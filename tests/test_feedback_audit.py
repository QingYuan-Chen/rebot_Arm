"""有界审计、真实 PTY 字节转发和生产假设备停止事务；不接触真机串口。"""
import importlib.util
import json
import logging
import os
from pathlib import Path
import select
import threading
import time
import tty
from types import SimpleNamespace

import pytest

from test_controller_local_runtime import make_local_manager, wait_until

SPEC = importlib.util.spec_from_file_location('audit_feedback_session',
    Path(__file__).parents[1] / 'scripts/audit_feedback_session.py')
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def test_queue_overflow_is_counted_without_waiting_for_disk(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = AUDIT.AuditWriter._line
    def blocked(self, item):
        entered.set()
        assert release.wait(2.)
        original(self, item)
    monkeypatch.setattr(AUDIT.AuditWriter, '_line', blocked)
    writer = AUDIT.AuditWriter(tmp_path/'audit.jsonl', 'offline', capacity=2)
    record = logging.makeLogRecord(dict(msg='probe', audit={'sequence': 1}))
    writer.handle(record)
    assert entered.wait(1.)
    for _ in range(30):
        writer.handle(record)
    assert writer.dropped == 28
    release.set()
    summary = writer.finish()
    assert summary['written'] == summary['accepted'] == 3
    assert summary['dropped'] == 28 and summary['errors'] == 0
    assert json.loads((tmp_path/'audit.jsonl').read_text().splitlines()[-1]) == summary


def test_audit_encoding_error_is_not_reported_as_complete(tmp_path):
    writer = AUDIT.AuditWriter(tmp_path/'error.jsonl', 'offline')
    writer.handle(logging.makeLogRecord(dict(msg='invalid', audit={'value': float('nan')})))
    summary = writer.finish()
    assert summary['errors'] == 1 and summary['written'] == 0
    assert 'JSON' in summary['failure']


def test_serial_relay_preserves_raw_bytes_both_directions_on_virtual_ports(caplog):
    pytest.importorskip('serial')
    master, slave = os.openpty()
    tty.setraw(slave)
    relay = AUDIT.SerialRelay(os.ttyname(slave), 921600)
    payload = bytes.fromhex('aa11080100000000123456789abcdef055')
    with caplog.at_level(logging.INFO, logger='rebotarm.audit'):
        try:
            os.write(master, payload)
            assert select.select([relay.slave], [], [], 1.)[0]
            assert os.read(relay.slave, 100) == payload
            command = bytes.fromhex('55aa1e03010000000a00000000ff070000000800000700cc000000000000')
            os.write(relay.slave, command)
            assert select.select([master], [], [], 1.)[0]
            assert os.read(master, 100) == command
            assert not relay.error
        finally:
            relay.close()
            os.close(master)
            os.close(slave)
    rows = [record.audit for record in caplog.records if record.msg == 'raw_serial']
    assert b''.join(bytes.fromhex(row['hex']) for row in rows if row['direction'] == 'rx') == payload
    assert b''.join(bytes.fromhex(row['hex']) for row in rows if row['direction'] == 'tx') == command


def test_relay_splits_coalesced_commands_and_waits_for_fragment_completion(monkeypatch, caplog):
    serial = pytest.importorskip('serial')
    original = serial.Serial
    writes = []
    class ObservedSerial(original):
        def write(self, data):
            writes.append((time.monotonic_ns(), bytes(data)))
            return super().write(data)
    monkeypatch.setattr(serial, 'Serial', ObservedSerial)
    master, slave = os.openpty()
    tty.setraw(slave)
    relay = AUDIT.SerialRelay(os.ttyname(slave), 921600)
    frame = bytes.fromhex('55aa1e03010000000a00000000ff070000000800000700cc000000000000')
    with caplog.at_level(logging.INFO, logger='rebotarm.audit'):
        try:
            os.write(relay.slave, frame[:9])
            time.sleep(.02)
            assert not writes  # 不把半帧送到 USB 适配器。
            os.write(relay.slave, frame[9:] + frame * 6)
            wait_until(lambda: len(writes) == 7 or bool(relay.error))
            assert not relay.error
            assert [data for _, data in writes] == [frame] * 7
            assert all(b[0]-a[0] >= 120_000 for a, b in zip(writes, writes[1:]))
            received = bytearray()
            while len(received) < 210:
                assert select.select([master], [], [], 1.)[0]
                received.extend(os.read(master, 4096))
            assert received == frame * 7
        finally:
            relay.close()
            os.close(master)
            os.close(slave)
    forwarded = [row.audit for row in caplog.records if row.msg == 'relay_forward']
    assert len(forwarded) == 7
    assert all(row['completed_ns'] >= row['started_ns'] >= row['read_ns'] for row in forwarded)


@pytest.mark.parametrize('payload', [b'bad header', b'\x55\xaa\x1e\x03'])
def test_relay_invalid_or_incomplete_command_marks_evidence_incomplete(payload):
    pytest.importorskip('serial')
    master, slave = os.openpty()
    tty.setraw(slave)
    relay = AUDIT.SerialRelay(os.ttyname(slave), 921600)
    try:
        os.write(relay.slave, payload)
        time.sleep(.03)
        relay.close()
        assert relay.error
        assert not select.select([master], [], [], .01)[0]
        relay.close()  # 清理入口可以重复调用。
    finally:
        relay.close()
        os.close(master)
        os.close(slave)


def test_relay_close_reports_unread_commands_without_sending_them(monkeypatch):
    pytest.importorskip('serial')
    monkeypatch.setattr(AUDIT.SerialRelay, '_run', lambda self: self.stop.wait())
    master, slave = os.openpty()
    tty.setraw(slave)
    relay = AUDIT.SerialRelay(os.ttyname(slave), 921600)
    try:
        os.write(relay.slave, b'queued command')
        relay.close()
        assert 'unforwarded' in relay.error
        assert not select.select([master], [], [], .01)[0]
    finally:
        relay.close()
        os.close(master);os.close(slave)


@pytest.mark.parametrize('close_error', [False, True])
def test_entrypoint_drains_writer_when_node_startup_or_relay_cleanup_fails(tmp_path, monkeypatch, close_error):
    class Relay:
        channel = '/dev/pts/offline'
        def __init__(self, *args):
            self.error = ''
        def close(self):
            if close_error:
                raise RuntimeError('relay cleanup failed')
    def startup(**kwargs):
        logging.getLogger('rebotarm.audit').info('offline_startup_failure')
        raise RuntimeError('node startup failed')
    monkeypatch.setattr(AUDIT, 'SerialRelay', Relay)
    module = SimpleNamespace(__file__=str(tmp_path/'node.py'), main=startup)
    monkeypatch.setattr(AUDIT.importlib, 'import_module', lambda name: module)
    path = tmp_path/'failure.jsonl'
    with pytest.raises(RuntimeError, match='relay cleanup failed' if close_error else 'node startup failed'):
        AUDIT.main(['--output', str(path), '--session', 'offline', '--module', 'offline',
                    '--serial', '/dev/pts/fake', '--', '{audit_serial}'])
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[-1]['event'] == 'audit_summary'
    assert rows[-1]['accepted'] == rows[-1]['written'] == 2 + int(close_error)
    assert any(row['event'] == 'relay_cleanup_error' for row in rows) is close_error
    assert rows[-1]['dropped'] == rows[-1]['errors'] == 0
    assert not any(worker.name == 'feedback-audit' for worker in threading.enumerate())


@pytest.mark.parametrize('through_relay', [False, True])
def test_installed_sdk_seven_motor_startup_on_virtual_serial(through_relay):
    """使用真实 ABI 和生产反馈组件；虚拟响应端含分段及粘包，不触碰硬件。"""
    motorbridge = pytest.importorskip('motorbridge')
    if motorbridge.__version__ != '0.4.7+rebotarm.1':
        pytest.skip('requires the verified MotorBridge wheel')
    from rebotarmcontroller.feedback_acquisition import FeedbackAcquisition
    from test_motorbridge_transport_safety import packet
    master, slave = os.openpty()
    tty.setraw(slave)
    relay = AUDIT.SerialRelay(os.ttyname(slave), 921600) if through_relay else None
    controller = None
    motors = []
    stop = threading.Event()
    commands, errors = [], []
    def respond():
        pending = bytearray()
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .01)[0]:
                    continue
                pending.extend(os.read(master, 4096))
                while len(pending) >= 30:
                    frame = bytes(pending[:30]);del pending[:30]
                    commands.append(frame)
                    assert frame[:4] == bytes.fromhex('55aa1e03')
                    number = frame[21]
                    assert frame[21:29] == bytes([number, 0, 0xcc, 0, 0, 0, 0, 0])
                    data = packet(can_id=0x10+number, motor_id=number)
                    os.write(master, data[:5])
                    os.write(master, data[5:])
        except BaseException as exc:
            errors.append(exc)
    worker = threading.Thread(target=respond, name='virtual-motors')
    worker.start()
    try:
        controller = motorbridge.Controller.from_dm_serial(relay.channel if relay else os.ttyname(slave), 921600)
        motors = [controller.add_damiao_motor(i, 0x10+i, '4340P' if i <= 3 else '4310') for i in range(1, 8)]
        labels = [f'joint{i}' for i in range(1, 7)]+['gripper']
        feedback = FeedbackAcquisition(
            groups=lambda: [(controller, list(zip(labels, motors)))], joint_names=lambda: labels[:6],
            gripper_motor=lambda: motors[-1],
            gripper_values=lambda state: (state.pos, state.vel, state.torq, state.status_code),
            gripper_sample=lambda *args, **kwargs: None, gripper_error=lambda reason: None,
            period=lambda: .02, stale_timeout=lambda: .15,
            backend=lambda: SimpleNamespace(control_thread=None), loop_active=lambda: False,
        )
        feedback._force_feedback_refresh()
        assert set(feedback._verified_feedback_by_label) == set(labels)
        assert all(sample.sequence > 0 and sample.state.status_code == 0
                   for sample in feedback._verified_feedback_by_label.values())
        assert not feedback._feedback_error_by_label
    finally:
        for motor in motors:
            motor.close()
        if controller:
            controller.close()
        stop.set();worker.join(1.)
        if relay:
            relay.close()
        os.close(master);os.close(slave)
    assert not worker.is_alive() and not errors
    assert {frame[21] for frame in commands} == set(range(1, 8))
    assert not relay or not relay.error


def test_stable_idle_stop_preserves_actual_old_target_and_audits_receipts(monkeypatch, caplog):
    manager, _ = make_local_manager(monkeypatch)
    with caplog.at_level(logging.INFO, logger='rebotarm.audit'):
        manager.connect()
        manager.enable()
        try:
            wait_until(lambda: manager.arm._hold_evidence.confirmed())
            original = manager.joint_target().copy()
            writes = []
            set_target = manager._backend.set_target
            monkeypatch.setattr(manager._backend, 'set_target', lambda value: writes.append(value) or set_target(value))
            generation = manager._backend.target_generation
            manager.stop_active_motion()
            assert not writes
            assert manager._backend.target_generation == generation+1
            assert list(manager.joint_target()) == list(original)
            assert manager.enabled
        finally:
            manager.disable()
            manager.shutdown()
    assert any(row.msg == 'stop_target' and row.audit['preserved'] for row in caplog.records)
    assert any(row.msg == 'position_receipt' and row.audit['success'] for row in caplog.records)
    assert any(row.msg == 'motor_state' and row.audit['sequence'] > 0 for row in caplog.records)


def test_duplicate_stop_joins_one_transaction_and_moving_cancel_retargets(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    errors, results = [], []
    def stop():
        try:
            manager.cancel_arm_trajectory()
            results.append(True)
        except Exception as exc:
            errors.append(str(exc))
    workers = [threading.Thread(target=stop) for _ in range(2)]
    try:
        # 正在运动的轨迹不能保留远处目标。
        task = manager.submit_arm_trajectory(manager.joint_names,
            [(0.,)*6, (.1, -.1, -.1, 0., 0., 0.)], [0., 10.])
        for motor in opened[0].motors[:6]:
            motor.state.vel = .2
        workers[0].start()
        wait_until(lambda: manager.arm._stop.pending)
        generation = manager._backend.target_generation
        workers[1].start()
        time.sleep(.04)
        assert workers[1].is_alive()
        assert generation == manager._backend.target_generation
        for motor in opened[0].motors[:6]:
            motor.state.vel = 0.
        for worker in workers:
            worker.join(1.)
            assert not worker.is_alive()
        assert results == [True, True] and not errors
        assert manager.arm_execution_status(task).state == 'canceled'
        assert list(manager.joint_target()) == [0.]*6
    finally:
        for worker in workers:
            if worker.ident is not None:
                worker.join(3.)
        manager.disable()
        manager.shutdown()


def test_obsolete_stop_cleanup_cannot_abort_successor_transaction(monkeypatch):
    manager, _ = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    try:
        def transactions():
            first = manager.arm.stop_active_motion()
            old = manager.arm._stop
            old.abort('old session ended')
            second = manager.arm.stop_active_motion()
            current = manager.arm._stop
            assert current is not old and current.pending
            first.cancel('obsolete deferred request')
            assert current.pending
            second.cancel('test cleanup')
        manager._backend.call(transactions)
        assert manager.enabled
    finally:
        manager.disable()
        manager.shutdown()


def test_isolated_velocity_peak_does_not_reanchor_recently_confirmed_idle_hold(monkeypatch, caplog):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    timer = None
    try:
        wait_until(lambda: manager.arm._hold_evidence.confirmed())
        motor = opened[0].motors[3]
        motor.state.vel = -.10988998413085938
        wait_until(lambda: manager.arm._hold_evidence.reason == 'velocity')
        writes = []
        original = manager._backend.set_target
        monkeypatch.setattr(manager._backend, 'set_target', lambda value: writes.append(value) or original(value))
        timer = threading.Timer(.08, lambda: setattr(motor.state, 'vel', 0.))
        timer.start()
        started = time.monotonic()
        with caplog.at_level(logging.INFO, logger='rebotarm.audit'):
            manager.stop_active_motion()
        assert time.monotonic()-started >= .20
        assert not writes
        assert any(row.msg == 'stop_target' and row.audit['preserved'] for row in caplog.records)
        assert manager.enabled
    finally:
        if timer is not None:
            timer.join(1.)
        manager.disable()
        manager.shutdown()
