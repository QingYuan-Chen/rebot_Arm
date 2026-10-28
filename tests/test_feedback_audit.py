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
            os.write(relay.slave, b'\x55\xaa\x00')
            assert select.select([master], [], [], 1.)[0]
            assert os.read(master, 100) == b'\x55\xaa\x00'
            assert not relay.error
        finally:
            relay.close()
            os.close(master)
            os.close(slave)
    rows = [record.audit for record in caplog.records if record.msg == 'raw_serial']
    assert b''.join(bytes.fromhex(row['hex']) for row in rows if row['direction'] == 'rx') == payload
    assert b''.join(bytes.fromhex(row['hex']) for row in rows if row['direction'] == 'tx') == b'\x55\xaa\x00'


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
