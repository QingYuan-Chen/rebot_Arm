"""现场回归门禁只用模拟 future，绝不连接设备。"""
import concurrent.futures
import threading

import pytest

from scripts.hardware_regression_gate import MonitorFault, RegressionGate, SustainedSpeed


class Handle:
    def __init__(self):
        self.accepted = True
        self.result = concurrent.futures.Future()
        self.cancels = 0
        self.cancelled = threading.Event()

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        self.cancelled.set()
        return concurrent.futures.Future()


def fresh_gate():
    stops = []
    now = [1.0]
    gate = RegressionGate(lambda: stops.append('hold'), clock=lambda: now[0])
    assert gate.monitor_check(lambda active: []) == ()
    return gate, stops, now


def test_fault_before_late_acceptance_cancels_and_blocks_return_goal():
    gate, stops, _ = fresh_gate()
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    assert gate.gripper_motion
    gate.monitor_check(lambda active: ['source_stale'])
    assert stops == ['hold']
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('不能发送返程目标'), gripper=True)
    handle = Handle()
    pending.set_result(handle)
    assert handle.cancelled.wait(1) and handle.cancels == 1
    handle.result.set_result('cancelled')
    assert not gate.gripper_motion
    assert gate.fault == ('source_stale',)
    with pytest.raises(MonitorFault):
        gate.guarded_call(lambda: pytest.fail('不能再次使能'))


def test_accepted_goal_fault_cancels_and_fault_survives_recovered_feedback():
    gate, stops, _ = fresh_gate()
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = Handle()
    pending.set_result(handle)
    assert gate.result_for(pending) == (handle, handle.result)
    gate.monitor_check(lambda active: ['unexpected_position'])
    assert handle.cancelled.wait(1) and handle.cancels == 1 and stops == ['hold']
    gate.monitor_check(lambda active: [])
    handle.result.set_result('cancelled')
    assert gate.fault == ('unexpected_position',)
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('已恢复反馈不能清除锁存'))


def test_gripper_motion_uses_its_bound_without_relaxing_six_axis():
    gate, _, _ = fresh_gate()
    speed = SustainedSpeed(.07, .5)
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    for stamp in (1_000_000_000, 1_110_000_000):
        assert not speed.observe('gripper', stamp, .2, gate.gripper_motion)
    assert not speed.observe('joint4', 1_000_000_000, .2, gate.gripper_motion)
    assert speed.observe('joint4', 1_110_000_000, .2, gate.gripper_motion)
    assert not speed.observe('gripper', 1_120_000_000, .6, gate.gripper_motion)
    assert speed.observe('gripper', 1_230_000_000, .6, gate.gripper_motion)
    handle = Handle()
    pending.set_result(handle)
    handle.result.set_result('done')
    assert not gate.gripper_motion
    assert not speed.observe('gripper', 1_240_000_000, .2, gate.gripper_motion)
    assert speed.observe('gripper', 1_350_000_000, .2, gate.gripper_motion)


def test_repeated_source_frames_do_not_accumulate_and_regression_rejected():
    speed = SustainedSpeed(.07, .5)
    assert not speed.observe('joint1', 100, .2, False)
    for _ in range(100):
        assert not speed.observe('joint1', 100, .2, False)
    assert speed.observe('joint1', 99, .2, False)


def test_watchdog_stops_if_monitor_is_stuck_inside_check():
    gate, stops, now = fresh_gate()
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = Handle()
    pending.set_result(handle)
    entered, release = threading.Event(), threading.Event()
    def stuck(_):
        entered.set()
        assert release.wait(2)
        return []
    thread = threading.Thread(target=lambda: gate.monitor_check(stuck))
    thread.start()
    assert entered.wait(1)
    now[0] += .2
    watchdog = threading.Thread(target=gate.watchdog)
    watchdog.start()
    for _ in range(1000):
        if stops: break
        threading.Event().wait(.001)
    assert stops == ['hold']
    assert gate.fault == ('monitor_stale',)
    assert handle.cancelled.wait(1) and handle.cancels == 1
    release.set()
    thread.join(1); watchdog.join(1)
    assert not thread.is_alive() and not watchdog.is_alive()
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('监测失活不能发送目标'))


def test_late_acceptance_is_cancelled_while_monitor_lock_is_stuck():
    gate, _, now = fresh_gate()
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    entered, release = threading.Event(), threading.Event()
    def stuck(_):
        entered.set()
        assert release.wait(2)
        return []
    monitor = threading.Thread(target=lambda: gate.monitor_check(stuck))
    monitor.start()
    assert entered.wait(1)
    now[0] += .2
    gate.watchdog()
    handle = Handle()
    accept = threading.Thread(target=lambda: pending.set_result(handle))
    accept.start()
    for _ in range(1000):
        if handle.cancels: break
        threading.Event().wait(.001)
    assert handle.cancels == 1
    release.set()
    monitor.join(1); accept.join(1)
    assert not monitor.is_alive() and not accept.is_alive()


def test_monitor_exception_and_send_failure_fail_closed():
    gate, stops, _ = fresh_gate()
    def broken(_):
        raise ValueError('broken')
    assert gate.monitor_check(broken) == ('monitor_exception: broken',)
    assert stops == ['hold']
    gate2, stops2, _ = fresh_gate()
    def send_failure():
        raise RuntimeError('unknown submission')
    with pytest.raises(RuntimeError):
        gate2.submit(send_failure, gripper=True)
    assert gate2.fault == ('goal_send_unknown',) and stops2 == ['hold']


def test_pending_or_unknown_terminal_never_allows_new_goal():
    gate, stops, _ = fresh_gate()
    future = concurrent.futures.Future()
    gate.submit(lambda: future, gripper=True)
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('旧动作接纳未确认'))
    handle = Handle()
    future.set_result(handle)
    handle.result.set_exception(RuntimeError('unknown terminal'))
    assert stops == ['hold']
    assert gate.fault == ('goal_terminal_unknown',)
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('未知终态不能继续'))


def test_unclaimed_late_terminal_remains_unresolved():
    gate, _, _ = fresh_gate()
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    gate.trip('acceptance_timeout')
    handle = Handle()
    pending.set_result(handle)
    handle.result.set_result('cancelled')
    assert gate.has_pending


def test_client_audit_write_errors_are_visible(tmp_path):
    from scripts.hardware_regression_io import Audit
    audit = Audit(tmp_path / 'events.jsonl')
    audit.record('invalid', value=object())
    for _ in range(1000):
        if audit.failed: break
        threading.Event().wait(.001)
    assert audit.failed
    with pytest.raises(RuntimeError, match='审计不完整'):
        audit.close()


def test_cancel_request_is_reported_separately_from_terminal():
    calls = []
    gate = RegressionGate(lambda: None, on_cancel=lambda future, error: calls.append((future, error)))
    gate.monitor_check(lambda active: [])
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = Handle()
    pending.set_result(handle)
    gate.trip('test_fault')
    assert handle.cancelled.wait(1)
    for _ in range(1000):
        if calls: break
        threading.Event().wait(.001)
    assert len(calls) == 1 and calls[0][0] is not None and calls[0][1] is None
    assert gate.has_pending and not handle.result.done()


def test_late_rclpy_future_acceptance_is_cancelled_without_ros_or_hardware():
    rclpy_task = pytest.importorskip('rclpy.task')
    gate, stops, _ = fresh_gate()
    pending = rclpy_task.Future()
    gate.submit(lambda: pending, gripper=True)
    gate.trip('simulated_fault')
    handle = Handle()
    pending.set_result(handle)
    assert handle.cancelled.wait(1) and handle.cancels == 1 and stops == ['hold']
    assert gate.has_pending


def test_watchdog_stop_block_does_not_delay_accepted_gripper_cancel():
    stop_entered, release_stop = threading.Event(), threading.Event()
    def blocked_stop():
        stop_entered.set()
        assert release_stop.wait(2)
    gate = RegressionGate(blocked_stop)
    gate.monitor_check(lambda active: [])
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = Handle()
    pending.set_result(handle)
    thread = threading.Thread(target=lambda: gate.trip('monitor_stale'))
    thread.start()
    try:
        assert stop_entered.wait(1)
        assert handle.cancelled.wait(1)
        assert gate.fault == ('monitor_stale',) and gate.fault_at is not None
        with pytest.raises(MonitorFault):
            gate.submit(lambda: pytest.fail('故障时不能继续发送'))
    finally:
        release_stop.set()
        thread.join(2)
    assert not thread.is_alive()


def test_slow_cancel_does_not_delay_stop_or_keep_monitor_gate_locked():
    cancel_entered, release_cancel, stop_called = threading.Event(), threading.Event(), threading.Event()
    cancel_start = threading.Event()
    class SlowHandle(Handle):
        def cancel_goal_async(self):
            cancel_entered.set()
            assert release_cancel.wait(2)
            return super().cancel_goal_async()
    gate = RegressionGate(stop_called.set, on_cancel_start=cancel_start.set)
    gate.monitor_check(lambda active: [])
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = SlowHandle()
    pending.set_result(handle)
    try:
        gate.monitor_check(lambda active: ['source_stale'])
        assert cancel_entered.wait(1) and cancel_start.is_set() and stop_called.is_set()
        with pytest.raises(MonitorFault):
            gate.submit(lambda: pytest.fail('故障时不能继续发送'))
    finally:
        release_cancel.set()
        assert handle.cancelled.wait(1)


def test_blocked_goal_send_does_not_block_watchdog_or_late_cancel():
    send_entered, release_send, stop_called = threading.Event(), threading.Event(), threading.Event()
    gate = RegressionGate(stop_called.set)
    gate.monitor_check(lambda active: [])
    pending = concurrent.futures.Future()
    def blocked_send():
        send_entered.set()
        assert release_send.wait(2)
        return pending
    send_thread = threading.Thread(target=lambda: gate.submit(blocked_send, gripper=True))
    send_thread.start()
    assert send_entered.wait(1)
    fault_thread = threading.Thread(target=lambda: gate.trip('monitor_stale'))
    fault_thread.start()
    try:
        assert stop_called.wait(1)
        assert gate.fault == ('monitor_stale',)
    finally:
        release_send.set()
        send_thread.join(2)
        fault_thread.join(2)
    assert not send_thread.is_alive() and not fault_thread.is_alive()
    handle = Handle()
    pending.set_result(handle)
    assert handle.cancelled.wait(1) and handle.cancels == 1
    with pytest.raises(MonitorFault):
        gate.submit(lambda: pytest.fail('故障锁存后不能准入新目标'))


def test_blocked_enable_send_does_not_block_watchdog():
    send_entered, release_send, stop_called = threading.Event(), threading.Event(), threading.Event()
    gate = RegressionGate(stop_called.set)
    gate.monitor_check(lambda active: [])
    def blocked_send():
        send_entered.set()
        assert release_send.wait(2)
    send_thread = threading.Thread(target=lambda: gate.guarded_call(blocked_send))
    send_thread.start()
    assert send_entered.wait(1)
    fault_thread = threading.Thread(target=lambda: gate.trip('monitor_stale'))
    fault_thread.start()
    try:
        assert stop_called.wait(1)
        assert gate.fault == ('monitor_stale',)
    finally:
        release_send.set()
        send_thread.join(2)
        fault_thread.join(2)
    assert not send_thread.is_alive() and not fault_thread.is_alive()
    with pytest.raises(MonitorFault):
        gate.guarded_call(lambda: pytest.fail('故障锁存后不能准入使能'))


def test_cancel_in_flight_blocks_shutdown_even_after_action_terminal():
    cancel_entered, release_cancel, cancel_returned = threading.Event(), threading.Event(), threading.Event()
    cancel_response = concurrent.futures.Future()
    class SlowHandle(Handle):
        def cancel_goal_async(self):
            cancel_entered.set()
            assert release_cancel.wait(2)
            return cancel_response
    gate = RegressionGate(lambda: None, on_cancel=lambda future, error: cancel_returned.set())
    gate.monitor_check(lambda active: [])
    pending = concurrent.futures.Future()
    gate.submit(lambda: pending, gripper=True)
    handle = SlowHandle()
    pending.set_result(handle)
    gate.result_for(pending)
    gate.trip('monitor_stale')
    try:
        assert cancel_entered.wait(1)
        handle.result.set_result('terminal')
        assert gate.has_pending  # Action 虽已结束，取消调用仍未返回。
    finally:
        release_cancel.set()
    assert cancel_returned.wait(1)
    assert gate.has_pending  # 调用返回后仍要等待取消结果。
    cancel_response.set_result('cancel response')
    for _ in range(1000):
        if not gate.has_pending: break
        threading.Event().wait(.001)
    assert not gate.has_pending
