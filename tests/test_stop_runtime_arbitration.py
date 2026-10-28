"""停止确认使用生产运行时与假设备；验证紧急请求不会等待静止窗口。"""
import threading
import time
from types import SimpleNamespace

import pytest

from test_controller_local_runtime import make_local_manager, wait_until


def test_disable_preempts_pending_stop_and_rejects_new_motion(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    bus = opened[0]
    for motor in bus.motors[:6]:
        motor.state.vel = .2
    errors = []
    done = threading.Event()
    def stop():
        try:
            manager.stop_active_motion()
        except Exception as exc:
            errors.append(str(exc))
        finally:
            done.set()
    worker = threading.Thread(target=stop)
    worker.start()
    try:
        wait_until(lambda: manager.arm._stop.pending)
        with pytest.raises(RuntimeError, match='pending'):
            manager.set_joint_target([0.] * 6)
        with pytest.raises(RuntimeError, match='pending'):
            manager.start_gravity_compensation()
        started = time.monotonic()
        manager.disable()
        # 假设备无硬件延迟；若写入线程仍阻塞式等静止，这里会接近 2 s。
        assert time.monotonic() - started < .5
        assert done.wait(.5) and errors
        assert not manager.enabled and manager.lifecycle_state == 'CONNECTED_DISABLED'
        count = len(bus.events)
        time.sleep(.03)
        assert not any(method == 'position' for _, method, _ in bus.events[count:])
    finally:
        worker.join(3.)
        if manager.enabled:
            manager.disable()
        manager.shutdown()


def test_stop_timeout_keeps_healthy_hardware_enabled_and_holding(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    bus = opened[0]
    for motor in bus.motors[:6]:
        motor.state.vel = .2
    before = bus.disables
    try:
        with pytest.raises(RuntimeError, match='stationary'):
            manager.stop_active_motion()
        assert manager.enabled and manager.control_loop_active
        assert bus.disables == before
        assert manager.arm._stop.result.state == 'timeout'
        assert len([event for event in bus.events if event[1] == 'position']) > 12
    finally:
        manager.disable()
        manager.shutdown()


def test_disable_cancels_passthrough_continuation_before_any_direct_write(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.connect()
    manager.enable()
    bus = opened[0]
    for motor in bus.motors[:6]:
        motor.state.vel = .2
    task = manager.submit_arm_trajectory(manager.joint_names,
        [(0.,) * 6, (.01, -.01, -.01, 0., 0., 0.)], [0., 10.])
    command = SimpleNamespace(mode=1, pos=.02, vel=0., kp=1., kd=.1, tau=0., vlim=.3,
        **{f'use_{key}': True for key in ('pos', 'vel', 'kp', 'kd', 'tau', 'vlim')})
    errors = []
    def preempt():
        try:
            manager.send_joint_motor_cmd('joint1', command, arbitration='preempt')
        except Exception as exc:
            errors.append(str(exc))
    worker = threading.Thread(target=preempt)
    worker.start()
    try:
        wait_until(lambda: manager.arm._stop.pending)
        manager.disable()
        worker.join(.5)
        assert not worker.is_alive() and errors
        assert task.event.is_set() and manager.arm_execution_status(task).state == 'aborted'
        # .02 只属于抢占请求，周期保持输出为 0；不能因停止等待结束而迟到发送。
        assert not any(number == 1 and method == 'position' and args[0] == .02
                       for number, method, args in bus.events)
        assert not manager.enabled
    finally:
        worker.join(3.)
        if manager.enabled:
            manager.disable()
        manager.shutdown()


def test_runtime_close_finishes_waiter_of_deferred_request(monkeypatch):
    from rebotarmcontroller.arm_stop import CycleResult
    manager, _ = make_local_manager(monkeypatch)
    runtime = manager._backend
    accepted = threading.Event()
    errors = []
    def operation():
        accepted.set()
        return CycleResult(lambda: (False, None))
    def call():
        try:
            runtime.call(operation)
        except Exception as exc:
            errors.append(str(exc))
    worker = threading.Thread(target=call)
    worker.start()
    assert accepted.wait(1.)
    runtime.close_worker()
    worker.join(1.)
    assert not worker.is_alive() and errors
