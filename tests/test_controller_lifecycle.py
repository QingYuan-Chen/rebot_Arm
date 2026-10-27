"""验证生命周期与关闭协调，不连接设备。"""
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.arm_execution import ArmExecution
from rebotarmcontroller.lifecycle import ControllerLifecycle, shutdown_policy


class Hardware:
    def __init__(self, enabled=True):
        self.connected = True
        self.enabled = enabled
        self.calls = []
    def stop_gravity_compensation(self): self.calls.append('gravity_stop')
    def ensure_pos_vel_control(self): self.calls.append('position_hold')
    def stop_active_motion(self): self.calls.append('stop')
    def shutdown(self): self.calls.append('shutdown')


def logger():
    messages = []
    return SimpleNamespace(error=messages.append, warn=messages.append), messages


def test_safe_home_failure_retains_enabled_control():
    hardware = Hardware()
    log, messages = logger()
    def fail(): raise TimeoutError('cannot reach target')
    assert not shutdown_policy(hardware, safe_home=fail, logger=log)
    assert hardware.enabled
    assert hardware.calls == ['gravity_stop', 'position_hold', 'stop']
    assert any('cannot reach target' in message for message in messages)


def test_successful_parking_still_requires_explicit_disable():
    hardware = Hardware()
    log, _ = logger()
    assert not shutdown_policy(hardware, safe_home=lambda: hardware.calls.append('park'), logger=log)
    assert hardware.calls == ['park', 'gravity_stop', 'position_hold', 'stop']
    hardware.enabled = False
    assert shutdown_policy(hardware, safe_home=None, logger=log)
    assert hardware.calls[-1] == 'shutdown'


def test_stop_failure_does_not_disable_healthy_arm():
    hardware = Hardware()
    hardware.stop_active_motion = lambda: (_ for _ in ()).throw(RuntimeError('stop failed'))
    log, messages = logger()
    assert not shutdown_policy(hardware, safe_home=None, logger=log)
    assert hardware.enabled and 'shutdown' not in hardware.calls
    assert any('stop failed' in message for message in messages)


def test_lifecycle_does_not_hide_unconfirmed_disable():
    state = ControllerLifecycle()
    state.set_state('DISABLING')
    state.set_execution('IDLE', enabled=True)
    assert state.state == 'DISABLING'
    state.set_state('ENABLED_HOLD')
    state.set_execution('TRAJ_RUNNING', enabled=True)
    assert state.state == 'TRAJECTORY_RUNNING'
    state.set_execution('IDLE', enabled=True)
    assert state.state == 'ENABLED_HOLD'
    with pytest.raises(ValueError): state.set_state('UNKNOWN')


def test_scoped_target_cannot_clear_or_overwrite_successor():
    arm = ArmExecution()
    arm.running = True
    home = arm.begin_scoped_target((0.,)*6, (.3,)*6)
    assert arm.sample(0.).velocity_override == (.3,)*6
    successor = (0.1, -0.1, -0.1, 0., 0., 0.)
    arm.set_target(successor)
    assert not arm.finish_scoped_target(home, (0.,)*6)
    assert arm.target() == pytest.approx(successor)
    assert arm.sample(0.).velocity_override is None


def test_new_trajectory_revokes_scoped_override():
    arm = ArmExecution()
    arm.running = True
    home = arm.begin_scoped_target((0.,)*6, (.3,)*6)
    assert arm.submit([(0.,)*6], [0.], home, now=0.)
    assert arm.generation != home
    assert arm.sample(0.).velocity_override is None
    assert not arm.finish_scoped_target(home, np.zeros(6))


def test_signal_shutdown_keeps_executor_running_after_refusal(monkeypatch):
    import signal
    from rebotarmcontroller import rebotarm_controller as module

    events = []
    class Node:
        def __init__(self):
            self.hardware = None
        def shutdown(self):
            events.append('shutdown')
            return events.count('shutdown') == 2
        def destroy_node(self): events.append('destroy')
        def get_logger(self): return logger()[0]
    class Executor:
        def __init__(self, **kwargs): pass
        def add_node(self, node): pass
        def spin_once(self, timeout_sec):
            events.append('spin')
            if events.count('spin') == 1 or (events.count('shutdown') == 1 and events.count('spin') > 2):
                signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        def shutdown(self): events.append('executor_shutdown')
    monkeypatch.setattr(module, 'reBotArmController', Node)
    monkeypatch.setattr(module, 'MultiThreadedExecutor', Executor)
    monkeypatch.setattr(module.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: events.append('ros_shutdown'))
    module.main()
    first = events.index('shutdown')
    second = events.index('shutdown', first + 1)
    assert 'spin' in events[first + 1:second]
    assert events[-3:] == ['executor_shutdown', 'destroy', 'ros_shutdown']


def test_gravity_and_lifecycle_fixed_vectors():
    import json
    from pathlib import Path
    from rebotarmcontroller.gravity_control import GravityExecutionState
    vectors = json.loads((Path(__file__).parent / 'fixtures/controller_arm_vectors.json').read_text())
    for case in vectors['gravity_steps']:
        gravity = GravityExecutionState()
        gravity.target = np.full(6, case['target'])
        settings = SimpleNamespace(integral_gain=case['gain'], integral_limit_nm=case['limit'])
        result = gravity.step(np.full(6, case['position']), case['dt'], np.zeros(6),
                              [case['linear_speed'], 0., 0., 0., 0., 0.], settings)
        assert result == pytest.approx([case['integral']] * 6)
        assert gravity.target == pytest.approx([case['position'] if case['follow'] else case['target']] * 6)
    for case in vectors['lifecycle']:
        lifecycle = ControllerLifecycle()
        lifecycle.set_state(case['initial'])
        lifecycle.set_execution(case['execution'], enabled=case['enabled'])
        assert lifecycle.state == case['expected']
