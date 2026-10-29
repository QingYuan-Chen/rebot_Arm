"""固定数值样本驱动生产组件；C++ 应复用 JSON 输入、输出和同一容差。"""
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.arm_execution import ArmExecution
from rebotarmcontroller.arm_position_control import ArmPositionControl
from rebotarmcontroller.arm_trajectory import ArmTrajectory
from rebotarmcontroller.arm_stop import ArmStop
from rebotarmcontroller.command_guard import CommandGuard
from rebotarmcontroller.controller_values import GripperTarget
from rebotarmcontroller.gripper_execution import GripperExecution
from rebotarmcontroller.sdk_driver import CommandTransportError
from test_controller_local_runtime import make_driver, make_local_manager, wait_until
from test_lifecycle_transactions import lifecycle

DATA = json.loads((Path(__file__).parent / 'fixtures/controller_reference_v1.json').read_text())


@pytest.mark.parametrize('velocity', [[0.] * 6, [-.1] * 6, [.1] * 5,
                                      [float('nan')] * 6, [float('inf')] * 6])
def test_rejected_velocity_preserves_hold_and_allows_valid_retry(velocity):
    driver, bus, _ = make_driver()
    driver.connect()
    execution = ArmExecution()
    execution.running = True
    execution.set_speed_override([.1] * 6)
    generation = execution.generation
    trajectory = ArmTrajectory(execution, clock=lambda: 10.)
    control = ArmPositionControl(j.vlim for j in driver.config.joints)
    points = [[0.] * 6, [.01, 0., 0., 0., 0., 0.]]
    try:
        with pytest.raises(ValueError, match='velocity override'):
            trajectory.submit(points, [0., 1.], np.zeros(6), velocity=velocity)
        assert trajectory.task is None and not execution.moving
        assert execution.generation == generation
        # 即使已过被拒轨迹的末点, 发送也必须沿用旧保持目标与速度覆盖.
        command = control.command(execution.sample(12.))
        driver.send_positions(command)
        assert command.positions == (0.,) * 6 and command.velocity_limits == (.1,) * 6
        assert all(args[0] == 0. for _, method, args in bus.events if method == 'position')
        task = trajectory.submit(points, [0., 1.], np.zeros(6), velocity=[.2] * 6)
        target = execution.sample(11.)
        assert target.positions == tuple(points[-1]) and target.velocity_override == (.2,) * 6
        assert task.generation == target.generation == generation + 1
    finally:
        driver.close()


def test_rejected_trajectory_never_reaches_fake_device_and_valid_retry_completes(monkeypatch):
    manager, opened = make_local_manager(monkeypatch)
    manager.lifecycle.connect()
    try:
        manager.lifecycle.enable()
        current = manager.feedback.get_joint_state()[0].tolist()
        goal = list(current)
        goal[0] += .01
        generation = manager.runtime.target_generation
        with pytest.raises(ValueError, match='velocity override'):
            manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
                [current, goal], [0., .01], velocity=[0.] * 6)
        assert manager.runtime.target_generation == generation
        assert manager.arm._arm_trajectory.task is None
        # 经过被拒请求的末点后, 唯一运行时仍只能发送原保持目标.
        after = time.monotonic() + .02
        wait_until(lambda: manager.feedback.get_cached_joint_sample()[-1] > after)
        assert all(args[0] == current[0] for number, method, args in opened[0].events
                   if number == 1 and method == 'position')
        task = manager.arm.submit_arm_trajectory(manager.runtime.joint_names,
            [current, goal], [0., .01], velocity=[.2] * 6)
        assert task.event.wait(1.)
        assert manager.arm.arm_execution_status(task).state == 'succeeded'
        assert opened[0].motors[0].state.pos == goal[0]
        assert manager.lifecycle.enabled and opened[0].disables == 0
    finally:
        manager.lifecycle.disable()
        manager.lifecycle.shutdown()


@pytest.mark.parametrize('case', DATA['arm'], ids=lambda case: case['name'])
def test_arm_request_command_receipt_feedback_result(case):
    driver, bus, _ = make_driver()
    driver.connect()
    clock = [0.]
    execution = ArmExecution()
    execution.running = True
    trajectory = ArmTrajectory(execution, clock=lambda: clock[0])
    control = ArmPositionControl(j.vlim for j in driver.config.joints)
    task = trajectory.submit([[0.] * 6, case['goal']], [0., 1.], np.zeros(6))
    try:
        for step in case['steps']:
            clock[0] = step['time']
            target = execution.sample(clock[0])
            command = control.command(target)
            receipt = driver.send_positions(command)
            assert receipt.success and receipt.sent == tuple(j.name for j in driver.config.joints)
            np.testing.assert_allclose(command.positions,
                [0.] * 6 if clock[0] < 1 else case['goal'], atol=DATA['absolute_tolerance'], rtol=0)
            assert command.generation == task.generation
            execution.acknowledge(target)
            hold = trajectory.update(step['feedback'], step['observed_at'])
            result = trajectory.snapshot(task)
            assert result.state == step['state'] and execution.progress == step['progress']
        assert (None if result.decision is None else result.decision.action.value) == case['decision']
        if 'hold' in case:
            np.testing.assert_allclose(hold, case['hold'], atol=DATA['absolute_tolerance'], rtol=0)
        assert not any(method == 'disable' for _, method, _ in bus.events)
    finally:
        driver.close()


def test_partial_send_retains_cursor_and_protection_decision():
    case = DATA['partial_write']
    driver, bus, _ = make_driver()
    driver.connect()
    execution = ArmExecution()
    execution.running = True
    execution.submit([case['goal']], [0.], execution.generation, now=0.)
    target = execution.sample(0.)
    command = ArmPositionControl(j.vlim for j in driver.config.joints).command(target)
    bus.failure = (case['failure_motor'], 'position')
    try:
        with pytest.raises(CommandTransportError) as raised:
            driver.send_positions(command)
        receipt = raised.value.receipt
        assert list(receipt.sent) == case['sent'] and receipt.failed == case['failed']
        assert list(receipt.not_attempted) == case['not_attempted']
        assert execution.progress == case['progress'] and execution.moving
        guard = CommandGuard()
        assert guard.begin_protection(str(raised.value), 'COMMAND')
        assert guard.protection.action.value == case['decision']
        assert not guard.begin_protection('same fault', 'COMMAND')
    finally:
        driver.close()


# v1 停止样本作为历史协议保留；当前策略使用 v2，不能改写旧预期。
STOP_V2 = json.loads((Path(__file__).parent / 'fixtures/controller_stop_reference_v2.json').read_text())


@pytest.mark.parametrize('case', STOP_V2['stop'], ids=lambda case: case['name'])
def test_stop_reference(case):
    clock = [0.]
    stop = ArmStop(clock=lambda: clock[0])
    stop.begin([0.] * 6, 1)
    stop.sent(1)
    for now, stamp in zip(case['clock_times'], case['feedback_times']):
        clock[0] = now
        result = stop.update([0.] * 6, [case['velocity']] * 6, stamp, generation=1, enabled=True)
    assert result.state == case['state']
    assert (None if result.decision is None else result.decision.action.value) == case['decision']


def test_lifecycle_reference():
    owner, device = lifecycle()
    for step in DATA['lifecycle']:
        device.calls.clear()
        getattr(owner, step['request'])()
        assert owner.state == step['state'] and owner.enabled == step['enabled']
        assert device.calls == step['writes']


def test_gripper_request_neutral_write_before_success():
    case = DATA['gripper']
    writes = []
    state = SimpleNamespace(pos=case['start_angle'], vel=0., torq=0., status_code=1)
    execution = GripperExecution(
        motor=lambda: object(), verified=lambda: state, feedback_failure=lambda: None,
        record_feedback_error=lambda reason: None, validate=lambda sample: (sample.pos, 0., 0., 1),
        feedback_age=lambda: 0., feedback_updated=lambda: 1.,
        speed=lambda: .3, send=lambda *args, **kwargs: writes.append((args, kwargs)) or True,
        clock=lambda: 1.)
    generation = execution.accept_position(GripperTarget(**{
        name: case[name] for name in ('start_angle', 'goal_angle', 'effort', 'timeout')}), 0.)
    assert execution.snapshot(generation).state == 'active'
    execution._gripper_tick()
    assert execution.snapshot(generation).state == case['state']
    np.testing.assert_allclose(writes[-1][0], case['command'], atol=DATA['absolute_tolerance'], rtol=0)
    assert writes[-1][1]['tau_limit'] == case['tau_limit']
    assert execution.snapshot(generation - 1).state == 'superseded'
