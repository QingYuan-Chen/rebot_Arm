"""固定数值样本驱动生产组件；C++ 应复用 JSON 输入、输出和同一容差。"""
import json
from pathlib import Path
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
from test_controller_local_runtime import make_driver
from test_lifecycle_transactions import lifecycle

DATA = json.loads((Path(__file__).parent / 'fixtures/controller_reference_v1.json').read_text())


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


@pytest.mark.parametrize('case', DATA['stop'], ids=lambda case: case['name'])
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
