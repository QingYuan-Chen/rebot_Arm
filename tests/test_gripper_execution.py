"""用假时钟及假写入验证夹爪周期决策与独立对照样本."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarmcontroller.gripper.gripper_control import GripperControl
from rebotarmcontroller.gripper.gripper_execution import GripperExecution
from rebotarmcontroller.contracts.controller_values import GripperTarget


VECTORS = json.loads((Path(__file__).parent / 'fixtures/controller_gripper_vectors.json').read_text())


@pytest.mark.parametrize('case', VECTORS['ramp'])
def test_ramp_vectors(case):
    assert GripperControl.ramp(case['target'], case['goal'], case['speed'], case['elapsed']) == pytest.approx(case['expected'])


@pytest.mark.parametrize('case', VECTORS['torque'])
def test_torque_vectors(case):
    pos, _, kp, kd, tau = GripperControl.bounded_mit(
        case['pos'], 0., case['kp'], case['kd'], case['feedforward'], case['limit'], case['q'], case['qd'])
    assert kp * (pos - case['q']) - kd * case['qd'] + tau == pytest.approx(case['expected_total'])


def fixture():
    now = [0.0]
    state = SimpleNamespace(pos=-3., vel=-0.1, torq=0., status_code=1)
    commands = []
    fail = [False]
    def send(*args, **kwargs):
        if fail[0]:
            raise RuntimeError('write failed')
        commands.append((args, kwargs))
    execution = GripperExecution(
        motor=lambda: object(), verified=lambda: state, feedback_failure=lambda: None,
        record_feedback_error=lambda reason: None,
        validate=lambda s: (s.pos, s.vel, s.torq, s.status_code),
        feedback_age=lambda: 0., feedback_updated=lambda: now[0], speed=lambda: .5,
        send=send, clock=lambda: now[0])
    execution._gripper_pos = state.pos
    return execution, state, now, commands, fail


def start(execution, now):
    return execution.accept_position(GripperTarget(-3., -2., .4, 2.), now)


def test_cancel_position_emits_neutral():
    execution, _, _, commands, _ = fixture()
    generation = start(execution, 0.)
    assert execution.cancel_gripper_position_command('cancelled', expected_generation=generation)
    execution._gripper_tick()
    assert execution.snapshot(generation).state == 'failed'
    assert commands[-1][0][2:4] == (0., 0.)


def test_old_position_waiter_cannot_cancel_identical_new_goal():
    execution, _, _, _, _ = fixture()
    with execution._gripper_lock:
        execution.supersede_locked()
        old_generation = execution._generation
        execution.supersede_locked()
        execution._gripper_active = True
        execution._gripper_mode = 'position'
        execution._gripper_goal_angle = -2.
    assert not execution.cancel_gripper_position_command(expected_generation=old_generation)
    assert not execution._release_gripper_position_target(-2., require_arrived=False,
                                                           expected_generation=old_generation)
    assert execution._gripper_active
    assert execution._gripper_neutral_pending is None


def test_old_neutral_confirmation_cannot_complete_new_position_command():
    execution, _, _, commands, _ = fixture()
    with execution._gripper_lock:
        execution._queue_gripper_neutral_locked('old target', marks_success=True)
        execution.supersede_locked()
        execution._gripper_mode = 'position'
        execution._gripper_active = True
        execution._gripper_position_result = 'active'
    execution._gripper_tick()
    assert commands[-1][0][2:4] == (0., 0.)
    assert execution._gripper_position_result == 'active'


def test_neutral_failure_blocks_superseding_position():
    execution, _, _, _, fail = fixture()
    with execution._gripper_lock:
        execution._queue_gripper_neutral_locked('old target', marks_success=True)
    generation = start(execution, 0.)
    fail[0] = True
    execution._gripper_tick()
    result = execution.snapshot(generation)
    assert result.state == 'failed'
    assert 'neutral command failed' in result.message
    assert not execution._gripper_active


def test_feedback_from_old_generation_does_not_advance_new_target():
    execution, state, _, commands, _ = fixture()
    start(execution, 0.)
    def replace_while_reading():
        start(execution, 0.)
        return state
    execution._verified = replace_while_reading
    execution._gripper_tick()
    assert execution._gripper_mode == 'position'
    assert commands == []


def test_position_deadline_releases_without_waiter():
    execution, _, now, commands, _ = fixture()
    execution._gripper_active = True
    execution._gripper_mode = 'position'
    execution._gripper_goal_angle = -4.
    execution._gripper_target_deadline_monotonic = 1.
    now[0] = 1.
    execution._gripper_tick()
    assert execution._gripper_position_result == 'failed'
    assert not execution._gripper_active
    assert commands[-1][0][2:4] == (0., 0.)
