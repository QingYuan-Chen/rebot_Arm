"""用假时钟及假写入验证夹爪周期决策与独立对照样本。"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarmcontroller.gripper_control import GripperControl
from rebotarmcontroller.gripper_execution import GripperExecution


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
    return execution.start_grasp(close_force=.4, hold_force=.3, timeout=2., min_time=.1,
        velocity_limit=.04, min_closure=.006, hold_timeout=.5, start_position=.054, now=now)


def test_stall_is_decided_by_tick_and_hold_expires_to_neutral():
    execution, state, now, commands, _ = fixture()
    request = start(execution, now[0])
    state.pos = -2.5
    state.vel = 0.
    assert not request['event'].is_set()
    now[0] = .05
    execution._gripper_tick()
    assert not request['event'].is_set()
    now[0] = .2
    execution._gripper_tick()
    assert request['result'][:2] == (True, True)
    assert execution._gripper_mode == 'grasp_holding'
    assert commands[-1][0][2:4] == (5., 1.)
    now[0] = .7
    execution._gripper_tick()
    assert execution._gripper_mode == 'idle'
    assert commands[-1][0][2:4] == (0., 0.)
    count = len(commands)
    execution._gripper_tick()
    assert len(commands) == count


def test_hold_write_failure_does_not_report_success():
    execution, state, now, _, fail = fixture()
    request = start(execution, 0.)
    state.pos, state.vel = -2.5, 0.
    now[0], fail[0] = .2, True
    execution._gripper_tick()
    assert request['event'].is_set()
    assert request['result'][:2] == (False, False)
    assert 'write failed' in request['result'][-1]
    assert execution._gripper_mode == 'idle'


def test_cancel_or_supersede_never_emits_old_closing_command():
    execution, state, now, commands, _ = fixture()
    old = start(execution, 0.)
    assert execution.release_grasp_hold('cancelled')
    assert old['result'][-1] == 'cancelled'
    execution._gripper_tick()
    assert commands[-1][0][2:4] == (0., 0.)
    current = start(execution, 0.)
    newer = start(execution, 0.)
    assert current['result'][-1] == 'grasp superseded'
    execution.expire_grasp(current)
    assert execution._gripper_mode == 'grasp_closing'
    assert newer['result'] is None


def test_timeout_queues_neutral_without_reporting_contact():
    execution, state, now, commands, _ = fixture()
    request = start(execution, 0.)
    now[0] = 2.
    execution._gripper_tick()
    assert request['result'][:2] == (False, False)
    assert 'timeout' in request['result'][-1]
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


def test_neutral_failure_blocks_superseding_grasp():
    execution, _, _, _, fail = fixture()
    with execution._gripper_lock:
        execution._queue_gripper_neutral_locked('old target', marks_success=True)
    request = start(execution, 0.)
    fail[0] = True
    execution._gripper_tick()
    assert request['result'][:2] == (False, False)
    assert 'neutral command failed' in request['result'][-1]
    assert not execution._gripper_active


def test_feedback_from_old_generation_does_not_advance_new_target():
    execution, state, _, commands, _ = fixture()
    start(execution, 0.)
    def replace_while_reading():
        start(execution, 0.)
        return state
    execution._verified = replace_while_reading
    execution._gripper_tick()
    assert execution._gripper_mode == 'grasp_closing'
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
