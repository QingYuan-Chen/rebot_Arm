"""不构造硬件管理器，验证反馈组件及可供跨语言复用的固定样本。"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarmcontroller.feedback_acquisition import FeedbackAcquisition
import rebotarmcontroller.feedback_acquisition as acquisition


VECTORS = json.loads((Path(__file__).parent / 'fixtures/controller_feedback_vectors.json').read_text())


def component():
    return FeedbackAcquisition(
        groups=lambda: [], joint_names=lambda: ['joint1'], gripper_motor=lambda: None,
        gripper_values=lambda state: (state.pos, state.vel, state.torq, state.status_code),
        gripper_sample=lambda *args, **kwargs: None, gripper_error=lambda reason: None,
        period=lambda: .02, stale_timeout=lambda: .15,
        backend=lambda: SimpleNamespace(control_thread=None), loop_active=lambda: False,
    )


@pytest.mark.parametrize('case', VECTORS['sequence_cases'])
def test_sequence_vectors(case):
    assert FeedbackAcquisition._feedback_sequence_advanced(case['sequence'], case['baseline']) is case['advanced']


@pytest.mark.parametrize('case', VECTORS['health_cases'])
def test_health_vectors_are_read_only(case):
    feedback = component()
    reason = feedback._feedback_failure_reason(case['label'], error=case['error'],
                                                updated=case['updated'], now=case['now'])
    if case['code'] is None:
        assert reason is None
    else:
        assert f"[{case['code']}]" in reason
    assert not feedback._verified_feedback_by_label
    assert not feedback._feedback_error_by_label


def test_new_invalid_frame_preserves_old_sample_but_marks_failure():
    feedback = component()
    valid = SimpleNamespace(pos=0., vel=0., torq=0., status_code=0)
    invalid = SimpleNamespace(pos=float('nan'), vel=0., torq=0., status_code=0)
    feedback._record_verified_feedback('joint1', valid, 10, 1.)
    previous = feedback._verified_feedback_sample('joint1')
    feedback._feedback_request_baseline_by_label['joint1'] = 10
    feedback._feedback_request_deadline_by_label['joint1'] = 2.
    feedback._inspect_pending_feedback({'joint1': (invalid, 11)}, observed_at=1.1)
    assert feedback._verified_feedback_sample('joint1') is previous
    assert '[INVALID_VALUES]' in feedback._feedback_error_by_label['joint1']
    result = feedback._cached_motor_feedback_locked('joint1', now=1.1)
    assert result.status_code == 255
    assert result.sample is previous
    feedback._feedback_request_baseline_by_label['joint1'] = 11
    feedback._feedback_request_deadline_by_label['joint1'] = 2.
    feedback._inspect_pending_feedback({'joint1': (valid, 12)}, observed_at=1.2)
    recovered = feedback._cached_motor_feedback_locked('joint1', now=1.2)
    assert recovered.status_code == 0
    assert recovered.sample.sequence == 12
    assert recovered.sample.observed_at == 1.2


@pytest.mark.parametrize('reply_at,oversleep', [(.11, False), (None, False), (.16, False), (.11, True)])
def test_force_refresh_waits_for_last_request_inside_existing_deadline(monkeypatch, reply_at, oversleep):
    """第三次请求的延迟应答可用；缺路或原有 150 ms 窗口外的应答仍失败。"""
    clock = [0.]
    monkeypatch.setattr(acquisition.time, 'monotonic', lambda: clock[0])
    def sleep(delay):
        clock[0] += .06 if oversleep and delay < .01 else delay
    monkeypatch.setattr(acquisition.time, 'sleep', sleep)
    feedback = component()
    motors = []
    for index in range(7):
        motor = SimpleNamespace(sequence=0, requests=0)
        motor.state = SimpleNamespace(pos=0., vel=0., torq=0., status_code=0)
        def request(motor=motor):
            motor.requests += 1
        motor.request_feedback = request
        motor.get_state_with_sequence = lambda motor=motor: (motor.state, motor.sequence)
        motors.append(motor)
    def poll():
        for index, motor in enumerate(motors):
            due = .001 if index != 3 else reply_at
            if due is not None and clock[0] >= due:
                motor.sequence = index+1
    controller = SimpleNamespace(poll_feedback_once=poll)
    labels = [f'joint{i}' for i in range(1, 7)]+['gripper']
    feedback._feedback_controller_groups = lambda: [(controller, list(zip(labels, motors)))]
    feedback._joint_names = lambda: labels[:6]
    feedback._gripper_motor = lambda: motors[-1]
    if reply_at == .11 and not oversleep:
        feedback._force_feedback_refresh()
        assert set(feedback._verified_feedback_by_label) == set(labels)
        assert not feedback._feedback_error_by_label
    else:
        with pytest.raises(RuntimeError, match='joint4.*sequence'):
            feedback._force_feedback_refresh()
        assert 'joint4' in feedback._feedback_error_by_label
    assert all(motor.requests == 3 for motor in motors)
    assert clock[0] <= (.16 if oversleep else .15) + 1e-9
