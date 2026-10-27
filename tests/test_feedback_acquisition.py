"""不构造硬件管理器，验证反馈组件及可供跨语言复用的固定样本。"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarmcontroller.feedback_acquisition import FeedbackAcquisition


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
