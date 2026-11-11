"""公共停止反馈的时效,连续性和静止窗口验证."""

import pytest
from rebotarm_motion.stop_feedback import StopFeedback
from sensor_msgs.msg import JointState


def sample(t, position=0., velocity=0.):
    msg = JointState()
    msg.name = [f'joint{i}' for i in range(1, 7)]
    msg.position = [float(position)]*6
    msg.velocity = [float(velocity)]*6
    ns = int(t*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return msg


def test_stop_requires_distinct_fresh_stationary_samples_after_terminal():
    feedback = StopFeedback()
    for t in (10., 10.1, 10.2, 10.3, 10.4):
        feedback.update(sample(t), t, int(t*1e9))
    assert feedback.stationary(10., 10.4, int(10.4e9))
    assert not feedback.stationary(10.2, 10.4, int(10.4e9))
    assert not feedback.stationary(10., 11., int(11e9))
    for t in (10.5, 10.6, 10.7):
        feedback.update(sample(10.4), t, int(t*1e9))
    assert not feedback.stationary(10.4, 10.7, int(10.7e9))


@pytest.mark.parametrize('position,velocity', [(0., .1), (float('nan'), 0.)])
def test_invalid_or_moving_feedback_breaks_quiet_window(position, velocity):
    feedback = StopFeedback()
    for t in (10., 10.1, 10.2, 10.3, 10.4):
        feedback.update(sample(t), t, int(t*1e9))
    feedback.update(sample(10.5, position, velocity), 10.5, int(10.5e9))
    assert not feedback.stationary(10., 10.5, int(10.5e9))


def test_position_changes_do_not_reject_when_speed_and_continuity_qualify():
    feedback = StopFeedback()
    for t,p in ((10., 0.), (10.1,.01), (10.2,.02), (10.3,.03), (10.4,.04)):
        feedback.update(sample(t,p), t, int(t*1e9))
    assert feedback.stationary(10., 10.4, int(10.4e9))
