"""Stop evidence and late-action regression tests; no hardware."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
import threading
import time

import pytest
from rebotarm_motion.stop_feedback import StopFeedback
from rebotarm_motion.pose_execution_node import PoseExecutionNode
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
    for t in (10., 10.1, 10.2, 10.4):
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
    for t in (10., 10.1, 10.2, 10.4):
        feedback.update(sample(t), t, int(t*1e9))
    feedback.update(sample(10.5, position, velocity), 10.5, int(10.5e9))
    assert not feedback.stationary(10., 10.5, int(10.5e9))


def test_position_drift_is_not_stationary_even_if_velocity_reports_zero():
    feedback = StopFeedback()
    for t,p in ((10., 0.), (10.1,.01), (10.2,.02), (10.4,.03)):
        feedback.update(sample(t,p), t, int(t*1e9))
    assert not feedback.stationary(10., 10.4, int(10.4e9))


def completed(value):
    future = Future()
    future.set_result(value)
    return future


def motion_stub():
    node = NS(_motion_lock=threading.RLock(), _stop_lock=threading.Lock(),
              _request_active=False, _stopping=False, _generation=0,
              _send_future=None, _result_future=None, _active_goal_handle=None,
              _stop_feedback=NS(stationary=lambda *_: True),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(time.time()*1e9))),
              get_parameter=lambda _: NS(value=.2),
              _trajectory_stop_client=NS(wait_for_service=lambda **_: True,
                 call_async=lambda _: completed(NS(success=True))),
              _hardware_idle=lambda _: True)
    node._action_terminal = lambda: PoseExecutionNode._action_terminal(node)
    return node


def test_late_acceptance_and_terminal_result_are_both_required(monkeypatch):
    monkeypatch.setattr('rebotarm_motion.pose_execution_node.rclpy.ok', lambda: True)
    node = motion_stub()
    node._send_future = Future()
    result_future = Future()
    canceled = []
    handle = NS(accepted=True, get_result_async=lambda: result_future,
                cancel_goal_async=lambda: canceled.append(True))
    response = NS()
    worker = threading.Thread(target=lambda: PoseExecutionNode._stop(node, None, response))
    worker.start()
    time.sleep(.025)
    assert node._stopping
    assert not PoseExecutionNode._execute_pose(node, NS(), NS()).success
    node._send_future.set_result(handle)
    time.sleep(.025)
    assert node._stopping and worker.is_alive()
    result_future.set_result(NS(status=5))
    worker.join(1)
    assert response.success and canceled
    assert not node._stopping


def test_stop_timeout_does_not_clear_motion_admission(monkeypatch):
    monkeypatch.setattr('rebotarm_motion.pose_execution_node.rclpy.ok', lambda: True)
    node = motion_stub()
    node._send_future = Future()
    response = PoseExecutionNode._stop(node, None, NS())
    assert not response.success and node._stopping
    assert not PoseExecutionNode._execute_pose(node, NS(), NS()).success


def test_hardware_fault_or_stale_status_cannot_confirm_stop():
    node = motion_stub()
    node.get_parameter = lambda _: NS(value=True)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(10e9)))
    status = NS(header=NS(stamp=NS(sec=10,nanosec=0)), error_codes=[],
                state_machine='IDLE', per_joint_status_code=[1]*6)
    node._arm_status = (10., status)
    assert PoseExecutionNode._hardware_idle(node, 10.)
    status.error_codes = ['ARM_FEEDBACK: stale']
    assert not PoseExecutionNode._hardware_idle(node, 10.)
    status.error_codes = []
    assert not PoseExecutionNode._hardware_idle(node, 13.)
    status.state_machine = 'TRAJ_RUNNING'
    assert not PoseExecutionNode._hardware_idle(node, 10.)
