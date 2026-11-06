"""消费端反馈交接：空档保留、快照隔离、撤销顺序及真实 ROS 并发。"""
from concurrent.futures import Future
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from rebotarm_motion.feedback_receiver import MotionFeedbackStore


def joints(seconds):
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(seconds * 1e9), 10**9)
    msg.name = [f'joint{i}' for i in range(1, 7)]
    msg.position, msg.velocity, msg.effort = [0.]*6, [0.]*6, [0.]*6
    return msg


def test_recovered_fresh_frame_retains_gap_without_monitoring_during_outage():
    store = MotionFeedbackStore()
    store.receive('joints', joints(10.), 10., 10_000_000_000)
    before = store.snapshot(10., 10_000_000_000)
    store.receive('joints', joints(10.3), 10.3, 10_300_000_000)
    after = store.snapshot(10.3, 10_300_000_000)
    assert after.epoch == before.epoch + 1
    assert after.joint_gaps == 1
    assert after.maximum_joint_age == pytest.approx(.3)
    assert after.joints.header.stamp.nanosec == 300_000_000
    assert store.snapshot(10.31, 10_310_000_000).joint_gaps == 1


@pytest.mark.parametrize('bad_stamp', [9., 10.2, 9.99, 0.])
def test_bad_incoming_frame_invalidates_old_pose_even_if_monitor_only_sees_recovery(bad_stamp):
    from test_pose_compat_protocol import monitor_node
    from rebotarm_motion.pose_compat_node import PoseCompatNode
    store = MotionFeedbackStore()
    store.receive('joints', joints(10.), 10., 10_000_000_000)
    store.receive('joints', joints(bad_stamp), 10.02, 10_020_000_000)
    store.receive('joints', joints(10.04), 10.04, 10_040_000_000)
    recovered = store.snapshot(10.04, 10_040_000_000)
    assert recovered.epoch == recovered.joint_gaps == 1
    assert recovered.joints.header.stamp.nanosec == 40_000_000
    assert store.stop_feedback.broken_at == 10.02
    node, run = monitor_node()
    node._feedback_receiver = NS(snapshot=lambda: recovered)
    node._ready = lambda: PoseCompatNode._refresh_feedback(node) or (0.,) * 6
    run.handle = NS(accepted=True, cancel_goal_async=lambda: Future())
    run.result = NS(done=lambda: True, result=lambda: NS(status=4, result=NS(error_code=0)))
    node._monitor()
    assert run.cancel and not run.done.is_set() and node.active is run


def test_duplicate_source_stamps_age_and_outage_is_latched_once():
    store = MotionFeedbackStore()
    for t in (10., 10.1, 10.2, 10.3):
        store.receive('joints', joints(10.), t, round(t*1e9))
    snapshot = store.snapshot(10.3, 10_300_000_000)
    assert snapshot.joint_gaps == snapshot.epoch == 1
    assert snapshot.joints.header.stamp.sec == 10
    assert snapshot.joints.header.stamp.nanosec == 0
    assert not store.stop_feedback.stationary(10., 10.3, 10_300_000_000)


def test_mode_round_trip_and_status_gap_cannot_reuse_old_plan():
    store = MotionFeedbackStore()
    status = lambda enabled: NS(mode='pos_vel', enabled=enabled, control_loop_active=enabled, error_codes=[])
    store.receive('status', status(True), 10., 10_000_000_000)
    store.receive('status', status(False), 10.1, 10_100_000_000)
    store.receive('status', status(True), 10.2, 10_200_000_000)
    assert store.snapshot(10.2, 10_200_000_000).epoch == 2
    store.receive('status', status(True), 11., 11_000_000_000)
    assert store.snapshot(11., 11_000_000_000).epoch == 3


def test_snapshot_cannot_mutate_receiver_owned_messages():
    store = MotionFeedbackStore()
    store.receive('joints', joints(10.), 10., 10_000_000_000)
    copy = store.snapshot(10., 10_000_000_000)
    copy.joints.position[0] = 4.
    copy.joints.header.stamp.sec = 99
    again = store.snapshot(10., 10_000_000_000)
    assert again.joints.position[0] == 0.
    assert again.joints.header.stamp.sec == 10
    store.error = 'injected executor failure'
    with pytest.raises(RuntimeError, match='executor failure'):
        store.snapshot(10., 10_000_000_000)


def test_success_result_after_feedback_gap_still_cancels_old_pose():
    from test_pose_compat_protocol import monitor_node
    node, run = monitor_node()
    run.handle = NS(accepted=True, cancel_goal_async=lambda: Future())
    result = NS(status=4, result=NS(error_code=0))
    run.result = NS(done=lambda: True, result=lambda: result)
    node.epoch = run.epoch + 1
    node._monitor()
    assert run.cancel and not run.done.is_set() and node.active is run


def test_isolated_ros_consumers_receive_while_business_waits_and_exit_cleanly(tmp_path):
    script = Path(__file__).parent / 'fixtures/feedback_consumers_probe.py'
    result = subprocess.run([sys.executable, str(script), str(tmp_path)],
        env=dict(os.environ, ROS_DOMAIN_ID='212', ROS_LOCALHOST_ONLY='1'),
        capture_output=True, text=True, timeout=35)
    assert result.returncode == 0, result.stdout + result.stderr
    print(result.stdout)
