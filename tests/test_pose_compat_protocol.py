"""位姿兼容协议：碰撞未知拒绝、唯一服务端、迟到接纳和取消确认。"""
import threading
import time
from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import Pose
from rebotarm_motion.pose_compat_node import PoseCompatNode, PoseRun
from rebotarm_motion.pose_kinematics import PoseKinematics
import rebotarm_motion.pose_compat_node as module
from rebotarm_motion.paired_trajectory_protocol import build_quintic_command


class Future:
    def __init__(self, result=None, ready=False):
        self.value, self.ready = result, ready
    def done(self): return self.ready
    def result(self): return self.value


def monitor_node():
    node = PoseCompatNode.__new__(PoseCompatNode)
    node.lock = threading.RLock()
    node._refresh_feedback = lambda: None
    node.epoch = 0
    node._ready = lambda: (0.,)*6
    node.feedback = NS(header=NS(stamp=NS(sec=0, nanosec=1)))
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=time.monotonic_ns()))
    node.stop_feedback = NS(stationary=lambda *_: False)
    run = PoseRun(0, time.monotonic()+30.)
    run.command = build_quintic_command((0.,)*6, (.1,0.,0.,0.,0.,0.),
                                       duration_sec=1., label='test')
    run.dispatched = time.monotonic()-4.
    run.send = Future()
    node.active = run
    return node, run


def test_late_acceptance_is_canceled_and_blocks_new_requests_until_stationary():
    node, run = monitor_node()
    node._monitor()
    assert run.cancel and node.active is run and not run.done.is_set()
    with pytest.raises(RuntimeError, match='busy'):
        node._begin()
    result = Future()
    cancellations = []
    handle = NS(accepted=True, get_result_async=lambda: result,
                cancel_goal_async=lambda: cancellations.append(1) or Future(ready=True))
    run.send.value, run.send.ready = handle, True
    node._monitor()
    assert cancellations == [1] and node.active is run
    result.value, result.ready = NS(status=5, result=NS(error_code=-1)), True
    node._monitor()
    assert node.active is run and not run.done.is_set()
    node.stop_feedback = NS(stationary=lambda *_: True)
    node._monitor()
    assert node.active is None and run.done.is_set()
    assert 'acceptance timeout' in run.error


def test_rejected_goal_releases_request_without_late_motion():
    node, run = monitor_node()
    run.send = Future(NS(accepted=False), ready=True)
    node._monitor()
    assert run.done.is_set() and run.error == 'FJT rejected'
    assert node.active is None


def planning_node(state):
    node, run = monitor_node()
    node.kinematics = PoseKinematics()
    seed = (0.,-1.,-1.,0.,.3,0.)
    node._check_run = lambda *_: node.kinematics.joints(seed)
    node._fresh = lambda _: True
    node.gripper = NS(position=.08, status_code=1)
    node.get_parameter = lambda _: NS(value='arm')
    node.validity = NS(srv_name='/check_state_validity')
    node.checker = NS(check_positions=lambda **_: dict(state=state, message='injected check'))
    node.action = NS(wait_for_server=lambda **_: True,
                     send_goal_async=lambda _: pytest.fail('must not dispatch'))
    node.action_name = '/rebotarm/follow_joint_trajectory'
    target = node.kinematics.pose(seed)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, target.translation)
    import pinocchio as pin
    q = pin.Quaternion(target.rotation).coeffs()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return node, run, pose


@pytest.mark.parametrize('state', ['unknown', 'collision', 'disabled'])
def test_collision_unknown_or_failed_never_dispatches_fjt(state):
    node, run, pose = planning_node(state)
    with pytest.raises(RuntimeError, match='collision check'):
        node._prepare(run, pose, 0., False)
    assert run.send is not None  # 该测试替身的旧 future 没有被替换/下发。
    assert not run.send.done()


def test_duplicate_fjt_servers_rejected_before_send(monkeypatch):
    node, run, pose = planning_node('pass')
    node.get_node_names_and_namespaces = lambda: [('one','/'), ('two','/')]
    monkeypatch.setattr(module, 'get_action_server_names_and_types_by_node',
        lambda *_: [(node.action_name, ['control_msgs/action/FollowJointTrajectory'])])
    with pytest.raises(RuntimeError, match='found 2'):
        node._prepare(run, pose, 0., False)


def test_canceled_planning_does_not_submit_late_solution():
    node, run, pose = planning_node('pass')
    node._check_run = PoseCompatNode._check_run.__get__(node)
    run.cancel = True
    with pytest.raises(RuntimeError, match='canceled'):
        node._prepare(run, pose, 0., False)


def test_monitor_uses_source_time_and_audits_actual_quantized_samples(monkeypatch, caplog):
    import logging
    import numpy as np
    from rebotarm_motion.trajectory_runtime_limits import JointRuntimeLimit, TrajectoryRuntimeLimitGuard
    from rebotarm_motion.paired_trajectory_protocol import sample_command
    node, run = monitor_node()
    run.dispatched, run.dispatched_ns = 100., int(10e9)
    run.guard = TrajectoryRuntimeLimitGuard({f'joint{i}': JointRuntimeLimit(
        max_velocity=1., max_effort=2., max_acceleration=5., max_jerk=20.) for i in range(1, 7)},
        derivative_policy='diagnostic')
    handle = NS(accepted=True, get_result_async=lambda: Future(), cancel_goal_async=lambda: Future())
    run.send = Future(handle, ready=True)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 100.5)
    # 到达时间固定为调度起点后 .5 s，源时间只有 .10/.12/.14 s。
    with caplog.at_level(logging.INFO, logger='rebotarm.audit'):
        for k, velocity in enumerate([-.0024423599243164062, .0024423599243164062, -.0024423599243164062]):
            elapsed = .1+k*.02
            stamp = run.dispatched_ns+round(elapsed*1e9)
            sec, ns = divmod(stamp, 10**9)
            desired = sample_command(run.command, elapsed)
            node.feedback = NS(header=NS(stamp=NS(sec=sec, nanosec=ns)),
                name=[f'joint{i}' for i in range(1, 7)], velocity=[velocity]+[0.]*5, effort=[0.]*6)
            node._ready = lambda: np.asarray(desired)
            node._monitor()
            assert not run.cancel and not run.error
    rows = [record.audit for record in caplog.records if record.msg == 'pose_guard_sample']
    assert [row['elapsed'] for row in rows] == pytest.approx([.1, .12, .14])
    assert rows[-1]['tracking_error'] == 0.
    assert rows[-1]['jerk']['joint1'] == pytest.approx(-24.423599, rel=.001)
    assert rows[-1]['derivative_diagnostics'][0]['kind'] == 'jerk'


def test_canceled_old_epoch_finishes_as_failure_only_after_terminal_and_new_hold_window(monkeypatch):
    import numpy as np
    from rebotarm_motion.stop_feedback import StopFeedback
    from test_motion_stop_confirmation import sample
    node, run = monitor_node()
    run.cancel, run.error = True, 'feedback continuity lost'
    run.dispatched = 10.
    node.epoch = 1
    node._ready = lambda: np.zeros(6)
    node.stop_feedback = StopFeedback()
    now = [10.4]
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=round(now[0]*1e9)))
    result = Future()
    handle = NS(accepted=True, get_result_async=lambda: result,
                cancel_goal_async=lambda: Future(ready=True))
    run.send = Future(handle, ready=True)
    node.stop_feedback.invalidate(10.3)
    for k in range(21):
        now[0] = 10.4+k*.02
        node.stop_feedback.update(sample(now[0]), now[0], round(now[0]*1e9))
        if k == 1:
            result.value, result.ready = NS(status=5, result=NS(error_code=-1)), True
        node._monitor()
        if k < 16:
            assert not run.done.is_set()
    assert run.done.is_set() and node.active is None
    assert run.error == 'feedback continuity lost'


@pytest.mark.parametrize('status,code', [(4, 0), (6, -1)])
def test_pose_action_cannot_succeed_without_measured_final_pose(status, code):
    node, run = monitor_node()
    run.done.set()
    run.terminal = NS(status=status, result=NS(error_code=code, error_string='finished'))
    node._begin = lambda: run
    node._prepare = lambda *_: None
    def unavailable():
        raise RuntimeError('final pose unavailable: feedback lost')
    node._pose_message = unavailable
    events = []
    goal = NS(request=NS(target_pose=Pose(), duration=1.), is_cancel_requested=False,
              succeed=lambda: events.append('succeeded'), abort=lambda: events.append('aborted'))
    result = node._execute(goal)
    assert not result.success and events == ['aborted']
    assert 'final pose unavailable' in result.message
