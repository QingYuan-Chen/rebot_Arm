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
