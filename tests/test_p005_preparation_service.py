"""轨迹准备服务的场景和覆盖边界回归。

文件职责：验证碰撞版本、整批只读锁、附着物副本、固定双指状态及失败响应边界。
对外接口：pytest 用例；LockedMonitor 为检查锁和 RobotState 使用方式的场景替身。
安全约束：不启动真实 MoveIt 或设备；替身只能证明调用范围，不能替代真实场景集成验收。
文件结构：场景版本测试 → 状态/锁替身 → 完整请求工厂 → 覆盖与拒绝用例。
"""
import copy
from contextlib import contextmanager
from types import SimpleNamespace
import time

import pytest
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from rebotarm_msgs.srv import PrepareArmTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_motion.trajectory_messages import point_to_point_message
from rebotarm_motion.trajectory_preparation_node import scene_revision, SceneCollisionChecker
from test_polynomial_trajectory import LIMITS, LOWER, UPPER


def test_scene_version_tracks_environment_attached_objects_and_rules_only():
    scene = PlanningScene()
    initial = scene_revision(scene)
    scene.robot_state.joint_state.name = ['joint1']
    scene.robot_state.joint_state.position = [.1]
    scene.robot_state.joint_state.header.stamp.sec = 42
    scene.world.octomap.header.stamp.sec = 43
    assert scene_revision(scene) == initial
    scene.world.collision_objects = [CollisionObject(id='obstacle')]
    assert scene_revision(scene) != initial
    scene.world.collision_objects = []
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name='end_link')]
    assert scene_revision(scene) != initial
    scene.robot_state.attached_collision_objects = []
    scene.allowed_collision_matrix.entry_names = ['end_link']
    assert scene_revision(scene) != initial


class FakeState:
    def __init__(self):
        self.attached_body = 'held_object'
        self.joint_positions = {}
    def update(self):
        pass


class LockedMonitor:
    """每次查询同时断言锁、附着物和全机器人范围；不能以单纯计数掩盖检查对象错误。"""
    def __init__(self):
        self.locked = False
        self.entries = 0
        self.checked = []
        self.scene = SimpleNamespace(planning_scene_message=PlanningScene(), current_state=FakeState(),
            is_state_colliding=self.collision)
    @contextmanager
    def read_only(self):
        assert not self.locked
        self.locked=True
        self.entries+=1
        try:
            yield self.scene
        finally:
            self.locked=False
    def collision(self, state, group, verbose):
        # 空 group 才覆盖完整机器人；复制状态必须保留附着物，同时不得污染场景当前状态。
        assert self.locked
        assert state is not self.scene.current_state
        assert state.attached_body=='held_object'
        assert group==''
        assert tuple(state.joint_positions)[-2:]==('left_finger_joint','right_finger_joint')
        self.checked.append(dict(state.joint_positions))
        return False


def request(monitor):
    req=PrepareArmTrajectory.Request()
    req.trajectory=point_to_point_message(tuple(f'joint{i}' for i in range(1,7)),
        (0.,)*6, (.02,0.,0.,0.,0.,0.), .4,
        trajectory_factory=JointTrajectory,point_factory=JointTrajectoryPoint)
    req.session=1;req.generation=2
    req.hold_positions=req.feedback_positions=[0.]*6
    req.feedback_observed_at=time.monotonic()
    req.stationary=req.enabled=True
    req.gripper_positions=[.01,-.01]
    req.scene_revision=scene_revision(monitor.scene.planning_scene_message)
    req.deadline_monotonic=time.monotonic()+.33
    return req


def test_one_read_lock_and_copied_attached_state_cover_every_curve_sample():
    monitor=LockedMonitor()
    payload,revision=SceneCollisionChecker(monitor).prepare(request(monitor), LOWER, UPPER, LIMITS)
    assert monitor.entries==1 and len(monitor.checked)>=2
    assert payload['scene_revision']==revision
    assert payload['gripper_positions']==(.01,-.01)
    assert monitor.scene.current_state.joint_positions=={}
    assert all(item['left_finger_joint']==.01 and item['right_finger_joint']==-.01 for item in monitor.checked)


def test_web_closed_collision_policy_preserves_actual_snapshot_and_public_scene():
    monitor = LockedMonitor()
    req = request(monitor)
    req.gripper_collision_policy = req.WEB_CLOSED
    payload,_ = SceneCollisionChecker(monitor).prepare(req,LOWER,UPPER,LIMITS)
    assert payload['gripper_positions'] == (.01,-.01)
    assert payload['gripper_collision_policy'] == req.WEB_CLOSED
    assert all(item['left_finger_joint'] == item['right_finger_joint'] == 0. for item in monitor.checked)
    assert monitor.scene.current_state.joint_positions == {}


@pytest.mark.parametrize('names', [('left_finger_joint',),(*tuple(f'joint{i}' for i in range(1,7)),'left_finger_joint'),(*tuple(f'joint{i}' for i in range(1,7)),'right_finger_joint')])
def test_prepare_rejects_gripper_fjt_before_collision(names):
    monitor=LockedMonitor();req=request(monitor)
    req.trajectory.joint_names=list(names)
    with pytest.raises(ValueError,match='exactly'):
        SceneCollisionChecker(monitor).prepare(req,LOWER,UPPER,LIMITS)
    assert not monitor.checked


@pytest.mark.parametrize('damage', ['scene','deadline','mixed','limits'])
def test_prepare_rejects_invalid_context_or_curve_without_partial_handoff(damage):
    monitor=LockedMonitor();req=request(monitor)
    if damage=='scene':req.scene_revision='old'
    elif damage=='deadline':req.deadline_monotonic=time.monotonic()-.1
    elif damage=='mixed':req.trajectory.points[-1].velocities=[]
    else:req.trajectory.points[-1].positions[0]=2.
    with pytest.raises((ValueError,TimeoutError)):
        SceneCollisionChecker(monitor).prepare(req,LOWER,UPPER,LIMITS)
