"""准备服务锁范围、附着物及场景版本，不借反馈刷新伪造场景变更。"""
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


@pytest.mark.parametrize('damage', ['scene','deadline','mixed','limits'])
def test_prepare_rejects_invalid_context_or_curve_without_partial_handoff(damage):
    monitor=LockedMonitor();req=request(monitor)
    if damage=='scene':req.scene_revision='old'
    elif damage=='deadline':req.deadline_monotonic=time.monotonic()-.1
    elif damage=='mixed':req.trajectory.points[-1].velocities=[]
    else:req.trajectory.points[-1].positions[0]=2.
    with pytest.raises((ValueError,TimeoutError)):
        SceneCollisionChecker(monitor).prepare(req,LOWER,UPPER,LIMITS)
