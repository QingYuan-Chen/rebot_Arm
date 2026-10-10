from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
for path in (
    ROS2_ROOT / "src" / "rebotarm_vision",
    ROS2_ROOT / "src" / "rebotarm_msgs",
):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)


def _install_ros_stubs_if_needed():
    existing_geometry_msg = sys.modules.get("geometry_msgs.msg")
    existing_pose = getattr(existing_geometry_msg, "Pose", None) if existing_geometry_msg is not None else None
    if existing_pose is not None:
        try:
            if hasattr(existing_pose(), "position"):
                return
        except Exception:
            pass
    try:
        if importlib.util.find_spec("geometry_msgs") is not None:
            return
    except ValueError:
        pass

    class Header:
        def __init__(self):
            self.frame_id = ""

    class Point:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

    class Quaternion:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.w = 1.0

    class Pose:
        def __init__(self):
            self.position = Point()
            self.orientation = Quaternion()

    class PoseStamped:
        def __init__(self):
            self.header = Header()
            self.pose = Pose()

    class Transform:
        def __init__(self):
            self.translation = Point()
            self.rotation = Quaternion()

    class TransformStamped:
        def __init__(self):
            self.header = Header()
            self.child_frame_id = ""
            self.transform = Transform()

    class Detection2D:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.center_u = 0
            self.center_v = 0
            self.x_min = 0
            self.y_min = 0
            self.x_max = 0
            self.y_max = 0
            self.has_obb = False
            self.obb_cx = 0.0
            self.obb_cy = 0.0
            self.obb_w = 0.0
            self.obb_h = 0.0
            self.obb_theta = 0.0
            self.obb_points_xy = []
            self.has_mask = False
            self.mask_polygon_xy = []

    class Detection2DArray:
        def __init__(self):
            self.header = Header()
            self.detections = []

    class GraspCandidate:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.pose = Pose()
            self.jaw_width = 0.0
            self.object_length = 0.0
            self.valid = False
            self.source = ""

    class GraspCandidateArray:
        def __init__(self):
            self.header = Header()
            self.candidates = []
            self.best_index = -1

    class GraspPlan:
        def __init__(self):
            self.header = Header()
            self.candidate = GraspCandidate()
            self.pregrasp_pose = Pose()
            self.grasp_pose = Pose()
            self.jaw_width = 0.0
            self.valid = False
            self.source = ""
            self.reason = ""

    geometry_msgs = sys.modules.get("geometry_msgs", types.ModuleType("geometry_msgs"))
    geometry_msgs_msg = sys.modules.get(
        "geometry_msgs.msg",
        types.ModuleType("geometry_msgs.msg"),
    )
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.TransformStamped = TransformStamped
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    rebotarm_msgs = sys.modules.get("rebotarm_msgs", types.ModuleType("rebotarm_msgs"))
    rebotarm_msgs_msg = sys.modules.get("rebotarm_msgs.msg", types.ModuleType("rebotarm_msgs.msg"))
    rebotarm_msgs_msg.Detection2D = Detection2D
    rebotarm_msgs_msg.Detection2DArray = Detection2DArray
    rebotarm_msgs_msg.GraspCandidate = GraspCandidate
    rebotarm_msgs_msg.GraspCandidateArray = GraspCandidateArray
    rebotarm_msgs_msg.GraspPlan = GraspPlan
    sys.modules["rebotarm_msgs"] = rebotarm_msgs
    sys.modules["rebotarm_msgs.msg"] = rebotarm_msgs_msg

    rclpy = types.ModuleType("rclpy")
    rclpy.duration = types.SimpleNamespace(Duration=lambda seconds=0.0: seconds)
    rclpy.time = types.SimpleNamespace(Time=lambda: None)
    sys.modules["rclpy"] = rclpy

    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = RuntimeError
    sys.modules["rclpy.executors"] = rclpy_executors

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node

    tf2_ros = sys.modules.get("tf2_ros", types.ModuleType("tf2_ros"))
    tf2_ros.Buffer = object
    tf2_ros.TransformException = RuntimeError
    tf2_ros.TransformListener = object
    sys.modules["tf2_ros"] = tf2_ros


_install_ros_stubs_if_needed()


def test_select_pregrasp_pose_from_valid_plan():
    from geometry_msgs.msg import Pose
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes.grasp_preview_sender_node import select_grasp_plan_pose

    plan = GraspPlan()
    plan.valid = True
    plan.pregrasp_pose.position.x = 0.1
    plan.grasp_pose.position.x = 0.2

    pose = select_grasp_plan_pose(plan, "pregrasp")

    assert isinstance(pose, Pose)
    assert pose.position.x == pytest.approx(0.1)


def test_select_grasp_pose_from_valid_plan():
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes.grasp_preview_sender_node import select_grasp_plan_pose

    plan = GraspPlan()
    plan.valid = True
    plan.pregrasp_pose.position.x = 0.1
    plan.grasp_pose.position.x = 0.2

    pose = select_grasp_plan_pose(plan, "grasp")

    assert pose.position.x == pytest.approx(0.2)


def test_select_pose_rejects_invalid_plan():
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes.grasp_preview_sender_node import select_grasp_plan_pose

    plan = GraspPlan()
    plan.valid = False
    plan.reason = "invalid candidate"

    with pytest.raises(ValueError, match="invalid candidate"):
        select_grasp_plan_pose(plan, "pregrasp")


def test_select_pose_rejects_unknown_mode():
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes.grasp_preview_sender_node import select_grasp_plan_pose

    plan = GraspPlan()
    plan.valid = True

    with pytest.raises(ValueError, match="unsupported pose mode"):
        select_grasp_plan_pose(plan, "drop")


def test_transform_pose_message_applies_tf_to_position_and_orientation():
    from rebotarm_vision.nodes.grasp_preview_sender_node import transform_pose_message
    from rebotarm_vision.utils.transform_points import Transform3D
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = 1.0
    pose.orientation.w = 1.0
    transform = Transform3D(
        translation=(1.0, 2.0, 3.0),
        rotation_xyzw=(0.0, 0.0, 0.7071067811865476, 0.7071067811865476),
    )

    transformed = transform_pose_message(pose, transform)

    assert transformed.position.x == pytest.approx(1.0)
    assert transformed.position.y == pytest.approx(3.0)
    assert transformed.position.z == pytest.approx(3.0)
    assert transformed.orientation.z == pytest.approx(0.7071067811865476)
    assert transformed.orientation.w == pytest.approx(0.7071067811865476)


def test_apply_tcp_offset_converts_grasp_tcp_target_to_end_link_target():
    from rebotarm_vision.nodes.grasp_preview_sender_node import apply_tcp_offset_to_pose
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = 0.50
    pose.position.y = 0.00
    pose.position.z = 0.20
    pose.orientation.w = 1.0

    target = apply_tcp_offset_to_pose(pose, (0.10, 0.0, 0.0))

    assert target.position.x == pytest.approx(0.40)
    assert target.position.y == pytest.approx(0.00)
    assert target.position.z == pytest.approx(0.20)
