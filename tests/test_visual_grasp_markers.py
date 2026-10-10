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
    existing_rebot_msg = sys.modules.get("rebotarm_msgs.msg")
    if (
        existing_pose is not None
        and hasattr(existing_rebot_msg, "GraspPlan")
        and "visualization_msgs.msg" in sys.modules
    ):
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

    class Stamp:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class Header:
        def __init__(self):
            self.stamp = Stamp()
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

    class Marker:
        ARROW = 0
        ADD = 0
        DELETEALL = 3
        CUBE = 1
        SPHERE = 2
        CYLINDER = 3
        LINE_LIST = 5
        TEXT_VIEW_FACING = 9

        def __init__(self):
            self.header = Header()
            self.ns = ""
            self.id = 0
            self.type = 0
            self.action = 0
            self.pose = Pose()
            self.scale = Point()
            self.color = types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
            self.lifetime = Stamp()
            self.frame_locked = False
            self.text = ""
            self.points = []

    class MarkerArray:
        def __init__(self):
            self.markers = []

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
    geometry_msgs_msg = sys.modules.get("geometry_msgs.msg", types.ModuleType("geometry_msgs.msg"))
    geometry_msgs_msg.Pose = Pose
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    visualization_msgs = sys.modules.get("visualization_msgs", types.ModuleType("visualization_msgs"))
    visualization_msgs_msg = sys.modules.get(
        "visualization_msgs.msg", types.ModuleType("visualization_msgs.msg")
    )
    visualization_msgs_msg.Marker = Marker
    visualization_msgs_msg.MarkerArray = MarkerArray
    sys.modules["visualization_msgs"] = visualization_msgs
    sys.modules["visualization_msgs.msg"] = visualization_msgs_msg

    rebotarm_msgs = sys.modules.get("rebotarm_msgs", types.ModuleType("rebotarm_msgs"))
    rebotarm_msgs_msg = sys.modules.get("rebotarm_msgs.msg", types.ModuleType("rebotarm_msgs.msg"))
    rebotarm_msgs_msg.GraspCandidate = GraspCandidate
    rebotarm_msgs_msg.GraspPlan = GraspPlan
    sys.modules["rebotarm_msgs"] = rebotarm_msgs
    sys.modules["rebotarm_msgs.msg"] = rebotarm_msgs_msg

    rclpy = types.ModuleType("rclpy")
    rclpy.time = types.SimpleNamespace(Time=lambda: types.SimpleNamespace(to_msg=lambda: Stamp()))
    rclpy.duration = types.SimpleNamespace(Duration=lambda seconds=0.0: seconds)
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


def _plan():
    from rebotarm_msgs.msg import GraspPlan

    plan = GraspPlan()
    plan.valid = True
    plan.header.frame_id = "base_link"
    plan.candidate.class_name = "bottle"
    plan.candidate.confidence = 0.83
    plan.candidate.pose.position.x = 0.32
    plan.candidate.pose.position.z = 0.30
    plan.candidate.pose.orientation.x = 0.3
    plan.candidate.pose.orientation.w = 0.95
    plan.candidate.jaw_width = 0.04
    plan.candidate.object_length = 0.20
    plan.jaw_width = 0.04
    plan.pregrasp_pose.position.z = 0.38
    plan.grasp_pose.position.z = 0.30
    return plan


def test_visual_marker_builder_uses_upright_object_marker():
    from rebotarm_vision.nodes.visual_grasp_marker_node import VisualGraspMarkerBuilder

    markers = VisualGraspMarkerBuilder(upright_object_marker=True).build(
        _plan(),
        frame_id="base_link",
        stamp=types.SimpleNamespace(sec=0, nanosec=0),
    )

    object_marker = next(marker for marker in markers.markers if marker.ns == "visual_object")
    assert object_marker.pose.position.x == pytest.approx(0.32)
    assert object_marker.pose.position.z == pytest.approx(0.30)
    assert object_marker.pose.orientation.x == pytest.approx(0.0)
    assert object_marker.pose.orientation.w == pytest.approx(1.0)
    assert object_marker.scale.x == pytest.approx(0.072)
    assert object_marker.scale.z == pytest.approx(0.20)
    assert object_marker.frame_locked is True


def test_visual_marker_builder_shows_tcp_approach_and_gripper_open_axis():
    from visualization_msgs.msg import Marker
    from rebotarm_vision.nodes.visual_grasp_marker_node import VisualGraspMarkerBuilder

    plan = _plan()
    plan.pregrasp_pose.position.x = 0.20
    plan.pregrasp_pose.position.y = 0.00
    plan.pregrasp_pose.position.z = 0.35
    plan.grasp_pose.position.x = 0.30
    plan.grasp_pose.position.y = 0.00
    plan.grasp_pose.position.z = 0.30
    plan.jaw_width = 0.04

    markers = VisualGraspMarkerBuilder(
        tcp_offset_xyz=(-0.04, 0.0, 0.0),
        gripper_open_axis_local_xyz=(0.0, 1.0, 0.0),
    ).build(
        plan,
        frame_id="base_link",
        stamp=types.SimpleNamespace(sec=0, nanosec=0),
    )

    by_ns = {marker.ns: marker for marker in markers.markers}

    assert by_ns["visual_object_center"].pose.position.x == pytest.approx(0.32)
    assert by_ns["visual_pregrasp_tcp"].pose.position.x == pytest.approx(0.16)
    assert by_ns["visual_grasp_tcp"].pose.position.x == pytest.approx(0.26)
    assert by_ns["visual_approach_axis"].type == Marker.ARROW
    assert len(by_ns["visual_approach_axis"].points) == 2
    assert by_ns["visual_approach_axis"].points[0].x == pytest.approx(0.16)
    assert by_ns["visual_approach_axis"].points[1].x == pytest.approx(0.26)
    assert by_ns["visual_gripper_open_axis"].type == Marker.LINE_LIST
    assert len(by_ns["visual_gripper_open_axis"].points) == 2
    assert by_ns["visual_gripper_open_axis"].points[0].y == pytest.approx(-0.02)
    assert by_ns["visual_gripper_open_axis"].points[1].y == pytest.approx(0.02)


def test_preview_markers_hide_object_shape_center_and_label():
    from rebotarm_vision.nodes.visual_grasp_marker_node import VisualGraspMarkerBuilder

    plan = _plan()
    markers = VisualGraspMarkerBuilder(
        show_object_marker=False,
        show_object_center_marker=False,
        show_object_label=False,
    ).build(
        plan, frame_id="base_link", stamp=types.SimpleNamespace(sec=0, nanosec=0)
    )
    by_ns = {marker.ns: marker for marker in markers.markers}

    assert "visual_object" not in by_ns
    assert "visual_object_center" not in by_ns
    assert "visual_object_label" not in by_ns
    assert by_ns["visual_grasp_tcp"].pose.position.z == pytest.approx(0.30)


def test_visual_marker_builder_deletes_markers_for_invalid_plan():
    from rebotarm_msgs.msg import GraspPlan
    from visualization_msgs.msg import Marker
    from rebotarm_vision.nodes.visual_grasp_marker_node import VisualGraspMarkerBuilder

    plan = GraspPlan()
    plan.valid = False

    markers = VisualGraspMarkerBuilder().build(
        plan,
        frame_id="base_link",
        stamp=types.SimpleNamespace(sec=0, nanosec=0),
    )

    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL
