from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_PATH = str(ROS2_ROOT / "src" / "rebotarm_vision")
if VISION_PATH not in sys.path:
    sys.path.insert(0, VISION_PATH)


def _install_ros_stubs_if_needed():
    existing_geometry_msg = sys.modules.get("geometry_msgs.msg")
    if existing_geometry_msg is not None and hasattr(existing_geometry_msg, "TransformStamped"):
        tf2_ros = sys.modules.get("tf2_ros", types.ModuleType("tf2_ros"))
        tf2_ros.Buffer = getattr(tf2_ros, "Buffer", object)
        tf2_ros.TransformException = getattr(tf2_ros, "TransformException", RuntimeError)
        tf2_ros.TransformListener = getattr(tf2_ros, "TransformListener", object)
        tf2_ros.StaticTransformBroadcaster = getattr(tf2_ros, "StaticTransformBroadcaster", object)
        sys.modules["tf2_ros"] = tf2_ros
        return
    try:
        if importlib.util.find_spec("geometry_msgs") is not None:
            return
    except ValueError:
        pass

    class Header:
        def __init__(self):
            self.frame_id = ""

    class Vector3:
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

    class Transform:
        def __init__(self):
            self.translation = Vector3()
            self.rotation = Quaternion()

    class TransformStamped:
        def __init__(self):
            self.header = Header()
            self.child_frame_id = ""
            self.transform = Transform()

    geometry_msgs = sys.modules.get("geometry_msgs", types.ModuleType("geometry_msgs"))
    geometry_msgs_msg = sys.modules.get(
        "geometry_msgs.msg",
        types.ModuleType("geometry_msgs.msg"),
    )
    geometry_msgs_msg.TransformStamped = TransformStamped
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    rclpy = types.ModuleType("rclpy")
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
    tf2_ros.StaticTransformBroadcaster = object
    sys.modules["tf2_ros"] = tf2_ros


_install_ros_stubs_if_needed()


def test_build_grasp_tcp_transform_uses_end_link_parent_and_offset():
    from rebotarm_vision.nodes.grasp_tcp_frame_node import build_grasp_tcp_transform

    tf_msg = build_grasp_tcp_transform(
        parent_frame="end_link",
        child_frame="grasp_tcp",
        tcp_offset_xyz=(0.01, -0.02, 0.03),
    )

    assert tf_msg.header.frame_id == "end_link"
    assert tf_msg.child_frame_id == "grasp_tcp"
    assert tf_msg.transform.translation.x == pytest.approx(0.01)
    assert tf_msg.transform.translation.y == pytest.approx(-0.02)
    assert tf_msg.transform.translation.z == pytest.approx(0.03)
    assert tf_msg.transform.rotation.w == pytest.approx(1.0)
