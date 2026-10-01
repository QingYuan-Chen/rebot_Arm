from __future__ import annotations

from pathlib import Path
import sys
import types

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = str(ROS2_ROOT / "src" / "rebotarm_calibration")
if CALIBRATION_PATH not in sys.path:
    sys.path.insert(0, CALIBRATION_PATH)


class _FakeAruco:
    DICT_4X4_50 = 0
    DICT_5X5_100 = 5


class _FakeCv2:
    aruco = _FakeAruco()


class _Point:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(self, x: float, y: float, z: float, w: float):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Transform:
    def __init__(self):
        self.translation = _Point(1.0, 2.0, 3.0)
        self.rotation = _Quaternion(0.0, 0.0, 0.0, 1.0)


class _TransformStamped:
    def __init__(self):
        self.transform = _Transform()


def _install_ros_stubs_if_needed():
    if "rclpy" in sys.modules:
        rclpy = sys.modules["rclpy"]
        if not hasattr(rclpy, "duration"):
            rclpy.duration = types.SimpleNamespace(Duration=lambda seconds=0.0: seconds)
        if not hasattr(rclpy, "time"):
            rclpy.time = types.SimpleNamespace(Time=lambda: None)
        if "rclpy.qos" not in sys.modules:
            rclpy_qos = types.ModuleType("rclpy.qos")
            rclpy_qos.qos_profile_sensor_data = object()
            sys.modules["rclpy.qos"] = rclpy_qos
        return

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

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sys.modules["rclpy.qos"] = rclpy_qos

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformListener = object
    sys.modules["tf2_ros"] = tf2_ros


def test_dictionary_name_accepts_dict_prefix_and_short_name():
    from rebotarm_calibration.aruco_reference import resolve_aruco_dictionary_id

    assert resolve_aruco_dictionary_id("DICT_4X4_50", cv2_module=_FakeCv2) == 0
    assert resolve_aruco_dictionary_id("4X4_50", cv2_module=_FakeCv2) == 0


def test_dictionary_name_rejects_unknown_dictionary():
    from rebotarm_calibration.aruco_reference import resolve_aruco_dictionary_id

    with pytest.raises(ValueError, match="unsupported ArUco dictionary"):
        resolve_aruco_dictionary_id("DICT_7X7_999", cv2_module=_FakeCv2)


def test_transform_camera_point_to_base_uses_tf_transform():
    from rebotarm_calibration.aruco_reference import transform_camera_point_to_base

    point = transform_camera_point_to_base(
        _TransformStamped(),
        camera_point_xyz=(0.1, 0.2, 0.3),
    )

    assert point == pytest.approx((1.1, 2.2, 3.3))


def test_transform_camera_point_to_base_rotates_point():
    from rebotarm_calibration.aruco_reference import transform_camera_point_to_base

    stamped = _TransformStamped()
    stamped.transform.rotation = _Quaternion(0.0, 0.0, 2**-0.5, 2**-0.5)
    point = transform_camera_point_to_base(
        stamped,
        camera_point_xyz=(0.1, 0.0, 0.0),
    )
    assert point == pytest.approx((1.0, 2.1, 3.0))


def test_auto_reference_provider_transforms_detected_aruco_center_to_base():
    _install_ros_stubs_if_needed()

    from rebotarm_calibration.tcp_calibration_node import AutoArucoReferenceProvider

    class Driver:
        def get_frame(self):
            return object(), None

    class Buffer:
        def lookup_transform(self, target, source, time, timeout):
            assert target == "base_link"
            assert source == "camera_depth_frame"
            return _TransformStamped()

    provider = AutoArucoReferenceProvider(
        driver=Driver(),
        tf_buffer=Buffer(),
        base_frame="base_link",
        camera_frame="camera_depth_frame",
        lookup_timeout_sec=0.5,
        detector=lambda image: (0.1, 0.2, 0.3),
    )

    assert provider.reference_position() == pytest.approx((1.1, 2.2, 3.3))
