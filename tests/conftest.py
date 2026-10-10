from __future__ import annotations

import importlib.machinery
import importlib.util
from pathlib import Path
import sys
import types


ROS2_ROOT = Path(__file__).resolve().parents[1]
for package_path in (ROS2_ROOT / "src").iterdir():
    if package_path.is_dir():
        path_str = str(package_path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


def _has_importable_package(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except ValueError:
        return False


def _module(name: str) -> types.ModuleType:
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    if getattr(module, "__spec__", None) is None:
        module.__spec__ = importlib.machinery.ModuleSpec(name, loader=None)
    return module


def _ensure_ros_stubs() -> None:
    if _has_importable_package("geometry_msgs"):
        return

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

    class Image:
        def __init__(self):
            self.header = Header()
            self.height = 0
            self.width = 0
            self.encoding = ""
            self.is_bigendian = 0
            self.step = 0
            self.data = b""

    class JointState:
        def __init__(self):
            self.header = Header()
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

    class CameraInfo:
        def __init__(self):
            self.header = Header()
            self.height = 0
            self.width = 0
            self.distortion_model = ""
            self.d = []
            self.k = [0.0] * 9
            self.r = [0.0] * 9
            self.p = [0.0] * 12

    class _Request:
        pass

    class _Response:
        def __init__(self):
            self.success = False
            self.message = ""

    class _Service:
        Request = _Request
        Response = _Response

    geometry_msgs_msg = _module("geometry_msgs.msg")
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.TransformStamped = TransformStamped
    _module("geometry_msgs").msg = geometry_msgs_msg

    visualization_msgs_msg = _module("visualization_msgs.msg")
    visualization_msgs_msg.Marker = Marker
    visualization_msgs_msg.MarkerArray = MarkerArray
    _module("visualization_msgs").msg = visualization_msgs_msg

    rebotarm_msgs_msg = _module("rebotarm_msgs.msg")
    rebotarm_msgs_msg.Detection2D = Detection2D
    rebotarm_msgs_msg.Detection2DArray = Detection2DArray
    rebotarm_msgs_msg.GraspCandidate = GraspCandidate
    rebotarm_msgs_msg.GraspCandidateArray = GraspCandidateArray
    rebotarm_msgs_msg.GraspPlan = GraspPlan
    _module("rebotarm_msgs").msg = rebotarm_msgs_msg
    rebotarm_msgs_srv = _module("rebotarm_msgs.srv")
    rebotarm_msgs_srv.ExecutePose = _Service
    rebotarm_msgs_srv.PublishTrajectoryPreview = _Service
    rebotarm_msgs_srv.GraspGripper = _Service
    rebotarm_msgs_srv.SetGripper = _Service
    _module("rebotarm_msgs").srv = rebotarm_msgs_srv

    std_srvs_srv = _module("std_srvs.srv")
    std_srvs_srv.Trigger = _Service
    _module("std_srvs").srv = std_srvs_srv

    moveit_msgs_srv = _module("moveit_msgs.srv")
    moveit_msgs_srv.GetPositionIK = _Service
    moveit_msgs_srv.GetStateValidity = _Service
    _module("moveit_msgs").srv = moveit_msgs_srv

    sensor_msgs_msg = _module("sensor_msgs.msg")
    sensor_msgs_msg.Image = Image
    sensor_msgs_msg.JointState = JointState
    sensor_msgs_msg.CameraInfo = CameraInfo
    _module("sensor_msgs").msg = sensor_msgs_msg

    rclpy = _module("rclpy")
    rclpy.duration = types.SimpleNamespace(Duration=lambda seconds=0.0: seconds)
    rclpy.time = types.SimpleNamespace(Time=lambda: types.SimpleNamespace(to_msg=lambda: Stamp()))
    rclpy.ok = lambda: False
    rclpy_executors = _module("rclpy.executors")
    rclpy_executors.ExternalShutdownException = RuntimeError
    rclpy_executors.MultiThreadedExecutor = object
    _module("rclpy.callback_groups").ReentrantCallbackGroup = object
    _module("rclpy.node").Node = object

    tf2_ros = _module("tf2_ros")
    tf2_ros.Buffer = object
    tf2_ros.TransformException = RuntimeError
    tf2_ros.TransformListener = object
    tf2_ros.StaticTransformBroadcaster = object


def pytest_configure(config):
    _ensure_ros_stubs()


def pytest_runtest_setup(item):
    _ensure_ros_stubs()


import pytest

@pytest.fixture
def bottle_scene(tmp_path):
    """Explicit test-only target scene; production defaults remain an empty table."""
    import xml.etree.ElementTree as ET
    import shutil
    models = ROS2_ROOT / 'src/rebotarm_simulation/models/rebotarm'
    directory = tmp_path / 'models'
    shutil.copytree(models, directory)
    scene = directory / 'scene.xml'
    root = ET.parse(scene).getroot()
    body = ET.parse(ROS2_ROOT / 'tests/fixtures/bottle_body.xml').getroot().find('body')
    root.find('worldbody').append(body)
    for key in root.findall('keyframe/key'):
        key.set('qpos', key.get('qpos') + ' 0.28 0 0 1 0 0 0')
    ET.ElementTree(root).write(scene, encoding='unicode')
    return scene
