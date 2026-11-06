from types import SimpleNamespace
import sys
import types

import numpy as np
import pytest

from rebotarm_vision.policies.camera_info_policy import calibrated_intrinsics
from rebotarm_vision.policies.candidate_motion_policy import joint_positions_by_name


def test_calibrated_intrinsics_requires_real_matching_camera_geometry():
    info = {"width": 2, "height": 1, "fx": 500, "fy": 501, "cx": 1, "cy": 0.5}
    assert calibrated_intrinsics(info, width=2, height=1)
    assert not calibrated_intrinsics(None, width=2, height=1)
    assert not calibrated_intrinsics(info, width=640, height=480)
    assert not calibrated_intrinsics({**info, "fx": 0}, width=2, height=1)
    assert not calibrated_intrinsics({**info, "fy": float("nan")}, width=2, height=1)


def test_joint_positions_by_name_is_ros_free_and_drops_non_finite_values():
    assert joint_positions_by_name(["joint1", "gripper"], [0.25, float("nan")]) == {
        "joint1": 0.25
    }


def test_vision_node_requests_current_frame_without_cache_fallback(monkeypatch):
    detector_module = types.ModuleType("rebotarm_vision.detector.yolo_detector")
    detector_module.YoloDetector = object
    monkeypatch.setitem(sys.modules, detector_module.__name__, detector_module)
    from rebotarm_vision.nodes.vision_node import RebotArmVisionNode

    calls = []
    published = []
    node = object.__new__(RebotArmVisionNode)
    node.camera = SimpleNamespace(
        get_frame=lambda **kwargs: calls.append(kwargs) or (np.zeros((1, 2, 3), dtype=np.uint8), None),
        last_debug_message="depth_frame_none",
        get_camera_info=lambda stream: None,
    )
    node.enable_depth = True
    node.empty_frame_count = 0
    node.max_empty_frames = 30
    node._vision_failure_latched = False
    node._record_frame_failure = lambda kind: calls.append(kind)
    node._frame_stamp = lambda stream, stamp: stamp
    node._publish_empty_detection = lambda stamp=None: published.append(stamp)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object()))
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)
    node.color_pub = SimpleNamespace(publish=lambda message: None)
    node.frame_id_color = "camera_color_frame"
    node.color_camera_info_pub = SimpleNamespace(publish=lambda message: None)

    node._on_timer()

    assert calls[0] == {"allow_partial": True}
    assert "partial" in calls
    assert len(published) == 1
    assert node.empty_frame_count == 1


def test_vision_node_close_failure_propagates_after_ros_cleanup(monkeypatch):
    detector_module = types.ModuleType("rebotarm_vision.detector.yolo_detector")
    detector_module.YoloDetector = object
    monkeypatch.setitem(sys.modules, detector_module.__name__, detector_module)
    from rclpy.node import Node
    from rebotarm_vision.nodes.vision_node import RebotArmVisionNode

    cleaned = []
    monkeypatch.setattr(Node, "destroy_node", lambda node: cleaned.append(True))
    node = object.__new__(RebotArmVisionNode)
    def close():
        raise RuntimeError("USB stop failed")
    node.camera = SimpleNamespace(close=close)
    node._preview_window_created = False
    with pytest.raises(RuntimeError, match="USB stop failed"):
        node.destroy_node()
    assert cleaned == [True]
