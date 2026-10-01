from types import SimpleNamespace
import sys
import types

import numpy as np

from rebotarm_vision.policies.camera_info_policy import calibrated_intrinsics
from rebotarm_vision.policies.candidate_motion_policy import joint_positions_by_name
from rebotarm_vision.policies.visual_grasp_runtime_policy import (
    detected_jaw_width,
    velocity_scaling_for_stage,
)
from rebotarm_vision.visual_grasp_state import plan_snapshot


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


def test_runtime_policy_isolated_from_ros_parameters():
    plan = SimpleNamespace(jaw_width=0.0, candidate=SimpleNamespace(jaw_width=0.04))
    assert detected_jaw_width(plan) == 0.04
    assert velocity_scaling_for_stage("safe_retreat", approach=0.1, retreat=0.2, normal=0.3) == 0.2


def test_plan_snapshot_extracts_diagnostic_data_without_ros_node():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    plan = SimpleNamespace(
        valid=True,
        source="test",
        reason="",
        header=SimpleNamespace(
            frame_id="camera_depth_frame",
            stamp=SimpleNamespace(sec=2, nanosec=3),
        ),
        candidate=SimpleNamespace(class_name="cup", confidence=0.8, jaw_width=0.04),
        jaw_width=0.0,
        pregrasp_pose=pose,
        grasp_pose=pose,
    )
    snapshot = plan_snapshot(plan)
    assert snapshot.frame_id == "camera_depth_frame"
    assert snapshot.stamp_ns == 2_000_000_003
    assert snapshot.jaw_width_m == 0.04


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
