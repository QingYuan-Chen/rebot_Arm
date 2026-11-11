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


@pytest.fixture
def online_detection_node(monkeypatch):
    """使用真实帧转换与节点回调,替换相机和发布端口以保持测试离线."""
    detector_module = types.ModuleType("rebotarm_vision.detector.yolo_detector")
    detector_module.YoloDetector = object
    monkeypatch.setitem(sys.modules, detector_module.__name__, detector_module)
    from rebotarm_vision.nodes.vision_node import RebotArmVisionNode
    from sensor_msgs.msg import Image

    stamp = Image().header.stamp
    stamp.sec, stamp.nanosec = 10, 123
    node = object.__new__(RebotArmVisionNode)
    published, logs = [], []
    node.camera = SimpleNamespace(
        get_frame=lambda **kwargs: (np.zeros((2, 2, 3), dtype=np.uint8), np.ones((2, 2), dtype=np.uint16)),
        get_camera_info=lambda stream: {"width": 2, "height": 2, "fx": 2., "fy": 2., "cx": 1., "cy": 1.},
    )
    node.enable_depth = True
    node.empty_frame_count = 0
    node._vision_failure_latched = False
    node._detection_failure_latched = False
    node._detection_log_countdown = 0
    node._frame_stamp = lambda stream, receipt: stamp
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    node.get_logger = lambda: SimpleNamespace(
        error=lambda message: logs.append(("error", message)),
        info=lambda message: logs.append(("info", message)),
    )
    node.frame_id_color = "camera_color_frame"
    node.frame_id_depth = "camera_depth_frame"
    for attr in ("color_pub", "depth_pub", "color_camera_info_pub", "depth_camera_info_pub"):
        setattr(node, attr, SimpleNamespace(publish=lambda message: None))
    node.detection_pub = SimpleNamespace(publish=published.append)
    node.annotated_pub = None
    node.show_preview = False
    node.custom_classes = ["bottle"]
    result = SimpleNamespace(
        names={0: "bottle"}, boxes=[SimpleNamespace(xyxy=[[0, 0, 2, 2]], cls=[0], conf=[.9])],
        masks=SimpleNamespace(xy=[[[0, 0], [2, 0], [0, 2]]]),
    )
    node.detector = SimpleNamespace(infer=lambda image: [result])
    return node, result, published, logs, stamp


def test_online_mask_failure_clears_current_frame_latches_and_recovers(online_detection_node):
    node, result, published, logs, stamp = online_detection_node
    node._on_timer()
    assert len(published[-1].detections) == 1
    result.masks.xy = [None]
    for sec in (11, 12):
        stamp.sec = sec
        node._on_timer()
        assert not published[-1].detections
        assert published[-1].header.stamp.sec == sec
        assert published[-1].header.frame_id == "camera_color_frame"
    assert node._detection_failure_latched
    assert not node._vision_failure_latched
    assert [level for level, _ in logs].count("error") == 1
    assert "mask[0]" in next(message for level, message in logs if level == "error")
    result.masks.xy = [[[0, 0], [2, 0], [0, 2]]]
    stamp.sec = 13
    node._on_timer()
    assert len(published[-1].detections) == 1
    assert not node._detection_failure_latched
    assert any("detections recovered" in message for _, message in logs)


@pytest.mark.parametrize("stage", ["infer", "convert"])
def test_online_internal_errors_propagate(online_detection_node, monkeypatch, stage):
    from rebotarm_vision.nodes import vision_node as module
    node, _, published, _, _ = online_detection_node
    failure = RuntimeError("internal detector failure")
    def fail(*args, **kwargs):
        raise failure
    if stage == "infer":
        node.detector.infer = fail
    else:
        monkeypatch.setattr(module, "result_to_detection_array_msg", fail)
    with pytest.raises(RuntimeError) as error:
        node._on_timer()
    assert error.value is failure
    assert not published
    assert not node._detection_failure_latched


def test_online_bad_mask_empty_frame_clears_graspnet_pending_candidates(online_detection_node):
    from rebotarm_vision.nodes.graspnet_baseline_node import GraspNetBaselineNode
    node, result, published, _, stamp = online_detection_node
    result.masks.xy = [None]
    node._on_timer()

    consumer = object.__new__(GraspNetBaselineNode)
    candidates = []
    # 用旧的待处理检测模拟 RGB-D 尚未配对的上一帧;新空帧须立即替换它并发布空候选.
    consumer.pending_inprocess_detections = SimpleNamespace(detections=[object()])
    consumer.last_inprocess_detection_timestamp_ns = 1
    consumer.latest_depth_frame_id = "camera_depth_frame"
    consumer.output_frame_id = "camera_depth_frame"
    consumer.candidates_pub = SimpleNamespace(publish=candidates.append)
    consumer._on_inprocess_detections(published[-1])
    assert consumer.pending_inprocess_detections is None
    assert consumer.last_inprocess_detection_timestamp_ns == stamp.sec * 1_000_000_000 + stamp.nanosec
    assert len(candidates) == 1
    assert not candidates[0].candidates and candidates[0].best_index == -1
    assert candidates[0].header.stamp == stamp
    consumer._try_process_inprocess_detection()
    assert len(candidates) == 1
