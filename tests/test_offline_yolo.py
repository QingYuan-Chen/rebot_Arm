from types import SimpleNamespace
import importlib
import sys
import types

import numpy as np
import pytest

from rebotarm_vision.offline_yolo import (
    decode_image_to_bgr,
    filter_detection_array,
    parse_target_classes,
)
from sensor_msgs.msg import Image


def _image(*, encoding, width, height, step, data):
    message = Image()
    message.encoding = encoding
    message.width = width
    message.height = height
    message.step = step
    message.data = data
    return message


def test_parse_target_classes_accepts_launch_yaml_and_normalizes():
    assert parse_target_classes("['Cube', ' bottle ']") == ("cube", "bottle")
    assert parse_target_classes("bottle, cup") == ("bottle", "cup")
    assert parse_target_classes([]) == ()


def test_decode_rgb8_with_row_padding_returns_bgr():
    message = _image(
        encoding="rgb8",
        width=2,
        height=2,
        step=8,
        data=bytes(
            [
                1,
                2,
                3,
                4,
                5,
                6,
                99,
                99,
                7,
                8,
                9,
                10,
                11,
                12,
                98,
                98,
            ]
        ),
    )
    result = decode_image_to_bgr(message)
    assert result.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(
        result,
        np.asarray([[[3, 2, 1], [6, 5, 4]], [[9, 8, 7], [12, 11, 10]]], dtype=np.uint8),
    )


def test_decode_rejects_unsupported_or_truncated_image():
    with pytest.raises(ValueError, match="supports rgb8"):
        decode_image_to_bgr(_image(encoding="mono8", width=1, height=1, step=1, data=b"\x00"))
    with pytest.raises(ValueError, match="needs at least"):
        decode_image_to_bgr(_image(encoding="rgb8", width=2, height=1, step=6, data=b"\x00" * 5))


def test_filter_detection_array_is_explicit_allowlist_and_fail_closed():
    detections = SimpleNamespace(header=SimpleNamespace(), detections=[])
    for class_name in ("Cube", "fire hydrant"):
        detection = SimpleNamespace(class_name=class_name)
        detections.detections.append(detection)

    filtered = filter_detection_array(detections, ["cube"])
    assert [item.class_name for item in filtered.detections] == ["Cube"]
    assert filter_detection_array(detections, []).detections == []


@pytest.fixture
def offline_node_factory(monkeypatch, tmp_path):
    """构造真实节点类与检测器，只替换模型依赖和 ROS 通信端口，避免创建 DDS 实体。"""
    from rebotarm_vision.nodes import offline_yolo_node as module
    from rclpy.node import Node

    model_path = tmp_path / "test.pt"
    model_path.touch()
    models = []
    class Model:
        def __init__(self, path):
            assert path == str(model_path)
            self.results = []
            models.append(self)

        def predict(self, image, **kwargs):
            return self.results

    monkeypatch.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=Model))
    detector_name = "rebotarm_vision.detector.yolo_detector"
    detector_package = importlib.import_module("rebotarm_vision.detector")
    missing = object()
    previous_child = getattr(detector_package, "yolo_detector", missing)
    monkeypatch.delitem(sys.modules, detector_name, raising=False)
    detector_module = importlib.import_module(detector_name)
    monkeypatch.setattr(Node, "__init__", lambda self, name: None)

    def build(*, annotated=None):
        params = {"offline_yolo.model_path": str(model_path), "offline_yolo.target_classes": "['bottle']"}
        if annotated is not None:
            params["offline_yolo.publish_annotated"] = annotated
        output, logs = [], []
        monkeypatch.setattr(Node, "declare_parameter", lambda self, key, default: params.setdefault(key, default))
        monkeypatch.setattr(Node, "get_parameter", lambda self, key: SimpleNamespace(value=params[key]))
        monkeypatch.setattr(Node, "create_publisher", lambda self, msg_type, topic, qos:
                            SimpleNamespace(publish=lambda message: output.append((topic, message))))
        monkeypatch.setattr(Node, "create_subscription", lambda self, *args: args)
        monkeypatch.setattr(Node, "get_logger", lambda self: SimpleNamespace(
            info=lambda message: logs.append(("info", message)),
            error=lambda message: logs.append(("error", message)),
        ))
        node = module.OfflineYoloNode()
        assert isinstance(node._detector, detector_module.YoloDetector)
        return node, models[-1], output, logs

    yield build
    # 防止真实检测器模块持有本测试的模型替身，影响后续独立模块测试。
    sys.modules.pop(detector_name, None)
    if previous_child is missing:
        delattr(detector_package, "yolo_detector")
    else:
        detector_package.yolo_detector = previous_child


def _result(*, bad_other_mask=False):
    return SimpleNamespace(
        names={0: "bottle", 1: "person"},
        boxes=[SimpleNamespace(xyxy=[[1, 1, 6, 6]], cls=[value], conf=[.9]) for value in (0, 1)],
        masks=SimpleNamespace(xy=[
            [[1, 1], [6, 1], [1, 6]], None if bad_other_mask else [[1, 1], [6, 1], [1, 6]],
        ]),
    )


def _frame(sec=7):
    message = _image(encoding="bgr8", width=8, height=8, step=24, data=bytes(8 * 8 * 3))
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = 123
    message.header.frame_id = "external_camera"
    return message


@pytest.mark.parametrize("annotated", [None, False])
def test_offline_node_constructs_and_publishes_default_or_disabled_annotation(offline_node_factory, annotated):
    node, model, output, _ = offline_node_factory(annotated=annotated)
    model.results = [_result()]
    frame = _frame()
    node._on_image(frame)
    expected = [node.raw_detection_topic, node.detection_topic]
    if annotated is None:
        expected.append(node.annotated_topic)
    assert [topic for topic, _ in output] == expected
    assert [item.class_name for item in output[0][1].detections] == ["bottle", "person"]
    assert [item.class_name for item in output[1][1].detections] == ["bottle"]
    for _, message in output:
        assert message.header.stamp == frame.header.stamp
        assert message.header.frame_id == frame.header.frame_id
    if annotated is None:
        assert output[-1][1].encoding == "bgr8"
        assert bytes(output[-1][1].data) != bytes(frame.data)


def test_offline_model_load_failure_reaches_constructor(offline_node_factory, monkeypatch):
    def fail_load(path):
        raise RuntimeError("model load failed")
    detector = sys.modules["rebotarm_vision.detector.yolo_detector"]
    monkeypatch.setattr(detector, "YOLO", fail_load)
    with pytest.raises(RuntimeError, match="model load failed"):
        offline_node_factory()


def test_offline_annotation_failure_propagates_after_detection_publication(offline_node_factory, monkeypatch):
    from rebotarm_vision.nodes import offline_yolo_node as module
    node, model, output, _ = offline_node_factory()
    model.results = [_result()]
    def fail_draw(*args):
        raise RuntimeError("drawing failed")
    monkeypatch.setattr(module, "draw_detections", fail_draw)
    with pytest.raises(RuntimeError, match="drawing failed"):
        node._on_image(_frame())
    assert [topic for topic, _ in output] == [node.raw_detection_topic, node.detection_topic]
    assert [len(message.detections) for _, message in output] == [2, 1]
    assert not node._failure_latched


def test_offline_bad_mask_outside_allowlist_clears_both_streams_and_recovers(offline_node_factory):
    node, model, output, logs = offline_node_factory(annotated=False)
    model.results = [_result(bad_other_mask=True)]
    for sec in (7, 8):
        node._on_image(_frame(sec))
        assert all(not message.detections for _, message in output[-2:])
        assert all(message.header.stamp.sec == sec for _, message in output[-2:])
        assert all(message.header.frame_id == "external_camera" for _, message in output[-2:])
    assert node._failure_latched
    assert len([entry for level, entry in logs if level == "error"]) == 1
    assert "mask[1]" in next(entry for level, entry in logs if level == "error")
    model.results = [_result()]
    node._on_image(_frame(9))
    assert [len(message.detections) for _, message in output[-2:]] == [2, 1]
    assert not node._failure_latched
    assert any("recovered" in entry for _, entry in logs)
