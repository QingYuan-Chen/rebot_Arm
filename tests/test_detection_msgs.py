from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Image


STAMP = Image().header.stamp


def test_ultralytics_segmentation_polygon_is_preserved_in_detection_message():
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    box = SimpleNamespace(
        xyxy=np.asarray([[10.0, 20.0, 110.0, 220.0]], dtype=np.float32),
        cls=np.asarray([0.0], dtype=np.float32),
        conf=np.asarray([0.91], dtype=np.float32),
    )
    result = SimpleNamespace(
        names={0: "bottle"},
        boxes=[box],
        obb=None,
        masks=SimpleNamespace(
            xy=[
                np.asarray(
                    [[20.0, 30.0], [100.0, 30.0], [100.0, 210.0], [20.0, 210.0]],
                    dtype=np.float32,
                )
            ]
        ),
    )

    msg = result_to_detection_array_msg([result], stamp=STAMP, frame_id="camera_color_frame")

    assert len(msg.detections) == 1
    detection = msg.detections[0]
    assert detection.class_name == "bottle"
    assert detection.has_mask is True
    assert list(detection.mask_polygon_xy) == [
        20.0,
        30.0,
        100.0,
        30.0,
        100.0,
        210.0,
        20.0,
        210.0,
    ]


@pytest.mark.parametrize("masks, reason", [
    (SimpleNamespace(), "xy is missing"),
    (SimpleNamespace(xy=None), "xy is missing"),
    (SimpleNamespace(xy=[]), "polygon is missing"),
    (SimpleNamespace(xy=[None]), "polygon is None"),
    (SimpleNamespace(xy=[[[0, 0], [1, 1]]]), "at least 3 vertices"),
    (SimpleNamespace(xy=[[0, 0, 1, 0, 0, 1]]), "expected shape"),
    (SimpleNamespace(xy=[np.zeros((2, 3))]), "expected shape"),
    (SimpleNamespace(xy=[[[0, 0], [1, 0], [0, float("nan")]]]), "must be finite"),
    (SimpleNamespace(xy=[[[0, 0], [1, 0], [0, float("inf")]]]), "must be finite"),
    (SimpleNamespace(xy=[[[0, 0], [1, 0], [0, "bad"]]]), "numeric coordinates"),
    (SimpleNamespace(xy=[[[0, 0], [1], [0, 1]]]), "numeric coordinates"),
])
def test_invalid_ultralytics_segmentation_polygon_rejects_frame(masks, reason):
    from rebotarm_vision.converters.detection_msgs import MaskPolygonError, result_to_detection_array_msg

    box = SimpleNamespace(
        xyxy=np.asarray([[10.0, 20.0, 110.0, 220.0]], dtype=np.float32),
        cls=np.asarray([0.0], dtype=np.float32),
        conf=np.asarray([0.91], dtype=np.float32),
    )
    result = SimpleNamespace(
        names={0: "bottle"},
        boxes=[box],
        obb=None,
        masks=masks,
    )

    with pytest.raises(MaskPolygonError, match=rf"mask\[0\].*{reason}"):
        result_to_detection_array_msg([result], stamp=STAMP, frame_id="camera_color_frame")


@pytest.mark.parametrize("provide_masks", [False, True])
def test_detection_model_without_masks_keeps_bbox(provide_masks):
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    box = SimpleNamespace(xyxy=[[1, 2, 10, 20]], cls=[0], conf=[.9])
    result = SimpleNamespace(names={0: "bottle"}, boxes=[box])
    if provide_masks:
        result.masks = None
    msg = result_to_detection_array_msg([result], stamp=STAMP, frame_id="camera")
    assert len(msg.detections) == 1
    detection = msg.detections[0]
    assert not detection.has_mask and list(detection.mask_polygon_xy) == []
    assert (detection.x_min, detection.y_min, detection.x_max, detection.y_max) == (1, 2, 10, 20)


def test_bad_second_mask_rejects_whole_frame_but_allowlist_precedes_conversion():
    from rebotarm_vision.converters.detection_msgs import MaskPolygonError, result_to_detection_array_msg

    result = SimpleNamespace(
        names={0: "bottle", 1: "person"},
        boxes=[SimpleNamespace(xyxy=[[1, 2, 10, 20]], cls=[value], conf=[.9]) for value in (0, 1)],
        masks=SimpleNamespace(xy=[[[0, 0], [1, 0], [0, 1]], None]),
    )
    with pytest.raises(MaskPolygonError, match=r"mask\[1\]: polygon is None"):
        result_to_detection_array_msg([result], stamp=STAMP, frame_id="camera")
    filtered = result_to_detection_array_msg(
        [result], stamp=STAMP, frame_id="camera", allowed_classes={"bottle"}
    )
    assert [item.class_name for item in filtered.detections] == ["bottle"]


def test_mask_tensor_transfer_error_keeps_original_exception():
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    failure = RuntimeError("device transfer failed")
    def fail_transfer():
        raise failure
    result = SimpleNamespace(
        names={0: "bottle"}, boxes=[SimpleNamespace(xyxy=[[1, 2, 10, 20]], cls=[0], conf=[.9])],
        masks=SimpleNamespace(xy=[SimpleNamespace(cpu=fail_transfer)]),
    )
    with pytest.raises(RuntimeError) as error:
        result_to_detection_array_msg([result], stamp=STAMP, frame_id="camera")
    assert error.value is failure


@pytest.mark.parametrize("with_mask, expected", [(False, [True, True]), (True, [True, False])])
def test_valid_detection_retains_graspnet_bbox_or_mask_sampling(with_mask, expected):
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg
    from rebotarm_vision.backends.graspnet_baseline_inference import detection_contains_pixels

    result = SimpleNamespace(
        names={0: "bottle"}, boxes=[SimpleNamespace(xyxy=[[0, 0, 8, 8]], cls=[0], conf=[.9])],
        masks=SimpleNamespace(xy=[[[0, 0], [8, 0], [0, 8]]]) if with_mask else None,
    )
    detection = result_to_detection_array_msg([result], STAMP, "camera").detections[0]
    payload = {key: getattr(detection, key) for key in ("x_min", "y_min", "x_max", "y_max")}
    if detection.has_mask:
        payload["mask_polygon_xy"] = list(detection.mask_polygon_xy)
    keep = detection_contains_pixels(payload, u=np.asarray([1., 7.]), v=np.asarray([1., 7.]))
    assert keep.tolist() == expected


def test_detection_converter_applies_allowed_class_filter():
    from rebotarm_vision.converters.detection_msgs import result_to_detection_array_msg

    person_box = SimpleNamespace(
        xyxy=np.asarray([[10.0, 20.0, 110.0, 220.0]], dtype=np.float32),
        cls=np.asarray([0.0], dtype=np.float32),
        conf=np.asarray([0.95], dtype=np.float32),
    )
    bottle_box = SimpleNamespace(
        xyxy=np.asarray([[20.0, 30.0, 100.0, 210.0]], dtype=np.float32),
        cls=np.asarray([1.0], dtype=np.float32),
        conf=np.asarray([0.90], dtype=np.float32),
    )
    result = SimpleNamespace(
        names={0: "person", 1: "bottle"},
        boxes=[person_box, bottle_box],
        obb=None,
        masks=None,
    )

    msg = result_to_detection_array_msg(
        [result],
        stamp=STAMP,
        frame_id="camera_color_frame",
        allowed_classes={"bottle"},
    )

    assert [detection.class_name for detection in msg.detections] == ["bottle"]
