from __future__ import annotations

import numpy as np
import pytest
import sys
import types


def _detection():
    from rebotarm_msgs.msg import Detection2D

    det = Detection2D()
    det.class_name = "bottle"
    det.confidence = 0.8
    det.x_min = 1
    det.y_min = 1
    det.x_max = 3
    det.y_max = 3
    det.center_u = 2
    det.center_v = 2
    return det


def test_graspnet_depth_crop_projects_detection_roi_to_point_cloud():
    from rebotarm_vision.graspnet_baseline_adapter import (
        CameraIntrinsics,
        build_point_cloud_for_detection,
    )

    depth_mm = np.array(
        [
            [0, 0, 0, 0],
            [0, 500, 510, 0],
            [0, 520, 0, 0],
            [0, 0, 0, 900],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    color_bgr[1, 1] = [10, 20, 30]

    cloud = build_point_cloud_for_detection(
        depth_mm,
        color_bgr,
        _detection(),
        CameraIntrinsics(fx=100.0, fy=100.0, cx=2.0, cy=2.0),
        min_depth_m=0.15,
        max_depth_m=0.8,
    )

    assert cloud.points.shape == (3, 3)
    assert cloud.colors.shape == (3, 3)
    assert cloud.points[0, 2] == pytest.approx(0.5)
    assert cloud.points[0, 0] == pytest.approx(-0.005)
    assert cloud.points[0, 1] == pytest.approx(-0.005)
    assert cloud.colors[0].tolist() == pytest.approx([30 / 255.0, 20 / 255.0, 10 / 255.0])


def test_graspnet_outputs_convert_to_ranked_grasp_candidates():
    from rebotarm_vision.graspnet_baseline_adapter import (
        GraspNetPrediction,
        predictions_to_candidate_array,
    )

    predictions = [
        GraspNetPrediction(
            score=0.92,
            translation_xyz=(0.10, 0.02, 0.40),
            rotation_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            width_m=0.045,
            object_length_m=0.11,
        ),
        GraspNetPrediction(
            score=0.75,
            translation_xyz=(0.11, 0.02, 0.39),
            rotation_matrix=((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            width_m=0.040,
            object_length_m=0.10,
        ),
    ]

    candidates = predictions_to_candidate_array(
        predictions,
        frame_id="camera_depth_frame",
        class_name="bottle",
        max_candidates=2,
    )

    assert candidates.header.frame_id == "camera_depth_frame"
    assert candidates.best_index == 0
    assert len(candidates.candidates) == 2
    assert candidates.candidates[0].source == "graspnet_baseline"
    assert candidates.candidates[0].confidence == pytest.approx(0.92)
    assert candidates.candidates[0].pose.position.z == pytest.approx(0.40)
    assert candidates.candidates[0].jaw_width == pytest.approx(0.045)
    assert candidates.candidates[0].valid is True
    assert candidates.candidates[1].confidence == pytest.approx(0.75)


def test_inprocess_graspnet_payload_converts_to_candidates():
    from rebotarm_vision.graspnet_baseline_adapter import payload_to_candidate_array

    payload = {
        "frame_id": "camera_depth_frame",
        "source": "graspnet_baseline",
        "backend_configured": True,
        "timestamp_ns": 1_700_000_000_123_456_789,
        "candidates": [
            {
                "class_name": "bottle",
                "score": 0.88,
                "translation_xyz": [0.12, -0.01, 0.42],
                "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "width_m": 0.042,
                "object_length_m": 0.12,
            }
        ],
    }

    candidates = payload_to_candidate_array(payload, fallback_frame_id="camera_depth_frame", max_candidates=10)

    assert candidates.best_index == 0
    assert candidates.header.frame_id == "camera_depth_frame"
    assert len(candidates.candidates) == 1
    assert candidates.candidates[0].source == "graspnet_baseline"
    assert candidates.candidates[0].confidence == pytest.approx(0.88)
    assert candidates.candidates[0].pose.position.x == pytest.approx(0.12)
    assert candidates.candidates[0].jaw_width == pytest.approx(0.042)
    assert candidates.header.stamp.sec == 1_700_000_000
    assert candidates.header.stamp.nanosec == 123_456_789
    assert candidates.candidates[0].header.stamp.sec == 1_700_000_000


def test_corrupt_prediction_rejects_whole_payload():
    from rebotarm_vision.graspnet_baseline_adapter import payload_to_candidate_array
    good = {"score": .8, "width_m": .04, "translation_xyz": [0, 0, .4],
            "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}
    for bad in ({**good, "score": float("nan")}, {key: value for key, value in good.items() if key != "width_m"},
                {**good, "rotation_matrix": [[0, 0, 0]] * 3}, None):
        with pytest.raises((ValueError, KeyError)):
            payload_to_candidate_array({"candidates": [good, bad]}, fallback_frame_id="camera", max_candidates=1)
    assert payload_to_candidate_array({"candidates": []}, fallback_frame_id="camera", max_candidates=1).candidates == []


def test_graspnet_node_fails_startup_when_backend_unavailable(monkeypatch):
    from types import SimpleNamespace
    from rebotarm_vision.nodes import graspnet_baseline_node as module
    monkeypatch.setattr(module, "InProcessGraspNetBackend",
                        lambda **_: SimpleNamespace(available=False, backend_error="missing weights"))
    values = {"model_root": "", "checkpoint_path": "", "device": "cpu",
              "backend_module": "", "backend_module_path": "", "max_points": 32}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(value=values[name]))
    with pytest.raises(RuntimeError, match="missing weights"):
        module.GraspNetBaselineNode._create_inprocess_backend(node)


def test_inprocess_graspnet_calls_full_rgbd_backend_without_http(tmp_path):
    from rebotarm_vision.graspnet_baseline_adapter import InProcessGraspNetBackend

    model_root = tmp_path / "graspnet-baseline"
    model_root.mkdir()
    checkpoint = tmp_path / "checkpoint-rs.tar"
    checkpoint.write_bytes(b"test")
    calls = []

    class FakeRunner:
        def __init__(self, **kwargs):
            calls.append(("init", kwargs))
            self.last_stage_counts = {"raw": 4, "published": 1, "empty_reason": ""}

        def infer(self, **kwargs):
            calls.append(("infer", kwargs))
            return [
                {
                    "class_name": "bottle",
                    "score": 0.91,
                    "translation_xyz": [0.1, -0.02, 0.32],
                    "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    "width_m": 0.073,
                }
            ]

    module_name = "test_inprocess_graspnet_backend"
    module = types.ModuleType(module_name)
    module.GraspNetBaselineInference = FakeRunner
    sys.modules[module_name] = module
    try:
        backend = InProcessGraspNetBackend(
            model_root=str(model_root),
            checkpoint_path=str(checkpoint),
            device="cuda:0",
            module_name=module_name,
            num_point=20_000,
        )
        color = np.zeros((4, 5, 3), dtype=np.uint8)
        depth_m = np.full((4, 5), 0.32, dtype=np.float32)
        payload = backend.infer(
            timestamp_ns=1_700_000_000_123_456_789,
            frame_id="camera_depth_frame",
            color_bgr=color,
            depth_m=depth_m,
            camera_info={"fx": 300.0, "fy": 301.0, "cx": 2.0, "cy": 1.5},
            detection={
                "x_min": 0,
                "y_min": 0,
                "x_max": 5,
                "y_max": 4,
                "confidence": 0.9,
                "class_name": "bottle",
                "mask_polygon_xy": [0, 0, 4, 0, 4, 3, 0, 3],
            },
            max_grasps=10,
            max_jaw_width_m=0.085,
        )
    finally:
        sys.modules.pop(module_name, None)

    assert backend.available is True
    assert payload["source"] == "ubuntu_inprocess_graspnet"
    assert payload["timestamp_ns"] == 1_700_000_000_123_456_789
    assert payload["frame_id"] == "camera_depth_frame"
    assert payload["candidates"][0]["score"] == pytest.approx(0.91)
    infer_call = dict(calls)["infer"]
    assert np.array_equal(infer_call["color_bgr"], color)
    assert np.array_equal(infer_call["depth_mm"], depth_m)
    assert infer_call["camera_info"]["depth_scale_m"] == 1.0
    assert infer_call["camera_info"]["workspace_min_depth_m"] == pytest.approx(0.05)
    assert infer_call["camera_info"]["workspace_max_depth_m"] == pytest.approx(1.5)
    assert infer_call["detections"][0]["mask_polygon_xy"] == [0, 0, 4, 0, 4, 3, 0, 3]
    assert infer_call["max_grasps"] == 10
    assert infer_call["max_jaw_width_m"] == pytest.approx(0.085)
    assert backend.last_stage_counts["published"] == 1
