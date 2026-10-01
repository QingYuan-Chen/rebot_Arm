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


def _candidate():
    from rebotarm_msgs.msg import GraspCandidate

    candidate = GraspCandidate()
    candidate.valid = True
    candidate.jaw_width = 0.04
    candidate.confidence = 0.9
    return candidate


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


def test_network_graspnet_payload_converts_to_candidates():
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


def test_candidate_filter_tf_failure_publishes_no_ranked_candidates():
    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy
    from rebotarm_vision.candidate_ik_runtime import CandidateIkRuntime
    from types import SimpleNamespace

    msg = GraspCandidateArray()
    msg.header.frame_id = "camera_depth_frame"
    candidate = GraspCandidate()
    candidate.confidence = 0.8
    candidate.jaw_width = 0.04
    msg.candidates.append(candidate)
    published = []
    warnings = []
    node = object.__new__(CandidateIkPolicy)
    parameters = {
        "max_candidates_per_frame": 10,
        "candidate_min_confidence": 0.4,
        "candidate_min_jaw_width_m": 0.006,
        "candidate_max_jaw_width_m": 0.085,
    }
    node.get_parameter = lambda name: type(
        "Parameter", (), {"value": parameters[name]}
    )()
    node._candidate_target_variants = lambda _msg, _pose: (_ for _ in ()).throw(
        RuntimeError("TF lookup unavailable")
    )
    node._publish_ranked = lambda original, ranked: published.append((original, ranked))
    node.get_logger = lambda: type("Logger", (), {"warn": warnings.append})()

    CandidateIkRuntime(
        policy=node, gateway=SimpleNamespace(
            check_target=getattr(node, "_check_ik_and_collision", None),
            publish_ranked=node._publish_ranked, publish_empty=lambda message: None,
        ), max_candidates=10, logger=node.get_logger(),
    ).filter_frame(msg)

    assert published == [(msg, [])]
    assert warnings == ["candidate IK filter rejected candidate: TF lookup unavailable"]


def test_preserve_input_safety_gate_allows_low_grasp_when_width_is_valid():
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy
    from rebotarm_vision.candidate_ik_runtime import CandidateIkRuntime
    from types import SimpleNamespace
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    class NodeForGate:
        def get_parameter(self, name):
            values = {
                "candidate_min_jaw_width_m": 0.006,
                "candidate_max_jaw_width_m": 0.082,
                "candidate_min_grasp_z_m": 0.0,
            }
            return type("Param", (), {"value": values[name]})()

        def get_logger(self):
            return type("Logger", (), {"warn": lambda self, message: None})()

    candidate = _candidate()
    candidate.jaw_width = 0.04
    low_grasp = PoseTarget(position=(0.38, 0.0, 0.035), orientation=(0.0, 0.0, 0.0, 1.0))
    safe_grasp = PoseTarget(position=(0.38, 0.0, 0.18), orientation=(0.0, 0.0, 0.0, 1.0))
    node = NodeForGate()

    assert CandidateIkPolicy._candidate_gate_allows(node, candidate, grasp=low_grasp) is True
    assert CandidateIkPolicy._candidate_gate_allows(node, candidate, grasp=safe_grasp) is True
