from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_wrapper():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "src/rebotarm_vision/rebotarm_vision/backends/graspnet_baseline_inference.py"
    spec = importlib.util.spec_from_file_location("graspnet_baseline_inference", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_graspnet_wrapper_converts_graspnet_array_rows_to_json_candidates():
    module = _load_wrapper()
    row = np.array(
        [
            0.91,
            0.042,
            0.02,
            0.03,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.10,
            -0.02,
            0.35,
            3.0,
        ],
        dtype=np.float32,
    )

    candidates = module.graspnet_array_to_candidates(
        np.asarray([row]),
        class_name="bottle",
        max_grasps=5,
    )

    assert candidates[0]["class_name"] == "bottle"
    assert candidates[0]["score"] == pytest.approx(0.91)
    assert candidates[0]["width_m"] == pytest.approx(0.042)
    assert candidates[0]["translation_xyz"] == pytest.approx([0.10, -0.02, 0.35])
    assert np.asarray(candidates[0]["rotation_matrix"]) == pytest.approx(np.eye(3))


def test_graspnet_wrapper_builds_scene_cloud_from_full_depth_image():
    module = _load_wrapper()
    depth_mm = np.array(
        [
            [0, 0, 0, 0],
            [0, 500, 500, 0],
            [0, 500, 500, 0],
            [0, 0, 0, 0],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 2.0, "depth_scale_m": 0.001}

    points, colors = module.build_scene_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
    )

    assert points.shape == (4, 3)
    assert colors.shape == (4, 3)
    assert points[:, 2].tolist() == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_graspnet_wrapper_scene_cloud_rejects_far_depth_outliers():
    module = _load_wrapper()
    depth_mm = np.array(
        [
            [500, 500, 4000],
            [500, 65511, 500],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((2, 3, 3), dtype=np.uint8)
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 1.0, "cy": 1.0, "depth_scale_m": 0.001}

    points, _colors = module.build_scene_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
    )

    assert points.shape[0] == 4
    assert points[:, 2].max() == pytest.approx(0.5)


def test_graspnet_sampling_is_deterministic_and_preserves_requested_shape():
    module = _load_wrapper()
    points = np.asarray(
        [[index * 0.0005, (index % 7) * 0.001, 0.3] for index in range(64)],
        dtype=np.float64,
    )
    colors = np.arange(64 * 3, dtype=np.uint8).reshape(64, 3)

    first_points, first_colors = module.sample_cloud(points, colors, num_point=40)
    second_points, second_colors = module.sample_cloud(points, colors, num_point=40)

    assert first_points.shape == (40, 3)
    assert first_colors.shape == (40, 3)
    assert first_points.dtype == np.float32
    assert first_colors.dtype == np.float32
    np.testing.assert_array_equal(first_points, second_points)
    np.testing.assert_array_equal(first_colors, second_colors)


def test_graspnet_sampling_covers_sparse_voxels_before_dense_repeats():
    module = _load_wrapper()
    points = np.asarray(
        [
            [0.0001, 0.0, 0.3],
            [0.0002, 0.0, 0.3],
            [0.0003, 0.0, 0.3],
            [0.0101, 0.0, 0.3],
            [0.0201, 0.0, 0.3],
            [0.0301, 0.0, 0.3],
        ],
        dtype=np.float32,
    )
    colors = np.zeros_like(points)

    sampled, _ = module.sample_cloud(
        points,
        colors,
        num_point=4,
        voxel_size_m=0.002,
    )
    sampled_voxels = np.unique(np.floor(sampled / 0.002).astype(np.int64), axis=0)

    assert len(sampled_voxels) == 4


def test_graspnet_sampling_pads_small_cloud_deterministically():
    module = _load_wrapper()
    points = np.asarray([[0.0, 0.0, 0.3], [0.01, 0.0, 0.3]], dtype=np.float32)
    colors = np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)

    first_points, first_colors = module.sample_cloud(points, colors, num_point=5)
    second_points, second_colors = module.sample_cloud(points, colors, num_point=5)

    assert first_points.shape == (5, 3)
    assert {tuple(row) for row in first_points} == {tuple(row) for row in points}
    np.testing.assert_array_equal(first_points, second_points)
    np.testing.assert_array_equal(first_colors, second_colors)


def test_graspnet_sampling_rejects_invalid_configuration():
    module = _load_wrapper()
    points = np.zeros((2, 3), dtype=np.float32)
    colors = np.zeros((2, 3), dtype=np.float32)

    with pytest.raises(ValueError, match="num_point must be positive"):
        module.sample_cloud(points, colors, num_point=0)
    with pytest.raises(ValueError, match="voxel_size_m must be finite and positive"):
        module.sample_cloud(points, colors, num_point=1, voxel_size_m=0.0)
    with pytest.raises(ValueError, match="same number of rows"):
        module.sample_cloud(points, colors[:1], num_point=1)


def test_graspnet_wrapper_builds_detection_cloud_in_original_camera_coordinates():
    module = _load_wrapper()
    depth_mm = np.ones((4, 4), dtype=np.uint16) * 500
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [1.0, 1.0, 3.0, 3.0], "class_name": "bottle"}
    camera_info = {
        "fx": 100.0,
        "fy": 100.0,
        "cx": 2.0,
        "cy": 2.0,
        "depth_scale_m": 0.001,
        "foreground_min_seed_points": 1,
    }

    points, colors = module.build_detection_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
        detection=detection,
    )

    assert points.shape == (4, 3)
    assert colors.shape == (4, 3)
    np.testing.assert_allclose(
        points,
        np.asarray(
            [
                [-0.005, -0.005, 0.5],
                [0.0, -0.005, 0.5],
                [-0.005, 0.0, 0.5],
                [0.0, 0.0, 0.5],
            ],
            dtype=np.float32,
        ),
    )


def test_graspnet_wrapper_detection_cloud_removes_far_background_depth():
    module = _load_wrapper()
    depth_mm = np.ones((10, 10), dtype=np.uint16) * 900
    depth_mm[3:7, 3:7] = 500
    color_bgr = np.zeros((10, 10, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [0.0, 0.0, 10.0, 10.0], "class_name": "bottle"}
    camera_info = {
        "fx": 100.0,
        "fy": 100.0,
        "cx": 5.0,
        "cy": 5.0,
        "depth_scale_m": 0.001,
        "foreground_center_fraction": 0.4,
        "foreground_min_seed_points": 4,
    }

    points, _colors = module.build_detection_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
        detection=detection,
    )

    assert points.shape == (16, 3)
    assert points[:, 2].tolist() == pytest.approx([0.5] * 16)


def test_graspnet_wrapper_detection_cloud_fails_closed_without_center_depth_seed():
    module = _load_wrapper()
    depth_mm = np.ones((10, 10), dtype=np.uint16) * 900
    depth_mm[3:7, 3:7] = 0
    color_bgr = np.zeros((10, 10, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [0.0, 0.0, 10.0, 10.0], "class_name": "bottle"}
    camera_info = {
        "fx": 100.0,
        "fy": 100.0,
        "cx": 5.0,
        "cy": 5.0,
        "depth_scale_m": 0.001,
        "foreground_center_fraction": 0.4,
        "foreground_min_seed_points": 4,
    }

    points, colors = module.build_detection_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
        detection=detection,
    )

    assert points.shape == (0, 3)
    assert colors.shape == (0, 3)


def test_graspnet_inference_uses_detection_cloud_and_full_scene_for_collision(monkeypatch):
    module = _load_wrapper()

    class FakeInference(module.GraspNetBaselineInference):
        def _load_network(self):
            self._torch = object()
            self._pred_decode = object()
            self._GraspGroup = object()
            return object()

        def _infer_grasp_array(self, points, colors, *, full_points):
            self.seen_point_count = len(points)
            self.seen_full_point_count = len(full_points)
            return np.asarray(
                [
                    [
                        0.9,
                        0.04,
                        0.02,
                        0.03,
                        1,
                        0,
                        0,
                        0,
                        1,
                        0,
                        0,
                        0,
                        1,
                        0.0,
                        0.0,
                        0.5,
                        -1,
                    ]
                ],
                dtype=np.float32,
            )

    monkeypatch.setattr(
        module,
        "sample_cloud",
        lambda points, colors, *, num_point: (points.astype(np.float32), colors.astype(np.float32)),
    )

    depth_mm = np.ones((4, 4), dtype=np.uint16) * 500
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [1.0, 1.0, 3.0, 3.0], "class_name": "bottle", "confidence": 0.9}
    camera_info = {
        "fx": 100.0,
        "fy": 100.0,
        "cx": 2.0,
        "cy": 2.0,
        "depth_scale_m": 0.001,
        "foreground_min_seed_points": 1,
    }

    backend = FakeInference(model_root="", checkpoint_path="")
    candidates = backend.infer(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detections=[detection],
        camera_info=camera_info,
        max_grasps=1,
    )

    assert backend.seen_point_count == 4
    assert backend.seen_full_point_count == 16
    assert candidates[0]["class_name"] == "bottle"
    assert backend.last_stage_counts == {
        "scene_points": 16,
        "object_points": 4,
        "raw": 1,
        "after_collision": 1,
        "after_nms": 1,
        "after_score_sort": 1,
        "after_projection": 1,
        "after_object_geometry": 1,
        "after_jaw_width": 1,
        "published": 1,
        "empty_reason": "",
    }


def test_graspnet_inference_fails_closed_when_detection_has_no_valid_depth(monkeypatch):
    module = _load_wrapper()

    class FakeInference(module.GraspNetBaselineInference):
        def _load_network(self):
            self._torch = object()
            self._pred_decode = object()
            self._GraspGroup = object()
            return object()

        def _infer_grasp_array(self, points, colors, *, full_points):
            raise AssertionError("inference must not run for an empty detection cloud")

    depth_mm = np.ones((4, 4), dtype=np.uint16) * 500
    depth_mm[1:3, 1:3] = 0
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [1.0, 1.0, 3.0, 3.0], "class_name": "bottle", "confidence": 0.9}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 2.0, "depth_scale_m": 0.001}

    backend = FakeInference(model_root="", checkpoint_path="")
    candidates = backend.infer(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detections=[detection],
        camera_info=camera_info,
        max_grasps=1,
    )

    assert candidates == []


def test_graspnet_candidates_are_filtered_by_target_bbox_projection():
    module = _load_wrapper()
    grasp_array = np.asarray(
        [
            [
                0.9,
                0.04,
                0.02,
                0.03,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
                0.0,
                0.0,
                1.0,
                -1,
            ],
            [
                0.8,
                0.04,
                0.02,
                0.03,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
                1.0,
                0.0,
                1.0,
                -1,
            ],
        ],
        dtype=np.float32,
    )
    detection = {"bbox_xyxy": [90.0, 90.0, 110.0, 110.0], "class_name": "bottle"}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 100.0, "cy": 100.0}

    candidates = module.graspnet_array_to_candidates(
        grasp_array,
        class_name="bottle",
        max_grasps=2,
        target_detection=detection,
        camera_info=camera_info,
    )

    assert len(candidates) == 1
    assert candidates[0]["score"] == pytest.approx(0.9)
    assert candidates[0]["target_filter"] == "yolo_projection"


def test_graspnet_candidates_apply_jaw_width_filter_before_top_n_limit():
    module = _load_wrapper()
    base_pose = [
        0.02,
        0.03,
        1,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        1,
        0.0,
        0.0,
        1.0,
        -1,
    ]
    grasp_array = np.asarray(
        [
            [0.95, 0.091, *base_pose],
            [0.90, 0.087, *base_pose],
            [0.82, 0.079, *base_pose],
        ],
        dtype=np.float32,
    )
    detection = {"bbox_xyxy": [90.0, 90.0, 110.0, 110.0], "class_name": "bottle"}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 100.0, "cy": 100.0}

    stage_counts = {}
    candidates = module.graspnet_array_to_candidates(
        grasp_array,
        class_name="bottle",
        max_grasps=1,
        max_jaw_width_m=0.082,
        target_detection=detection,
        camera_info=camera_info,
        stage_counts=stage_counts,
    )

    assert len(candidates) == 1
    assert candidates[0]["score"] == pytest.approx(0.82)
    assert candidates[0]["width_m"] == pytest.approx(0.079)
    assert stage_counts == {
        "after_projection": 3,
        "after_object_geometry": 3,
        "after_jaw_width": 1,
        "published": 1,
    }


def test_training_knn_fallback_matches_legacy_one_based_indices():
    pytest.importorskip("torch")
    module = _load_wrapper()
    module.sys.modules.pop("knn_modules", None)
    module.install_training_knn_fallback()

    import torch

    ref = torch.tensor([[[0.0, 1.0, 3.0]]])
    query = torch.tensor([[[0.1, 2.8]]])
    indices = module.sys.modules["knn_modules"].knn(ref, query, k=1)

    assert indices.tolist() == [[[1, 3]]]
