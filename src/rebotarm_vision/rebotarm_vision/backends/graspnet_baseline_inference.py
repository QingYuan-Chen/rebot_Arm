from __future__ import annotations

from pathlib import Path
import importlib.util
import os
import sys
import types
from typing import Any

import numpy as np


DEFAULT_NUM_POINT = 20000
DEFAULT_SAMPLE_VOXEL_SIZE_M = 0.002
DEFAULT_WORKSPACE_MIN_DEPTH_M = 0.05
DEFAULT_WORKSPACE_MAX_DEPTH_M = 1.5
DEFAULT_FOREGROUND_CENTER_FRACTION = 0.35
DEFAULT_FOREGROUND_NEAR_MARGIN_M = 0.03
DEFAULT_FOREGROUND_FAR_MARGIN_M = 0.08
DEFAULT_FOREGROUND_MIN_SEED_POINTS = 32


def install_training_knn_fallback() -> None:
    """Provide the training-only knn import without building the legacy extension.

    The official model imports label-generation helpers even in eval mode. Those
    helpers import ``knn_modules``, although inference never calls them. A small
    torch implementation keeps that import valid and is also correct if a caller
    explicitly uses the helper on modest tensors.
    """
    if "knn_modules" in sys.modules:
        return
    import torch

    module = types.ModuleType("knn_modules")

    def knn(ref, query, k=1):
        ref_points = ref.transpose(1, 2).contiguous()
        query_points = query.transpose(1, 2).contiguous()
        distances = torch.cdist(query_points.float(), ref_points.float())
        return torch.topk(distances, int(k), dim=-1, largest=False).indices.transpose(1, 2) + 1

    module.knn = knn
    sys.modules["knn_modules"] = module


def load_grasp_group(model_root: str):
    """Load GraspGroup without importing graspnetAPI's evaluation-only stack."""
    configured = os.environ.get("GRASPNET_API_ROOT", "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(Path(model_root).resolve().parent / "graspnetAPI" / "graspnetAPI")
    for api_root in candidates:
        grasp_path = api_root / "grasp.py"
        if not grasp_path.is_file():
            continue
        package = types.ModuleType("graspnetAPI")
        package.__path__ = [str(api_root)]
        package.__package__ = "graspnetAPI"
        sys.modules["graspnetAPI"] = package
        spec = importlib.util.spec_from_file_location("graspnetAPI.grasp", grasp_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load GraspNetAPI from {grasp_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules["graspnetAPI.grasp"] = module
        spec.loader.exec_module(module)
        return module.GraspGroup
    from graspnetAPI import GraspGroup

    return GraspGroup


def _build_cloud(
    *,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    camera_info: dict[str, Any],
    pixel_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    depth = np.asarray(depth_mm)
    color = np.asarray(color_bgr)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    if color.shape[:2] != depth.shape:
        raise ValueError("color_bgr and depth_mm must have the same image size")

    height, width = depth.shape
    z = depth.astype(np.float32) * float(camera_info.get("depth_scale_m", 0.001))
    min_depth_m = float(camera_info.get("workspace_min_depth_m", DEFAULT_WORKSPACE_MIN_DEPTH_M))
    max_depth_m = float(camera_info.get("workspace_max_depth_m", DEFAULT_WORKSPACE_MAX_DEPTH_M))
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if pixel_mask is not None:
        selected = np.asarray(pixel_mask, dtype=bool)
        if selected.shape != depth.shape:
            raise ValueError("pixel_mask must match the depth image shape")
        valid &= selected
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    z_values = z[v, u]
    x = (u.astype(np.float32) - cx) * z_values / fx
    y = (v.astype(np.float32) - cy) * z_values / fy
    points = np.column_stack((x, y, z_values)).astype(np.float32)
    colors = color[v, u, :3].astype(np.float32)[:, ::-1] / 255.0
    return points, colors.astype(np.float32)


def build_scene_cloud(
    *,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    camera_info: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    return _build_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
    )


def build_detection_cloud(
    *,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    camera_info: dict[str, Any],
    detection: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    """Build an object-centric cloud from the selected YOLO detection.

    Pixel coordinates remain in the original image coordinate system, so the
    original camera intrinsics stay valid. Bounding-box maxima follow normal
    image-slice semantics and are exclusive. A polygon mask is preferred when
    one is present in the detection payload.
    """
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    height, width = depth.shape
    detection_mask = _detection_pixel_mask((height, width), detection)
    pixel_mask = _foreground_depth_mask(
        depth_mm=depth,
        detection_mask=detection_mask,
        detection=detection,
        camera_info=camera_info,
    )
    return _build_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
        pixel_mask=pixel_mask,
    )


def _detection_pixel_mask(
    image_shape: tuple[int, int],
    detection: dict[str, Any],
) -> np.ndarray:
    height, width = image_shape
    polygon = _detection_polygon(detection)
    if polygon is not None:
        v, u = np.indices((height, width), dtype=np.float32)
        return _points_in_polygon(u.reshape(-1), v.reshape(-1), polygon).reshape(
            height,
            width,
        )
    left, top, right, bottom = _clamped_detection_bbox(
        image_shape,
        detection,
    )
    pixel_mask = np.zeros((height, width), dtype=bool)
    if right > left and bottom > top:
        pixel_mask[top:bottom, left:right] = True
    return pixel_mask


def _foreground_depth_mask(
    *,
    depth_mm: np.ndarray,
    detection_mask: np.ndarray,
    detection: dict[str, Any],
    camera_info: dict[str, Any],
) -> np.ndarray:
    depth = np.asarray(depth_mm)
    selected = np.asarray(detection_mask, dtype=bool)
    if selected.shape != depth.shape:
        raise ValueError("detection_mask must match the depth image shape")

    depth_scale_m = float(camera_info.get("depth_scale_m", 0.001))
    z = depth.astype(np.float32) * depth_scale_m
    min_depth_m = float(camera_info.get("workspace_min_depth_m", DEFAULT_WORKSPACE_MIN_DEPTH_M))
    max_depth_m = float(camera_info.get("workspace_max_depth_m", DEFAULT_WORKSPACE_MAX_DEPTH_M))
    valid = selected & np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if not np.any(valid):
        return np.zeros(depth.shape, dtype=bool)

    center_fraction = float(
        camera_info.get("foreground_center_fraction", DEFAULT_FOREGROUND_CENTER_FRACTION)
    )
    near_margin_m = float(
        camera_info.get("foreground_near_margin_m", DEFAULT_FOREGROUND_NEAR_MARGIN_M)
    )
    far_margin_m = float(
        camera_info.get("foreground_far_margin_m", DEFAULT_FOREGROUND_FAR_MARGIN_M)
    )
    min_seed_points = int(
        camera_info.get("foreground_min_seed_points", DEFAULT_FOREGROUND_MIN_SEED_POINTS)
    )
    if not 0.0 < center_fraction <= 1.0:
        raise ValueError("foreground_center_fraction must be in (0, 1]")
    if near_margin_m < 0.0 or far_margin_m <= 0.0:
        raise ValueError("foreground depth margins must be non-negative with a positive far margin")
    if min_seed_points <= 0:
        raise ValueError("foreground_min_seed_points must be positive")

    left, top, right, bottom = _clamped_detection_bbox(depth.shape, detection)
    if right <= left or bottom <= top:
        return np.zeros(depth.shape, dtype=bool)
    center_width = max(1, int(round((right - left) * center_fraction)))
    center_height = max(1, int(round((bottom - top) * center_fraction)))
    center_u = (left + right) // 2
    center_v = (top + bottom) // 2
    center_left = max(left, center_u - center_width // 2)
    center_top = max(top, center_v - center_height // 2)
    center_right = min(right, center_left + center_width)
    center_bottom = min(bottom, center_top + center_height)
    seed_mask = np.zeros(depth.shape, dtype=bool)
    seed_mask[center_top:center_bottom, center_left:center_right] = True
    seed_values = z[valid & seed_mask]
    if seed_values.size < min_seed_points:
        return np.zeros(depth.shape, dtype=bool)

    foreground_depth_m = float(np.median(seed_values))
    return valid & (z >= foreground_depth_m - near_margin_m) & (
        z <= foreground_depth_m + far_margin_m
    )


def _clamped_detection_bbox(
    image_shape: tuple[int, int],
    detection: dict[str, Any],
) -> tuple[int, int, int, int]:
    height, width = image_shape
    x_min, y_min, x_max, y_max = _detection_bbox(detection)
    left = max(0, min(width, int(np.floor(x_min))))
    top = max(0, min(height, int(np.floor(y_min))))
    right = max(0, min(width, int(np.ceil(x_max))))
    bottom = max(0, min(height, int(np.ceil(y_max))))
    return left, top, right, bottom


def _splitmix64(values: np.ndarray) -> np.ndarray:
    """Return a stable pseudo-random hash for unsigned integer values."""
    with np.errstate(over="ignore"):
        hashed = np.asarray(values, dtype=np.uint64) + np.uint64(0x9E3779B97F4A7C15)
        hashed = (hashed ^ (hashed >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
        hashed = (hashed ^ (hashed >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
        return hashed ^ (hashed >> np.uint64(31))


def sample_cloud(
    points: np.ndarray,
    colors: np.ndarray,
    *,
    num_point: int,
    voxel_size_m: float = DEFAULT_SAMPLE_VOXEL_SIZE_M,
) -> tuple[np.ndarray, np.ndarray]:
    """Select a deterministic, spatially covered GraspNet input cloud.

    Points are grouped into metric voxels and selected one layer at a time so
    sparse surface regions are represented before dense voxels contribute more
    points.  The selected set is then placed in a stable hash order.  Keeping
    that final order spatially decorrelated matters because GraspNet's internal
    point sampling is sensitive to an axis-sorted input order.
    """
    point_array = np.asarray(points)
    color_array = np.asarray(colors)
    if len(point_array) == 0:
        return point_array.astype(np.float32), color_array.astype(np.float32)
    if len(color_array) != len(point_array):
        raise ValueError("points and colors must contain the same number of rows")

    count = int(num_point)
    if count <= 0:
        raise ValueError("num_point must be positive")
    voxel_size = float(voxel_size_m)
    if not np.isfinite(voxel_size) or voxel_size <= 0.0:
        raise ValueError("voxel_size_m must be finite and positive")

    voxel = np.floor(point_array.astype(np.float64) / voxel_size).astype(np.int64)
    original_indices = np.arange(len(point_array), dtype=np.int64)
    voxel_order = np.lexsort(
        (original_indices, voxel[:, 2], voxel[:, 1], voxel[:, 0])
    )
    sorted_voxel = voxel[voxel_order]
    group_start = np.empty(len(point_array), dtype=bool)
    group_start[0] = True
    group_start[1:] = np.any(sorted_voxel[1:] != sorted_voxel[:-1], axis=1)
    group_starts = np.flatnonzero(group_start)
    group_ids = np.cumsum(group_start, dtype=np.int64) - 1
    ranks_within_group = (
        np.arange(len(point_array), dtype=np.int64) - group_starts[group_ids]
    )
    layered_order = np.lexsort((group_ids, ranks_within_group))
    selected = voxel_order[layered_order[: min(count, len(point_array))]]
    if len(selected) < count:
        selected = np.resize(selected, count)

    stable_keys = _splitmix64(selected.astype(np.uint64))
    selected = selected[np.argsort(stable_keys, kind="stable")]
    return point_array[selected].astype(np.float32), color_array[selected].astype(np.float32)


def graspnet_array_to_candidates(
    grasp_array,
    *,
    class_name: str,
    max_grasps: int,
    max_jaw_width_m: float | None = None,
    target_detection: dict[str, Any] | None = None,
    camera_info: dict[str, Any] | None = None,
    stage_counts: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    array = np.asarray(grasp_array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if target_detection is not None and camera_info is not None:
        array = filter_grasp_array_by_detection_projection(array, target_detection=target_detection, camera_info=camera_info)
    if stage_counts is not None:
        stage_counts["after_projection"] = int(len(array))
    if max_jaw_width_m is not None:
        width_limit = float(max_jaw_width_m)
        if not np.isfinite(width_limit) or width_limit <= 0.0:
            raise ValueError("max_jaw_width_m must be finite and positive")
        if array.ndim != 2 or array.shape[1] < 2:
            array = array[:0]
        else:
            widths = array[:, 1]
            array = array[
                np.isfinite(widths)
                & (widths > 0.0)
                & (widths <= width_limit)
            ]
    if stage_counts is not None:
        stage_counts["after_jaw_width"] = int(len(array))
    candidates: list[dict[str, Any]] = []
    for row in array[: max(0, int(max_grasps))]:
        if row.size < 16:
            continue
        rotation = row[4:13].reshape(3, 3)
        translation = row[13:16]
        candidates.append(
            {
                "source": "graspnet_baseline",
                "class_name": str(class_name),
                "score": float(row[0]),
                "width_m": float(row[1]),
                "height_m": float(row[2]),
                "depth_m": float(row[3]),
                "rotation_matrix": rotation.astype(float).tolist(),
                "translation_xyz": translation.astype(float).tolist(),
                "object_length_m": float(row[2]),
                "target_filter": "yolo_projection" if target_detection is not None and camera_info is not None else "",
            }
        )
    if stage_counts is not None:
        stage_counts["published"] = int(len(candidates))
    return candidates


def filter_grasp_array_by_detection_projection(
    grasp_array,
    *,
    target_detection: dict[str, Any],
    camera_info: dict[str, Any],
) -> np.ndarray:
    array = np.asarray(grasp_array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.size == 0:
        return array
    translations = array[:, 13:16]
    u, v = project_points_to_image(translations, camera_info=camera_info)
    keep = detection_contains_pixels(target_detection, u=u, v=v)
    return array[keep]


def project_points_to_image(points_xyz: np.ndarray, *, camera_info: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    z = points[:, 2]
    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    valid_z = np.where(np.abs(z) > 1e-6, z, np.nan)
    u = fx * points[:, 0] / valid_z + cx
    v = fy * points[:, 1] / valid_z + cy
    return u, v


def detection_contains_pixels(target_detection: dict[str, Any], *, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    u_values = np.asarray(u, dtype=np.float32).reshape(-1)
    v_values = np.asarray(v, dtype=np.float32).reshape(-1)
    finite = np.isfinite(u_values) & np.isfinite(v_values)
    polygon = _detection_polygon(target_detection)
    if polygon is not None:
        return finite & _points_in_polygon(u_values, v_values, polygon)
    x_min, y_min, x_max, y_max = _detection_bbox(target_detection)
    return finite & (u_values >= x_min) & (u_values <= x_max) & (v_values >= y_min) & (v_values <= y_max)


def _detection_polygon(detection: dict[str, Any]) -> np.ndarray | None:
    raw = None
    mask = detection.get("mask")
    if isinstance(mask, dict):
        raw = mask.get("polygon_xy")
    if raw is None:
        raw = detection.get("mask_polygon_xy")
    if raw is None:
        return None
    arr = np.asarray(raw, dtype=np.float32).reshape(-1, 2)
    if arr.shape[0] < 3:
        return None
    return arr


def _detection_bbox(detection: dict[str, Any]) -> tuple[float, float, float, float]:
    if "bbox_xyxy" in detection:
        raw = np.asarray(detection.get("bbox_xyxy"), dtype=np.float32).reshape(-1)
        if raw.size >= 4:
            return float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3])
    return (
        float(detection.get("x_min", 0.0)),
        float(detection.get("y_min", 0.0)),
        float(detection.get("x_max", 0.0)),
        float(detection.get("y_max", 0.0)),
    )


def _points_in_polygon(u: np.ndarray, v: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    x = np.asarray(u, dtype=np.float32)
    y = np.asarray(v, dtype=np.float32)
    px = polygon[:, 0]
    py = polygon[:, 1]
    inside = np.zeros(x.shape, dtype=bool)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        crosses = ((py[i] > y) != (py[j] > y)) & (
            x < (px[j] - px[i]) * (y - py[i]) / ((py[j] - py[i]) + 1e-12) + px[i]
        )
        inside ^= crosses
        j = i
    return inside


class GraspNetBaselineInference:
    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str = "cuda:0",
        num_point: int = DEFAULT_NUM_POINT,
        collision_thresh: float = 0.01,
        voxel_size: float = 0.01,
    ) -> None:
        self.model_root = str(model_root)
        self.checkpoint_path = str(checkpoint_path)
        self.device = str(device)
        self.num_point = int(num_point)
        self.collision_thresh = float(collision_thresh)
        self.voxel_size = float(voxel_size)
        self._torch = None
        self._GraspGroup = None
        self._ModelFreeCollisionDetector = None
        self._pred_decode = None
        self.last_stage_counts: dict[str, Any] = self._empty_stage_counts()
        self.net = self._load_network()

    @staticmethod
    def _empty_stage_counts() -> dict[str, Any]:
        return {
            "scene_points": 0,
            "object_points": 0,
            "raw": 0,
            "after_collision": 0,
            "after_nms": 0,
            "after_score_sort": 0,
            "after_projection": 0,
            "after_jaw_width": 0,
            "published": 0,
            "empty_reason": "",
        }

    @staticmethod
    def _empty_reason(stage_counts: dict[str, Any]) -> str:
        ordered_stages = (
            ("raw", "raw_candidates_empty"),
            ("after_collision", "collision_filter_empty"),
            ("after_nms", "nms_empty"),
            ("after_projection", "projection_filter_empty"),
            ("after_jaw_width", "jaw_width_filter_empty"),
            ("published", "publish_conversion_empty"),
        )
        for key, reason in ordered_stages:
            if int(stage_counts.get(key, 0)) == 0:
                return reason
        return ""

    def _load_network(self):
        root = Path(self.model_root)
        for child in ("models", "dataset", "utils"):
            path = str(root / child)
            if path not in sys.path:
                sys.path.insert(0, path)
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        import torch
        install_training_knn_fallback()
        from graspnet import GraspNet, pred_decode
        GraspGroup = load_grasp_group(self.model_root)

        self._torch = torch
        self._pred_decode = pred_decode
        self._GraspGroup = GraspGroup
        try:
            from collision_detector import ModelFreeCollisionDetector

            self._ModelFreeCollisionDetector = ModelFreeCollisionDetector
        except Exception:
            self._ModelFreeCollisionDetector = None

        net = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        net.load_state_dict(state_dict)
        net.to(self.device)
        net.eval()
        return net

    def infer(
        self,
        *,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        detections: list[dict[str, Any]],
        camera_info: dict[str, Any],
        max_grasps: int,
        max_jaw_width_m: float | None = None,
    ) -> list[dict[str, Any]]:
        self.last_stage_counts = self._empty_stage_counts()
        stage_counts = self.last_stage_counts
        if not detections:
            stage_counts["empty_reason"] = "no_detections"
            return []
        detection = max(detections, key=lambda item: float(item.get("confidence", 0.0)))
        scene_points, _scene_colors = build_scene_cloud(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            camera_info=camera_info,
        )
        stage_counts["scene_points"] = int(len(scene_points))
        if len(scene_points) == 0:
            stage_counts["empty_reason"] = "empty_scene_cloud"
            return []
        object_points, object_colors = build_detection_cloud(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            camera_info=camera_info,
            detection=detection,
        )
        stage_counts["object_points"] = int(len(object_points))
        if len(object_points) == 0:
            stage_counts["empty_reason"] = "empty_object_cloud"
            return []
        points_sampled, colors_sampled = sample_cloud(
            object_points,
            object_colors,
            num_point=self.num_point,
        )
        grasp_array = self._infer_grasp_array(
            points_sampled,
            colors_sampled,
            full_points=scene_points,
        )
        inferred_array = np.asarray(grasp_array)
        inferred_count = int(
            1 if inferred_array.ndim == 1 and inferred_array.size else len(inferred_array)
        )
        if int(stage_counts["after_score_sort"]) == 0 and inferred_count > 0:
            stage_counts["raw"] = inferred_count
            stage_counts["after_collision"] = inferred_count
            stage_counts["after_nms"] = inferred_count
            stage_counts["after_score_sort"] = inferred_count
        candidates = graspnet_array_to_candidates(
            grasp_array,
            class_name=str(detection.get("class_name", "")),
            max_grasps=max_grasps,
            max_jaw_width_m=max_jaw_width_m,
            target_detection=detection,
            camera_info=camera_info,
            stage_counts=stage_counts,
        )
        if not candidates:
            stage_counts["empty_reason"] = self._empty_reason(stage_counts)
        return candidates

    def _infer_grasp_array(self, points: np.ndarray, colors: np.ndarray, *, full_points: np.ndarray):
        torch = self._torch
        if torch is None or self._pred_decode is None or self._GraspGroup is None:
            raise RuntimeError("GraspNet backend is not loaded")
        cloud = torch.from_numpy(points[np.newaxis].astype(np.float32)).to(self.device)
        color = torch.from_numpy(colors[np.newaxis].astype(np.float32)).to(self.device)
        end_points = {"point_clouds": cloud, "cloud_colors": color}
        with torch.no_grad():
            end_points = self.net(end_points)
            decoded = self._pred_decode(end_points)
        grasp_group = self._GraspGroup(decoded[0].detach().cpu().numpy())
        self.last_stage_counts["raw"] = int(len(grasp_group))
        if self._ModelFreeCollisionDetector is not None and self.collision_thresh > 0:
            detector = self._ModelFreeCollisionDetector(full_points, voxel_size=self.voxel_size)
            collision_mask = detector.detect(grasp_group, approach_dist=0.05, collision_thresh=self.collision_thresh)
            grasp_group = grasp_group[~collision_mask]
        self.last_stage_counts["after_collision"] = int(len(grasp_group))
        try:
            grasp_group = grasp_group.nms()
        except ModuleNotFoundError:
            pass
        self.last_stage_counts["after_nms"] = int(len(grasp_group))
        grasp_group = grasp_group.sort_by_score()
        self.last_stage_counts["after_score_sort"] = int(len(grasp_group))
        return getattr(grasp_group, "grasp_group_array", np.asarray(grasp_group))
