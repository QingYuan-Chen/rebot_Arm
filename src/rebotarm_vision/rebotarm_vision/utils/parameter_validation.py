"""视觉节点参数快照的公共校验逻辑."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence


def _finite(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _range(values: Mapping[str, object], name: str, *, minimum: float | None = None, maximum: float | None = None) -> None:
    if name not in values:
        return
    value = _finite(values[name], name)
    if minimum is not None and value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    if maximum is not None and value > maximum:
        raise ValueError(f"{name} must be <= {maximum}")


def _vector(values: Mapping[str, object], name: str, size: int) -> None:
    if name not in values:
        return
    value = values[name]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    for index, item in enumerate(value):
        _finite(item, f"{name}[{index}]")


def validate_candidate_parameters(values: Mapping[str, object]) -> None:
    validate_parameter_geometry(values)
    _range(values, "service_timeout_sec", minimum=0.0)
    for name in ("base_pregrasp_distance_m", "candidate_score_joint_distance_weight", "candidate_score_joint6_weight", "candidate_max_grasp_to_object_center_m"):
        _range(values, name, minimum=0.0)
    _range(values, "candidate_min_confidence", minimum=0.0, maximum=1.0)
    _range(values, "candidate_min_jaw_width_m", minimum=0.0)
    _range(values, "candidate_max_jaw_width_m", minimum=0.0)
    _range(values, "candidate_max_joint6_delta_rad", minimum=0.0)
    if "candidate_min_jaw_width_m" in values and "candidate_max_jaw_width_m" in values and float(values["candidate_min_jaw_width_m"]) > float(values["candidate_max_jaw_width_m"]):
        raise ValueError("candidate_min_jaw_width_m must be <= candidate_max_jaw_width_m")
    if "max_candidates_per_frame" in values and int(values["max_candidates_per_frame"]) < 1:
        raise ValueError("max_candidates_per_frame must be >= 1")
    for name, size in (
        ("candidate_workspace_min_xyz", 3), ("candidate_workspace_max_xyz", 3),
        ("fixed_grasp_orientation_xyzw", 4), ("base_approach_axis_xyz", 3),
        ("tcp_offset_xyz", 3), ("target_base_offset_xyz", 3),
    ):
        _vector(values, name, size)
    if "candidate_workspace_min_xyz" in values and "candidate_workspace_max_xyz" in values:
        if any(float(lo) > float(hi) for lo, hi in zip(values["candidate_workspace_min_xyz"], values["candidate_workspace_max_xyz"])):
            raise ValueError("candidate_workspace_min_xyz must be <= candidate_workspace_max_xyz")


def validate_parameter_geometry(values: Mapping[str, object]) -> None:
    """公共几何数值校验:要求数值有限,且方向向量和四元数非零."""
    for name in ("fixed_grasp_orientation_xyzw", "place_orientation_xyzw", "base_approach_axis_xyz"):
        if name in values:
            size = 4 if name.endswith("xyzw") else 3
            _vector(values, name, size)
            if sum(float(v) ** 2 for v in values[name]) <= 1e-12:
                raise ValueError(f"{name} must be nonzero")
    for name, value in values.items():
        if name in ("orientation_yaw_offsets_rad", "candidate_grasp_z_offsets_m"):
            if not value:
                raise ValueError(f"{name} must not be empty")
            for index, element in enumerate(value):
                _finite(element, f"{name}[{index}]")
        elif isinstance(value, (float, int)) and not isinstance(value, bool):
            _finite(value, name)
