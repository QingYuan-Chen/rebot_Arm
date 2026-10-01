"""Candidate IK filter configuration snapshot.

The ROS node remains the owner of parameter declaration.  This module only
copies values once so frame processing can be tested without a parameter
server or a live ROS node.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from .utils.parameter_validation import validate_candidate_parameters


def _tuple(node, name: str, size: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in node[name])
    if len(values) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    return values


@dataclass(frozen=True)
class CandidateIkConfig:
    values: Mapping[str, Any]
    input_topic: str
    target_frame: str
    service_timeout_sec: float
    max_candidates_per_frame: int
    candidate_min_confidence: float
    candidate_min_jaw_width_m: float
    candidate_max_jaw_width_m: float
    candidate_workspace_gate_enabled: bool
    candidate_workspace_min_xyz: tuple[float, float, float]
    candidate_workspace_max_xyz: tuple[float, float, float]
    candidate_max_grasp_to_object_center_m: float
    candidate_min_grasp_z_m: float
    pose_policy: str
    fixed_grasp_orientation_xyzw: tuple[float, float, float, float]
    base_approach_axis_xyz: tuple[float, float, float]
    base_pregrasp_distance_m: float
    tcp_offset_xyz: tuple[float, float, float]
    target_base_offset_xyz: tuple[float, float, float]
    candidate_pregrasp_min_z_m: float
    grasp_base_z_offset_m: float
    orientation_yaw_offsets_rad: tuple[float, ...]
    candidate_grasp_z_offsets_m: tuple[float, ...]
    candidate_joint6_symmetry_enabled: bool
    candidate_joint6_symmetry_angle_rad: float
    candidate_score_joint_distance_weight: float
    candidate_score_joint6_weight: float
    candidate_max_joint6_delta_rad: float
    moveit_group_name: str
    ee_frame_id: str
    collision_check_enabled: bool
    collision_group_name: str

    @classmethod
    def from_node(cls, node) -> "CandidateIkConfig":
        try:
            names = node.list_parameters([], depth=10).names
        except AttributeError:
            names = list(_candidate_parameter_names())
        values = MappingProxyType({name: (tuple(value) if isinstance(value, (list, tuple)) else deepcopy(value)) for name, value in ((name, node.get_parameter(name).value) for name in names)})
        validate_candidate_parameters(values)
        value = values.__getitem__
        return cls(
            values=values,
            input_topic=str(value("input_topic")),
            target_frame=str(value("target_frame")),
            service_timeout_sec=float(value("service_timeout_sec")),
            max_candidates_per_frame=max(1, int(value("max_candidates_per_frame"))),
            candidate_min_confidence=float(value("candidate_min_confidence")),
            candidate_min_jaw_width_m=float(value("candidate_min_jaw_width_m")),
            candidate_max_jaw_width_m=float(value("candidate_max_jaw_width_m")),
            candidate_workspace_gate_enabled=bool(value("candidate_workspace_gate_enabled")),
            candidate_workspace_min_xyz=_tuple(values, "candidate_workspace_min_xyz", 3),
            candidate_workspace_max_xyz=_tuple(values, "candidate_workspace_max_xyz", 3),
            candidate_max_grasp_to_object_center_m=float(value("candidate_max_grasp_to_object_center_m")),
            candidate_min_grasp_z_m=float(value("candidate_min_grasp_z_m")),
            pose_policy=str(value("pose_policy")),
            fixed_grasp_orientation_xyzw=_tuple(values, "fixed_grasp_orientation_xyzw", 4),
            base_approach_axis_xyz=_tuple(values, "base_approach_axis_xyz", 3),
            base_pregrasp_distance_m=float(value("base_pregrasp_distance_m")),
            tcp_offset_xyz=_tuple(values, "tcp_offset_xyz", 3),
            target_base_offset_xyz=_tuple(values, "target_base_offset_xyz", 3),
            candidate_pregrasp_min_z_m=float(value("candidate_pregrasp_min_z_m")),
            grasp_base_z_offset_m=float(value("grasp_base_z_offset_m")),
            orientation_yaw_offsets_rad=tuple(float(v) for v in value("orientation_yaw_offsets_rad")),
            candidate_grasp_z_offsets_m=tuple(float(v) for v in value("candidate_grasp_z_offsets_m")),
            candidate_joint6_symmetry_enabled=bool(value("candidate_joint6_symmetry_enabled")),
            candidate_joint6_symmetry_angle_rad=float(value("candidate_joint6_symmetry_angle_rad")),
            candidate_score_joint_distance_weight=float(value("candidate_score_joint_distance_weight")),
            candidate_score_joint6_weight=float(value("candidate_score_joint6_weight")),
            candidate_max_joint6_delta_rad=float(value("candidate_max_joint6_delta_rad")),
            moveit_group_name=str(value("moveit_group_name")),
            ee_frame_id=str(value("ee_frame_id")),
            collision_check_enabled=bool(value("collision_check_enabled")),
            collision_group_name=str(value("collision_group_name")),
        )


def _candidate_parameter_names() -> tuple[str, ...]:
    return (
        "input_topic", "target_frame", "service_timeout_sec", "max_candidates_per_frame",
        "candidate_min_confidence", "candidate_min_jaw_width_m", "candidate_max_jaw_width_m",
        "candidate_workspace_gate_enabled", "candidate_workspace_min_xyz", "candidate_workspace_max_xyz",
        "candidate_max_grasp_to_object_center_m", "candidate_min_grasp_z_m", "pose_policy",
        "fixed_grasp_orientation_xyzw", "base_approach_axis_xyz", "base_pregrasp_distance_m",
        "tcp_offset_xyz", "target_base_offset_xyz", "candidate_pregrasp_min_z_m", "grasp_base_z_offset_m",
        "orientation_yaw_offsets_rad", "candidate_grasp_z_offsets_m", "candidate_joint6_symmetry_enabled",
        "candidate_joint6_symmetry_angle_rad", "candidate_score_joint_distance_weight",
        "candidate_score_joint6_weight", "candidate_max_joint6_delta_rad", "moveit_group_name",
        "ee_frame_id", "collision_check_enabled", "collision_group_name",
    )
