from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_base_axis_policy_uses_visual_position_but_fixed_reachable_orientation():
    from rebotarm_vision.policies.visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets

    config = BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_axis_xyz=(1.0, 0.0, 0.0),
        pregrasp_distance_m=0.08,
        tcp_offset_xyz=(-0.04, 0.0, 0.0),
        target_base_offset_xyz=(0.0, 0.01, 0.0),
        grasp_z_offset_m=0.0,
    )

    pregrasp, grasp = build_base_axis_grasp_targets(
        grasp_position_xyz=(0.44, -0.027, 0.229),
        config=config,
    )

    assert grasp.position == pytest.approx((0.48, -0.017, 0.229))
    assert grasp.orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert pregrasp.position == pytest.approx((0.40, -0.017, 0.229))
    assert pregrasp.orientation == grasp.orientation


def test_base_axis_policy_normalizes_approach_axis():
    from rebotarm_vision.policies.visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets

    config = BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_axis_xyz=(2.0, 0.0, 0.0),
        pregrasp_distance_m=0.10,
        tcp_offset_xyz=(0.0, 0.0, 0.0),
        target_base_offset_xyz=(0.0, 0.0, 0.0),
        grasp_z_offset_m=0.0,
    )

    pregrasp, grasp = build_base_axis_grasp_targets(
        grasp_position_xyz=(0.5, 0.0, 0.2),
        config=config,
    )

    assert grasp.position == pytest.approx((0.5, 0.0, 0.2))
    assert pregrasp.position == pytest.approx((0.4, 0.0, 0.2))


def test_base_axis_policy_rejects_zero_approach_axis():
    from rebotarm_vision.policies.visual_grasp_pose_policy import BaseAxisGraspPolicyConfig, build_base_axis_grasp_targets

    config = BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_axis_xyz=(0.0, 0.0, 0.0),
        pregrasp_distance_m=0.08,
    )

    with pytest.raises(ValueError, match="approach_axis_xyz"):
        build_base_axis_grasp_targets(grasp_position_xyz=(0.4, 0.0, 0.2), config=config)


def test_official_geometry_policy_uses_candidate_tcp_orientation_for_pregrasp_axis():
    from rebotarm_vision.policies.visual_grasp_pose_policy import (
        OfficialGeometryGraspPolicyConfig,
        build_official_geometry_grasp_targets,
    )

    config = OfficialGeometryGraspPolicyConfig(
        pregrasp_distance_m=0.08,
        tcp_offset_xyz=(0.0, 0.0, 0.0),
        target_base_offset_xyz=(0.0, 0.0, 0.0),
        grasp_z_offset_m=0.0,
    )

    pregrasp, grasp = build_official_geometry_grasp_targets(
        grasp_position_xyz=(0.44, -0.02, 0.18),
        grasp_orientation_xyzw=(0.0, 0.0, 0.70710678, 0.70710678),
        config=config,
    )

    assert grasp.position == pytest.approx((0.44, -0.02, 0.18))
    assert grasp.orientation == pytest.approx((0.0, 0.0, 0.70710678, 0.70710678))
    assert pregrasp.position == pytest.approx((0.44, -0.10, 0.18))
    assert pregrasp.orientation == grasp.orientation


def test_hybrid_geometry_policy_uses_base_axis_pregrasp_with_candidate_yaw():
    from rebotarm_vision.policies.visual_grasp_pose_policy import (
        BaseAxisGraspPolicyConfig,
        build_hybrid_geometry_grasp_targets,
    )

    config = BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        approach_axis_xyz=(1.0, 0.0, 0.0),
        pregrasp_distance_m=0.08,
        tcp_offset_xyz=(0.0, 0.0, 0.0),
        target_base_offset_xyz=(0.0, 0.0, 0.0),
        grasp_z_offset_m=0.0,
    )

    pregrasp, grasp = build_hybrid_geometry_grasp_targets(
        grasp_position_xyz=(0.44, -0.02, 0.18),
        candidate_orientation_xyzw=(0.0, 0.0, 0.70710678, 0.70710678),
        config=config,
    )

    assert grasp.position == pytest.approx((0.44, -0.02, 0.18))
    assert pregrasp.position == pytest.approx((0.36, -0.02, 0.18))
    assert grasp.orientation == pytest.approx((0.0, 0.0, 0.70710678, 0.70710678))
    assert pregrasp.orientation == grasp.orientation


def test_preserve_candidate_pose_keeps_graspnet_tcp_orientation_and_uses_end_link_center_when_tcp_offset_zero():
    from rebotarm_vision.policies.visual_grasp_pose_policy import (
        OfficialGeometryGraspPolicyConfig,
        build_preserve_candidate_grasp_targets,
    )

    config = OfficialGeometryGraspPolicyConfig(
        pregrasp_distance_m=0.08,
        tcp_offset_xyz=(0.0, 0.0, 0.0),
        target_base_offset_xyz=(0.0, 0.0, 0.0),
        grasp_z_offset_m=0.0,
    )

    pregrasp, grasp = build_preserve_candidate_grasp_targets(
        grasp_position_xyz=(0.42, 0.03, 0.055),
        grasp_orientation_xyzw=(0.0, 0.70710678, 0.0, 0.70710678),
        config=config,
    )

    assert grasp.position == pytest.approx((0.42, 0.03, 0.055))
    assert grasp.orientation == pytest.approx((0.0, 0.70710678, 0.0, 0.70710678))
    assert pregrasp.orientation == grasp.orientation
    assert pregrasp.position != pytest.approx((0.42, 0.03, 0.055))


def test_parallel_jaw_symmetric_orientation_preserves_approach_axis_and_flips_open_axis():
    from rebotarm_vision.policies.visual_grasp_pose_policy import (
        build_parallel_jaw_symmetric_orientation,
        quaternion_to_rotation_matrix,
    )

    original = (0.0, 0.0, 0.0, 1.0)
    symmetric = build_parallel_jaw_symmetric_orientation(original)
    original_rotation = quaternion_to_rotation_matrix(original)
    symmetric_rotation = quaternion_to_rotation_matrix(symmetric)

    assert symmetric == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert [row[0] for row in symmetric_rotation] == pytest.approx([row[0] for row in original_rotation])
    assert [row[1] for row in symmetric_rotation] == pytest.approx([-row[1] for row in original_rotation])
    assert [row[2] for row in symmetric_rotation] == pytest.approx([-row[2] for row in original_rotation])


def test_candidate_workspace_gate_accepts_only_base_link_workspace_box_and_object_distance():
    from rebotarm_vision.policies.candidate_workspace_gate import CandidateWorkspaceGateConfig, candidate_workspace_gate

    config = CandidateWorkspaceGateConfig(
        enabled=True,
        min_xyz=(0.18, -0.35, 0.0),
        max_xyz=(0.64, 0.35, 0.45),
        max_grasp_to_object_center_m=0.15,
    )

    assert candidate_workspace_gate(
        grasp_position_xyz=(0.42, 0.03, 0.055),
        object_center_xyz=(0.40, 0.02, 0.050),
        config=config,
    ).accepted
    assert not candidate_workspace_gate(
        grasp_position_xyz=(2.7, -1.4, 0.37),
        object_center_xyz=(0.40, 0.02, 0.050),
        config=config,
    ).accepted
    assert not candidate_workspace_gate(
        grasp_position_xyz=(0.42, 0.03, 0.055),
        object_center_xyz=(0.10, 0.30, 0.050),
        config=config,
    ).accepted
