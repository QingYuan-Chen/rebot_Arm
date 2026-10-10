from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_candidate_target_policy_preserve_pose_adds_parallel_jaw_symmetric_variant():
    from rebotarm_vision.policies.candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants

    variants = build_candidate_target_variants(
        grasp_position_xyz=(0.42, 0.03, 0.055),
        candidate_orientation_xyzw=(0.0, 0.70710678, 0.0, 0.70710678),
        config=CandidateTargetPolicyConfig(
            pose_policy="preserve_candidate_pose",
            base_pregrasp_distance_m=0.08,
            tcp_offset_xyz=(0.0, 0.0, 0.0),
            target_base_offset_xyz=(0.0, 0.0, 0.0),
            grasp_base_z_offset_m=0.0,
        ),
    )

    assert [variant.label for variant in variants] == [
        "preserve_candidate_pose",
        "preserve_candidate_pose_parallel_jaw_symmetric",
    ]
    assert variants[0].grasp.position == pytest.approx((0.42, 0.03, 0.055))
    assert variants[0].pregrasp.orientation == variants[0].grasp.orientation


def test_candidate_target_policy_rejects_retired_pose_variants():
    from rebotarm_vision.policies.candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants
    with pytest.raises(ValueError, match="only supports preserve_candidate_pose"):
        build_candidate_target_variants(
            grasp_position_xyz=(0.44, -0.027, 0.229),
            candidate_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            config=CandidateTargetPolicyConfig(pose_policy="base_axis"),
        )


def test_candidate_target_policy_uses_pose_agnostic_pregrasp_minimum_height_only_when_needed():
    from rebotarm_vision.policies.candidate_target_policy import CandidateTargetPolicyConfig, build_candidate_target_variants
    from rebotarm_vision.policies.pose_variant_policy import PoseVariantConfig

    low_variants = build_candidate_target_variants(
        grasp_position_xyz=(0.31, -0.01, 0.039),
        candidate_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        config=CandidateTargetPolicyConfig(
            pose_policy="preserve_candidate_pose",
            base_approach_axis_xyz=(1.0, 0.0, 0.0),
            base_pregrasp_distance_m=0.06,
            pregrasp_min_z_m=0.12,
            tcp_offset_xyz=(0.0, 0.0, 0.0),
            target_base_offset_xyz=(0.0, 0.0, 0.0),
            pose_variant_config=PoseVariantConfig(joint6_symmetry_enabled=False),
        ),
    )
    high_variants = build_candidate_target_variants(
        grasp_position_xyz=(0.31, -0.01, 0.20),
        candidate_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        config=CandidateTargetPolicyConfig(
            pose_policy="preserve_candidate_pose",
            base_approach_axis_xyz=(1.0, 0.0, 0.0),
            base_pregrasp_distance_m=0.06,
            pregrasp_min_z_m=0.12,
            tcp_offset_xyz=(0.0, 0.0, 0.0),
            target_base_offset_xyz=(0.0, 0.0, 0.0),
            pose_variant_config=PoseVariantConfig(joint6_symmetry_enabled=False),
        ),
    )

    assert low_variants[0].grasp.position == pytest.approx((0.31, -0.01, 0.039))
    assert low_variants[0].pregrasp.position == pytest.approx((0.25, -0.01, 0.12))
    assert high_variants[0].grasp.position == pytest.approx((0.31, -0.01, 0.20))
    assert high_variants[0].pregrasp.position == pytest.approx((0.25, -0.01, 0.20))
