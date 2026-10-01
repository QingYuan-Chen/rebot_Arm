from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_parallel_jaw_pose_variants_adds_180_degree_symmetric_orientation():
    from rebotarm_vision.policies.pose_variant_policy import (
        PoseVariantConfig,
        build_parallel_jaw_pose_variants,
        quaternion_to_rotation_matrix,
    )

    variants = build_parallel_jaw_pose_variants(
        base_label="preserve_candidate_pose",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        config=PoseVariantConfig(joint6_symmetry_enabled=True),
    )

    assert [label for label, _orientation in variants] == [
        "preserve_candidate_pose",
        "preserve_candidate_pose_parallel_jaw_symmetric",
    ]
    original_rotation = quaternion_to_rotation_matrix(variants[0][1])
    symmetric_rotation = quaternion_to_rotation_matrix(variants[1][1])
    assert [row[0] for row in symmetric_rotation] == pytest.approx([row[0] for row in original_rotation])
    assert [row[1] for row in symmetric_rotation] == pytest.approx([-row[1] for row in original_rotation])


def test_parallel_jaw_pose_variants_can_disable_symmetric_orientation():
    from rebotarm_vision.policies.pose_variant_policy import PoseVariantConfig, build_parallel_jaw_pose_variants

    variants = build_parallel_jaw_pose_variants(
        base_label="preserve_candidate_pose",
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        config=PoseVariantConfig(joint6_symmetry_enabled=False),
    )

    assert [label for label, _orientation in variants] == ["preserve_candidate_pose"]
