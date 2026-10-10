from __future__ import annotations

from pathlib import Path
import sys


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_candidate_gate_policy_rejects_jaw_width_outside_gripper_range():
    from rebotarm_vision.policies.candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate

    too_small = evaluate_candidate_gate(
        jaw_width_m=0.004,
        grasp_position_xyz=(0.4, 0.0, 0.12),
        object_center_xyz=None,
        config=CandidateGateConfig(min_jaw_width_m=0.006, max_jaw_width_m=0.082),
    )
    too_large = evaluate_candidate_gate(
        jaw_width_m=0.095,
        grasp_position_xyz=(0.4, 0.0, 0.12),
        object_center_xyz=None,
        config=CandidateGateConfig(min_jaw_width_m=0.006, max_jaw_width_m=0.082),
    )

    assert not too_small.accepted
    assert too_small.reason == "jaw_width too small (0.004m)"
    assert not too_large.accepted
    assert too_large.reason == "jaw_width too large (0.095m)"


def test_candidate_gate_default_accepts_85mm_and_rejects_anything_wider():
    from rebotarm_vision.policies.candidate_gate_policy import evaluate_candidate_gate

    accepted = evaluate_candidate_gate(
        jaw_width_m=0.085,
        grasp_position_xyz=(0.4, 0.0, 0.12),
        object_center_xyz=None,
    )
    rejected = evaluate_candidate_gate(
        jaw_width_m=0.085001,
        grasp_position_xyz=(0.4, 0.0, 0.12),
        object_center_xyz=None,
    )

    assert accepted.accepted
    assert not rejected.accepted
    assert rejected.reason == "jaw_width too large (0.085m)"


def test_candidate_gate_policy_rejects_grasp_below_minimum_without_blocking_low_table_grasp_by_default():
    from rebotarm_vision.policies.candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate

    default_low_grasp = evaluate_candidate_gate(
        jaw_width_m=0.04,
        grasp_position_xyz=(0.4, 0.0, 0.039),
        object_center_xyz=None,
        config=CandidateGateConfig(min_grasp_z_m=0.0),
    )
    rejected_low_grasp = evaluate_candidate_gate(
        jaw_width_m=0.04,
        grasp_position_xyz=(0.4, 0.0, 0.039),
        object_center_xyz=None,
        config=CandidateGateConfig(min_grasp_z_m=0.05),
    )

    assert default_low_grasp.accepted
    assert not rejected_low_grasp.accepted
    assert rejected_low_grasp.reason == "grasp z too low (0.039m)"


def test_candidate_gate_policy_rejects_workspace_outliers_and_object_center_outliers():
    from rebotarm_vision.policies.candidate_gate_policy import CandidateGateConfig, evaluate_candidate_gate

    outside_workspace = evaluate_candidate_gate(
        jaw_width_m=0.04,
        grasp_position_xyz=(2.7, -1.4, 0.20),
        object_center_xyz=(0.4, 0.0, 0.12),
        config=CandidateGateConfig(workspace_gate_enabled=True),
    )
    far_from_object = evaluate_candidate_gate(
        jaw_width_m=0.04,
        grasp_position_xyz=(0.4, 0.0, 0.12),
        object_center_xyz=(0.1, 0.0, 0.12),
        config=CandidateGateConfig(workspace_gate_enabled=True, max_grasp_to_object_center_m=0.15),
    )

    assert not outside_workspace.accepted
    assert "outside workspace" in outside_workspace.reason
    assert not far_from_object.accepted
    assert far_from_object.reason == "grasp too far from object center (0.300m)"
