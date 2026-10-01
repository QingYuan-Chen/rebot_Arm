from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_candidate_scoring_policy_preserves_graspnet_order_before_motion_penalty():
    from rebotarm_vision.policies.candidate_scoring_policy import CandidateScoringInput, score_candidate

    first = score_candidate(
        CandidateScoringInput(original_index=0, variant_label="preserve_candidate_pose", motion_penalty=0.2)
    )
    second = score_candidate(
        CandidateScoringInput(original_index=1, variant_label="preserve_candidate_pose", motion_penalty=0.2)
    )

    assert first.score == pytest.approx(-0.2)
    assert second.score == pytest.approx(-1.2)
    assert first.score > second.score


def test_candidate_scoring_policy_prefers_less_motion_within_same_graspnet_rank():
    from rebotarm_vision.policies.candidate_scoring_policy import CandidateScoringInput, score_candidate

    small_motion = score_candidate(
        CandidateScoringInput(original_index=0, variant_label="preserve_candidate_pose", motion_penalty=0.1)
    )
    large_motion = score_candidate(
        CandidateScoringInput(original_index=0, variant_label="preserve_candidate_pose", motion_penalty=0.5)
    )

    assert small_motion.score > large_motion.score


def test_candidate_scoring_policy_keeps_small_z_variant_tie_breaker():
    from rebotarm_vision.policies.candidate_scoring_policy import CandidateScoringInput, score_candidate

    z0 = score_candidate(CandidateScoringInput(original_index=0, variant_label="base_axis_yaw0_z0"))
    z1 = score_candidate(CandidateScoringInput(original_index=0, variant_label="base_axis_yaw0_z1"))

    assert z0.score == pytest.approx(0.0)
    assert z1.score == pytest.approx(-0.001)
    assert z0.score > z1.score
