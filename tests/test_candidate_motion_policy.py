from __future__ import annotations

from pathlib import Path
import math
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_joint_motion_rejects_actual_joint6_delta_over_limit():
    from rebotarm_vision.policies.candidate_motion_policy import JointMotionPolicyConfig, evaluate_joint_motion

    result = evaluate_joint_motion(
        current_positions={"joint1": 0.0, "joint6": 0.0},
        target_positions={"joint1": 0.0, "joint6": math.radians(100.0)},
        config=JointMotionPolicyConfig(max_joint6_delta_rad=math.radians(90.0)),
    )

    assert not result.accepted
    assert result.joint6_delta == pytest.approx(math.radians(100.0))
    assert "joint6_delta=1.745" in result.reason


def test_joint_motion_accepts_actual_joint6_delta_at_limit():
    from rebotarm_vision.policies.candidate_motion_policy import JointMotionPolicyConfig, evaluate_joint_motion

    result = evaluate_joint_motion(
        current_positions={"joint1": 0.0, "joint6": 0.0},
        target_positions={"joint1": 0.0, "joint6": math.radians(90.0)},
        config=JointMotionPolicyConfig(max_joint6_delta_rad=math.radians(90.0)),
    )

    assert result.accepted
    assert result.joint6_delta == pytest.approx(math.radians(90.0))
