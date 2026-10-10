from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_approach_policy_builds_pregrasp_by_retreating_along_approach_axis():
    from rebotarm_vision.policies.approach_policy import ApproachPolicyConfig, build_pregrasp_tcp

    pregrasp = build_pregrasp_tcp(
        grasp_tcp_xyz=(0.42, 0.03, 0.055),
        approach_axis_xyz=(1.0, 0.0, 0.0),
        config=ApproachPolicyConfig(
            pregrasp_distance_m=0.08,
        ),
    )

    assert pregrasp == pytest.approx((0.34, 0.03, 0.055))


def test_approach_policy_can_raise_flat_pregrasp_to_minimum_height_without_changing_grasp():
    from rebotarm_vision.policies.approach_policy import ApproachPolicyConfig, build_pregrasp_tcp

    pregrasp = build_pregrasp_tcp(
        grasp_tcp_xyz=(0.31, -0.01, 0.039),
        approach_axis_xyz=(0.5, 0.0, -0.8660254),
        config=ApproachPolicyConfig(
            pregrasp_distance_m=0.06,
            pregrasp_min_z_m=0.12,
        ),
    )

    assert pregrasp[0] == pytest.approx(0.28)
    assert pregrasp[1] == pytest.approx(-0.01)
    assert pregrasp[2] == pytest.approx(0.12)
