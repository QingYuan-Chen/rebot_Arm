from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
PREVIEW_SRC = ROS2_ROOT / "src" / "rebotarm_preview"
if str(PREVIEW_SRC) not in sys.path:
    sys.path.insert(0, str(PREVIEW_SRC))


def test_gripper_width_maps_to_symmetric_finger_joints():
    from rebotarm_preview.sim_gripper import gripper_joint_positions_for_width

    left, right, reached = gripper_joint_positions_for_width(0.06, min_width=0.0, max_width=0.09)

    assert left == pytest.approx(0.03)
    assert right == pytest.approx(-0.03)
    assert reached == pytest.approx(0.06)


def test_gripper_width_is_clamped_before_mapping():
    from rebotarm_preview.sim_gripper import gripper_joint_positions_for_width

    left, right, reached = gripper_joint_positions_for_width(0.12, min_width=0.0, max_width=0.09)

    assert left == pytest.approx(0.045)
    assert right == pytest.approx(-0.045)
    assert reached == pytest.approx(0.09)
