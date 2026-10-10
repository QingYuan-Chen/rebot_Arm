from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"


@pytest.fixture()
def simulation():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as sim:
        yield sim


def test_delta_and_options_validate_public_inputs() -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, IkOptions

    assert CartesianDelta().frame == "world"
    with pytest.raises(ValueError, match="3 values"):
        CartesianDelta(xyz_m=(0.0, 0.0))
    with pytest.raises(ValueError, match="finite"):
        CartesianDelta(rpy_rad=(0.0, float("nan"), 0.0))
    with pytest.raises(ValueError, match="frame"):
        CartesianDelta(frame="base")
    with pytest.raises(ValueError, match="positive"):
        IkOptions(damping=0.0)
    with pytest.raises(ValueError, match="positive integer"):
        IkOptions(max_iterations=True)


def test_world_translation_converges_without_mutating_live_simulation(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    before_state = simulation.get_state()
    before_controls = simulation.get_control_status()
    controller = MujocoCartesianController(simulation)
    result = controller.solve_delta(CartesianDelta(xyz_m=(0.01, 0.0, 0.0)))

    assert result.success is True
    assert result.status == "converged"
    assert result.position_error_m <= controller.options.position_tolerance_m
    assert result.target_position_m == pytest.approx(
        np.asarray(before_state.end_effector_position) + (0.01, 0.0, 0.0)
    )
    assert result.reached_position_m == pytest.approx(result.target_position_m, abs=5e-4)
    assert simulation.get_state() == before_state
    assert simulation.get_control_status() == before_controls


def test_tool_translation_uses_current_end_effector_axes(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    model, data = simulation._unsafe_viewer_handles()
    mujoco = pytest.importorskip("mujoco")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    rotation = np.asarray(data.site_xmat[site_id]).reshape(3, 3)
    start = np.asarray(data.site_xpos[site_id])
    local_delta = np.asarray((0.01, 0.0, 0.0))

    result = MujocoCartesianController(simulation).solve_delta(
        CartesianDelta(xyz_m=tuple(local_delta), frame="tool")
    )

    assert result.success is True
    assert result.target_position_m == pytest.approx(start + rotation @ local_delta)
    assert result.target_position_m != pytest.approx(start + local_delta)


@pytest.mark.parametrize("frame", ["world", "tool"])
def test_roll_pitch_yaw_delta_converges_in_both_frames(simulation, frame: str) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    result = MujocoCartesianController(simulation).solve_delta(
        CartesianDelta(rpy_rad=(0.02, -0.01, 0.015), frame=frame)
    )

    assert result.success is True
    assert result.orientation_error_rad <= 2e-3
    assert all(np.isfinite(result.joint_positions))


def test_command_delta_only_submits_successful_position_solution(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    controller = MujocoCartesianController(simulation)
    before = simulation.control_targets[:6]
    solved = controller.command_delta(CartesianDelta(xyz_m=(0.0, 0.01, 0.0)))

    assert solved.success is True
    assert simulation.control_mode == "position"
    assert simulation.control_targets[:6] == pytest.approx(solved.joint_positions)
    assert simulation.control_targets[:6] != pytest.approx(before)

    unchanged = simulation.control_targets[:6]
    failed = controller.command_delta(CartesianDelta(xyz_m=(2.0, 0.0, 0.0)))
    assert failed.success is False
    assert simulation.control_targets[:6] == pytest.approx(unchanged)


def test_absolute_pose_solve_has_no_live_side_effects(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import MujocoCartesianController

    before_state = simulation.get_state()
    before_control = simulation.get_control_status()
    xyzw = before_state.end_effector_orientation
    quaternion_wxyz = (xyzw[3], xyzw[0], xyzw[1], xyzw[2])
    target_position = np.asarray(before_state.end_effector_position) + (0.01, 0.0, 0.0)

    result = MujocoCartesianController(simulation).solve_pose(
        target_position, quaternion_wxyz
    )

    assert result.success is True
    assert result.target_position_m == pytest.approx(target_position)
    assert simulation.get_state() == before_state
    assert simulation.get_control_status() == before_control


def test_absolute_pose_command_submits_only_converged_solution(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import MujocoCartesianController

    state = simulation.get_state()
    xyzw = state.end_effector_orientation
    quaternion_wxyz = (xyzw[3], xyzw[0], xyzw[1], xyzw[2])
    target_position = np.asarray(state.end_effector_position) + (0.0, 0.01, 0.0)
    controller = MujocoCartesianController(simulation)

    result = controller.command_pose(target_position, quaternion_wxyz)

    assert result.success is True
    assert simulation.control_mode == "position"
    assert simulation.control_targets[:6] == pytest.approx(result.joint_positions)


@pytest.mark.parametrize(
    ("position", "quaternion", "message"),
    [
        ((0.0, 0.0), (1.0, 0.0, 0.0, 0.0), "3 values"),
        ((0.0, 0.0, float("nan")), (1.0, 0.0, 0.0, 0.0), "finite"),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), "4 values"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "non-zero"),
        ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0), "unit length"),
    ],
)
def test_absolute_pose_strictly_validates_position_and_quaternion(
    simulation, position, quaternion, message: str
) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import MujocoCartesianController

    with pytest.raises((TypeError, ValueError), match=message):
        MujocoCartesianController(simulation).solve_pose(position, quaternion)


def test_unreachable_target_is_bounded_and_reported(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    controller = MujocoCartesianController(simulation)
    result = controller.solve_delta(CartesianDelta(xyz_m=(2.0, 0.0, 0.0)))

    assert result.success is False
    assert result.status == "unreachable"
    assert result.message
    assert result.iterations <= controller.options.max_iterations
    for value, (lower, upper) in zip(result.joint_positions, simulation.arm_joint_limits):
        assert lower <= value <= upper
    assert all(np.isfinite(result.joint_positions))
    assert np.isfinite(result.position_error_m)


def test_damping_keeps_zero_motion_and_near_singular_solves_finite(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, IkOptions, MujocoCartesianController

    controller = MujocoCartesianController(
        simulation,
        options=IkOptions(damping=0.1, max_iterations=20),
    )
    zero = controller.solve_delta(CartesianDelta())
    difficult = controller.solve_delta(CartesianDelta(rpy_rad=(0.0, 0.0, 0.1)))

    assert zero.success is True
    assert zero.iterations == 0
    assert all(np.isfinite(difficult.joint_positions))
    assert difficult.status != "numerical_failure"


def test_result_records_are_immutable(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    result = MujocoCartesianController(simulation).solve_delta(CartesianDelta())
    with pytest.raises(FrozenInstanceError):
        result.success = False


def test_result_exposes_finite_target_and_reached_rpy(simulation) -> None:
    from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController

    result = MujocoCartesianController(simulation).solve_delta(
        CartesianDelta(rpy_rad=(0.01, -0.01, 0.005), frame="world")
    )
    assert len(result.target_rpy_rad) == 3
    assert len(result.reached_rpy_rad) == 3
    assert all(np.isfinite(result.target_rpy_rad))
    assert all(np.isfinite(result.reached_rpy_rad))
