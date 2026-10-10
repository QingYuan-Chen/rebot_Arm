from __future__ import annotations

from dataclasses import FrozenInstanceError
import importlib
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
ALL_JOINTS = ARM_JOINTS + ("left_finger_joint", "right_finger_joint")


def test_state_records_are_immutable() -> None:
    from rebotarm_simulation.core.mujoco_types import ContactInfo, RandomizedScene, SimulationState

    state = SimulationState(
        joint_names=ALL_JOINTS,
        joint_positions=[0.0] * 8,
        joint_velocities=(0.0,) * 8,
        actuator_forces=(0.0,) * 8,
        end_effector_position=(0.0, 0.0, 0.0),
        end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
        gripper_width=0.0,
        object_poses={"cube": (0.0,) * 7},
        simulation_time=0.0,
    )
    contact = ContactInfo("a", "b", "ga", "gb", (0.0, 0.0, 0.0), 0.0)
    scene = RandomizedScene(
        cube_pose=(0.28, 0.0, 0.04, 0.0, 0.0, 0.0, 1.0),
        reach_target_position=(0.3, 0.0, 0.2),
        seed=1,
    )

    with pytest.raises(FrozenInstanceError):
        state.simulation_time = 1.0
    with pytest.raises(FrozenInstanceError):
        contact.force = 1.0
    with pytest.raises(FrozenInstanceError):
        scene.seed = 2
    with pytest.raises(TypeError):
        state.object_poses["cube"] = (1.0,) * 7
    with pytest.raises(TypeError):
        state.joint_positions[0] = 1.0


def test_mirror_joint_state_sets_exact_arm_and_gripper_state(runtime_sim) -> None:
    target = (0.1, -0.8, -1.0, 0.3, 0.0, 0.0)
    velocity = (0.01, 0.0, 0.0, 0.0, 0.0, -0.01)

    state = runtime_sim.mirror_joint_state(
        target,
        velocity,
        gripper_width=0.04,
    )

    assert state.joint_positions[:6] == pytest.approx(target)
    assert state.joint_velocities[:6] == pytest.approx(velocity)
    assert state.gripper_width == pytest.approx(0.04)
    assert runtime_sim.control_targets[:6] == pytest.approx(target)


def test_mirror_joint_state_rejects_out_of_range_position(runtime_sim) -> None:
    with pytest.raises(ValueError, match="outside"):
        runtime_sim.mirror_joint_state((0.0, 0.1, -1.0, 0.3, 0.0, 0.0))


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"end_effector_position": (0.0, 0.0)}, "3 values"),
        ({"end_effector_orientation": (0.0, 0.0, 1.0)}, "4 values"),
        ({"object_poses": {"cube": (0.0,) * 6}}, "7 values"),
        ({"joint_positions": (float("nan"),) + (0.0,) * 7}, "finite"),
        ({"gripper_width": float("inf")}, "finite"),
    ],
)
def test_state_record_rejects_invalid_numeric_shapes(overrides, message: str) -> None:
    from rebotarm_simulation.core.mujoco_types import SimulationState

    values = dict(
        joint_names=ALL_JOINTS,
        joint_positions=(0.0,) * 8,
        joint_velocities=(0.0,) * 8,
        actuator_forces=(0.0,) * 8,
        end_effector_position=(0.0, 0.0, 0.0),
        end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
        gripper_width=0.0,
        object_poses={"cube": (0.0,) * 7},
        simulation_time=0.0,
    )
    values.update(overrides)
    with pytest.raises(ValueError, match=message):
        SimulationState(**values)


def test_contact_record_validates_names_shape_finiteness_and_force() -> None:
    from rebotarm_simulation.core.mujoco_types import ContactInfo

    with pytest.raises(ValueError, match="names"):
        ContactInfo("", "b", "ga", "gb", (0.0, 0.0, 0.0), 0.0)
    with pytest.raises(ValueError, match="3 values"):
        ContactInfo("a", "b", "ga", "gb", (0.0, 0.0), 0.0)
    with pytest.raises(ValueError, match="finite"):
        ContactInfo("a", "b", "ga", "gb", (0.0, float("nan"), 0.0), 0.0)
    with pytest.raises(ValueError, match="non-negative"):
        ContactInfo("a", "b", "ga", "gb", (0.0, 0.0, 0.0), -1.0)
    with pytest.raises(ValueError, match="finite"):
        ContactInfo(
            "a",
            "b",
            "ga",
            "gb",
            (0.0, 0.0, 0.0),
            1.0,
            normal=(0.0, float("nan"), 0.0),
        )


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"cube_pose": (0.0,) * 6}, "7 values"),
        ({"reach_target_position": (0.0, 0.0)}, "3 values"),
        ({"reach_target_position": (0.0, float("inf"), 0.0)}, "finite"),
    ],
)
def test_randomized_scene_record_validates_shapes_and_finiteness(overrides, message: str) -> None:
    from rebotarm_simulation.core.mujoco_types import RandomizedScene

    values = dict(
        cube_pose=(0.28, 0.0, 0.04, 0.0, 0.0, 0.0, 1.0),
        reach_target_position=(0.3, 0.0, 0.2),
        seed=1,
    )
    values.update(overrides)
    with pytest.raises(ValueError, match=message):
        RandomizedScene(**values)


def test_missing_mujoco_dependency_has_actionable_error(monkeypatch: pytest.MonkeyPatch) -> None:
    from rebotarm_simulation.core import mujoco_sim

    original = importlib.import_module

    def missing(name: str, *args, **kwargs):
        if name == "mujoco":
            raise ModuleNotFoundError("No module named 'mujoco'")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(importlib, "import_module", missing)
    with pytest.raises(RuntimeError, match=r"requirements-mujoco\.txt"):
        mujoco_sim.RebotArmMujoco(SCENE)


@pytest.fixture()
def runtime_sim():
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco(SCENE) as sim:
        yield sim


def test_default_model_path_is_portable_and_model_uses_canonical_order() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco() as sim:
        assert sim.joint_names == ALL_JOINTS
        assert Path(sim.model_path).name == "scene.xml"


def test_reset_is_deterministic_and_returns_finite_state(runtime_sim) -> None:
    first = runtime_sim.reset(seed=17)
    runtime_sim.step(4)
    second = runtime_sim.reset(seed=17)

    assert first == second
    assert first.joint_names == ALL_JOINTS
    assert len(first.joint_positions) == len(first.joint_velocities) == 8
    assert len(first.actuator_forces) == 8
    assert len(first.end_effector_position) == 3
    assert len(first.end_effector_orientation) == 4
    assert first.gripper_width == pytest.approx(
        first.joint_positions[-2] - first.joint_positions[-1]
    )
    assert "test_cube" in first.object_poses
    assert all(math.isfinite(value) for value in (
        *first.joint_positions,
        *first.joint_velocities,
        *first.actuator_forces,
        *first.end_effector_position,
        *first.end_effector_orientation,
        first.simulation_time,
    ))
    assert all(
        math.isfinite(value)
        for pose in first.object_poses.values()
        for value in pose
    )


def test_reset_home_uses_the_scene_home_keyframe(runtime_sim) -> None:
    state = runtime_sim.reset_home()

    assert state.joint_positions == pytest.approx(
        (0.0, -0.8, -1.0, 0.3, 0.0, 0.0, 0.03, -0.03)
    )
    assert state.object_poses["test_cube"][:3] == pytest.approx((0.28, 0.0, 0.04))


def test_control_modes_use_public_names_and_accept_legacy_pos_vel_alias(runtime_sim) -> None:
    runtime_sim.reset_home()
    assert runtime_sim.control_mode == "hold"
    assert runtime_sim.set_control_mode("gravity_comp") == "gravity_comp"
    assert runtime_sim.control_mode == "gravity_comp"
    before = runtime_sim.get_state().joint_positions[:6]
    runtime_sim.step(50)
    after = runtime_sim.get_state().joint_positions[:6]
    assert all(math.isfinite(value) for value in after)
    assert max(abs(a - b) for a, b in zip(after, before)) < 0.4

    assert runtime_sim.set_control_mode("hold") == "hold"
    assert runtime_sim.control_mode == "hold"
    runtime_sim.command_joint_positions((0.05, -0.85, -1.05, 0.25, 0.0, 0.0))
    assert runtime_sim.control_mode == "position"
    assert runtime_sim.set_control_mode("pos_vel") == "position"
    with pytest.raises(ValueError):
        runtime_sim.set_control_mode("unknown")


def test_reentering_hold_is_an_exact_noop(runtime_sim) -> None:
    runtime_sim.reset_home()
    runtime_sim.step(200)
    targets_before = tuple(runtime_sim.control_targets)
    status_before = runtime_sim.get_control_status()
    controller_state_before = runtime_sim.save_state().applied_torque
    actuator_control_before = runtime_sim._data.ctrl.copy()
    control_phase_before = runtime_sim.save_state().control_phase

    assert runtime_sim.set_mode("hold") == "hold"

    status_after = runtime_sim.get_control_status()
    assert tuple(runtime_sim.control_targets) == targets_before
    assert status_after.joint_targets == status_before.joint_targets
    assert status_after.requested_torques == status_before.requested_torques
    assert status_after.applied_torques == status_before.applied_torques
    assert runtime_sim.save_state().applied_torque == pytest.approx(
        controller_state_before
    )
    assert runtime_sim._data.ctrl == pytest.approx(actuator_control_before)
    assert runtime_sim.save_state().control_phase == control_phase_before


def test_entering_hold_from_gravity_comp_captures_pose(runtime_sim) -> None:
    runtime_sim.reset_home()
    runtime_sim.set_mode("gravity_comp")
    runtime_sim.step(20)
    position_before_hold = runtime_sim.get_state().joint_positions[:6]

    assert runtime_sim.set_mode("hold") == "hold"
    assert runtime_sim.get_control_status().joint_targets == pytest.approx(
        position_before_hold
    )


def test_default_reset_uses_home_and_hold(runtime_sim) -> None:
    state = runtime_sim.reset()

    assert state.joint_positions == pytest.approx(
        (0.0, -0.8, -1.0, 0.3, 0.0, 0.0, 0.03, -0.03)
    )
    assert runtime_sim.control_mode == "hold"
    assert runtime_sim.get_control_status().joint_targets == pytest.approx(
        state.joint_positions[:6]
    )


def test_raw_torque_is_limited_reported_and_expires_to_hold(runtime_sim) -> None:
    runtime_sim.reset()
    requested = (99.0, -99.0, 3.0, -99.0, 2.0, -1.0)
    reached = runtime_sim.command_joint_torques(requested, timeout_s=0.1)
    status = runtime_sim.get_control_status()

    assert reached == pytest.approx((27.0, -27.0, 3.0, -7.0, 2.0, -1.0))
    assert status.mode == "raw_torque"
    assert status.requested_torques == pytest.approx(requested)
    assert status.applied_torques == pytest.approx(reached)
    assert status.saturated == (True, True, False, True, False, False)
    assert status.watchdog_remaining_s == pytest.approx(0.1)

    runtime_sim.step(math.ceil(0.1 / runtime_sim.timestep))
    expired = runtime_sim.get_control_status()
    assert expired.mode == "hold"
    assert expired.watchdog_remaining_s is None
    assert runtime_sim.control_targets[:6] == pytest.approx(expired.joint_positions, abs=0.01)


@pytest.mark.parametrize(
    ("torques", "timeout"),
    [([0.0] * 5, 0.1), ([0.0] * 5 + [float("nan")], 0.1), ([0.0] * 6, 0.0)],
)
def test_raw_torque_rejects_invalid_commands(runtime_sim, torques, timeout) -> None:
    with pytest.raises((TypeError, ValueError)):
        runtime_sim.command_joint_torques(torques, timeout_s=timeout)


def test_home_pose_stays_stable_under_motor_control(runtime_sim) -> None:
    state = runtime_sim.reset_home()
    target = np.asarray(runtime_sim.control_targets[:6])
    max_error = 0.0
    max_speed = 0.0

    for _ in range(1000):
        state = runtime_sim.step()
        position = np.asarray(state.joint_positions[:6])
        velocity = np.asarray(state.joint_velocities[:6])
        max_error = max(max_error, float(np.max(np.abs(position - target))))
        max_speed = max(max_speed, float(np.max(np.abs(velocity))))

    assert max_error < 0.005
    assert max_speed < 0.02
    assert max(abs(force) for force in state.actuator_forces[:6]) < 9.0


def test_small_joint_step_settles_under_motor_control(runtime_sim) -> None:
    runtime_sim.reset_home()
    target = np.asarray((0.05, -0.85, -1.05, 0.25, 0.0, 0.0))
    runtime_sim.set_joint_position_targets(target)
    max_speed = 0.0

    for _ in range(3000):
        state = runtime_sim.step()
        max_speed = max(max_speed, float(np.max(np.abs(state.joint_velocities[:6]))))

    final_error = np.max(np.abs(np.asarray(state.joint_positions[:6]) - target))
    assert final_error < 0.01
    assert max_speed < 1.0


def test_end_effector_orientation_comes_from_site_frame_in_xyzw_order(tmp_path: Path) -> None:
    mujoco = pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    model_dir = tmp_path / "rebotarm"
    shutil.copytree(SCENE.parent, model_dir)
    robot_path = model_dir / "robot.xml"
    robot = ET.parse(robot_path)
    site = robot.find('.//site[@name="ee_site"]')
    assert site is not None
    site.set("quat", "0.7071067811865476 0 0 0.7071067811865476")
    robot.write(robot_path, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(model_dir / "scene.xml"))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    body_id = int(model.site_bodyid[site_id])
    expected_wxyz = np.empty(4)
    mujoco.mju_mat2Quat(expected_wxyz, data.site_xmat[site_id])
    expected_xyzw = (*expected_wxyz[1:], expected_wxyz[0])
    body_xyzw = (*data.xquat[body_id][1:], data.xquat[body_id][0])
    assert expected_xyzw != pytest.approx(body_xyzw)

    with RebotArmMujoco(model_dir / "scene.xml") as sim:
        assert sim.get_state().end_effector_orientation == pytest.approx(expected_xyzw)


def test_joint_targets_accept_mapping_or_arm_sequence_and_clamp(runtime_sim) -> None:
    reached = runtime_sim.set_joint_position_targets({"joint1": 99.0, "joint2": -1.0})
    assert reached == pytest.approx((2.8, -1.0, -1.0, 0.3, 0.0, 0.0))

    reached = runtime_sim.set_joint_position_targets([0.1, -0.2, -0.3, 0.4, 0.5, 0.6])
    assert reached == pytest.approx((0.1, -0.2, -0.3, 0.4, 0.5, 0.6))


@pytest.mark.parametrize("targets", [
    {"unknown": 0.0},
    {"joint1": float("nan")},
    [0.0] * 5,
    [0.0] * 5 + [float("inf")],
])
def test_joint_targets_reject_invalid_inputs(runtime_sim, targets) -> None:
    with pytest.raises((ValueError, TypeError)):
        runtime_sim.set_joint_position_targets(targets)


def test_gripper_width_uses_equal_and_opposite_joint_targets(runtime_sim) -> None:
    assert runtime_sim.set_gripper_width(0.20) == pytest.approx(0.09)
    assert runtime_sim.control_targets[-2:] == pytest.approx((0.045, -0.045))
    runtime_sim.step(50)
    state = runtime_sim.get_state()
    assert 0.0 <= state.gripper_width <= 0.09
    assert runtime_sim.set_gripper_width(-1.0) == pytest.approx(0.0)
    with pytest.raises(ValueError):
        runtime_sim.set_gripper_width(float("nan"))


def test_gripper_public_command_applies_optional_force_limit(runtime_sim) -> None:
    runtime_sim.reset()
    assert runtime_sim.command_gripper_width(0.09, max_force_n=2.5) == pytest.approx(0.09)
    runtime_sim.step()
    status = runtime_sim.get_control_status()
    assert status.gripper_target_width_m == pytest.approx(0.09)
    assert max(abs(value) for value in status.gripper_control_force_n) <= 2.5
    with pytest.raises(ValueError):
        runtime_sim.command_gripper_width(0.04, max_force_n=0.0)


def test_gripper_motor_control_is_clamped_before_viewer_exposes_ctrl(runtime_sim) -> None:
    runtime_sim.reset_home()
    runtime_sim.set_gripper_width(0.09)
    model, data = runtime_sim._unsafe_viewer_handles()

    runtime_sim.step()

    for actuator_id in range(model.nu - 2, model.nu):
        lower, upper = model.actuator_ctrlrange[actuator_id]
        assert lower <= data.ctrl[actuator_id] <= upper
    assert 1.0 < data.ctrl[-2] <= 20.0
    assert data.ctrl[-1] == pytest.approx(-data.ctrl[-2])


def test_gripper_opening_force_does_not_flip_sign_while_below_target(runtime_sim) -> None:
    runtime_sim.reset_home()
    runtime_sim.set_gripper_width(0.09)
    _, data = runtime_sim._unsafe_viewer_handles()

    left_forces = []
    widths = []
    for _ in range(200):
        state = runtime_sim.step()
        left_forces.append(float(data.ctrl[-2]))
        widths.append(float(state.gripper_width))

    assert max(widths) < 0.09
    assert min(left_forces) >= 0.0


@pytest.mark.parametrize("steps", [0, -1, 1.5, True])
def test_step_requires_a_positive_integer(runtime_sim, steps) -> None:
    with pytest.raises((ValueError, TypeError)):
        runtime_sim.step(steps)


def test_step_advances_exact_number_of_physics_steps(runtime_sim) -> None:
    before = runtime_sim.get_state().simulation_time
    runtime_sim.step(5)
    after = runtime_sim.get_state().simulation_time
    assert after - before == pytest.approx(5 * runtime_sim.timestep)


def test_save_restore_recovers_full_runtime_state(runtime_sim) -> None:
    runtime_sim.set_joint_position_targets([0.2, -0.3, -0.4, 0.2, 0.1, -0.2])
    runtime_sim.step(10)
    saved = runtime_sim.save_state()
    expected = runtime_sim.get_state()
    runtime_sim.step(20)
    runtime_sim.restore_state(saved)
    restored = runtime_sim.get_state()
    assert restored.joint_positions == pytest.approx(expected.joint_positions)
    assert restored.joint_velocities == pytest.approx(expected.joint_velocities)
    assert restored.actuator_forces == pytest.approx(expected.actuator_forces)
    assert restored.end_effector_position == pytest.approx(expected.end_effector_position)
    assert restored.end_effector_orientation == pytest.approx(expected.end_effector_orientation)
    assert restored.object_poses == pytest.approx(expected.object_poses)
    assert restored.simulation_time == pytest.approx(expected.simulation_time)


def test_saved_state_rejects_a_different_model_instance(runtime_sim) -> None:
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    saved = runtime_sim.save_state()
    with RebotArmMujoco(SCENE) as other:
        with pytest.raises(ValueError, match="same MuJoCo model"):
            other.restore_state(saved)


def test_saved_state_restores_arm_and_gripper_control_targets(runtime_sim) -> None:
    arm_a = (0.2, -0.4, -0.5, 0.3, -0.2, 0.4)
    runtime_sim.set_joint_position_targets(arm_a)
    runtime_sim.set_gripper_width(0.04)
    controls_a = runtime_sim.control_targets
    saved = runtime_sim.save_state()
    mujoco = pytest.importorskip("mujoco")
    assert saved.state_spec & int(mujoco.mjtState.mjSTATE_CTRL)
    assert saved.state_spec & int(mujoco.mjtState.mjSTATE_USER) == int(
        mujoco.mjtState.mjSTATE_USER
    )

    runtime_sim.set_joint_position_targets((-0.3, -1.0, -1.2, -0.4, 0.6, -0.7))
    runtime_sim.set_gripper_width(0.08)
    assert runtime_sim.control_targets != pytest.approx(controls_a)
    runtime_sim.restore_state(saved)

    assert runtime_sim.control_targets == pytest.approx(controls_a)


def test_saved_state_restores_raw_torque_watchdog(runtime_sim) -> None:
    requested = (40.0, -30.0, 3.0, -14.0, 2.0, -1.0)
    runtime_sim.command_joint_torques(requested, timeout_s=0.2)
    runtime_sim.step(10)
    saved = runtime_sim.save_state()
    expected = runtime_sim.get_control_status()

    runtime_sim.set_mode("hold")
    runtime_sim.restore_state(saved)
    restored = runtime_sim.get_control_status()

    assert restored.mode == "raw_torque"
    assert restored.requested_torques == pytest.approx(expected.requested_torques)
    assert restored.applied_torques == pytest.approx(expected.applied_torques)
    assert restored.watchdog_remaining_s == pytest.approx(expected.watchdog_remaining_s)


def test_saved_integration_state_replays_deterministically(runtime_sim) -> None:
    runtime_sim.set_joint_position_targets([0.2, -0.4, -0.5, 0.3, -0.2, 0.4])
    runtime_sim.set_gripper_width(0.04)
    runtime_sim.step(15)
    saved = runtime_sim.save_state()
    assert isinstance(saved.state, tuple)
    assert saved.state_spec > 0

    runtime_sim.step(25)
    first_state = runtime_sim.get_state()
    first_contacts = runtime_sim.get_contacts()
    assert first_contacts, "replay must exercise contact solver state"
    runtime_sim.restore_state(saved)
    runtime_sim.step(25)
    replayed_state = runtime_sim.get_state()
    replayed_contacts = runtime_sim.get_contacts()

    assert replayed_state.joint_positions == pytest.approx(first_state.joint_positions)
    assert replayed_state.joint_velocities == pytest.approx(first_state.joint_velocities)
    assert replayed_state.actuator_forces == pytest.approx(first_state.actuator_forces)
    assert replayed_state.end_effector_position == pytest.approx(first_state.end_effector_position)
    assert replayed_state.end_effector_orientation == pytest.approx(first_state.end_effector_orientation)
    assert replayed_state.object_poses.keys() == first_state.object_poses.keys()
    for name, pose in first_state.object_poses.items():
        assert replayed_state.object_poses[name] == pytest.approx(pose, abs=1e-12)
    assert replayed_state.simulation_time == pytest.approx(first_state.simulation_time)
    assert [(c.body1, c.body2, c.geom1, c.geom2) for c in replayed_contacts] == [
        (c.body1, c.body2, c.geom1, c.geom2) for c in first_contacts
    ]
    assert [c.force for c in replayed_contacts] == pytest.approx(
        [c.force for c in first_contacts]
    )


def test_object_pose_sets_free_body_and_normalizes_quaternion(runtime_sim) -> None:
    runtime_sim.set_object_pose("test_cube", (0.3, -0.1, 0.6), (0.0, 0.0, 0.0, 2.0))
    pose = runtime_sim.get_state().object_poses["test_cube"]
    assert pose == pytest.approx((0.3, -0.1, 0.6, 0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        runtime_sim.set_object_pose("test_cube", (0.0, 0.0, 0.0), (0.0,) * 4)
    with pytest.raises(ValueError):
        runtime_sim.set_object_pose("test_cube", (float("nan"), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError):
        runtime_sim.set_object_pose("table", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_set_object_pose_can_zero_or_preserve_free_joint_velocity(runtime_sim) -> None:
    position = (0.3, 0.0, 0.8)
    orientation = (0.0, 0.0, 0.0, 1.0)

    runtime_sim.reset()
    runtime_sim.step(20)
    runtime_sim.set_object_pose("test_cube", position, orientation, zero_velocity=False)
    runtime_sim.step()
    preserved_z = runtime_sim.get_state().object_poses["test_cube"][2]

    runtime_sim.reset()
    runtime_sim.step(20)
    runtime_sim.set_object_pose("test_cube", position, orientation)
    runtime_sim.step()
    zeroed_z = runtime_sim.get_state().object_poses["test_cube"][2]
    assert preserved_z < zeroed_z


def test_randomize_scene_is_seeded_and_keeps_objects_inside_bounds(runtime_sim) -> None:
    cube_xy_bounds = ((0.25, 0.26), (-0.02, 0.02))
    target_bounds = ((0.30, 0.31), (-0.03, 0.03), (0.18, 0.19))

    first = runtime_sim.randomize_scene(
        seed=11,
        cube_xy_bounds=cube_xy_bounds,
        cube_z=0.05,
        reach_target_bounds=target_bounds,
    )
    runtime_sim.reset_home(seed=99)
    second = runtime_sim.randomize_scene(
        seed=11,
        cube_xy_bounds=cube_xy_bounds,
        cube_z=0.05,
        reach_target_bounds=target_bounds,
    )

    assert second == first
    assert runtime_sim.get_state().object_poses["test_cube"] == pytest.approx(first.cube_pose)
    assert cube_xy_bounds[0][0] <= first.cube_pose[0] <= cube_xy_bounds[0][1]
    assert cube_xy_bounds[1][0] <= first.cube_pose[1] <= cube_xy_bounds[1][1]
    assert first.cube_pose[2] == pytest.approx(0.05)
    assert target_bounds[0][0] <= first.reach_target_position[0] <= target_bounds[0][1]
    assert target_bounds[1][0] <= first.reach_target_position[1] <= target_bounds[1][1]
    assert target_bounds[2][0] <= first.reach_target_position[2] <= target_bounds[2][1]


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"cube_xy_bounds": ((0.3, 0.2), (0.0, 0.1))}, "lower bound"),
        ({"cube_xy_bounds": ((0.2, 0.3),)}, "2 bounds"),
        ({"reach_target_bounds": ((0.2, 0.3), (0.0, 0.1))}, "3 bounds"),
        ({"reach_target_bounds": ((0.2, 0.3), (0.0, 0.1), (0.2, float("nan")))}, "finite"),
    ],
)
def test_randomize_scene_rejects_invalid_bounds(runtime_sim, kwargs, message: str) -> None:
    with pytest.raises(ValueError, match=message):
        runtime_sim.randomize_scene(**kwargs)


def test_randomization_session_changes_and_restores_model_parameters(runtime_sim) -> None:
    from rebotarm_simulation.sim2real.randomization import RandomizationSample

    model, _data = runtime_sim._unsafe_viewer_handles()
    baseline_mass = np.asarray(model.body_mass).copy()
    baseline_damping = np.asarray(model.dof_damping).copy()
    baseline_friction = np.asarray(model.geom_friction).copy()
    sample = RandomizationSample(
        seed=3,
        mass_scale=1.05,
        damping_scale=0.9,
        friction_scale=1.1,
        torque_scale=0.95,
        control_latency_steps=1,
        action_noise_std=0.0,
        position_noise_std=0.0,
        velocity_noise_std=0.0,
    )

    with runtime_sim.randomization_session(sample):
        assert np.asarray(model.body_mass) == pytest.approx(baseline_mass * 1.05)
        assert np.asarray(model.dof_damping) == pytest.approx(baseline_damping * 0.9)
        assert np.asarray(model.geom_friction) == pytest.approx(baseline_friction * 1.1)
        assert runtime_sim.randomization_sample == sample

    assert np.asarray(model.body_mass) == pytest.approx(baseline_mass)
    assert np.asarray(model.dof_damping) == pytest.approx(baseline_damping)
    assert np.asarray(model.geom_friction) == pytest.approx(baseline_friction)
    assert runtime_sim.randomization_sample is None


def test_contacts_have_stable_named_schema(runtime_sim) -> None:
    runtime_sim.reset()
    runtime_sim.step(100)
    for contact in runtime_sim.get_contacts():
        assert contact.body1 and contact.body2 and contact.geom1 and contact.geom2
        assert len(contact.position) == 3
        assert math.isfinite(contact.force) and contact.force >= 0.0


def test_close_is_idempotent_and_closed_instance_rejects_use(runtime_sim) -> None:
    runtime_sim.close()
    runtime_sim.close()
    with pytest.raises(RuntimeError, match="closed"):
        runtime_sim.step()
