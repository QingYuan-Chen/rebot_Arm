from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from rebotarm_simulation.control.control_config import load_motor_control_parameters

from rebotarm_simulation.control.motor_control import PosVelController


ROOT = Path(__file__).resolve().parents[1]


def test_parameters_come_from_existing_motor_yaml_and_urdf() -> None:
    parameters = load_motor_control_parameters(ROOT)

    assert parameters.control_rate_hz == pytest.approx(100.0)
    assert parameters.motor_specs["DM4310_V1_2"].peak_torque_nm == pytest.approx(12.5)
    assert parameters.motor_specs["DM4310_V1_2"].rated_torque_nm == pytest.approx(3.5)
    assert parameters.arm.motor_models == (
        "DM4340P", "DM4340P", "DM4340P",
        "DM4310_V1_2", "DM4310_V1_2", "DM4310_V1_2",
    )
    assert parameters.arm.pos_kp == pytest.approx((150, 150, 150, 50, 50, 50))
    assert parameters.arm.pos_ki == pytest.approx((0.5, 0.5, 0.5, 1, 1, 1))
    assert parameters.arm.vel_kp == pytest.approx((0.0125, 0.0125, 0.0125, 0.0008, 0.0008, 0.0008))
    assert parameters.arm.vel_ki == pytest.approx((0.004, 0.004, 0.004, 0.002, 0.002, 0.002))
    assert parameters.arm.velocity_limit == pytest.approx((5, 5, 5, 3, 3, 3))
    assert parameters.arm.effort_limit == pytest.approx((27, 27, 27, 7, 7, 7))
    assert parameters.arm.rated_torque == pytest.approx((9, 9, 9, 3.5, 3.5, 3.5))
    assert parameters.arm.firmware_to_torque_scale == pytest.approx((200, 200, 200, 500, 500, 500))
    assert parameters.arm.velocity_damping_nm_s_per_rad == pytest.approx((9.3, 9.3, 9.3, 2.6, 2.6, 1.0))
    assert parameters.arm.torque_rate_limit_nm_s == pytest.approx((180, 180, 180, 90, 90, 90))
    assert parameters.arm.torque_lowpass_alpha == pytest.approx((0.35,) * 6)
    assert parameters.arm.gravity_compensation_scale == pytest.approx(1.0)
    assert parameters.gripper.motor_model == "DM4310_V1_2"
    assert parameters.gripper.finger_force_limit_n == pytest.approx(1.5)
    assert parameters.gripper.sim_force_kp_n_per_m == pytest.approx(250.0)
    assert parameters.gripper.sim_force_kd_n_s_per_m == pytest.approx(6.0)


def test_pos_vel_controller_uses_cascade_and_effort_scaling() -> None:
    parameters = load_motor_control_parameters(ROOT).arm
    controller = PosVelController(parameters)

    torque = controller.compute(
        target=np.array((0.01, 0, 0, 0, 0, 0), dtype=float),
        position=np.zeros(6),
        velocity=np.zeros(6),
        dt=0.01,
    )

    assert controller.target_velocity[0] == pytest.approx(1.50005)
    assert torque[0] == pytest.approx(0.63)
    assert torque[1:] == pytest.approx(np.zeros(5))


def test_pos_vel_controller_limits_torque_step_and_accepts_feedforward() -> None:
    parameters = load_motor_control_parameters(ROOT).arm
    controller = PosVelController(parameters)

    first = controller.compute(
        target=np.ones(6),
        position=np.zeros(6),
        velocity=np.zeros(6),
        dt=0.01,
        feedforward=np.array((2.0, 2.0, 2.0, 1.0, 1.0, 1.0)),
    )
    second = controller.compute(
        target=np.ones(6),
        position=np.zeros(6),
        velocity=np.zeros(6),
        dt=0.01,
        feedforward=np.array((2.0, 2.0, 2.0, 1.0, 1.0, 1.0)),
    )

    assert first == pytest.approx((0.63, 0.63, 0.63, 0.315, 0.315, 0.315))
    assert np.all(np.abs(second - first) <= np.array((0.63, 0.63, 0.63, 0.315, 0.315, 0.315)) + 1e-12)


def test_pos_vel_controller_saturates_and_reset_clears_integrators() -> None:
    parameters = load_motor_control_parameters(ROOT).arm
    controller = PosVelController(parameters)
    for _ in range(10_000):
        torque = controller.compute(
            target=np.full(6, 100.0),
            position=np.zeros(6),
            velocity=np.full(6, -100.0),
            dt=0.01,
        )
    assert np.all(np.abs(torque) <= np.asarray(parameters.effort_limit))
    assert np.all(np.isfinite(controller.position_integral))
    assert np.all(np.isfinite(controller.velocity_integral))

    controller.reset()
    assert controller.position_integral == pytest.approx(np.zeros(6))
    assert controller.velocity_integral == pytest.approx(np.zeros(6))
    assert controller.outer_torque == pytest.approx(np.zeros(6))


def test_pos_vel_controller_applies_inner_velocity_damping_and_rejects_invalid_values() -> None:
    from dataclasses import replace

    parameters = load_motor_control_parameters(ROOT).arm
    controller = PosVelController(parameters)
    outer_step = controller.compute(
        target=np.array((0.01, 0, 0, 0, 0, 0), dtype=float),
        position=np.zeros(6),
        velocity=np.zeros(6),
        dt=0.01,
    )
    assert outer_step[0] == pytest.approx(0.63)
    requested, bounded = controller.evaluate_inner_torque(np.array((0.1, 0, 0, 0, 0, 0), dtype=float))
    assert requested[0] == pytest.approx(0.63 - 9.3 * 0.1)
    assert bounded[0] == pytest.approx(0.63 - 9.3 * 0.1)
    assert controller.outer_torque[0] == pytest.approx(0.63)

    for bad in ((-0.1, 0, 0, 0, 0, 0), (float("nan"), 0, 0, 0, 0, 0), (1.0, 1.0)):
        with pytest.raises(ValueError, match="velocity_damping_nm_s_per_rad"):
            replace(parameters, velocity_damping_nm_s_per_rad=bad)


def test_pos_vel_inner_damping_stabilizes_step_even_at_near_zero_mechanical_damping() -> None:
    pytest.importorskip("mujoco")
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    with RebotArmMujoco() as sim:
        model, data = sim.borrow_viewer_handles()
        for joint_id in sim._joint_ids[:6]:
            dof = int(model.jnt_dofadr[joint_id])
            model.dof_damping[dof] = 0.01
            sim._randomization_baseline["dof_damping"][dof] = 0.01
        sim._mj.mj_setConst(model, data)
        sim.reset()
        target = np.asarray((0.05, -0.85, -1.05, 0.25, 0.05, -0.08), dtype=float)
        sim.command_joint_positions(target)
        max_speed = 0.0
        for _ in range(3000):
            state = sim.step()
            max_speed = max(max_speed, float(np.max(np.abs(state.joint_velocities[:6]))))
        final_error = float(np.max(np.abs(np.asarray(state.joint_positions[:6]) - target)))
        assert final_error < 0.01
        assert max_speed < 1.0
