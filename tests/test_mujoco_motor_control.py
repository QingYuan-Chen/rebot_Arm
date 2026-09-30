from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rebotarm_simulation.motor_control import (
    GripperMitController,
    PosVelController,
    load_motor_control_parameters,
)


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
    assert parameters.arm.rated_torque == pytest.approx((12, 12, 12, 3.5, 3.5, 3.5))
    assert parameters.arm.firmware_to_torque_scale == pytest.approx((200, 200, 200, 500, 500, 500))
    assert parameters.arm.torque_rate_limit_nm_s == pytest.approx((180, 180, 180, 90, 90, 90))
    assert parameters.arm.torque_lowpass_alpha == pytest.approx((0.35,) * 6)
    assert parameters.arm.gravity_compensation_scale == pytest.approx(1.0)
    assert parameters.gripper.firmware_default_kp == pytest.approx(8.0)
    assert parameters.gripper.firmware_default_kd == pytest.approx(1.0)
    assert parameters.gripper.move_kp == pytest.approx(5.0)
    assert parameters.gripper.closing_kp == pytest.approx(0.0)
    assert parameters.gripper.hold_kp == pytest.approx(5.0)
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


def test_gripper_mit_uses_state_gains_and_transmission_conversion() -> None:
    parameters = load_motor_control_parameters(ROOT).gripper
    controller = GripperMitController(parameters)

    command = controller.compute(
        target=0.03,
        position=0.02,
        velocity=0.0,
        mode="move",
    )

    assert command.kp == pytest.approx(5.0)
    assert command.kd == pytest.approx(1.0)
    assert command.motor_target_rad == pytest.approx(-1.6666666667)
    assert command.motor_torque_nm == pytest.approx(-1.5)
    assert command.finger_force_n == pytest.approx(66.6666666667)


def test_gripper_mit_supports_closing_and_hold_modes() -> None:
    parameters = load_motor_control_parameters(ROOT).gripper
    controller = GripperMitController(parameters)

    closing = controller.compute(
        target=0.0,
        position=0.02,
        velocity=-0.01,
        mode="closing",
        feedforward_torque=0.3,
    )
    hold = controller.compute(
        target=0.0,
        position=0.02,
        velocity=0.0,
        mode="hold",
        feedforward_torque=0.3,
    )

    assert closing.kp == pytest.approx(0.0)
    assert closing.kd == pytest.approx(0.5)
    assert closing.motor_torque_nm == pytest.approx(0.0222222222)
    assert hold.kp == pytest.approx(5.0)
    assert hold.kd == pytest.approx(1.0)
    assert hold.motor_torque_nm == pytest.approx(1.5)
