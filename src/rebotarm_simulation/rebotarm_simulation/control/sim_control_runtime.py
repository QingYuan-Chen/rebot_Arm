"""Own simulated control targets, torque watchdog, controller memory and cadence.

The caller lends model/data for each operation; this module never loads,
steps, resets, or closes a MuJoCo instance.
"""
from __future__ import annotations
import math
from typing import Mapping, Sequence
import numpy as np
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES, JOINT_NAMES
from rebotarm_simulation.core.validation import finite_vector as _finite_vector
from rebotarm_simulation.core.mujoco_types import ControlStatus
from rebotarm_simulation.control.motor_control import PosVelController, GripperMitController
from rebotarm_simulation.control.sim_gripper import gripper_joint_positions_for_width

CONTROL_MODES = ("position", "hold", "gravity_comp", "raw_torque")
_CONTROL_MODE_ALIASES = {"pos_vel": "position"}

class SimControlRuntime:
    def __init__(self, parameters, timestep, joint_ids, actuator_ids):
        self._motor_parameters = parameters
        self._arm_controller = PosVelController(parameters.arm)
        self._gripper_controller = GripperMitController(parameters.gripper)
        self._control_steps_per_update = max(1, int(round((1.0 / parameters.control_rate_hz) / timestep)))
        self._control_phase = 0
        self._position_targets = np.zeros(len(JOINT_NAMES), dtype=float)
        self._control_mode = "hold"
        self._raw_torque_command = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._raw_torque_requested = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._raw_torque_deadline = None
        self._requested_arm_torque = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._applied_arm_torque = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        self._arm_torque_saturated = np.zeros(len(ARM_JOINT_NAMES), dtype=bool)
        self._gripper_max_force_n = None
        self._gripper_control_force = np.zeros(2, dtype=float)
        self._joint_ids = joint_ids
        self._actuator_ids = actuator_ids
        self._randomization_torque_scale = 1.0

    @property
    def control_targets(self):
        return tuple(float(value) for value in self._position_targets)

    @property
    def control_mode(self):
        return self._control_mode

    def before_step(self, model, data):
        if self._raw_torque_watchdog_expired(model, data):
            self._expire_raw_torque(model, data)
            self._control_phase = 0
        if self._control_phase == 0:
            self._apply_motor_control(model, data)

    def after_step(self, model, data):
        self._control_phase = (self._control_phase + 1) % self._control_steps_per_update
        if self._raw_torque_watchdog_expired(model, data):
            self._expire_raw_torque(model, data)
            self._control_phase = 0
            self._apply_motor_control(model, data)

    def capture(self):
        return dict(
            control_targets=tuple(float(value) for value in self._position_targets),
            position_integral=tuple(float(value) for value in self._arm_controller.position_integral),
            velocity_integral=tuple(float(value) for value in self._arm_controller.velocity_integral),
            applied_torque=tuple(float(value) for value in self._arm_controller.applied_torque),
            control_phase=self._control_phase,
            control_mode=self._control_mode,
            raw_torque_command=tuple(float(value) for value in self._raw_torque_command),
            raw_torque_requested=tuple(float(value) for value in self._raw_torque_requested),
            raw_torque_deadline=self._raw_torque_deadline,
            gripper_max_force_n=self._gripper_max_force_n,
        )

    def restore(self, state, model, data):
        self._position_targets[:] = np.asarray(state.control_targets, dtype=float)
        self._arm_controller.position_integral[:] = np.asarray(state.position_integral, dtype=float)
        self._arm_controller.velocity_integral[:] = np.asarray(state.velocity_integral, dtype=float)
        self._arm_controller.applied_torque[:] = np.asarray(state.applied_torque, dtype=float)
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode
        self._raw_torque_command[:] = np.asarray(state.raw_torque_command, dtype=float)
        self._raw_torque_requested[:] = np.asarray(state.raw_torque_requested, dtype=float)
        self._raw_torque_deadline = state.raw_torque_deadline
        self._gripper_max_force_n = state.gripper_max_force_n
        self._applied_arm_torque[:] = np.asarray(
            [data.ctrl[actuator_id] for actuator_id in self._actuator_ids[:6]],
            dtype=float,
        )
        if self._control_mode == "raw_torque":
            self._requested_arm_torque[:] = self._raw_torque_requested
            self._arm_torque_saturated[:] = ~np.isclose(
                self._requested_arm_torque,
                self._applied_arm_torque,
                atol=1e-12,
                rtol=0.0,
            )
        else:
            self._requested_arm_torque[:] = self._applied_arm_torque
            self._arm_torque_saturated.fill(False)
        self._gripper_control_force[:] = tuple(
            float(data.ctrl[actuator_id]) for actuator_id in self._actuator_ids[-2:]
        )

    def get_control_status(self, state) -> ControlStatus:
        """Return an immutable diagnostic snapshot without changing control state."""
        remaining = None
        if self._control_mode == "raw_torque" and self._raw_torque_deadline is not None:
            remaining = max(0.0, self._raw_torque_deadline - state.simulation_time)
        return ControlStatus(
            mode=self._control_mode,
            joint_targets=tuple(float(value) for value in self._position_targets[:6]),
            joint_positions=state.joint_positions[:6],
            joint_velocities=state.joint_velocities[:6],
            requested_torques=tuple(float(value) for value in self._requested_arm_torque),
            applied_torques=tuple(float(value) for value in self._applied_arm_torque),
            saturated=tuple(bool(value) for value in self._arm_torque_saturated),
            watchdog_remaining_s=remaining,
            gripper_target_width_m=float(self._position_targets[-2] - self._position_targets[-1]),
            gripper_width_m=state.gripper_width,
            gripper_control_force_n=tuple(float(value) for value in self._gripper_control_force),
        )


    def set_mode(self, model, data, mode: str) -> str:
        mode = _CONTROL_MODE_ALIASES.get(str(mode), str(mode))
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        # Re-entering Hold must be a true no-op. Re-capturing the measured
        # position and resetting the simulated firmware controller on every
        # keyboard-repeat event creates a small target/torque discontinuity
        # that is visible as a shake even though the requested mode did not
        # change.
        if mode == "hold" and self._control_mode == "hold":
            return self._control_mode
        if mode == "hold":
            self._sync_arm_targets_to_current_position(model, data)
        if mode != "raw_torque":
            self._raw_torque_command.fill(0.0)
            self._raw_torque_requested.fill(0.0)
            self._raw_torque_deadline = None
        elif self._raw_torque_deadline is None:
            self._raw_torque_command.fill(0.0)
            self._raw_torque_requested.fill(0.0)
            self._raw_torque_deadline = float(data.time) + 0.1
        if mode in ("gravity_comp", "hold"):
            self._arm_controller.reset()
        self._control_mode = mode
        self._apply_motor_control(model, data)
        return self._control_mode


    def set_control_mode(self, model, data, mode: str) -> str:
        """Compatibility alias for :meth:`set_mode`.

        ``pos_vel`` is accepted as a legacy spelling and normalized to
        ``position``.
        """
        return self.set_mode(model, data, mode)


    def command_joint_positions(
        self, model, data, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        current = list(self.control_targets[: len(ARM_JOINT_NAMES)])
        if isinstance(targets, Mapping):
            unknown = set(targets) - set(ARM_JOINT_NAMES)
            if unknown:
                raise ValueError(f"Unknown arm joint names: {sorted(unknown)}")
            updates = {name: float(value) for name, value in targets.items()}
            if not all(math.isfinite(value) for value in updates.values()):
                raise ValueError("Joint targets must be finite")
            for name, value in updates.items():
                current[ARM_JOINT_NAMES.index(name)] = value
        else:
            current = list(_finite_vector(targets, len(ARM_JOINT_NAMES), "joint targets"))

        reached = []
        for index, joint_id in enumerate(self._joint_ids[:6]):
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            value = min(max(current[index], lower), upper)
            self._position_targets[index] = value
            reached.append(value)
        self.set_mode(model, data, "position")
        return tuple(reached)


    def set_joint_position_targets(
        self, model, data, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        return self.command_joint_positions(model, data, targets)


    def command_joint_torques(
        self, model, data, torques: Sequence[float], timeout_s: float = 0.1
    ) -> tuple[float, ...]:
        requested = np.asarray(
            _finite_vector(torques, len(ARM_JOINT_NAMES), "joint torques"), dtype=float
        )
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        effort = np.asarray(self._motor_parameters.arm.effort_limit, dtype=float)
        self._raw_torque_requested[:] = requested
        self._raw_torque_command[:] = np.clip(requested, -effort, effort)
        self._raw_torque_deadline = float(data.time) + timeout
        self._arm_controller.reset()
        self._control_mode = "raw_torque"
        self._apply_motor_control(model, data)
        return tuple(float(value) for value in self._raw_torque_command)


    def command_gripper_width(self, model, data, width_m: float, max_force_n: float | None = None) -> float:
        value = float(width_m)
        if not math.isfinite(value):
            raise ValueError("Gripper width must be finite")
        if max_force_n is not None:
            max_force_n = float(max_force_n)
            if not math.isfinite(max_force_n) or max_force_n <= 0.0:
                raise ValueError("max_force_n must be finite and positive")
            max_force_n = min(max_force_n, self._motor_parameters.gripper.finger_force_limit_n)
        self._gripper_max_force_n = max_force_n
        left, right, reached = gripper_joint_positions_for_width(value)
        self._position_targets[-2] = left
        self._position_targets[-1] = right
        return reached


    def set_gripper_width(self, model, data, width: float) -> float:
        return self.command_gripper_width(model, data, width)


    def _apply_motor_control(self, model, data) -> None:
        qpos_addresses = [int(model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        position = np.asarray([data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        gravity = self._gravity_compensation_torque(model, data, qvel_addresses)
        if self._control_mode == "raw_torque":
            requested_torque = self._raw_torque_requested.copy()
            arm_torque = self._raw_torque_command.copy()
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "gravity_comp":
            requested_torque = gravity
            arm_torque = requested_torque.copy()
            self._arm_controller.applied_torque[:] = arm_torque
        elif self._control_mode == "hold":
            # Hold is position regulation around the pose captured on entry.
            # It deliberately shares the simulated firmware loop with
            # position mode; the semantic difference is who owns the target.
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
            requested_torque = arm_torque.copy()
        else:
            arm_torque = self._arm_controller.compute(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
            requested_torque = arm_torque.copy()
        requested_torque = np.asarray(requested_torque, dtype=float)
        scaled_torque = np.asarray(arm_torque, dtype=float) * self._randomization_torque_scale
        applied_torque = np.asarray(
            [
                self._clamp_actuator_control(model, data, actuator_id, torque)
                for actuator_id, torque in zip(self._actuator_ids[:6], scaled_torque)
            ],
            dtype=float,
        )
        self._requested_arm_torque[:] = requested_torque
        self._applied_arm_torque[:] = applied_torque
        self._arm_torque_saturated[:] = ~np.isclose(requested_torque, applied_torque, atol=1e-12, rtol=0.0)
        for actuator_id, torque in zip(self._actuator_ids[:6], applied_torque):
            data.ctrl[actuator_id] = torque

        left_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-1]])])
        command = self._gripper_controller.compute(
            target=float(self._position_targets[-2] - self._position_targets[-1]),
            position=left_qpos - right_qpos,
            velocity=left_qvel - right_qvel,
            mode="move",
        )
        finger_force = self._stable_gripper_force(model, data,
            target_width=command.target_displacement_m,
            current_width=left_qpos - right_qpos,
            current_velocity=left_qvel - right_qvel,
        )
        if self._gripper_max_force_n is not None:
            finger_force = float(np.clip(
                finger_force, -self._gripper_max_force_n, self._gripper_max_force_n
            ))
        left_force = self._clamp_actuator_control(model, data, self._actuator_ids[-2], finger_force)
        right_force = self._clamp_actuator_control(model, data, self._actuator_ids[-1], -finger_force)
        self._gripper_control_force[:] = (left_force, right_force)
        data.ctrl[self._actuator_ids[-2]] = left_force
        data.ctrl[self._actuator_ids[-1]] = right_force


    def _expire_raw_torque(self, model, data) -> None:
        self._raw_torque_command.fill(0.0)
        self._raw_torque_requested.fill(0.0)
        self._raw_torque_deadline = None
        self._sync_arm_targets_to_current_position(model, data)
        self._arm_controller.reset()
        self._control_mode = "hold"


    def _raw_torque_watchdog_expired(self, model, data) -> bool:
        return (
            self._control_mode == "raw_torque"
            and self._raw_torque_deadline is not None
            and float(data.time) >= self._raw_torque_deadline
        )


    def _stable_gripper_force(
        self, model, data,
        *,
        target_width: float,
        current_width: float,
        current_velocity: float,
    ) -> float:
        gripper = self._motor_parameters.gripper
        error = float(target_width) - float(current_width)
        velocity = float(current_velocity)
        if (
            abs(error) < gripper.sim_force_deadband_m
            and abs(velocity) < gripper.sim_velocity_deadband_m_s
        ):
            return 0.0
        # MuJoCo force actuators act directly on the sliding finger joints in N.
        # The real MIT command still defines the motor-side torque limit, while
        # this linear-space PD keeps the simulated prismatic joints stable.
        return (
            gripper.sim_force_kp_n_per_m * error
            - gripper.sim_force_kd_n_s_per_m * velocity
        )


    def _clamp_actuator_control(self, model, data, actuator_id: int, value: float) -> float:
        if int(model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)


    def _seed_arm_torque_from_gravity(self, model, data) -> None:
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.applied_torque[:] = self._gravity_compensation_torque(model, data, qvel_addresses)


    def _sync_arm_targets_to_current_position(self, model, data) -> None:
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(data.qpos[int(model.jnt_qposadr[joint_id])])


    def _gravity_compensation_torque(self, model, data, qvel_addresses: Sequence[int]) -> np.ndarray:
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return np.asarray([data.qfrc_bias[address] for address in qvel_addresses], dtype=float) * scale
