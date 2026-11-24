"""职责:仿真控制目标,位置/保持双频内环与关节阻抗双频MIT会话接线,控制器记忆及控制周期.

对外接口:位置,力矩,关节阻抗命令和模式切换;物理实例由调用者持有.
安全约束:阻抗准入/两阶段MIT计算/交接限速委托 rebotarm_force_control.JointImpedanceSession,
          位置/保持 500 Hz 高频速度阻尼委托 PosVelController.evaluate_inner_torque;
          100 Hz 外环写 data.ctrl 确认后提交状态,500 Hz 物理子步分别执行 MIT/PosVel 内环;
          退出阻抗经 handoff 限速裁剪并回写 PosVelController 外环基线平滑交接回 hold.
文件结构:状态与快照 → 命令准入与验证 → 逐拍控制(外环/MIT内环/PosVel内环) → 交接与重力辅助函数.
"""
from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np

from rebotarm_force_control.contracts import package_file
from rebotarm_force_control.joint_impedance import JointImpedanceCommand
from rebotarm_force_control.joint_impedance_session import JointImpedanceSession
from rebotarm_force_control.joint_impedance_settings import JointImpedanceSettings
from rebotarm_force_control.mit_command import evaluate_mit_torque
from rebotarm_simulation.control.motor_control import PosVelController
from rebotarm_simulation.control.sim_gripper import gripper_joint_positions_for_width
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES, JOINT_NAMES
from rebotarm_simulation.core.mujoco_types import ControlStatus
from rebotarm_simulation.core.validation import finite_vector as _finite_vector


CONTROL_MODES = ("position", "hold", "gravity_comp", "raw_torque", "joint_impedance")


class SimControlRuntime:
    def __init__(
        self,
        parameters,
        timestep,
        joint_ids,
        actuator_ids,
        gravity_torque,
        impedance_settings: JointImpedanceSettings | None = None,
    ):
        self._motor_parameters = parameters
        self._arm_controller = PosVelController(parameters.arm)
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
        self._gravity_torque = gravity_torque
        self._impedance_settings = (
            impedance_settings
            if impedance_settings is not None
            else JointImpedanceSettings.from_yaml(
                package_file("rebotarm_force_control", "config/joint_impedance.yaml")
            )
        )
        self._impedance = JointImpedanceSession()

    @property
    def control_targets(self):
        return tuple(float(value) for value in self._position_targets)

    @property
    def control_mode(self):
        return self._control_mode

    @property
    def control_rate_hz(self) -> float:
        return float(self._motor_parameters.control_rate_hz)

    @property
    def arm_actuator_ids(self) -> tuple[int, ...]:
        return tuple(self._actuator_ids[:6])

    @property
    def randomization_torque_scale(self) -> float:
        return self._randomization_torque_scale

    @randomization_torque_scale.setter
    def randomization_torque_scale(self, scale: float) -> None:
        self._randomization_torque_scale = float(scale)

    def _arm_addresses(self, model) -> tuple[list[int], list[int]]:
        qpos_addresses = [int(model.jnt_qposadr[joint_id]) for joint_id in self._joint_ids[:6]]
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        return qpos_addresses, qvel_addresses

    def impedance_position_limits(self, model) -> tuple[tuple[float, float], ...]:
        """提取物理引擎当前模型六轴位置限位供力控状态机校验."""
        return tuple(
            (float(model.jnt_range[joint_id][0]), float(model.jnt_range[joint_id][1]))
            for joint_id in self._joint_ids[:6]
        )

    def impedance_effort_limits(self, model):
        """采用模型执行器与标定上限的交集,供接纳,快照和逐拍计算共用."""
        return tuple(
            min(
                limit,
                -float(model.actuator_ctrlrange[actuator_id][0]),
                float(model.actuator_ctrlrange[actuator_id][1]),
            )
            if model.actuator_ctrllimited[actuator_id]
            else limit
            for limit, actuator_id in zip(
                self._motor_parameters.arm.effort_limit, self._actuator_ids[:6]
            )
        )

    def _validate_impedance_feedback(self, data) -> None:
        """每个物理步先检查反馈,避免非控制拍的坏值被物理引擎自动复位掩盖."""
        self._impedance.validate_feedback(float(data.time), data.qpos, data.qvel, data.qacc)

    def reset_from_physics(
        self, mj, model, data, *, gripper_targets: Sequence[float] | None = None
    ) -> None:
        """物理状态复位后,将目标对齐 qpos,清零控制器/力矩/阻抗记忆并以重力前馈进入 hold."""
        for index, joint_id in enumerate(self._joint_ids):
            qpos_address = int(model.jnt_qposadr[joint_id])
            self._position_targets[index] = data.qpos[qpos_address]
        if gripper_targets is not None:
            self._position_targets[-2:] = gripper_targets
        data.ctrl[:] = 0.0
        self._arm_controller.reset()
        self._control_mode = "hold"
        self._raw_torque_command.fill(0.0)
        self._raw_torque_requested.fill(0.0)
        self._raw_torque_deadline = None
        self._impedance.reset(float(data.time))
        self._requested_arm_torque.fill(0.0)
        self._applied_arm_torque.fill(0.0)
        self._arm_torque_saturated.fill(False)
        self._gripper_max_force_n = None
        self._gripper_control_force.fill(0.0)
        self._control_phase = 0
        mj.mj_forward(model, data)
        self._seed_arm_torque_from_gravity(model, data)
        self._apply_motor_control(model, data)
        mj.mj_forward(model, data)

    def sync_after_mirror(self, mj, model, data, *, include_gripper: bool) -> None:
        """运动学镜像写入 qpos 后对齐目标,重置 PI 与控制相位并刷新重力前馈."""
        indices = range(len(self._joint_ids)) if include_gripper else range(len(ARM_JOINT_NAMES))
        for index in indices:
            joint_id = self._joint_ids[index]
            self._position_targets[index] = float(data.qpos[int(model.jnt_qposadr[joint_id])])
        self._arm_controller.reset()
        self._control_phase = 0
        mj.mj_forward(model, data)
        self._seed_arm_torque_from_gravity(model, data)
        self._apply_motor_control(model, data)
        mj.mj_forward(model, data)

    def before_step(self, model, data) -> None:
        """物理步开始前核对反馈;100 Hz 外环更新控制量,其余 500 Hz 步按当前模式运行内环."""
        if self._control_mode == "joint_impedance" or self._impedance.handoff:
            self._validate_impedance_feedback(data)
        if self._raw_torque_watchdog_expired(model, data):
            self._expire_raw_torque(model, data)
            self._control_phase = 0
        if self._control_phase == 0:
            self._apply_motor_control(model, data)
            # 后继位置任务沿用原定拍完成首次交接,随后恢复既有位置控制语义.
            if self._impedance.handoff and self._control_mode == "position":
                self._impedance.clear_handoff()
        elif self._control_mode == "joint_impedance":
            self._apply_mit_inner_loop(model, data)
        elif self._control_mode in ("position", "hold") and not self._impedance.handoff:
            # 退出阻抗的 handoff 过渡子步保持外环已限速力矩,避免越过变化率约束.
            self._apply_pos_vel_inner_loop(model, data)

    def after_step(self, model, data) -> None:
        """物理时间推进后核对反馈与相位;raw_torque 到期立即切回保持."""
        if self._control_mode == "joint_impedance" or self._impedance.handoff:
            self._validate_impedance_feedback(data)
        self._control_phase = (self._control_phase + 1) % self._control_steps_per_update
        if self._raw_torque_watchdog_expired(model, data):
            self._expire_raw_torque(model, data)
            self._control_phase = 0
            self._apply_motor_control(model, data)

    def capture(self) -> dict:
        snapshot = dict(
            control_targets=tuple(float(value) for value in self._position_targets),
            position_integral=tuple(float(value) for value in self._arm_controller.position_integral),
            velocity_integral=tuple(float(value) for value in self._arm_controller.velocity_integral),
            applied_torque=tuple(float(value) for value in self._arm_controller.applied_torque),
            pos_vel_outer_torque=tuple(float(value) for value in self._arm_controller.outer_torque),
            control_phase=self._control_phase,
            control_mode=self._control_mode,
            raw_torque_command=tuple(float(value) for value in self._raw_torque_command),
            raw_torque_requested=tuple(float(value) for value in self._raw_torque_requested),
            raw_torque_deadline=self._raw_torque_deadline,
            gripper_max_force_n=self._gripper_max_force_n,
        )
        snapshot.update(self._impedance.snapshot())
        return snapshot

    def validate_saved_impedance_state(self, model, candidate, state) -> None:
        """由状态机统一校验检查点内的阻抗命令,协议界限与溢出边界."""
        qpos_addresses, qvel_addresses = self._arm_addresses(model)
        torque = np.asarray(candidate.ctrl)[list(self.arm_actuator_ids)]
        self._impedance.validate_snapshot_state(
            command=state.impedance_command,
            mit_output=state.impedance_mit_output,
            requested_torques=state.impedance_requested_torques,
            handoff=state.impedance_handoff,
            last_control_time=state.last_control_time,
            candidate_time=float(candidate.time),
            candidate_torque=torque,
            position_limits=self.impedance_position_limits(model),
            position=candidate.qpos[qpos_addresses],
            velocity=candidate.qvel[qvel_addresses],
            gravity=self._gravity_torque(qvel_addresses, candidate),
            effort_limits=self.impedance_effort_limits(model),
            rate_limits=self._motor_parameters.arm.torque_rate_limit_nm_s,
            nominal_dt=1.0 / self._motor_parameters.control_rate_hz,
            torque_scale=self._randomization_torque_scale,
            settings=self._impedance_settings,
        )

    def restore(self, state, model, data) -> None:
        qpos_addresses, qvel_addresses = self._arm_addresses(model)
        restored_torque = np.asarray(
            [data.ctrl[actuator_id] for actuator_id in self._actuator_ids[:6]],
            dtype=float,
        )
        self._impedance.restore(
            command=state.impedance_command,
            mit_output=state.impedance_mit_output,
            requested_torques=state.impedance_requested_torques,
            handoff=state.impedance_handoff,
            last_control_time=state.last_control_time,
            candidate_time=float(data.time),
            candidate_torque=restored_torque,
            position_limits=self.impedance_position_limits(model),
            position=data.qpos[qpos_addresses],
            velocity=data.qvel[qvel_addresses],
            gravity=self._gravity_torque(qvel_addresses, data),
            effort_limits=self.impedance_effort_limits(model),
            rate_limits=self._motor_parameters.arm.torque_rate_limit_nm_s,
            nominal_dt=1.0 / self._motor_parameters.control_rate_hz,
            torque_scale=self._randomization_torque_scale,
            settings=self._impedance_settings,
        )
        self._position_targets[:] = np.asarray(state.control_targets, dtype=float)
        self._arm_controller.position_integral[:] = np.asarray(state.position_integral, dtype=float)
        self._arm_controller.velocity_integral[:] = np.asarray(state.velocity_integral, dtype=float)
        self._arm_controller.applied_torque[:] = np.asarray(state.applied_torque, dtype=float)
        self._arm_controller.outer_torque[:] = np.asarray(state.pos_vel_outer_torque, dtype=float)
        self._control_phase = int(state.control_phase) % self._control_steps_per_update
        self._control_mode = state.control_mode
        self._raw_torque_command[:] = np.asarray(state.raw_torque_command, dtype=float)
        self._raw_torque_requested[:] = np.asarray(state.raw_torque_requested, dtype=float)
        self._raw_torque_deadline = state.raw_torque_deadline
        self._gripper_max_force_n = state.gripper_max_force_n
        self._applied_arm_torque[:] = restored_torque
        if self._control_mode in ("raw_torque", "joint_impedance"):
            self._requested_arm_torque[:] = (
                state.impedance_requested_torques
                if self._control_mode == "joint_impedance"
                else self._raw_torque_requested
            )
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
        """复制不可变诊断快照;仅 raw_torque 报告剩余看门狗秒数."""
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
            impedance_command=self._impedance.command,
        )

    def set_mode(self, model, data, mode: str) -> str:
        mode = str(mode)
        if mode not in CONTROL_MODES:
            raise ValueError(f"control mode must be one of {CONTROL_MODES}")
        if mode == "joint_impedance":
            raise ValueError("joint_impedance requires command_joint_impedance")
        if self._control_mode == "joint_impedance":
            if mode != "hold":
                raise RuntimeError("joint impedance active; enter hold before changing mode")
            self._leave_impedance(model, data)
            return self._control_mode
        # 键盘重复 hold 时沿用原目标与积分,反复捕获当前位置会引入目标和支撑力矩跳变.
        if mode == "hold" and self._control_mode == "hold":
            return self._control_mode
        if self._impedance.handoff and mode in ("position", "hold"):
            if mode == "hold":
                self._sync_arm_targets_to_current_position(model, data)
            self._control_mode = mode
            return mode
        self._impedance.clear_handoff()
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
        # position 与 hold 共用位置控制器;position→hold 保留积分和上一拍支撑力矩.
        # 不能在静止保持时清零后等待 PI 重建重力支撑;其余模式的既有复位规则保留.
        if mode == "gravity_comp" or (mode == "hold" and self._control_mode != "position"):
            self._arm_controller.reset()
        self._control_mode = mode
        self._apply_motor_control(model, data)
        return self._control_mode

    def command_joint_positions(
        self, model, data, targets: Mapping[str, float] | Sequence[float]
    ) -> tuple[float, ...]:
        if self._control_mode == "joint_impedance":
            raise RuntimeError("joint impedance active; enter hold before position commands")
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

        # 整批通过后才修改目标;越界不得截断后伪报成功或留下部分更新.
        for index, joint_id in enumerate(self._joint_ids[:6]):
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            if not lower <= current[index] <= upper:
                raise ValueError(f"joint target outside limits: {ARM_JOINT_NAMES[index]}")
        self._position_targets[:6] = current
        # 高频参考只更新目标,PI 的积分与力矩更新仍由配置控制频率推进.
        # 每次写目标都调用 set_mode 会额外执行控制计算,使高频轨迹改变有效增益.
        if self._control_mode != "position":
            self.set_mode(model, data, "position")
        return tuple(current)

    def command_joint_torques(
        self, model, data, torques: Sequence[float], timeout_s: float = 0.1
    ) -> tuple[float, ...]:
        if self._control_mode == "joint_impedance":
            raise RuntimeError("joint impedance active; enter hold before torque commands")
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
        self._impedance.clear_handoff()
        self._arm_controller.reset()
        self._control_mode = "raw_torque"
        self._apply_motor_control(model, data)
        return tuple(float(value) for value in self._raw_torque_command)

    def command_joint_impedance(
        self,
        model,
        data,
        targets,
        stiffness,
        damping,
        *,
        allow_from_position: bool = False,
    ) -> tuple[float, ...]:
        """登记静态平衡点与增益;热更新保持控制相位和上一拍实际力矩."""
        command = JointImpedanceCommand(targets, stiffness, damping)
        allowed_modes = (
            ("hold", "joint_impedance", "position")
            if allow_from_position
            else ("hold", "joint_impedance")
        )
        if self._control_mode not in allowed_modes:
            raise RuntimeError("enter hold before starting joint impedance")
        self._validate_impedance_feedback(data)
        qpos_addresses, qvel_addresses = self._arm_addresses(model)
        accepted = self._impedance.accept(
            command,
            now=float(data.time),
            position_limits=self.impedance_position_limits(model),
            position=data.qpos[qpos_addresses],
            velocity=data.qvel[qvel_addresses],
            gravity=self._gravity_torque(qvel_addresses, data),
            effort_limits=self.impedance_effort_limits(model),
            rate_limits=self._motor_parameters.arm.torque_rate_limit_nm_s,
            previous_torque=self._applied_arm_torque,
            nominal_dt=1.0 / self._motor_parameters.control_rate_hz,
            torque_scale=self._randomization_torque_scale,
            settings=self._impedance_settings,
        )
        self._position_targets[:6] = accepted
        self._control_mode = "joint_impedance"
        return accepted

    def command_gripper_width(
        self, model, data, width_m: float, max_force_n: float | None = None
    ) -> float:
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

    def command_gripper_position(self, model, data, width, torque_cap):
        """独立夹爪开口命令,电机限矩按冻结传动参数折算;时间生命周期由调用者持有."""
        if not math.isfinite(width) or not 0 <= width <= 0.09:
            raise ValueError("gripper opening outside limits")
        p = self._motor_parameters.gripper
        if not math.isfinite(torque_cap) or not 0 < torque_cap <= p.motor_torque_limit_nm:
            raise ValueError("coordinated torque cap outside configured simulation capability")
        force = torque_cap * abs(p.motor_radians_per_opening_m) * p.transmission_efficiency
        return self.command_gripper_width(model, data, width, max_force_n=force)

    def release_gripper_position(self, model, data) -> None:
        """协同位置任务结束后中性释放;保留目标记录而将实际输出归零."""
        self._gripper_max_force_n = 0.0
        self._gripper_control_force.fill(0.0)
        for actuator in self._actuator_ids[-2:]:
            data.ctrl[actuator] = 0.0

    def _apply_motor_control(self, model, data) -> None:
        qpos_addresses, qvel_addresses = self._arm_addresses(model)
        position = np.asarray([data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        gravity = self._gravity_compensation_torque(model, data, qvel_addresses)
        mit_output = None
        if self._control_mode == "joint_impedance":
            mit_output = self._impedance.compute_mit_step(
                position,
                velocity,
                self._gravity_torque(qvel_addresses, data),
                self.impedance_effort_limits(model),
                self._motor_parameters.arm.torque_rate_limit_nm_s,
                self._applied_arm_torque,
                float(data.time),
                previous_requested=self._requested_arm_torque,
                torque_scale=self._randomization_torque_scale,
                settings=self._impedance_settings,
            )
            requested_torque = np.asarray(mit_output.requested_torques, dtype=float)
            arm_torque = np.asarray(mit_output.estimated_torques, dtype=float)
            self._arm_controller.seed_torque(arm_torque)
        elif self._control_mode == "raw_torque":
            requested_torque = self._raw_torque_requested.copy()
            arm_torque = self._raw_torque_command.copy()
            self._arm_controller.seed_torque(arm_torque)
        elif self._control_mode == "gravity_comp":
            requested_torque = gravity
            arm_torque = requested_torque.copy()
            self._arm_controller.seed_torque(arm_torque)
        else:
            # position 与 hold 共用同一双频位置控制器;hold 固定进入时捕获的位置目标.
            requested_motor_torque, arm_torque = self._arm_controller.compute_step(
                target=self._position_targets[:6],
                position=position,
                velocity=velocity,
                dt=1.0 / self._motor_parameters.control_rate_hz,
                feedforward=gravity,
            )
            requested_torque = requested_motor_torque * self._randomization_torque_scale
        requested_torque = np.asarray(requested_torque, dtype=float)
        scaled_torque = np.asarray(arm_torque, dtype=float) * (
            1.0 if self._control_mode == "joint_impedance" else self._randomization_torque_scale
        )
        if self._impedance.handoff:
            scaled_torque = self._impedance.clamp_handoff_torque(
                scaled_torque,
                self.impedance_effort_limits(model),
                self._motor_parameters.arm.torque_rate_limit_nm_s,
                self._applied_arm_torque,
                float(data.time),
            )
        applied_torque = np.asarray(
            [
                self._clamp_actuator_control(model, data, actuator_id, torque)
                for actuator_id, torque in zip(self._actuator_ids[:6], scaled_torque)
            ],
            dtype=float,
        )
        for actuator_id, torque in zip(self._actuator_ids[:6], applied_torque):
            data.ctrl[actuator_id] = torque
        # 执行器写入完成后再提交本拍阻抗/MIT锁存与交接状态.
        self._impedance.commit(
            requested_torques=requested_torque,
            applied_torques=applied_torque,
            now=float(data.time),
            mit_output=mit_output,
        )
        self._requested_arm_torque[:] = requested_torque
        self._applied_arm_torque[:] = applied_torque
        if self._impedance.handoff:
            self._arm_controller.sync_outer_after_handoff(
                applied_torque / self._randomization_torque_scale,
                velocity,
            )
        self._arm_torque_saturated[:] = ~np.isclose(
            requested_torque, applied_torque, atol=1e-12, rtol=0.0
        )

        left_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-2]])])
        right_qpos = float(data.qpos[int(model.jnt_qposadr[self._joint_ids[-1]])])
        left_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-2]])])
        right_qvel = float(data.qvel[int(model.jnt_dofadr[self._joint_ids[-1]])])
        gripper_params = self._motor_parameters.gripper
        target_width = float(
            np.clip(
                float(self._position_targets[-2] - self._position_targets[-1]),
                gripper_params.displacement_min_m,
                gripper_params.displacement_max_m,
            )
        )
        finger_force = self._stable_gripper_force(
            model,
            data,
            target_width=target_width,
            current_width=left_qpos - right_qpos,
            current_velocity=left_qvel - right_qvel,
        )
        if self._gripper_max_force_n is not None:
            finger_force = float(
                np.clip(finger_force, -self._gripper_max_force_n, self._gripper_max_force_n)
            )
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

    def _enter_handoff_hold(self, model, data) -> None:
        """保留上一拍实际支撑力矩切入 hold,下一原定控制拍平滑过渡."""
        self._sync_arm_targets_to_current_position(model, data)
        self._arm_controller.reset()
        _, qvel_addresses = self._arm_addresses(model)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        self._arm_controller.sync_outer_after_handoff(
            self._applied_arm_torque / self._randomization_torque_scale,
            velocity,
        )
        self._control_mode = "hold"

    def _apply_pos_vel_inner_loop(self, model, data) -> None:
        """500 Hz 物理子步内环:委托 PosVelController 按实时速度施加位置控制高频速度阻尼."""
        _, qvel_addresses = self._arm_addresses(model)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        requested_motor, bounded_motor = self._arm_controller.evaluate_inner_torque(velocity)
        requested_torque = requested_motor * self._randomization_torque_scale
        scaled_torque = bounded_motor * self._randomization_torque_scale
        applied_torque = np.asarray(
            [
                self._clamp_actuator_control(model, data, actuator_id, torque)
                for actuator_id, torque in zip(self._actuator_ids[:6], scaled_torque)
            ],
            dtype=float,
        )
        for actuator_id, torque in zip(self._actuator_ids[:6], applied_torque):
            data.ctrl[actuator_id] = torque
        self._requested_arm_torque[:] = requested_torque
        self._applied_arm_torque[:] = applied_torque
        self._arm_torque_saturated[:] = ~np.isclose(
            requested_torque, applied_torque, atol=1e-12, rtol=0.0
        )

    def _apply_mit_inner_loop(self, model, data) -> None:
        """500 Hz 物理子步内环:沿用最近 100 Hz 锁存的 MIT 参数与当前 (q, qd) 实时求值."""
        mit = self._impedance.latched_mit
        if mit is None:
            return
        qpos_addresses, qvel_addresses = self._arm_addresses(model)
        position = np.asarray([data.qpos[address] for address in qpos_addresses], dtype=float)
        velocity = np.asarray([data.qvel[address] for address in qvel_addresses], dtype=float)
        effort_limits = np.asarray(self.impedance_effort_limits(model), dtype=float)
        raw_torque, bounded_torque = evaluate_mit_torque(
            mit.target,
            position,
            velocity,
            mit.kp,
            mit.kd,
            mit.feedforward_torques,
            effort_limits,
        )
        applied_torque = np.asarray(
            [
                self._clamp_actuator_control(model, data, actuator_id, torque)
                for actuator_id, torque in zip(self._actuator_ids[:6], bounded_torque)
            ],
            dtype=float,
        )
        for actuator_id, torque in zip(self._actuator_ids[:6], applied_torque):
            data.ctrl[actuator_id] = torque
        self._applied_arm_torque[:] = applied_torque
        self._arm_controller.seed_torque(applied_torque)
        self._arm_torque_saturated[:] = np.asarray(mit.saturated, dtype=bool) | ~np.isclose(
            raw_torque, applied_torque, atol=1e-12, rtol=0.0
        )

    def _leave_impedance(self, model, data) -> None:
        self._impedance.leave()
        self._enter_handoff_hold(model, data)

    def _raw_torque_watchdog_expired(self, model, data) -> bool:
        return (
            self._control_mode == "raw_torque"
            and self._raw_torque_deadline is not None
            and float(data.time) >= self._raw_torque_deadline
        )

    def _stable_gripper_force(
        self,
        model,
        data,
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
        # MuJoCo 手指执行器直接施加 N;电机限矩先换算为线性力上限,滑动关节以米制 PD 稳定.
        return gripper.sim_force_kp_n_per_m * error - gripper.sim_force_kd_n_s_per_m * velocity

    def _clamp_actuator_control(self, model, data, actuator_id: int, value: float) -> float:
        if int(model.actuator_ctrllimited[actuator_id]):
            lower, upper = (float(v) for v in model.actuator_ctrlrange[actuator_id])
            return min(max(float(value), lower), upper)
        return float(value)

    def _seed_arm_torque_from_gravity(self, model, data) -> None:
        qvel_addresses = [int(model.jnt_dofadr[joint_id]) for joint_id in self._joint_ids[:6]]
        self._arm_controller.seed_torque(
            self._gravity_compensation_torque(model, data, qvel_addresses)
        )

    def _sync_arm_targets_to_current_position(self, model, data) -> None:
        for index, joint_id in enumerate(self._joint_ids[:6]):
            self._position_targets[index] = float(data.qpos[int(model.jnt_qposadr[joint_id])])

    def _gravity_compensation_torque(
        self, model, data, qvel_addresses: Sequence[int]
    ) -> np.ndarray:
        scale = float(self._motor_parameters.arm.gravity_compensation_scale)
        if scale == 0.0:
            return np.zeros(len(ARM_JOINT_NAMES), dtype=float)
        return (
            np.asarray([data.qfrc_bias[address] for address in qvel_addresses], dtype=float)
            * scale
        )
