"""后端无关的六轴关节阻抗两阶段提交状态机.

职责与位置
    集中维护关节阻抗运行期会话状态:平衡点准入与热更新,反馈单调校验,
    退出交接限速,纯力矩/MIT 两阶段计算与锁存确认后提交,以及快照导出与一致性校验.

对外接口
    JointImpedanceSession: 状态机类.

安全与并发约束
    - 零 `rclpy`, `mujoco`, `motorbridge`, `threading` 依赖,由调用方串行持有;
    - `accept` 与同拍重入不推进 `last_control_time`,热更新不消耗变化率预算;
    - 阶段 1 (`compute_torque_step` / `compute_mit_step` / `clamp_handoff_torque`) 保持只读,
      仅在后端实际写入或发送成功后调用阶段 2 (`commit`) 提交输出与 MIT 内环锁存.

文件结构(按定义顺序)
    JointImpedanceSession(状态属性 → 准入/退出/复位 → 两阶段计算与提交 → 快照保存与恢复).
"""

from __future__ import annotations

from dataclasses import replace
import math
from typing import Sequence

import numpy as np

from .contracts import JOINT_NAMES, finite_scalar, six_vector, validate_position_limits
from .joint_impedance import (
    ImpedanceMitOutput,
    ImpedanceOutput,
    JointImpedanceCommand,
    compute_joint_impedance,
    scale_impedance_command,
)
from .joint_impedance_settings import JointImpedanceSettings
from .mit_command import PROTOCOL_KD_MAX, PROTOCOL_KP_MAX, bounded_mit_command, evaluate_mit_torque


class JointImpedanceSession:
    """后端无关的关节阻抗状态机,协调准入,热更新,交接与两阶段提交."""

    def __init__(self) -> None:
        self._command: JointImpedanceCommand | None = None
        self._handoff: bool = False
        self._last_control_time: float = 0.0
        self._committed_requested_torques: tuple[float, ...] = (0.0,) * 6
        self._latched_mit: ImpedanceMitOutput | None = None

    @property
    def active(self) -> bool:
        return self._command is not None

    @property
    def command(self) -> JointImpedanceCommand | None:
        return self._command

    @property
    def handoff(self) -> bool:
        return self._handoff

    @property
    def last_control_time(self) -> float:
        return self._last_control_time

    @property
    def last_requested_torques(self) -> tuple[float, ...]:
        return self._committed_requested_torques if self._command is not None else ()

    @property
    def latched_mit(self) -> ImpedanceMitOutput | None:
        return self._latched_mit if self._command is not None else None

    @staticmethod
    def validate_target(
        command: JointImpedanceCommand,
        position_limits: Sequence[tuple[float, float]],
    ) -> None:
        """整批校验六轴静态平衡角是否处于物理限位内."""
        if not isinstance(command, JointImpedanceCommand):
            raise TypeError("command must be JointImpedanceCommand")
        bounds = validate_position_limits(position_limits)
        for name, (lower, upper), target in zip(JOINT_NAMES, bounds, command.target):
            if not lower <= target <= upper:
                raise ValueError(f"impedance target outside limits: {name}")

    def validate_feedback(self, now: float, *arrays) -> float:
        """核对时间单调递增且输入反馈全部有限,阻止坏帧在计算前污染状态."""
        if isinstance(now, bool):
            raise ValueError("joint impedance feedback and time must be finite")
        current = float(now)
        if not math.isfinite(current) or not all(
            np.isfinite(np.asarray(item, dtype=float)).all() for item in arrays
        ):
            raise ValueError("joint impedance feedback and time must be finite")
        if current < self._last_control_time:
            raise ValueError("joint impedance simulation time moved backwards")
        return current

    def accept(
        self,
        command: JointImpedanceCommand,
        *,
        now: float,
        position_limits: Sequence[tuple[float, float]],
        position,
        velocity,
        gravity,
        effort_limits,
        rate_limits,
        previous_torque,
        nominal_dt: float,
        torque_scale: float = 1.0,
        settings: JointImpedanceSettings | None = None,
    ) -> tuple[float, ...]:
        """原子校验并接纳/热更新静态平衡点与增益;任何检查失败均保留原状态不变."""
        self.validate_target(command, position_limits)
        self.validate_feedback(now, position, velocity, gravity)
        if settings is not None:
            settings.validate_command(command)
        effort = six_vector(effort_limits, "effort limits")
        previous = six_vector(previous_torque, "previous torque")
        if (effort <= 0).any():
            raise ValueError("effort limits must be positive")
        if (np.abs(previous) > effort).any():
            raise ValueError("previous torque exceeds impedance effort limits")
        # 接纳前无副作用试算一拍,尽早暴露增益过大引起的浮点溢出.
        scale = finite_scalar(torque_scale, "torque_scale", allow_zero=True)
        compute_joint_impedance(
            scale_impedance_command(command, scale),
            position,
            velocity,
            np.asarray(gravity, dtype=float) * scale,
            effort,
            rate_limits,
            previous,
            nominal_dt,
        )
        self._command = replace(command)
        self._handoff = False
        return self._command.target

    def leave(self) -> None:
        """退出阻抗活动态并开启下一拍平滑交接标志,清除锁存 MIT 输出,保留历史力矩与时刻."""
        self._command = None
        self._latched_mit = None
        self._handoff = True

    def clear_handoff(self) -> None:
        self._handoff = False

    def reset(self, now: float = 0.0) -> None:
        """清空命令,锁存 MIT 输出,交接标志与历史请求,并重设控制时刻基准."""
        current = finite_scalar(now, "now", allow_zero=True)
        self._command = None
        self._latched_mit = None
        self._handoff = False
        self._committed_requested_torques = (0.0,) * 6
        self._last_control_time = current

    def compute_torque_step(
        self,
        position,
        velocity,
        gravity,
        effort_limits,
        rate_limits,
        previous_torque,
        now: float,
        *,
        previous_requested=None,
        torque_scale: float = 1.0,
    ) -> ImpedanceOutput:
        """阶段 1(纯力矩路径):计算本拍目标力矩与饱和标志,不提交内部状态."""
        if self._command is None:
            raise RuntimeError("joint impedance is not active")
        current = self.validate_feedback(now, position, velocity, gravity)
        scale = finite_scalar(torque_scale, "torque_scale", allow_zero=True)
        elapsed = current - self._last_control_time
        if elapsed <= 0.0:
            # 同一物理时刻首次激活或重入计算保持已有输出,避免消耗变化率预算.
            applied = tuple(float(v) for v in six_vector(previous_torque, "previous torque"))
            if previous_requested is not None:
                requested = tuple(float(v) for v in six_vector(previous_requested, "previous requested"))
            else:
                requested = self._committed_requested_torques
            saturated = tuple(
                not math.isclose(r, a, abs_tol=1e-12, rel_tol=0.0)
                for r, a in zip(requested, applied)
            )
            return ImpedanceOutput(requested, applied, saturated)
        return compute_joint_impedance(
            scale_impedance_command(self._command, scale),
            position,
            velocity,
            np.asarray(gravity, dtype=float) * scale,
            effort_limits,
            rate_limits,
            previous_torque,
            elapsed,
        )

    def compute_mit_step(
        self,
        position,
        velocity,
        gravity,
        effort_limits,
        rate_limits,
        previous_torque,
        now: float,
        *,
        dt_override: float | None = None,
        previous_requested=None,
        torque_scale: float = 1.0,
        settings: JointImpedanceSettings | None = None,
    ) -> ImpedanceMitOutput:
        """阶段 1(MIT 内环路径):计算有界 Kp/Kd/tau_ff 与估算总力矩,不提交内部状态."""
        if self._command is None:
            raise RuntimeError("joint impedance is not active")
        for index, name in enumerate(JOINT_NAMES):
            if self._command.stiffness[index] > PROTOCOL_KP_MAX:
                raise ValueError(f"{name}: stiffness exceeds MIT protocol limit {PROTOCOL_KP_MAX:.0f}")
            if self._command.damping[index] > PROTOCOL_KD_MAX + 1e-9:
                raise ValueError(f"{name}: damping exceeds MIT protocol limit {PROTOCOL_KD_MAX:.1f}")
        if settings is not None:
            settings.validate_command(self._command)
        scale = finite_scalar(torque_scale, "torque_scale", allow_zero=True)
        current = self.validate_feedback(now, position, velocity, gravity)
        elapsed = (
            current - self._last_control_time
            if dt_override is None
            else finite_scalar(dt_override, "dt")
        )
        effort = six_vector(effort_limits, "effort limits")
        rates = six_vector(rate_limits, "rate limits")
        prev = six_vector(previous_torque, "previous torque")
        if (effort <= 0).any() or (rates <= 0).any():
            raise ValueError("effort limits and rate limits must be positive")
        if (np.abs(prev) > effort + 1e-9).any():
            raise ValueError("previous torque exceeds effort limits")
        if dt_override is None and elapsed <= 0.0:
            # 同一物理时刻首次激活或重入计算不消耗变化率预算,以零增益前馈精确维持上一实际输出.
            applied = tuple(float(v) for v in prev)
            if previous_requested is not None:
                requested_seq = tuple(
                    float(v) for v in six_vector(previous_requested, "previous requested")
                )
            else:
                requested_seq = self._committed_requested_torques
            saturated = tuple(
                not math.isclose(r, a, abs_tol=1e-12, rel_tol=0.0)
                for r, a in zip(requested_seq, applied)
            )
            return ImpedanceMitOutput(
                target=self._command.target,
                velocity=(0.0,) * 6,
                kp=(0.0,) * 6,
                kd=(0.0,) * 6,
                feedforward_torques=applied,
                estimated_torques=applied,
                requested_torques=requested_seq,
                saturated=saturated,
            )
        q = six_vector(position, "position")
        qd = six_vector(velocity, "velocity")
        g = six_vector(gravity, "gravity") * scale
        scaled = scale_impedance_command(self._command, scale)
        try:
            with np.errstate(over="raise", invalid="raise"):
                requested = g + np.asarray(scaled.stiffness) * (scaled.target - q)
                requested -= np.asarray(scaled.damping) * qd
        except FloatingPointError as exc:
            raise ValueError("joint impedance calculation overflowed") from exc
        kp, kd, tau_ff, estimated = bounded_mit_command(
            scaled.target,
            q,
            qd,
            scaled.stiffness,
            scaled.damping,
            g,
            effort,
            rates,
            prev,
            elapsed,
        )
        saturated = tuple(
            not math.isclose(float(r), float(e), abs_tol=1e-12, rel_tol=0.0)
            for r, e in zip(requested, estimated)
        )
        return ImpedanceMitOutput(
            target=scaled.target,
            velocity=(0.0,) * 6,
            kp=tuple(float(v) for v in kp),
            kd=tuple(float(v) for v in kd),
            feedforward_torques=tuple(float(v) for v in tau_ff),
            estimated_torques=tuple(float(v) for v in estimated),
            requested_torques=tuple(float(v) for v in requested),
            saturated=saturated,
        )

    def clamp_handoff_torque(
        self,
        candidate_torque,
        effort_limits,
        rate_limits,
        previous_torque,
        now: float,
    ) -> np.ndarray:
        """退出阻抗后的首拍交接裁剪:同时受总力矩上限与上一拍实际输出变化率约束."""
        current = self.validate_feedback(now, candidate_torque, previous_torque)
        candidate = six_vector(candidate_torque, "candidate torque")
        effort = six_vector(effort_limits, "effort limits")
        rates = six_vector(rate_limits, "rate limits")
        previous = six_vector(previous_torque, "previous torque")
        if (effort <= 0.0).any() or (rates <= 0.0).any():
            raise ValueError("effort limits and rate limits must be positive")
        elapsed = max(0.0, current - self._last_control_time)
        allowance = rates * elapsed
        bounded = np.clip(candidate, -effort, effort)
        return np.clip(bounded, previous - allowance, previous + allowance)

    def commit(
        self,
        *,
        requested_torques,
        applied_torques,
        now: float,
        clear_handoff: bool = False,
        mit_output: ImpedanceMitOutput | None = None,
    ) -> None:
        """阶段 2:在后端确认写入或发送成功后,提交本拍实际结果,控制时刻与 MIT 锁存."""
        current = self.validate_feedback(now, requested_torques, applied_torques)
        requested = tuple(float(v) for v in six_vector(requested_torques, "requested torques"))
        six_vector(applied_torques, "applied torques")
        self._last_control_time = current
        self._committed_requested_torques = requested
        if self._command is not None and mit_output is not None:
            if not isinstance(mit_output, ImpedanceMitOutput):
                raise TypeError("mit_output must be ImpedanceMitOutput")
            self._latched_mit = replace(mit_output)
        elif self._command is None:
            self._latched_mit = None
        if clear_handoff:
            self._handoff = False

    def snapshot(self) -> dict:
        """导出与 SavedSimulationState 对齐的扁平阻抗会话字段."""
        return {
            "impedance_command": self._command,
            "impedance_mit_output": self.latched_mit,
            "impedance_requested_torques": self.last_requested_torques,
            "impedance_handoff": self._handoff,
            "last_control_time": self._last_control_time,
        }

    def validate_snapshot_state(
        self,
        *,
        command: JointImpedanceCommand | None,
        mit_output: ImpedanceMitOutput | None,
        requested_torques: Sequence[float],
        handoff: bool,
        last_control_time: float,
        candidate_time: float,
        candidate_torque,
        position_limits: Sequence[tuple[float, float]],
        position,
        velocity,
        gravity,
        effort_limits,
        rate_limits,
        nominal_dt: float,
        torque_scale: float = 1.0,
        settings: JointImpedanceSettings | None = None,
    ) -> None:
        """在覆写活动状态前核对快照一致性,协议边界与可计算性."""
        last_time = finite_scalar(last_control_time, "last_control_time", allow_zero=True)
        if command is not None:
            self.validate_target(command, position_limits)
            if settings is not None:
                settings.validate_command(command)
        elif mit_output is not None:
            raise ValueError("inactive impedance cannot carry mit output")
        if command is None and not handoff:
            return
        sim_time = finite_scalar(candidate_time, "candidate_time", allow_zero=True)
        if last_time > sim_time:
            raise ValueError("saved control time exceeds simulation time")
        effort = six_vector(effort_limits, "effort limits")
        torque = six_vector(candidate_torque, "candidate torque")
        if np.any(np.abs(torque) > effort):
            raise ValueError("saved impedance torque exceeds effort limits")
        if command is not None:
            six_vector(requested_torques, "impedance_requested_torques")
            scale = finite_scalar(torque_scale, "torque_scale", allow_zero=True)
            compute_joint_impedance(
                scale_impedance_command(command, scale),
                position,
                velocity,
                np.asarray(gravity, dtype=float) * scale,
                effort,
                rate_limits,
                torque,
                nominal_dt,
            )
            if mit_output is not None:
                if not isinstance(mit_output, ImpedanceMitOutput):
                    raise TypeError("mit_output must be ImpedanceMitOutput")
                validated_mit = replace(mit_output)
                evaluate_mit_torque(
                    validated_mit.target,
                    position,
                    velocity,
                    validated_mit.kp,
                    validated_mit.kd,
                    validated_mit.feedforward_torques,
                    effort,
                )

    def restore(
        self,
        *,
        command: JointImpedanceCommand | None,
        mit_output: ImpedanceMitOutput | None,
        requested_torques: Sequence[float],
        handoff: bool,
        last_control_time: float,
        candidate_time: float,
        candidate_torque,
        position_limits: Sequence[tuple[float, float]],
        position,
        velocity,
        gravity,
        effort_limits,
        rate_limits,
        nominal_dt: float,
        torque_scale: float = 1.0,
        settings: JointImpedanceSettings | None = None,
    ) -> None:
        """验证并恢复阻抗会话状态."""
        self.validate_snapshot_state(
            command=command,
            mit_output=mit_output,
            requested_torques=requested_torques,
            handoff=handoff,
            last_control_time=last_control_time,
            candidate_time=candidate_time,
            candidate_torque=candidate_torque,
            position_limits=position_limits,
            position=position,
            velocity=velocity,
            gravity=gravity,
            effort_limits=effort_limits,
            rate_limits=rate_limits,
            nominal_dt=nominal_dt,
            torque_scale=torque_scale,
            settings=settings,
        )
        self._command = replace(command) if command is not None else None
        self._latched_mit = (
            replace(mit_output) if self._command is not None and mit_output is not None else None
        )
        self._handoff = bool(handoff)
        self._last_control_time = float(last_control_time)
        if self._command is not None:
            self._committed_requested_torques = tuple(float(v) for v in requested_torques)
