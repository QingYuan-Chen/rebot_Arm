"""六轴关节阻抗值对象与无状态单拍纯计算控制律.

职责与位置
    计算 `tau = g(q) + K * (q_ref - q) - D * q_dot` (目标速度为零);
    只保存不可变命令/输出数据结构与无状态单拍数学公式,不持有配置边界或运行状态.

对外接口
    JointImpedanceCommand: 静态平衡点及六轴刚度/阻尼值对象.
    ImpedanceOutput / ImpedanceMitOutput: 纯力矩与 MIT 内环单拍输出值对象.
    compute_joint_impedance: 依次限幅总力矩及相对上一实际输出的变化率.
    scale_impedance_command: 按域随机化或标定尺度缩放平衡点刚度/阻尼.

安全与并发约束
    调用方负责模型限位,反馈新鲜度,看门狗期限与并发串行;
    有限输入发生乘法溢出时在限幅前显式抛错,不得静默截断.

文件结构(按定义顺序)
    值对象 → 增益缩放辅助 → compute_joint_impedance.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contracts import finite_scalar, six_vector


@dataclass(frozen=True)
class JointImpedanceCommand:
    """静态平衡点及逐轴刚度/阻尼;复制输入,防止调用方修改已接纳参数."""

    target: tuple[float, ...]
    stiffness: tuple[float, ...]
    damping: tuple[float, ...]

    def __post_init__(self) -> None:
        for name in ("target", "stiffness", "damping"):
            values = six_vector(getattr(self, name), name)
            if name != "target" and (values < 0).any():
                raise ValueError(f"{name} must be nonnegative")
            object.__setattr__(self, name, tuple(float(v) for v in values))


@dataclass(frozen=True)
class ImpedanceOutput:
    """requested 为控制律请求,applied 为已约束输出;限幅与限速均标记 saturated."""

    requested_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    saturated: tuple[bool, ...]


@dataclass(frozen=True)
class ImpedanceMitOutput:
    """MIT 模式两阶段输出:供电机高频内环发送的有界参数与估算总力矩."""

    target: tuple[float, ...]
    velocity: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]
    feedforward_torques: tuple[float, ...]
    estimated_torques: tuple[float, ...]
    requested_torques: tuple[float, ...]
    saturated: tuple[bool, ...]

    def __post_init__(self) -> None:
        for name in (
            "target",
            "velocity",
            "kp",
            "kd",
            "feedforward_torques",
            "estimated_torques",
            "requested_torques",
        ):
            values = six_vector(getattr(self, name), name)
            if name in ("kp", "kd") and (values < 0.0).any():
                raise ValueError(f"{name} must be nonnegative")
            object.__setattr__(self, name, tuple(float(v) for v in values))
        if len(self.saturated) != 6 or any(not isinstance(v, (bool, np.bool_)) for v in self.saturated):
            raise ValueError("saturated must contain exactly 6 booleans")
        object.__setattr__(self, "saturated", tuple(bool(v) for v in self.saturated))


def scale_impedance_command(
    command: JointImpedanceCommand, torque_scale: float
) -> JointImpedanceCommand:
    """按非负标量系数等比例缩放平衡点刚度与阻尼."""
    scale = finite_scalar(torque_scale, "torque_scale", allow_zero=True)
    if scale == 1.0:
        return command
    return JointImpedanceCommand(
        command.target,
        tuple(value * scale for value in command.stiffness),
        tuple(value * scale for value in command.damping),
    )


def compute_joint_impedance(
    command: JointImpedanceCommand,
    position,
    velocity,
    gravity,
    effort_limits,
    rate_limits,
    previous,
    dt: float,
) -> ImpedanceOutput:
    """按当前反馈计算一拍关节力矩,依次限制总幅值及相对实际上一输出的变化率.

    stiffness 单位 N·m/rad,damping 单位 N·m·s/rad;gravity 由调用方按其模型
    计算纯重力项,effort_limits/rate_limits 分别为 N·m 和 N·m/s.
    计算状态完全由调用方保存;发送/执行成功后才能将输出用于下一拍 previous.
    """
    if not isinstance(command, JointImpedanceCommand):
        raise TypeError("command must be JointImpedanceCommand")
    q, qd, g, effort, rate, prev = [
        six_vector(value, label)
        for value, label in (
            (position, "position"),
            (velocity, "velocity"),
            (gravity, "gravity"),
            (effort_limits, "effort limits"),
            (rate_limits, "rate limits"),
            (previous, "previous torque"),
        )
    ]
    step_dt = finite_scalar(dt, "dt")
    if (effort <= 0).any() or (rate <= 0).any():
        raise ValueError("effort limits and rate limits must be positive")
    if (np.abs(prev) > effort).any():
        raise ValueError("previous torque exceeds effort limits")
    # 有限输入仍可能在乘加时溢出;必须在限幅前暴露错误,避免掩盖无效控制律.
    try:
        with np.errstate(over="raise", invalid="raise"):
            requested = g + np.asarray(command.stiffness) * (command.target - q)
            requested -= np.asarray(command.damping) * qd
            allowance = rate * step_dt
            applied = np.clip(
                np.clip(requested, -effort, effort),
                prev - allowance,
                prev + allowance,
            )
    except FloatingPointError as exc:
        raise ValueError("joint impedance calculation overflowed") from exc
    return ImpedanceOutput(
        tuple(float(v) for v in requested),
        tuple(float(v) for v in applied),
        tuple(bool(v) for v in (requested != applied)),
    )
