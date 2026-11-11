"""六轴关节阻抗纯计算：重力前馈、弹簧阻尼与总力矩约束。

职责：计算 tau = g(q) + K * (q_ref - q) - D * q_dot，目标速度为零。
接口：JointImpedanceCommand 保存不可变参考；compute_joint_impedance 返回力矩快照。
约束：调用方校验模型限位、反馈新鲜度和命令期限，提供上一拍实际输出与实际 dt；
      本模块无设备、ROS、物理引擎或线程，非法输入和非有限计算结果直接报错。
结构：六轴校验 → 命令/输出值 → 单周期计算。单位为 rad、rad/s、N·m、s。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _six(values, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError(f'{label} must contain six finite values')
    return result


@dataclass(frozen=True)
class JointImpedanceCommand:
    """静态平衡点及逐轴刚度/阻尼；复制输入，防止调用方修改已接纳参数。"""

    target: tuple[float, ...]
    stiffness: tuple[float, ...]
    damping: tuple[float, ...]

    def __post_init__(self):
        for name in ('target', 'stiffness', 'damping'):
            values = _six(getattr(self, name), name)
            if name != 'target' and (values < 0).any():
                raise ValueError(f'{name} must be nonnegative')
            object.__setattr__(self, name, tuple(float(v) for v in values))


@dataclass(frozen=True)
class ImpedanceOutput:
    """requested 为控制律请求，applied 为已约束输出；限幅与限速均标记 saturated。"""

    requested_torques: tuple[float, ...]
    applied_torques: tuple[float, ...]
    saturated: tuple[bool, ...]


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
    """按当前反馈计算一拍关节力矩，依次限制总幅值及相对实际上一输出的变化率。

    stiffness 单位 N·m/rad，damping 单位 N·m·s/rad；gravity 由调用方按其模型
    计算纯重力项，effort_limits/rate_limits 分别为 N·m 和 N·m/s。
    计算状态完全由调用方保存；发送/执行成功后才能将输出用于下一拍 previous。
    """
    if not isinstance(command, JointImpedanceCommand):
        raise TypeError('command must be JointImpedanceCommand')
    q, qd, g, effort, rate, previous = [
        _six(value, label) for value, label in (
            (position, 'position'), (velocity, 'velocity'), (gravity, 'gravity'),
            (effort_limits, 'effort limits'), (rate_limits, 'rate limits'),
            (previous, 'previous torque'),
        )
    ]
    dt = float(dt)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be finite and positive')
    if (effort <= 0).any() or (rate <= 0).any():
        raise ValueError('effort limits and rate limits must be positive')
    if (np.abs(previous) > effort).any():
        raise ValueError('previous torque exceeds effort limits')
    # 有限输入仍可能在乘加时溢出；必须在限幅前暴露错误，避免掩盖无效控制律。
    try:
        with np.errstate(over='raise', invalid='raise'):
            requested = g + np.asarray(command.stiffness) * (command.target - q)
            requested -= np.asarray(command.damping) * qd
            allowance = rate * dt
            applied = np.clip(np.clip(requested, -effort, effort),
                              previous - allowance, previous + allowance)
    except FloatingPointError as exc:
        raise ValueError('joint impedance calculation overflowed') from exc
    return ImpedanceOutput(
        tuple(float(v) for v in requested),
        tuple(float(v) for v in applied),
        tuple(bool(v) for v in (requested != applied)),
    )
