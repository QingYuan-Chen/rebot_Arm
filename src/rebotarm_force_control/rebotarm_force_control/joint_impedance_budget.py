"""关节阻抗增益力矩预算纯计算与违规检查(离线整定/诊断工具).

职责与位置
    将给定动力学数据(额定力矩,effort 限幅,重力样本,对角惯量)换算为逐轴弹簧/重力/阻尼
    力矩预算与自然频率/阻尼比,并判定是否违背额定,限幅或 MIT 协议边界;不依赖 MuJoCo 运行时.

对外接口
    REFERENCE_DEFLECTION_RAD / DAMPING_RATIO: 整定参考偏角与目标阻尼比常量.
    evaluate_gains: 计算逐轴预算报告.
    budget_violations: 判定预算违规项列表.

安全与并发约束
    本模块为纯离线数值评估工具,不参与控制回路也不持有可变状态;
    输入维度与正定性错误一律显式抛错.

文件结构(按定义顺序)
    基准常量 → evaluate_gains → budget_violations.
"""

from __future__ import annotations

from typing import Mapping

import numpy as np

from .contracts import JOINT_NAMES, finite_scalar, six_vector
from .mit_command import PROTOCOL_KD_MAX, PROTOCOL_KP_MAX


# 关节阻抗整定规则基准:额定力矩对应的参考偏角与目标阻尼比.
REFERENCE_DEFLECTION_RAD = 0.2
DAMPING_RATIO = 0.7


def evaluate_gains(
    stiffness,
    damping,
    rated,
    effort,
    gravity,
    inertia,
    deflection: float = REFERENCE_DEFLECTION_RAD,
) -> dict:
    """按给定增益和动力学数据算出力矩预算;输入维度不符时直接报错.

    rated/effort/gravity/inertia 的顺序必须与 JOINT_NAMES 一致.
    gravity 可以是单个姿态的六轴向量,也可以是 (姿态数, 6) 的矩阵,按轴取最大值.
    """
    stiffness = six_vector(stiffness, "stiffness")
    damping = six_vector(damping, "damping")
    rated = six_vector(rated, "rated torque")
    effort = six_vector(effort, "effort")
    inertia = six_vector(inertia, "inertia")
    gravity = np.asarray(gravity, dtype=float)
    if gravity.ndim == 1:
        gravity = gravity.reshape(1, 6)
    if gravity.ndim != 2 or gravity.shape[1] != 6 or not np.isfinite(gravity).all():
        raise ValueError("gravity must be one or more six-axis rows")
    if (rated <= 0).any() or (effort <= 0).any() or (inertia <= 0).any():
        raise ValueError("rated torque, effort and inertia must be positive")
    deflection = finite_scalar(deflection, "reference deflection")

    worst_gravity = np.abs(gravity).max(axis=0)
    spring = stiffness * deflection
    natural = np.sqrt(stiffness / inertia)
    zeta = damping / (2.0 * np.sqrt(stiffness * inertia))
    gravity_parts = [
        {"joint": name, "gravity_nm": float(value)}
        for name, value in zip(JOINT_NAMES, worst_gravity)
    ]
    return {
        "deflection_rad": float(deflection),
        "stiffness": stiffness.tolist(),
        "damping": damping.tolist(),
        "spring_nm": spring.tolist(),
        "spring_over_rated": (spring / rated).tolist(),
        "spring_plus_gravity_nm": (spring + worst_gravity).tolist(),
        "spring_plus_gravity_over_effort": ((spring + worst_gravity) / effort).tolist(),
        "damping_at_1_rad_s_over_rated": (damping / rated).tolist(),
        "natural_frequency_hz": (natural / (2.0 * np.pi)).tolist(),
        "damping_ratio": zeta.tolist(),
        "worst_gravity_nm": worst_gravity.tolist(),
        "gravity_over_rated": (worst_gravity / rated).tolist(),
        "gravity_per_pose": gravity_parts,
        "violations": [],
    }


def budget_violations(
    report: Mapping,
    rated,
    effort,
    *,
    kp_max: float = PROTOCOL_KP_MAX,
    kd_max: float = PROTOCOL_KD_MAX,
) -> list[str]:
    """列出违反预算边界的具体项;空列表表示全部通过."""
    rated = six_vector(rated, "rated torque")
    effort = six_vector(effort, "effort")
    kp_max = finite_scalar(kp_max, "kp_max")
    kd_max = finite_scalar(kd_max, "kd_max")
    violations = []
    for index, name in enumerate(JOINT_NAMES):
        if report["spring_nm"][index] > rated[index] + 1e-9:
            violations.append(
                f"{name}: spring {report['spring_nm'][index]:.2f} N·m "
                f"exceeds rated {rated[index]:.2f} N·m"
            )
        if report["spring_plus_gravity_nm"][index] > effort[index] + 1e-9:
            violations.append(
                f"{name}: spring+gravity "
                f"{report['spring_plus_gravity_nm'][index]:.2f} N·m "
                f"exceeds effort {effort[index]:.2f} N·m"
            )
        if report["stiffness"][index] > kp_max:
            violations.append(f"{name}: stiffness exceeds MIT protocol limit {kp_max:.0f}")
        if report["damping"][index] > kd_max + 1e-9:
            violations.append(f"{name}: damping exceeds MIT protocol limit {kd_max:.1f}")
    return violations
