"""抓取候选的关节运动代价与可行性评估（纯函数，无 ROS 依赖）。

在系统中的位置：抓取候选 IK 过滤器在拿到 IK 解之后调用本模块，把"机械臂要动多少"
量化为一个可比较的代价（penalty），从而在同一帧的多个候选/多种姿态变体之间排序。
代价越小越优先——它不改变几何可达性判断，只影响排序，以及对 joint6 大幅翻转的硬性否决。

安全约束：joint6（末轴）大角度翻转会让夹爪绕着物体甩过去，既可能撞到周围障碍，
也会让线缆/夹爪姿态突变。因此当 joint6 的单关节转角超过阈值时，直接判为不可接受
（accepted=False），而不仅仅是加一个较大的罚分。

单位约定：所有角度为弧度；penalty 为无量纲加权和。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence


def joint_positions_by_name(names: Sequence[str], positions: Sequence[float]) -> dict[str, float]:
    """规范化关节采样数据，不依赖 sensor_msgs 或 ROS 节点。"""

    result: dict[str, float] = {}
    for name, position in zip(names, positions):
        value = float(position)
        if math.isfinite(value):
            result[str(name)] = value
    return result


def angle_delta(target_rad: float, current_rad: float) -> float:
    """求两个角度之间的最短有向角差，结果落在 (-pi, pi]。

    直接用 target - current 在跨 ±pi 时会得到约 2pi 的假大值（例如从 +3.1 转到 -3.1
    实际只差 0.08 rad）。这里借助 atan2(sin d, cos d) 把差值归一化到最短路径。
    """
    return math.atan2(math.sin(float(target_rad) - float(current_rad)), math.cos(float(target_rad) - float(current_rad)))


@dataclass(frozen=True)
class JointMotionPolicyConfig:
    """关节运动代价的权重配置。

    joint_distance_weight: 全体关节角差二范数的权重，默认 0.15。
        调大后更偏好"整体动作幅度小"的方案，运动更省时、更少扰动周边。
    joint6_weight: joint6（末轴）角差的额外权重，默认 0.35。
        调大后更强烈地避免末轴旋转。
    max_joint6_delta_rad: joint6 允许的最大单关节角差（弧度），默认 1.5708（约 90°）。
        超过即直接否决该候选；设为 <= 0 表示关闭这项否决（见 evaluate_joint_motion 中的判断）。
    """

    joint_distance_weight: float = 0.15
    joint6_weight: float = 0.35
    max_joint6_delta_rad: float = 1.5708


@dataclass(frozen=True)
class JointMotionEvaluation:
    """评估结果。

    accepted: 是否接受该目标关节位置。
    penalty: 加权运动代价（越小越好）；被否决时恒为 0.0，调用方不应使用该值参与排序。
    reason: 人类可读的度量描述，形如 "joint_distance=0.123, joint6_delta=0.456"，
        会被原样带入日志，因此保持英文格式。
    joint_distance: 所有可比关节角差的二范数（弧度）。
    joint6_delta: joint6 的绝对角差（弧度），不存在 joint6 时为 0.0。
    """

    accepted: bool
    penalty: float
    reason: str
    joint_distance: float
    joint6_delta: float


def evaluate_joint_motion(
    *,
    current_positions: dict[str, float],
    target_positions: dict[str, float],
    config: JointMotionPolicyConfig = JointMotionPolicyConfig(),
) -> JointMotionEvaluation:
    """评估从当前关节位置运动到目标位置的代价。

    参数:
        current_positions: 当前关节名 -> 弧度。
        target_positions: IK 解给出的关节名 -> 弧度。
        config: 权重与 joint6 阈值配置。
    返回:
        JointMotionEvaluation。两侧没有共同的 jointN 名称时视为"信息不足"，
        返回 accepted=True、penalty=0.0 的宽容结果（不因缺数据而误杀候选）。
    副作用: 无。异常: 无（无法转 float 的输入由 float() 抛出）。
    """
    # 只比较两侧都有、且以 "joint" 开头的关节：排除夹爪关节等非机械臂自由度
    common_names = [name for name in current_positions if name in target_positions and name.startswith("joint")]
    if not common_names:
        return JointMotionEvaluation(
            accepted=True,
            penalty=0.0,
            reason="joint_delta=unknown",
            joint_distance=0.0,
            joint6_delta=0.0,
        )

    # 用最短角差而非原始差值：避免 ±pi 附近的绕圈被算成大位移
    deltas = {
        name: abs(angle_delta(float(target_positions[name]), float(current_positions[name])))
        for name in common_names
    }
    # 二范数：对"多关节同时大角度"的惩罚强于单关节大角度，符合关节空间距离的直觉
    joint_distance = math.sqrt(sum(value * value for value in deltas.values()))
    joint6_delta = float(deltas.get("joint6", 0.0))
    max_joint6_delta = float(config.max_joint6_delta_rad)
    reason = f"joint_distance={joint_distance:.3f}, joint6_delta={joint6_delta:.3f}"
    # 阈值 <= 0 视为关闭该项检查（便于标定/调试时恢复所有候选）
    if max_joint6_delta > 0.0 and joint6_delta > max_joint6_delta:
        return JointMotionEvaluation(
            accepted=False,
            penalty=0.0,
            reason=reason,
            joint_distance=joint_distance,
            joint6_delta=joint6_delta,
        )

    penalty = float(config.joint_distance_weight) * joint_distance + float(config.joint6_weight) * joint6_delta
    return JointMotionEvaluation(
        accepted=True,
        penalty=penalty,
        reason=reason,
        joint_distance=joint_distance,
        joint6_delta=joint6_delta,
    )
