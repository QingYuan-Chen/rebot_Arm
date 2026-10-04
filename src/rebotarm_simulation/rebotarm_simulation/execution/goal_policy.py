"""execution goal_policy components; preserve existing execution contracts."""
from __future__ import annotations
import math
from dataclasses import dataclass
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES


@dataclass(frozen=True)
class GoalSettlingPolicy:
    """节点级"到位判定"策略：轨迹走完后判断仿真是否已稳定停住。

    三个阈值都是不可被单次目标请求覆盖的节点级默认值（构造节点时由参数注入）。
    只做判定，不发命令；`evaluate` 是纯函数，便于离线测试。
    """

    # 各关节位置误差上限，单位 rad（取六轴中的最大绝对误差）。
    position_tolerance: float = 0.02
    # 各关节速度上限，单位 rad/s（取六轴中的最大绝对速度）。
    velocity_tolerance: float = 0.05
    # 到达轨迹终点后允许的等待时间，单位 s；超时仍未满足上面两个阈值即判超时。
    time_tolerance_sec: float = 5.0

    def __post_init__(self) -> None:
        # 阈值必须为正的有限值：0 或负数会让"到位"判定永不可能满足，使目标
        # 只能等到超时，等于人为制造执行失败。
        values = (self.position_tolerance, self.velocity_tolerance, self.time_tolerance_sec)
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
            raise ValueError("goal tolerances must be positive finite values")

    def evaluate(self, desired, actual, velocities, settle_elapsed: float) -> str:
        """判定当前是否到位。

        参数：
            desired: 目标关节角，单位 rad，必须是 6 个手臂关节值；
            actual: 当前关节角，单位 rad，长度同上；
            velocities: 当前关节角速度，单位 rad/s，长度同上；
            settle_elapsed: 轨迹走完后已等待的时间，单位 s，必须非负。

        返回三态字符串（调用方据此决定继续等待/成功/超时）：
            "succeeded" —— 最大位置误差与最大速度都在阈值内；
            "timed_out" —— 等待时间已达 time_tolerance_sec 仍未满足阈值；
            "settling"  —— 仍在容差内等待。
        """
        vectors = (tuple(desired), tuple(actual), tuple(velocities))
        if any(len(vector) != len(ARM_JOINT_NAMES) for vector in vectors):
            raise ValueError("goal state must contain six arm values")
        numeric = tuple(tuple(float(value) for value in vector) for vector in vectors)
        elapsed = float(settle_elapsed)
        if any(not math.isfinite(value) for vector in numeric for value in vector) or not math.isfinite(elapsed):
            raise ValueError("goal state must be finite")
        if elapsed < 0.0:
            raise ValueError("settling time must be non-negative")
        # 位置与速度都用"最差关节"（各轴绝对值取最大）作为判据，任一轴不达标即未稳定。
        position_error = max(abs(target - reached) for target, reached in zip(numeric[0], numeric[1]))
        max_velocity = max(abs(value) for value in numeric[2])
        # 先判成功再判超时：恰好同时满足时按成功处理，避免边界抖动导致误报超时。
        if position_error <= self.position_tolerance and max_velocity <= self.velocity_tolerance:
            return "succeeded"
        if elapsed >= self.time_tolerance_sec:
            return "timed_out"
        return "settling"
