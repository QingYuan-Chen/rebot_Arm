"""视觉伺服逼近的单步限幅策略。

闭环逼近时每轮只允许末端移动一小段（``max_step_m``），避免一次跳变过大导致
碰撞或超出控制器跟随能力；当位置误差进入容差带即判定到位。本模块只算「下一
步该去哪、误差多大」，不做规划、不下发指令。

单位统一为米，坐标系与传入位姿一致（由调用方保证）。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from ..visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class VisualServoApproachConfig:
    """伺服逼近参数；默认值对应约 20 mm 的单步上限与 8 mm 的到位容差。"""

    # 单步最大位移，单位米。调大逼近更快但更易过冲或碰撞；<= 0 表示不限制
    # 步长（直接以期望位姿为目标）。
    max_step_m: float = 0.02
    # 到位判据，单位米：当前位置与期望位置的距离小于等于它即认为到达。
    # 调小精度更高，但可能因控制器跟随误差而反复迭代不收敛。
    position_tolerance_m: float = 0.008


@dataclass(frozen=True)
class VisualServoStep:
    """单步结果：``target`` 是本轮应下发的位置目标。"""

    # 本轮要去的位姿（误差已进容差时就是期望位姿本身）。
    target: PoseTarget
    # 限幅前的原始位置误差，单位米；供上层判断收敛趋势并记录诊断。
    error_m: float
    # 是否已进入容差带。
    reached: bool


def build_visual_servo_step(
    current: PoseTarget,
    desired: PoseTarget,
    config: VisualServoApproachConfig,
) -> VisualServoStep:
    """在当前位置与期望位置之间取一个有界的中间目标。

    计算流程：
        1. 求「当前 → 期望」的位移向量并取欧氏范数作为误差；
        2. 误差 <= 容差：直接返回期望位姿并标记 ``reached=True``；
        3. 否则按 ``max_step_m`` 限幅：缩放系数 = 步长 / 误差，沿位移方向插值
           出中间点；姿态始终取期望姿态（不做姿态插值），保持末端朝向一致；
        4. 步长 <= 0 或误差本身不超过步长时，直接以期望位姿为目标，但
           ``reached=False``（误差尚未进容差）。

    返回的 ``error_m`` 始终是限幅前的真实误差，因此上层看到的收敛曲线不受
    步长限制影响。
    """

    dx = float(desired.position[0]) - float(current.position[0])
    dy = float(desired.position[1]) - float(current.position[1])
    dz = float(desired.position[2]) - float(current.position[2])
    error_m = math.sqrt(dx * dx + dy * dy + dz * dz)
    tolerance = max(0.0, float(config.position_tolerance_m))
    if error_m <= tolerance:
        return VisualServoStep(target=desired, error_m=error_m, reached=True)

    max_step = max(0.0, float(config.max_step_m))
    if max_step <= 0.0 or error_m <= max_step:
        return VisualServoStep(target=desired, error_m=error_m, reached=False)

    scale = max_step / error_m
    target = PoseTarget(
        position=(
            float(current.position[0]) + dx * scale,
            float(current.position[1]) + dy * scale,
            float(current.position[2]) + dz * scale,
        ),
        orientation=desired.orientation,
    )
    return VisualServoStep(target=target, error_m=error_m, reached=False)
