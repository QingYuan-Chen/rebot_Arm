"""抓取后沿接近路径反向撤退的纯几何策略。

撤退方向由本次抓取的 ``grasp -> pregrasp`` 位移动态推导。闭合夹爪后沿进入
物体的路径反向退出，不额外插入基座 Z 方向的独立抬升动作。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class RetreatPolicyConfig:
    """沿接近路径反向撤退的参数。"""

    enabled: bool = False
    # 从抓取点沿 grasp->pregrasp 方向退出的距离，单位米。
    retreat_distance_m: float = 0.06


def build_retreat_pose(
    grasp: PoseTarget,
    pregrasp: PoseTarget,
    config: RetreatPolicyConfig,
) -> PoseTarget:
    """从抓取点沿接近轴反方向生成撤退位姿，姿态保持抓取姿态。"""

    from ..visual_grasp_sequence import PoseTarget

    direction = tuple(float(pregrasp.position[i]) - float(grasp.position[i]) for i in range(3))
    norm = sum(component * component for component in direction) ** 0.5
    if norm <= 1e-9:
        raise ValueError("pregrasp and grasp must differ to derive retreat direction")
    axis = tuple(component / norm for component in direction)
    distance = max(float(config.retreat_distance_m), 0.0)
    return PoseTarget(
        position=tuple(float(grasp.position[i]) + axis[i] * distance for i in range(3)),
        orientation=grasp.orientation,
    )
