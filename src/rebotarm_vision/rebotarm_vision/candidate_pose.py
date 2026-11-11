"""候选筛选共用的末端目标位姿值."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PoseTarget:
    """末端 TCP 目标位姿.

    position 为 (x, y, z),单位 m;orientation 为四元数 (x, y, z, w),单位无.
    使用前由调用方保证四元数已归一化.
    """

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
