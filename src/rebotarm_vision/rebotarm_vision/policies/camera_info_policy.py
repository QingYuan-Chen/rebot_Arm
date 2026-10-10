"""校验 RGB-D 抓取所需的真实相机内参（不依赖 ROS 节点）。"""

from __future__ import annotations

import math
from collections.abc import Mapping


def calibrated_intrinsics(info: Mapping[str, object] | None, *, width: int, height: int) -> bool:
    """只有尺寸一致、焦距有限且为正的驱动内参才可用于抓取。"""

    if info is None or width <= 0 or height <= 0:
        return False
    try:
        if int(info["width"]) != width or int(info["height"]) != height:
            return False
        fx, fy = float(info["fx"]), float(info["fy"])
        cx, cy = float(info["cx"]), float(info["cy"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return all(math.isfinite(value) for value in (fx, fy, cx, cy)) and fx > 0 and fy > 0
