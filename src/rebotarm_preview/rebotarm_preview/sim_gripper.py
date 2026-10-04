"""预演夹爪宽度到 RViz 手指关节位置的显示映射（单位 m）。

本模块只生成内存中的左右对称姿态，不计算力、接触或真实夹爪反馈。
预演和物理后端各自拥有状态映射；本包不导入 MuJoCo 或 simulation 实现。
"""

from __future__ import annotations


def clamp_width(width: float, *, min_width: float, max_width: float) -> float:
    """把宽度夹到 ``[min_width, max_width]`` 区间内（米）。

    先对两端排序，因此即使调用方把上下限传反，也不会得到空区间或错误结果。
    """
    lower = min(float(min_width), float(max_width))
    upper = max(float(min_width), float(max_width))
    return min(max(float(width), lower), upper)


def gripper_joint_positions_for_width(
    width: float,
    *,
    min_width: float = 0.0,
    max_width: float = 0.09,
) -> tuple[float, float, float]:
    """把开口宽度换算为 (左指位置, 右指位置, 实际生效宽度)，单位均为米。

    参数：
        width: 期望开口宽度（m）。
        min_width: 允许的最小开口（m），默认 0.0，即两指完全闭合。
        max_width: 允许的最大开口（m），默认 0.09，即两指完全张开；调小可用于
            模拟机械限位或负载导致的行程缩减。

    返回：
        ``(left, right, reached)``，左右对称：``left = +reached / 2``、
        ``right = -reached / 2``，与模型中两根手指关节的符号约定一致。
        ``reached`` 是夹紧后的实际宽度，请求越界时它不等于入参，调用方应使用它。
    """
    reached_width = clamp_width(width, min_width=min_width, max_width=max_width)
    return reached_width * 0.5, -reached_width * 0.5, reached_width
