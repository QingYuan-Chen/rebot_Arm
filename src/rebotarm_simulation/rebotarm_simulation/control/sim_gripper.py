"""夹爪开口宽度到手指关节位置的换算（纯函数）。

职责与位置
    仿真侧的夹爪运动学换算：把"两指之间的开口宽度"（米）换算成物理模型里左右手指
    滑移关节的目标位置。本模块供 MuJoCo 物理核心使用；轻量 RViz 预演由独立的 rebotarm_preview 包负责。

几何约定
    模型里两根手指沿各自的 Y 轴反向对称滑移：左指关节量程 0～0.045 m，右指
    -0.045～0 m。因此"开口宽度 = 左指位置 − 右指位置"，每根手指只走一半宽度；
    最大开口 0.09 m 对应两指各 0.045 m。

安全语义
    宽度先夹到合法区间再换算：越界指令既不会让手指冲出面，也不会被静默忽略；
    返回值中的实际生效宽度供调用方上报真实状态，而不是回显请求值。
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
