"""夹爪开口 → RViz 手指关节位移的纯几何换算。

真实夹爪只有一路开合自由度，而 URDF 里用左右两个反向的移动副手指来表示，
本模块负责在两者之间换算；不涉及 ROS，可在无运行时的环境下单独测试。
"""

from __future__ import annotations


# 历史开口接口映射范围（m）：0 对应关节零位，0.09 对应双侧总行程。
# 单侧关节范围为 left∈[0, 0.045]、right∈[-0.045, 0]。
# 新软指零位仍有间隙，真实净开口待实测标定；不能用此映射宣称物理闭合。
DEFAULT_GRIPPER_LIMITS_M = (0.0, 0.09)


def clamp_gripper_opening(position_m: float, limits_m: tuple[float, float]) -> float:
    """把请求开口夹紧到 ``limits_m`` 范围内，并返回相对下限的开口量（m）。

    先对 (下限, 上限) 排序，容忍配置里上下限写反；返回值减去下限，因此结果恒为
    非负，可直接当作「从闭合位置算起的开口距离」使用。单位：m。
    """
    lower, upper = float(limits_m[0]), float(limits_m[1])
    if upper < lower:
        lower, upper = upper, lower
    value = min(max(float(position_m), lower), upper)
    return value - lower


def gripper_opening_to_finger_joint_positions(
    position_m: float,
    limits_m: tuple[float, float] = DEFAULT_GRIPPER_LIMITS_M,
) -> tuple[float, float]:
    """把夹爪开口换算成 ``(左手指关节位移, 右手指关节位移)``，单位 m。

    两指对中开合、行程对半分：左指取 +开口/2，右指取 -开口/2，与 URDF 中两个
    手指 joint 沿同一轴反向安装、一正一负的限位约定一致；返回顺序与
    ``(left_finger_joint, right_finger_joint)`` 对应。
    """
    half_opening = 0.5 * clamp_gripper_opening(position_m, limits_m)
    return half_opening, -half_opening
