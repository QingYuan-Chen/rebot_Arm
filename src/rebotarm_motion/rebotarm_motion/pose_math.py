"""姿态角的四元数 ↔ RPY 互转（纯数学工具，无中间件依赖）。

约定
    采用 ZYX 欧拉角（等价于内旋 XYZ），即旋转矩阵 ``R = Rz(yaw) · Ry(pitch) · Rx(roll)``：
    先绕 X 轴转 roll，再绕 Y 轴转 pitch，最后绕 Z 轴转 yaw。这与位姿消息里四元数的常用解释
    一致，因此可以直接拿来做界面显示与目标位姿换算。

用途
    正向（四元数 → RPY）用于把求解器/规划结果里的姿态四元数转成操作者可读的三个角；
    反向（RPY → 四元数）用于把界面或参数给出的 RPY 目标转成位姿消息所需的四元数。

注意
    两个函数都按"输入已是单位四元数/标准欧拉角"处理，不做归一化、不做范围截断；角度单位
    一律是弧度（rad）。
"""

from __future__ import annotations

import math


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """单位四元数 ``(x, y, z, w)`` → 欧拉角 ``(roll, pitch, yaw)``，单位 rad。

    分量顺序是 ``x, y, z, w``（实部 w 在最后），与位姿消息中的字段顺序一致。返回的 roll 与
    yaw 来自 ``atan2``，取值在 ``[-π, π]``；pitch 来自 ``asin``，取值在 ``[-π/2, π/2]``。
    输入未归一化时结果会有偏差，本函数不做检查。
    """
    # roll：atan2 的分子/分母对应旋转矩阵元素 R[2,1] 与 R[1,1]。
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch：sin(pitch) 即 R[0,2]，可直接反解。
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        # 万向锁分支：pitch 达到 ±π/2。此时 asin 会因浮点误差得到的 |sinp| 略大于 1 而抛
        # ValueError，故按符号直接取 ±π/2；该姿态下 roll 与 yaw 不再唯一（只有和/差可确定）。
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw：与 roll 同构，分子/分母对应 R[1,0] 与 R[0,0]。
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """欧拉角 ``(roll, pitch, yaw)``（rad）→ 单位四元数 ``(x, y, z, w)``。

    与 :func:`quaternion_to_rpy` 使用同一 ZYX 约定，两者互为逆运算（万向锁附近 pitch=±π/2
    时角度不唯一，往返只保证旋转等价而非数值相等）。返回四元数模长为 1，分量顺序是
    ``x, y, z, w``。
    """
    # 半角三角函数各算一次：四元数分量由 roll/pitch/yaw 的 1/2 角组合而成。
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # 下面四项是 ZYX 顺序四元数乘法的展开结果（w 为实部，x/y/z 为虚部）。
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w
