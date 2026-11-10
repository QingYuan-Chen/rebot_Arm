"""控制器共享的参数, 单位与兼容常量.

职责与位置
    各控制组件共用的默认数值, 异常类型与无状态力矩缩放.

对外接口
    GravityCommandError 标记发送失败; apply_gravity_compensation_tau_scale 返回副本.

安全与并发约束
    本文件没有会话状态; 常量不是硬件已验收证明, 参数变更需另行评审.

文件结构(按定义顺序)
    GravityCommandError: 电机命令传输失败, 无法确认保持力矩.
    apply_gravity_compensation_tau_scale: 对六轴广义重力力矩施加补偿增益, 返回独立副本.
"""

from __future__ import annotations

import logging


import numpy as np


class GravityCommandError(RuntimeError):
    """电机命令传输失败, 无法确认保持力矩."""


_LOG = logging.getLogger('rebotarmcontroller.hardware_manager')

from .gripper.gripper_control import (
    _G_MAX_DIST_M,
    _G_VERIFIED_OPEN_LIMIT_M,
    _G_ANGLE_OPEN,
    _G_COORDINATE_TOL_RAD,
    _G_CLOSED_FEEDBACK_TOL_RAD,
    _G_ZERO_VERIFY_TIMEOUT_SEC,
    _G_ZERO_VERIFY_SAMPLES,
    _G_ARRIVE_TOL,
    _G_TAU_MAX,
    _G_POSITION_MAX_SPEED_RAD_S,
    _G_POSITION_MAX_SPEED_MIN_RAD_S,
    _G_POSITION_MAX_SPEED_MAX_RAD_S,
    _G_POSITION_TIMEOUT_MARGIN_SEC,
    _G_POSITION_TIMEOUT_MARGIN_MAX_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_MAX_SEC,
    _HARDWARE_FEEDBACK_RATE_HZ,
    _HARDWARE_FEEDBACK_RATE_MIN_HZ,
    _HARDWARE_FEEDBACK_RATE_MAX_HZ,
    _G_LARGE_MOVE_MAX_TAU_NM,
    _G_LARGE_MOVE_MAX_TAU_CAP_NM,
    _G_KP_MOVE,
    _G_KD_MOVE,
    _G_DEFAULT_FORCE,
    _G_CTRL_RATE,
)

_GC_VEL_THRESHOLD = 0.04
_GC_W_VEL_THRESHOLD = 0.08
_GC_EE_FRAME = "end_link"
_GC_KP = 7.0
_GC_KD = 0.8
_GC_TAU_SCALE = np.ones(6, dtype=np.float64)
# DM 串口桥可能分多个轮询周期才把六帧电机反馈取回. 5 ms 短于实测的帧节拍,
# 会把首帧晚于 joint1/joint2 到达的健康关节误判为失败.
_FEEDBACK_RETRY_INTERVAL_SEC = 0.05

# 厂商 ArmEndPos.safe_home 会把每个关节都驱到 0 rad. 对本机械臂而言那是"折叠"
# 姿态而非张开姿态: joint3 原点在 x 方向为 -0.264 m, joint4 为 +0.2426 m,
# 因此当 2-5 关节接近零时, 前臂会折回贴上臂, link2/link5 间距缩到约 1.6 mm.
# MoveIt 的 CheckStartStateCollision 随后会否决从该处发出的每一个规划请求
# ("1 contact(s) detected : link2 - link5"), 即零位不可用于规划, 也就无法作为
# 停放姿态. 这里改为停在"视觉就绪"姿态: link2/link5 实测间隙 33.2 mm, 且机械
# 上与零位同样是紧凑站姿(end_link 水平伸展 0.254 m 对 0.260 m, 高 24 mm),
# 因此不增加机械代价, 也不会让手臂在重力下长期外伸. 该姿态同时是
# tests/test_paired_trajectory_protocol.py 视为标准工作位姿的姿态, 并与 MoveIt
# SRDF 中的 "safe_home" 命名状态以及 MuJoCo keyframe 一致.
_SAFE_HOME_JOINT_POSITIONS = (0.0, 0.0, -0.017453292519943295, 0.0, 0.0, 0.0)
# safe_home 到位判据: 所有关节残差的最大值需小于该阈值(rad).
_SAFE_HOME_ARRIVE_TOL_RAD = 0.02
_SAFE_HOME_TIMEOUT_SEC = 30.0


def apply_gravity_compensation_tau_scale(tau: np.ndarray) -> np.ndarray:
    """对六轴广义重力力矩施加补偿增益, 返回独立副本; 维数不符明确失败."""
    scaled = np.array(tau, dtype=np.float64, copy=True)
    if scaled.shape != _GC_TAU_SCALE.shape:
        raise ValueError('gravity torque must contain 6 values')
    scaled *= _GC_TAU_SCALE
    return scaled
