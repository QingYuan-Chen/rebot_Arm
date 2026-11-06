"""夹爪输出计算及配置基线.

职责与位置
    夹爪纯计算层和配置常量, 使用原始电机角度计算斜坡与有界 MIT 参数.
    本模块不访问设备或 ROS; 调用方负责反馈校验, 命令发送及发送结果确认.

对外接口
    GripperControl.ramp 生成限速目标; bounded_mit 计算待发送的 MIT 参数.

安全与并发约束
    总力矩估算包含 PD 和前馈, 约束基于当前反馈, 不能代替真实设备时序验证.
    发送和反馈健康由 GripperController 协调, 执行状态由 GripperExecution 持有.

统一约定
    标称开口为 m, 原始电机角为 rad, 角速度为 rad/s, 电机力矩为 N·m.
    开口与电机角按标称行程线性映射, 不是对软指实际指间距的独立标定.

文件结构(按定义顺序)
    模块常量: 标称行程, 量化容差, 运动参数, 抓取参数及保持时限.
    GripperControl: 无状态斜坡和力矩计算, 供执行器和发送边界使用.
"""

import numpy as np

_G_MAX_DIST_M = 0.09
# 张开命令上限为 90 mm, 当前与软件标称行程一致; 不是实测软指净开口.
_G_VERIFIED_OPEN_LIMIT_M = 0.09
# 全开位置对应的电机原始角度(rad, 负方向为张开). 位置-角度线性映射的另一端.
_G_ANGLE_OPEN = -5.0
# DM4310 的 16 位反馈覆盖 +/-12.5 rad. 这里给张开端点与置零验证留两个量化步长,
# 它是"反馈有效性"容差, 不是运动到位容差.
_G_COORDINATE_TOL_RAD = 2.0 * 25.0 / 65535.0
# 操作员批准的 10 mm 双指总开口等效闭合反馈容差, 不改变命令限位或置零验证精度.
_G_CLOSED_FEEDBACK_TOL_RAD = 0.010 * abs(_G_ANGLE_OPEN) / _G_MAX_DIST_M
_G_ZERO_VERIFY_TIMEOUT_SEC = 0.5
_G_ZERO_VERIFY_SAMPLES = 3
_G_ARRIVE_TOL = 0.12
_G_TAU_MAX = 1.5
# 常规位置移动使用有界的位置斜坡命令. 默认 0.5 rad/s 与既有 Web/键盘遥操作
# 速度基线一致; 它仍是独立的夹爪参数, 以便两条机构分别整定.
_G_POSITION_MAX_SPEED_RAD_S = 0.5
_G_POSITION_MAX_SPEED_MIN_RAD_S = 0.05
_G_POSITION_MAX_SPEED_MAX_RAD_S = 3.0
_G_POSITION_TIMEOUT_MARGIN_SEC = 1.5
_G_POSITION_TIMEOUT_MARGIN_MAX_SEC = 10.0
_G_FEEDBACK_STALE_TIMEOUT_SEC = 0.15
_G_FEEDBACK_STALE_TIMEOUT_MAX_SEC = 2.0
_HARDWARE_FEEDBACK_RATE_HZ = 50.0
_HARDWARE_FEEDBACK_RATE_MIN_HZ = 20.0
_HARDWARE_FEEDBACK_RATE_MAX_HZ = 100.0
# 下面两个是 MOVE 阶段专用的电机侧力矩上限; 到位后防止继续顶住机械限位的,
# 是 _release_gripper_position_target 里的"中性 + 空闲"释放.
# 2026-09-04 操作员决定: 采用 Web 遥操作已验证可用的默认值 1.0 N.m, 仅把
# 1.5 N.m 保留为可配置上限, 并用上面的 0.5 rad/s 斜坡限制运动速度.
_G_LARGE_MOVE_MAX_TAU_NM = 1.0
_G_LARGE_MOVE_MAX_TAU_CAP_NM = 1.5
_G_KP_MOVE = 5.0
_G_KD_MOVE = 1.0
_G_DEFAULT_FORCE = 0.40
_G_CTRL_RATE = 500.0


class GripperControl:
    """无状态夹爪控制计算, 供周期执行器和驱动发送边界使用."""

    @staticmethod
    def ramp(target, goal, speed, elapsed):
        """按实际周期时长限制目标角变化, 不越过终点."""
        step = speed * max(elapsed, 0.0)
        remaining = goal - target
        if abs(remaining) <= step:
            return goal
        return target + float(np.copysign(step, remaining)) if step > 0.0 else target

    @staticmethod
    def bounded_mit(pos, vel, kp, kd, tau_ff, tau_limit, measured_pos, measured_vel):
        """夹紧目标并反解前馈, 使当前反馈下的估计总力矩不超限."""
        pos_cmd = float(np.clip(pos, _G_ANGLE_OPEN, 0.0))
        pos_term = kp * (pos_cmd - measured_pos) + kd * (-measured_vel)
        limit = float(np.clip(abs(tau_limit), 0.05, _G_TAU_MAX))
        tau_safe = float(np.clip(pos_term + tau_ff, -limit, limit)) - pos_term
        return pos_cmd, vel, kp, kd, tau_safe
