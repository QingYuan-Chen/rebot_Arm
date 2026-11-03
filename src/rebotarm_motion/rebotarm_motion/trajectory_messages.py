"""操作入口和示教对齐共用的静止到静止轨迹构造。

文件职责
    提供五次曲线的解析 q/v/a，以及含完整首末导数的点对点消息，避免前端只发送阶跃。
对外接口
    rest_to_rest_sample 在比例时间 ratio∈[0,1] 求值；point_to_point_message
    接收六轴名称、同序起终点和秒制时长，并通过调用者提供的工厂创建消息。
安全约束
    本文件只构造约束，不执行限位、碰撞或设备发送；生成消息后仍需正式准备与接纳。
    joint_names 必须恰好包含六轴；start/target 必须与名称同序，函数不暗中重排它们。
    不自动重定时；静止起终点的 v/a 为零不意味着中间离散点也应补零。
文件结构
    解析五次采样 → 两点完整约束的消息编码与纳秒时间拆分。
"""

from .polynomial_trajectory import JOINT_NAMES
from .trajectory_preparation import point_to_point
import math


def rest_to_rest_sample(start, target, duration, ratio):
    """合成对齐段的解析五次 q/v/a；保持点由调用者显式选择端点。"""
    if len(start) != len(target) or not math.isfinite(duration) or duration <= 0:
        raise ValueError('finite positive alignment duration and matching vectors required')
    u = float(ratio)
    if not 0 <= u <= 1:
        raise ValueError('alignment ratio outside [0, 1]')
    q, v, a = [], [], []
    for first, last in zip(start, target):
        delta = float(last)-float(first)
        # 10u³-15u⁴+6u⁵ 同时满足首末速度、加速度为零；导数按秒制时长换算。
        q.append(float(first)+delta*(10*u**3-15*u**4+6*u**5))
        v.append(delta*(30*u**2-60*u**3+30*u**4)/duration)
        a.append(delta*(60*u-180*u**2+120*u**3)/duration**2)
    return tuple(q), tuple(v), tuple(a)


def point_to_point_message(names, start, target, duration, *, trajectory_factory, point_factory):
    """只输出首末两个完整约束点，实际连续曲线由准备服务生成并由后端统一采样。"""
    names = tuple(names)
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError('exactly joint1..joint6 required')
    points = point_to_point(start, target, duration)
    trajectory = trajectory_factory()
    trajectory.joint_names = list(names)
    for value in points:
        point = point_factory()
        point.positions = list(value.positions)
        point.velocities = list(value.velocities)
        point.accelerations = list(value.accelerations)
        stamp = round(value.time*1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(stamp, 1_000_000_000)
        trajectory.points.append(point)
    return trajectory
