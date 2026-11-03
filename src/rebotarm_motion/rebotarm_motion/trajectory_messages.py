"""运动层生成明确的静止到静止轨迹，消息类型由调用者提供。"""

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
        q.append(float(first)+delta*(10*u**3-15*u**4+6*u**5))
        v.append(delta*(30*u**2-60*u**3+30*u**4)/duration)
        a.append(delta*(60*u-180*u**2+120*u**3)/duration**2)
    return tuple(q), tuple(v), tuple(a)


def point_to_point_message(names, start, target, duration, *, trajectory_factory, point_factory):
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
