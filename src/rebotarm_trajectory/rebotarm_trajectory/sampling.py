"""已校验离散轨迹的兼容采样，不持有执行游标或发送状态。"""

from bisect import bisect_right
import math

from .polynomial import vector6


def validate_samples(points, times):
    values = tuple(vector6(point) for point in points)
    stamps = tuple(float(stamp) for stamp in times)
    if not values or len(values) != len(stamps) or not all(
            math.isfinite(stamp) and stamp >= 0 for stamp in stamps):
        raise ValueError('trajectory needs finite nonnegative times and joint targets')
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('trajectory times must increase')
    return values, stamps


def step_index(times, elapsed, cursor=0):
    """返回当前已到期点的右边界；发送确认游标仍由后端拥有。"""
    return bisect_right(times, elapsed, lo=cursor)


def linear_sample(times, positions, initial, elapsed):
    if elapsed >= times[-1]:
        return positions[-1]
    upper = bisect_right(times, elapsed)
    lower_time = times[upper - 1] if upper else 0.
    lower = positions[upper - 1] if upper else initial
    if lower_time == elapsed and upper:
        return lower
    if times[upper] == lower_time:
        return positions[upper]
    ratio = (elapsed - lower_time) / (times[upper] - lower_time)
    return tuple(a + ratio * (b - a) for a, b in zip(lower, positions[upper]))
