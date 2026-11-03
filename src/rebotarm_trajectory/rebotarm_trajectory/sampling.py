"""离散轨迹的旧阶梯和线性兼容采样。

文件职责
    提供纯数值输入校验、到期点查找和线性位置采样，保留迁移前的时间边界语义。
对外接口
    validate_samples 返回复制后的六轴点与时间元组；step_index 返回到期点右边界；
    linear_sample 接收已校验的点、起始位置及非负经过时间，返回位置元组。
安全约束
    不读取时钟、不拥有游标、不确认设备发送；调用方负责位置限位与执行生命周期。
    step_index 的 cursor 由后端确认状态提供；计算出候选点不代表发送或到位成功。
文件结构
    离散输入校验 → 阶梯索引 → 线性插值。
"""

from bisect import bisect_right
import math

from .polynomial import vector6


def validate_samples(points, times):
    """检查非空、六轴有限值、点时一一对应及严格递增时间；允许首点晚于零时刻。"""
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
    """首点前从 initial 插值，节点取该点，末点后保持；不生成速度或加速度约束。"""
    if elapsed >= times[-1]:
        return positions[-1]
    upper = bisect_right(times, elapsed)
    # 部分旧输入没有 t=0 点，起始反馈只用于首段，不追加或改写原始轨迹点。
    lower_time = times[upper - 1] if upper else 0.
    lower = positions[upper - 1] if upper else initial
    if lower_time == elapsed and upper:
        return lower
    if times[upper] == lower_time:
        return positions[upper]
    ratio = (elapsed - lower_time) / (times[upper] - lower_time)
    return tuple(a + ratio * (b - a) for a, b in zip(lower, positions[upper]))
