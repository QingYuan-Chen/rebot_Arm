"""ROS 载荷与纯数值执行对象之间的转换和入口校验。

依次提供时长/轨迹转换、夹爪参数检查和单调仿真时间戳；字段通过属性读取，
消息结构错误交由 ROS 调用方映射成失败结果，本层不操作仿真实例。
"""
from __future__ import annotations
import math
import threading
from typing import Any
from rebotarm_trajectory.polynomial import TrajectoryPoint
from rebotarm_trajectory.joint_contract import canonical_input
DEFAULT_MAX_TRAJECTORY_POINTS = 10_000
DEFAULT_MAX_TRAJECTORY_DURATION_SEC = 300.0


def duration_to_seconds(duration: Any) -> float:
    """把带 sec/nanosec 字段的 ROS 时长对象换算为秒（float）。

    仅按属性读取，不依赖具体消息类型，因此可以在无 ROS 环境下测试。字段缺失、
    类型错误或结果非有限值时抛 ValueError（由调用方转成拒绝目标）。
    """
    try:
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("trajectory time is invalid") from exc
    if not math.isfinite(value):
        raise ValueError("trajectory time must be finite")
    return value



def trajectory_points(trajectory: Any, *, max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
                      max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC):
    """校验公共 FJT 的规模、完整六轴和原导数，返回规范顺序的不可变点。"""
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError('max points must be positive')
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.:
        raise ValueError('max duration must be finite and positive')
    names, raw_points = tuple(trajectory.joint_names), tuple(trajectory.points)
    if len(raw_points) > int(max_points):
        raise ValueError('trajectory contains too many points')
    points = tuple(TrajectoryPoint(duration_to_seconds(point.time_from_start),
        point.positions, point.velocities, point.accelerations) for point in raw_points)
    names, points = canonical_input(names, points)
    if points[-1].time > duration_cap:
        raise ValueError('trajectory duration exceeds configured limit')
    return names, points



def validate_gripper_width(width: Any) -> float:
    """校验夹爪目标开口宽度并返回 float。

    参数 width 单位为 m（0 = 完全闭合，正值 = 张开）。这里只拒绝非有限值，
    行程裁剪（夹到可信开度内）交由仿真侧完成，与本模块的尺度无关。
    """
    try:
        value = float(width)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper width must be finite") from exc
    if not math.isfinite(value):
        raise ValueError("gripper width must be finite")
    return value



def seconds_to_stamp_parts(simulation_time: Any) -> tuple[int, int]:
    """把仿真时间（秒，float）拆成 ROS 时间戳的 (sec, nanosec) 整数对。

    仿真时间必须有限且非负；四舍五入到纳秒后若进位到 1e9 则向秒进位，
    保证返回的纳秒字段始终落在 [0, 1e9) 内。
    """
    try:
        value = float(simulation_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("simulation time must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = math.floor(value)
    # 先按纳秒取整，再处理进位，避免 nanosec 越界导致下游时间戳非法。
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return int(seconds), nanoseconds



class MonotonicStamp:
    """单调时间戳生成器：保证对外发布的仿真时间只增不减。

    仿真重置可能使物理时间回退；本对象在当前生命周期内维护已发布的最大值，
    使 TF、消息缓存继续看到非递减时间。锁保护并发取值，较小候选沿用已有最大值。
    """

    def __init__(self) -> None:
        self._nanoseconds = 0
        self._lock = threading.Lock()

    def update(self, simulation_time: Any) -> tuple[int, int]:
        """登记一个仿真时间并返回不小于此前已返回值的 (sec, nanosec)。"""
        seconds, nanoseconds = seconds_to_stamp_parts(simulation_time)
        candidate = seconds * 1_000_000_000 + nanoseconds
        with self._lock:
            self._nanoseconds = max(self._nanoseconds, candidate)
            return divmod(self._nanoseconds, 1_000_000_000)

def validate_gripper_force(max_effort: Any) -> float | None:
    try:
        value = float(max_effort)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper max effort must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("gripper max effort must be finite and non-negative")
    return None if value == 0.0 else value
