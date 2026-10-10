"""ros message_codec components; preserve existing execution contracts."""
from __future__ import annotations
import math
import threading
from typing import Any, Sequence
from rebotarm_simulation.execution.trajectory_sampler import NamedTrajectoryPoint, TrajectorySampler
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



def trajectory_to_sampler(
    trajectory: Any,
    *,
    initial_positions: Sequence[float],
    max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
    max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC,
) -> TrajectorySampler:
    """校验外部传入的"类 JointTrajectory"对象并构造采样器。

    这是动作目标的第一道安全门：先把不可信输入挡在仿真之外，再交给采样器。
    本函数只做规模与结构层面的拒绝式校验（点数上限、时长上限、时长为有限值），
    关节名合法性、重复点、时间严格递增等语义校验由采样器负责。

    参数：
        trajectory: 具有 joint_names 与 points 属性的对象（points 需含
            time_from_start 与 positions）；
        initial_positions: 六个手臂关节的初始角，单位 rad；用于补齐目标中未列出的关节；
        max_points: 允许的最大路点数，正整数；
        max_duration_sec: 允许的最长轨迹时长，单位 s，正的有限值。

    异常：结构不可读、超规模或超时长时抛 ValueError。
    """
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError("max points must be positive")
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.0:
        raise ValueError("max duration must be finite and positive")
    try:
        names = tuple(trajectory.joint_names)
        raw_points = tuple(trajectory.points)
    except (AttributeError, TypeError) as exc:
        raise ValueError("trajectory structure is invalid") from exc
    if len(raw_points) > int(max_points):
        raise ValueError("trajectory contains too many points")

    points = tuple(
        NamedTrajectoryPoint(duration_to_seconds(point.time_from_start), point.positions)
        for point in raw_points
    )
    # 规范关节名校验、重复名/长度、非空、有限值与时间严格递增这些语义约束统一由
    # 采样器负责，本文只在上游控制规模。
    sampler = TrajectorySampler(names, points, initial_positions=initial_positions)
    # 时长上限在采样器规范化之后才能确知（此时 duration 已含补齐初始点的语义）。
    if sampler.duration > duration_cap:
        raise ValueError("trajectory duration exceeds configured limit")
    return sampler



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

    物理步进与虚拟相机线程可能并发取时间戳，若直接透传仿真时间，时钟会在
    重启/回退时倒退，使依赖时间戳的 TF、消息缓存与上层校验失效。这里用锁
    维护已发布的最大值，任何更小的候选都被夹到该值。
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
