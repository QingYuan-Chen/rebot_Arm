"""与 ROS 解耦的具名关节轨迹规范化与采样工具。

模块职责：把上层（轨迹动作目标、示教回放等）给出的“具名关节轨迹点”转换成仿真包内部使用的
规范形式，并按仿真已流逝时间做线性插值采样，供无头仿真控制器在物理时间轴上逐步执行。

关键约定与安全语义：

- 关节名必须属于六个机械臂关节的规范集合（joint1..joint6，顺序固定，见 ``ARM_JOINT_NAMES``）。
  允许只给出其中一部分关节，但此时必须显式提供其余关节的初始角度，否则无法还原完整目标向量，
  会在构造阶段直接拒绝，不做任何默认补齐；
- 轨迹点时间以秒为单位，必须有限、非负且严格递增（不允许出现重复时间戳）；
- 位置值必须有限；非法输入一律抛异常，绝不静默裁剪、截断或丢弃轨迹点；
- 采样为线性插值：第一个轨迹点之前从 t=0 的初始位置线性插值，最后一个点之后保持终点位置（不超调）；
- ``cancel()`` 之后继续采样会抛异常，用于保证停止指令与后续目标下发之间没有竞态窗口。

本模块刻意不导入任何 ROS 依赖，因此可以在未安装 ROS 的开发主机上对轨迹输入做单元测试与
模糊测试。
"""

from __future__ import annotations

from bisect import bisect_right
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math


# 规范关节顺序：六个机械臂关节固定按 joint1..joint6 排列。
# 所有内部目标向量、采样结果和示例中的“完整向量”都使用这一顺序，与外部传入顺序无关。
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES


@dataclass(frozen=True)
class NamedTrajectoryPoint:
    """规范化之前的单个具名轨迹点。

    ``time_from_start`` 为相对轨迹起点的时刻（秒，非负）；``positions`` 按调用方传入的
    ``joint_names`` 顺序排列（单位 rad，机械臂关节）。本记录不可变：构造时立即把序列
    固化成元组，避免调用方事后修改底层列表造成轨迹“漂移”。
    """

    time_from_start: float
    positions: tuple[float, ...]

    def __init__(self, time_from_start: float, positions: Sequence[float]) -> None:
        """手写 ``__init__`` 以便把任意序列固化为元组；frozen 数据类必须用 ``object.__setattr__`` 赋值。"""
        object.__setattr__(self, "time_from_start", time_from_start)
        object.__setattr__(self, "positions", tuple(positions))


class TrajectorySampler:
    """把具名机械臂目标规范化成六个关节的完整向量，并按仿真时间线性采样。

    生命周期：构造阶段完成全部校验与规范化（构造失败即表示该轨迹被拒绝），之后可被反复采样；
    ``cancel()`` / ``clear_cancel()`` / ``reset()`` 用于响应外部停止请求；``sample()`` 本身无副作用，
    不缓存、不落盘，每次按时间做二分查找定位相邻轨迹点。

    该类不依赖 ROS，也不接触真实硬件；它的输出只是目标关节角，实际下发与安全门由调用方负责。
    """

    def __init__(
        self,
        joint_names: Sequence[str],
        points: Sequence[NamedTrajectoryPoint],
        *,
        initial_positions: Sequence[float] | Mapping[str, float] | None = None,
    ) -> None:
        """校验并规范化一条具名轨迹。

        参数：
            joint_names: 本条轨迹涉及的关节名（允许是六个规范关节的子集），不得为空、不得重复，
                且每个名字都必须属于 ``ARM_JOINT_NAMES``。
            points: 具名轨迹点序列，至少一个；时间必须严格递增，位置个数必须与 ``joint_names`` 一致。
            initial_positions: 未出现在 ``joint_names`` 中的关节的初始角（rad）。可以是规范顺序的
                六元序列，也可以是“关节名 -> 角度”的映射；当 ``joint_names`` 已是完整六关节时可省略，
                省略时按全零位处理。

        异常：
            ValueError: 关节名非法/重复、点数为空、时间为负或非递增、位置个数不匹配、
                ``initial_positions`` 未恰好覆盖六个规范关节等。
            TypeError: ``points`` 中存在非 ``NamedTrajectoryPoint`` 记录。

        本方法只做校验与预计算，不修改传入的 ``points``，也不产生任何副作用。
        """
        names = tuple(joint_names)
        raw_points = tuple(points)
        self._validate_names(names)
        if not raw_points:
            raise ValueError("trajectory must contain at least one point")

        # 轨迹只覆盖部分关节时，无法从轨迹本身还原完整目标向量，必须由调用方补齐其余关节的初值。
        omitted = set(ARM_JOINT_NAMES).difference(names)
        if omitted and initial_positions is None:
            raise ValueError("partial trajectories require canonical initial positions")
        initial = self._normalize_initial(initial_positions)

        normalized_points: list[tuple[float, tuple[float, ...]]] = []
        # 用 -inf 起始，使第一个点的时间只要求满足“有限且非负”，严格递增检查可统一处理。
        previous_time = -math.inf
        # 预先求出每个具名关节在规范向量中的下标，用于把子集值写回完整向量。
        indexes = tuple(ARM_JOINT_NAMES.index(name) for name in names)
        for point in raw_points:
            if not isinstance(point, NamedTrajectoryPoint):
                raise TypeError("trajectory points must be NamedTrajectoryPoint records")
            time = float(point.time_from_start)
            if not math.isfinite(time) or time < 0.0:
                raise ValueError("point times must be finite and non-negative")
            if time <= previous_time:
                raise ValueError("point times must be strictly increasing")
            if len(point.positions) != len(names):
                raise ValueError("point position count must match joint names")

            values = tuple(float(value) for value in point.positions)
            if any(not math.isfinite(value) for value in values):
                raise ValueError("point positions must be finite")
            # 以初始位置为底，用当前点给出的关节值覆盖对应下标，得到完整六关节目标向量。
            canonical = list(initial)
            for index, value in zip(indexes, values):
                canonical[index] = value
            normalized_points.append((time, tuple(canonical)))
            previous_time = time

        self._initial = initial
        self._times = tuple(time for time, _ in normalized_points)
        self._positions = tuple(positions for _, positions in normalized_points)
        self._cancelled = False

    @staticmethod
    def _validate_names(names: tuple[str, ...]) -> None:
        """校验关节名：非空、全为非空字符串、无重复，且都属于规范关节集合。

        这里拒绝未知关节名而不是忽略它们，防止上层把关节名拼错后被静默当成“不控制该关节”。
        """
        if not names:
            raise ValueError("joint names must not be empty")
        if any(not isinstance(name, str) or not name for name in names):
            raise ValueError("joint names must be non-empty strings")
        if len(set(names)) != len(names):
            raise ValueError("duplicate joint names are not allowed")
        unknown = set(names).difference(ARM_JOINT_NAMES)
        if unknown:
            raise ValueError(f"unknown arm joint names: {sorted(unknown)}")

    @staticmethod
    def _normalize_initial(
        positions: Sequence[float] | Mapping[str, float] | None,
    ) -> tuple[float, ...]:
        """把初始位置统一成规范顺序的六元组（rad）。

        接受三种形式：``None``（缺省全零位）、关节名到角度的映射（必须恰好覆盖六个规范关节）、
        长度恰为 6 的数值序列。字符串/字节串会被逐元素展开，属于典型误用，因此显式拒绝。
        """
        if positions is None:
            values = (0.0,) * len(ARM_JOINT_NAMES)
        elif isinstance(positions, Mapping):
            # 映射形式必须“不多不少”地覆盖六个规范关节，避免漏给某个关节的初值。
            unknown = set(positions).difference(ARM_JOINT_NAMES)
            missing = set(ARM_JOINT_NAMES).difference(positions)
            if unknown or missing:
                raise ValueError("initial positions must contain exactly the canonical arm joints")
            values = tuple(float(positions[name]) for name in ARM_JOINT_NAMES)
        else:
            if isinstance(positions, (str, bytes)) or len(positions) != len(ARM_JOINT_NAMES):
                raise ValueError("initial positions must contain six canonical arm values")
            values = tuple(float(value) for value in positions)
        if any(not math.isfinite(value) for value in values):
            raise ValueError("initial positions must be finite")
        return values

    @property
    def joint_names(self) -> tuple[str, ...]:
        """采样结果向量的关节顺序：始终是规范六关节顺序，而不是构造时传入的子集。"""
        return ARM_JOINT_NAMES

    @property
    def points(self) -> tuple[NamedTrajectoryPoint, ...]:
        """Immutable canonical points for transport and reporting."""
        return tuple(NamedTrajectoryPoint(t, q) for t, q in zip(self._times, self._positions))

    @property
    def duration(self) -> float:
        """轨迹总时长（秒），即最后一个轨迹点的时刻。"""
        return self._times[-1]

    @property
    def is_cancelled(self) -> bool:
        """是否已收到取消请求；取消状态下 ``sample()`` 会拒绝继续采样。"""
        return self._cancelled

    def is_complete(self, simulation_time: float) -> bool:
        """仿真时间是否已到达或超过轨迹终点（用于判断“该发成功结果了”）。"""
        time = self._validate_sample_time(simulation_time)
        return time >= self.duration

    def cancel(self) -> None:
        """标记轨迹被取消；只置标志位，不修改已规范化的轨迹数据。"""
        self._cancelled = True

    def clear_cancel(self) -> None:
        """撤销取消标志，使轨迹可再次被采样。"""
        self._cancelled = False

    def reset(self) -> None:
        """把采样器恢复到“未取消”状态（当前语义等同于 ``clear_cancel()``）。"""
        self.clear_cancel()

    def sample(self, simulation_time: float) -> tuple[float, ...]:
        """按仿真时间采样，返回规范顺序的六关节目标角（rad）。

        插值规则：早于第一个轨迹点时从 t=0 的初始位置线性插值；
        到达或超过终点时保持最后一个点（不超调）；其余情况在相邻两点之间做线性插值。
        取消状态下调用会抛 RuntimeError，以保证停止请求生效后不再产生新的运动目标。
        """
        time = self._validate_sample_time(simulation_time)
        if self._cancelled:
            raise RuntimeError("trajectory sampling was cancelled")
        if time >= self.duration:
            return self._positions[-1]

        # 二分定位第一个严格大于 time 的轨迹点下标，因此待插值区间必为 [upper-1, upper]。
        upper = bisect_right(self._times, time)
        if upper == 0:
            # 早于第一个轨迹点：以 t=0 的初始位置作为插值区间左端。
            lower_time = 0.0
            lower_positions = self._initial
        else:
            lower_time = self._times[upper - 1]
            lower_positions = self._positions[upper - 1]
            if lower_time == time:
                # 正好命中已记录的轨迹点，直接返回，避免插值引入浮点误差。
                return lower_positions

        upper_time = self._times[upper]
        upper_positions = self._positions[upper]
        if upper_time == lower_time:
            # 时间严格递增时不会发生；此处仅作除零保护。
            return upper_positions
        fraction = (time - lower_time) / (upper_time - lower_time)
        return tuple(
            start + fraction * (end - start)
            for start, end in zip(lower_positions, upper_positions)
        )

    @staticmethod
    def _validate_sample_time(simulation_time: float) -> float:
        """把采样时刻统一成 float，要求有限且非负（负时间说明时间基准异常，必须拒绝）。"""
        time = float(simulation_time)
        if not math.isfinite(time) or time < 0.0:
            raise ValueError("simulation time must be finite and non-negative")
        return time
