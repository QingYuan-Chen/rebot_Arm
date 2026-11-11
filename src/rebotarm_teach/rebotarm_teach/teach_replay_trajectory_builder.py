"""示教回放轨迹构建：把预处理后的示教样本拼装成可下发的关节轨迹。

本模块只做"时间轴拼接"，不访问硬件、不做碰撞检查，也不决定是否允许回放：

    起始保持段 -> 起始对齐段（软启动插值，或由外部规划器生成） -> 录制段 -> 末尾保持段

轨迹对象与轨迹点对象都由调用方注入的工厂创建，因此本模块不直接依赖具体消息类型，
便于单元测试；位置单位统一为弧度（rad），时间单位统一为秒（s）。

门控说明：本模块只消费预处理结果 ``prepared``（其 ``samples`` 或 ``retimed_points``），
不读取记录文件；录制段一定先重定时再下发，不会把原始采样时刻直接写进轨迹，
yellow 风险等级还会在这里把速度压到配置上限以内。

对外接口：TeachReplayTrajectoryBuilder.build 拼接并返回轨迹，消息工厂和起点对齐器由调用方注入。
安全约束：按原输入等级拼接；示教对齐须显式转换并重新检查，不补造缺失的高阶导数。
文件结构：时间编码 → 构建配置/结果 → 起点分档与拼接 → 回放点编码 → 合法末点保持。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable

from .teach_recording import (
    build_replay_start_soft_points,
    retime_teach_samples,
)


def set_duration(duration_msg: Any, seconds: float) -> None:
    """把浮点秒数写进 ROS 时长消息的 ``sec`` / ``nanosec`` 两个字段。

    ``nanosec`` 由小数部分乘 1e9 后**截断取整**（非四舍五入），因此写入值最多比
    入参小 1 ns；``seconds`` 约定为非负值，负值不在此处做保护。
    """
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


@dataclass(frozen=True)
class TeachReplayTrajectoryConfig:
    """单次轨迹构建所需的、来自节点参数的只读常量集合。

    字段含义与单位：

    - ``use_moveit_start_align``：起始段是否改由外部起始对齐回调（规划器）生成；
      为 True 时软启动相关字段全部失效。
    - ``start_hold_sec``：轨迹最开始的原地保持时长（s），避免启动瞬间拉扯。
    - ``soft_start_duration``：软启动插值段时长（s），仅在未启用起始对齐时生效。
    - ``soft_start_steps``：软启动插值步数（含首尾点），实际插值点数为 max(该值, 2)。
    - ``first_hold_sec``：对齐到记录首点之后的保持时长（s）。
    - ``yellow_max_speed``：质量等级为 yellow 时的回放速度上限（倍率，1.0 表示原速）。
    - ``initial_replay_delay_sec``：录制段相对对齐段末尾的额外延迟（s）。
    - ``max_velocity_rad_s``：重定时用的每关节速度上限（rad/s），可以是标量、
      与关节同序的序列，或 ``{关节名: 上限}`` 映射，具体解释见重定时实现。
    - ``max_acceleration_rad_s2``：每关节加速度上限（rad/s^2）。
    - ``max_jerk_rad_s3``：每关节加加速度上限（rad/s^3）。
    """

    use_moveit_start_align: bool
    start_hold_sec: float
    soft_start_duration: float
    soft_start_steps: int
    first_hold_sec: float
    yellow_max_speed: float
    initial_replay_delay_sec: float
    max_velocity_rad_s: Any
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float


@dataclass(frozen=True)
class TeachReplayTrajectoryResult:
    """构建结果：只包含最终轨迹对象，后续由调用方下发给控制器。"""

    trajectory: Any


class TeachReplayTrajectoryBuilder:
    """把预处理样本拼装成控制器可接受的关节轨迹（FollowJointTrajectory 语义）。

    无状态工具类：同一个实例可被多次 ``build`` 复用；线程安全性由调用方保证
    （实践中回放请求串行处理，且活动轨迹在节点侧加锁）。

    轨迹/轨迹点对象来自构造时注入的工厂，因此同一份逻辑既可用于真实消息类型，
    也可在测试中替换为轻量替身。
    """

    def __init__(
        self,
        *,
        trajectory_factory: Callable[[], Any],
        trajectory_point_factory: Callable[[], Any],
    ) -> None:
        # 两个工厂分别生产"整条轨迹"和"单个轨迹点"的空对象，构建时逐个填充字段。
        self._trajectory_factory = trajectory_factory
        self._trajectory_point_factory = trajectory_point_factory

    def build(
        self,
        *,
        prepared: Any,
        current_positions: dict[str, float],
        start_band: str,
        settings: dict[str, float | int],
        config: TeachReplayTrajectoryConfig,
        moveit_start_alignment: Callable[..., float] | None = None,
    ) -> TeachReplayTrajectoryResult:
        """构建完整回放轨迹。

        参数：

        - ``prepared``：预处理结果（含 ``samples``、``retimed_points``、
          ``effective_replay_speed``、``after_quality.risk_level`` 等字段）。
        - ``current_positions``：当前关节反馈，``{关节名: 位置(rad)}``，须完整且有限。
        - ``start_band``：起始分级（direct / align / moveit_align / reject），
          决定软启动用哪一组时长与步数。
        - ``settings``：已规范化的回放设置，需含 ``align_duration`` / ``align_steps`` /
          ``final_hold_sec``。
        - ``config``：节点参数构成的只读常量集合。
        - ``moveit_start_alignment``：启用起始对齐时必需的回调，返回对齐段结束时刻（s）。

        异常：记录为空时抛 ``ValueError``；启用起始对齐但没有传回调时同样抛
        ``ValueError``（不静默降级为软启动）。
        """
        replay_samples = list(prepared.samples)
        if not replay_samples:
            raise ValueError("record contains no samples")
        first = replay_samples[0]
        grade = prepared.reference_grade
        if grade not in ("q", "q/v", "q/v/a"):
            raise ValueError(f"invalid prepared reference grade: {grade}")
        trajectory = self._trajectory_factory()
        # 关节顺序以记录首样本为准，后续所有点都按这个顺序排列。
        trajectory.joint_names = list(first.joint_names)
        # 对齐起点必须来自实际反馈，并按记录的关节顺序排列。
        try:
            current = tuple(float(current_positions[name]) for name in first.joint_names)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"missing or invalid current joint feedback: {exc}") from exc
        if not all(math.isfinite(value) for value in current):
            raise ValueError("non-finite current joint feedback")
        if config.use_moveit_start_align:
            if moveit_start_alignment is None:
                raise ValueError("MoveIt start alignment callback is required")
            # 规划器负责把"当前位置 -> 记录首点"这段写进同一条轨迹，并以关键字实参
            # 接收当前位置与首点位置，返回值是对齐段占用的总时长（s）。
            elapsed = float(
                moveit_start_alignment(
                    trajectory,
                    current_positions=current,
                    first_positions=tuple(first.positions),
                    reference_grade=grade,
                )
            )
        else:
            elapsed = self._append_soft_start(
                trajectory,
                current_positions=current,
                first_positions=tuple(first.positions),
                start_band=start_band,
                settings=settings,
                config=config,
                reference_grade=grade,
            )
        self._append_replay_points(trajectory, prepared=prepared, elapsed=elapsed, config=config)
        # 对齐适配已显式确定等级；这里仍严格核验，不自动补导数或静默删除另一段字段。
        for point in trajectory.points:
            for field in ('velocities', 'accelerations'):
                expected = grade != 'q' if field == 'velocities' else grade == 'q/v/a'
                if len(getattr(point,field)) != (len(trajectory.joint_names) if expected else 0):
                    raise ValueError(f'teach {grade} splice has incompatible {field}')
        self.append_final_hold(trajectory, final_hold_sec=float(settings["final_hold_sec"]))
        return TeachReplayTrajectoryResult(trajectory=trajectory)

    def _append_soft_start(
        self,
        trajectory: Any,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
        start_band: str,
        settings: dict[str, float | int],
        config: TeachReplayTrajectoryConfig,
        reference_grade: str,
    ) -> float:
        """追加"起始保持 + 插值对齐 + 首点保持"，返回对齐段结束时刻（s）。

        对齐段用静止到静止的五次曲线生成；``align`` 分级使用设置里的
        ``align_duration`` / ``align_steps``（可由起始误差自动推算），
        其余可回放分级使用配置里的软启动参数。
        合成段显式投影到示教参考等级；最终检查的是投影后实际曲线。
        """
        start_points = build_replay_start_soft_points(
            current_positions=current_positions,
            first_positions=first_positions,
            start_band=start_band,
            start_hold_sec=float(config.start_hold_sec),
            soft_start_duration=float(config.soft_start_duration),
            soft_start_steps=int(config.soft_start_steps),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            first_hold_sec=float(config.first_hold_sec),
        )
        for start_point in start_points:
            point = self._trajectory_point_factory()
            point.positions = [float(v) for v in start_point.positions]
            point.velocities = list(start_point.velocities) if reference_grade != 'q' else []
            point.accelerations = list(start_point.accelerations) if reference_grade == 'q/v/a' else []
            set_duration(point.time_from_start, start_point.time_from_start)
            trajectory.points.append(point)
        # 最后一个起始点的时刻即录制段的起点偏移；无起始点时从 0 开始。
        return float(start_points[-1].time_from_start) if start_points else 0.0

    def _append_replay_points(
        self,
        trajectory: Any,
        *,
        prepared: Any,
        elapsed: float,
        config: TeachReplayTrajectoryConfig,
    ) -> None:
        """追加录制段（重定时后的轨迹点），时间整体平移 ``elapsed``。"""
        # 速度下限 0.01 是除零保护：倍率会作为时间缩放的分母参与重定时。
        speed = max(float(prepared.effective_replay_speed), 0.01)
        # yellow 风险（存在中等跳变/超速）时按配置再压一次速度，不允许跑满原速。
        if str(prepared.after_quality.risk_level) == "yellow":
            speed = min(speed, float(config.yellow_max_speed))
        if prepared.retimed_points:
            # 预处理产出的重定点时间轴从 0 起算（不含额外延迟），需要在这里补上。
            initial_delay = max(float(config.initial_replay_delay_sec), 0.0)
            retimed_points = prepared.retimed_points
        else:
            # 没有预处理重定点时退化为"现场重定时"，此时延迟由重定时函数写进时间轴，
            # 因此这里不再重复叠加（initial_delay 保持 0）。
            initial_delay = 0.0
            retimed_points = retime_teach_samples(
                prepared.samples,
                replay_speed=speed,
                max_velocity_rad_s=config.max_velocity_rad_s,
                max_acceleration_rad_s2=float(config.max_acceleration_rad_s2),
                max_jerk_rad_s3=float(config.max_jerk_rad_s3),
                initial_delay_sec=float(config.initial_replay_delay_sec),
                boundary_zero_velocity=True,
            )
        from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
        from rebotarm_motion.trajectory_time_parameterization import prepare_teach_reference
        retimed_points = prepare_teach_reference(tuple(TrajectoryPoint(
            p.time_from_start, p.positions, p.velocities, p.accelerations)
            for p in retimed_points))
        for retimed in retimed_points:
            point = self._trajectory_point_factory()
            point.positions = [float(v) for v in retimed.positions]
            point.velocities = [float(v) for v in retimed.velocities]
            point.accelerations = [float(v) for v in retimed.accelerations]
            set_duration(point.time_from_start, float(elapsed) + initial_delay + retimed.time)
            if trajectory.points and point.time_from_start == trajectory.points[-1].time_from_start:
                if any(getattr(point, f) != getattr(trajectory.points[-1], f)
                       for f in ('positions','velocities','accelerations')):
                    raise ValueError('teach splice has a zero-duration state change')
                continue
            trajectory.points.append(point)

    def append_final_hold(self, trajectory: Any, *, final_hold_sec: float) -> None:
        """在轨迹末尾追加同等级保持点，位置和已有的零导数约束沿用末点。

        ``final_hold_sec <= 0`` 或轨迹还没有任何点时直接返回（不改动轨迹）；
        保持点的位置沿用最后一个点，时间 = 上一点时刻 + 保持时长。
        末点若提供非零 v/a 则拒绝，不能通过追加零导数点掩盖突变；q-only 保持 q-only。
        """
        final_hold = max(float(final_hold_sec), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        # 上一点时刻 = sec + nanosec * 1e-9，需先把两段式时长还原成浮点秒。
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = self._trajectory_point_factory()
        hold_point.positions = [float(v) for v in last_point.positions]
        for field in ('velocities', 'accelerations'):
            values = getattr(last_point, field)
            if values and any(float(value) != 0. for value in values):
                raise ValueError('final hold requires stationary endpoint derivatives')
            setattr(hold_point, field, [0.0]*len(hold_point.positions) if values else [])
        set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)
