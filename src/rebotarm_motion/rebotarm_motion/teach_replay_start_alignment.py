"""示教回放起始对齐:为回放轨迹拼接"起点保持 + 规划对齐段 + 首点保持".

回放要求机械臂从当前位姿平滑地过渡到示教轨迹的第一个采样点,本模块负责生成这段
过渡(对齐)轨迹并追加到回放轨迹前面,使整条 ``JointTrajectory`` 的时间戳从 0 开始
连续:

1. 在当前关节角上保持 ``start_hold_sec`` 秒(速度 0),给控制器留出收敛时间;
2. 若当前角与首个示教点的最大关节偏差达到 ``skip_threshold``(rad),则向规划服务
   请求一段关节空间规划并追加为对齐段.规划可能改变路径形状,但保证终点是首个示教
   点;偏差小于阈值时直接跳过对齐,避免无意义的规划与抖动;
3. 在首个示教点上再保持 ``first_hold_sec`` 秒(速度 0),用保持段吸收规划终点残差
   (规划器允许的关节容差),保证进入示教轨迹时刻的速度与位置都精确.

对外接口:MoveItStartAligner.append 与只读 last_report;仅用于示教对齐适配.
安全约束:显式转换到示教等级,原规划约束保留在报告中;转换后必须重新检查实际曲线.
文件结构:时间编码 → 配置 → 同等级保持 → 规划校验,转换与拼接.
时间基准:所有点的 ``time_from_start`` 都是相对轨迹起点的累计时长,由本模块自行
累加,不使用绝对时间.本模块只构造轨迹数据,不发送任何控制指令.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


def set_duration(duration_msg: Any, seconds: float) -> None:
    """把秒数写入 ROS 风格 Duration 消息的 ``sec``/``nanosec`` 字段.

    拆分为整秒 + 纳秒(1e9 进制),避免浮点累积误差;``nanosec`` 由小数部分截断得到,
    因此不保证四舍五入.本函数不做非负检查,调用方需自行保证秒数非负.
    """
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


@dataclass(frozen=True)
class MoveItStartAlignmentConfig:
    """起始对齐段的配置.

    - ``start_hold_sec``:对齐前的起点保持时长(秒),负数按 0 处理;
    - ``first_hold_sec``:对齐后的首个示教点保持时长(秒),负数按 0 处理,为 0 时不
      追加保持点;
    - ``skip_threshold``:跳过对齐的最大关节偏差(rad),当前角与首个示教点各关节偏
      差的**最大值**达到该值才规划对齐段;调大更容易跳过对齐,调小则更频繁规划;
    - ``joint_goal_tolerance``:规划目标的关节容差(rad),越小终点越准,但可能规划
      失败或耗时变长;
    - ``velocity_scaling``:规划速度缩放系数(0~1,越小越慢越安全);
    - ``acceleration_scaling``:规划加速度缩放系数(0~1,越小加速度越小).
    """

    start_hold_sec: float
    first_hold_sec: float
    skip_threshold: float
    joint_goal_tolerance: float
    velocity_scaling: float
    acceleration_scaling: float
    reference_grade: str = 'q/v'


class MoveItStartAligner:
    """把规划器生成的起始对齐段追加到回放轨迹上.

    构造时注入两个外部依赖,便于替换与测试:

    - ``planner``:关节空间规划端口,需提供 ``plan_joint_positions(...)``,返回带
      ``success``/``message``/``trajectory`` 的结果对象;
    - ``trajectory_point_factory``:轨迹点工厂(无参可调用),用于创建与目标消息类型
      一致的空轨迹点.

    规划失败或规划结果缺少回放所需关节时抛出 ``ValueError``,交由上层中止本次回放.
    """

    def __init__(
        self,
        *,
        planner: Any,
        trajectory_point_factory: Callable[[], Any],
    ) -> None:
        self._planner = planner
        self._trajectory_point_factory = trajectory_point_factory
        self.last_report = {}

    def append(
        self,
        trajectory: Any,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
        config: MoveItStartAlignmentConfig,
    ) -> float:
        """在 ``trajectory`` 头部追加对齐段,返回追加后轨迹的总时长(秒).

        参数:
        - ``trajectory``:就地修改的回放轨迹,需已设置 ``joint_names``(关节顺序以它
          为准,规划结果的关节顺序会被重映射到该顺序);
        - ``current_positions``:当前关节角(rad),作为起点保持点与规划起点;
        - ``first_positions``:示教轨迹第一个采样点的关节角(rad);
        - ``config``:对齐参数,见 :class:`MoveItStartAlignmentConfig`.

        返回值为累计时长:起点保持 + 对齐段实际时长 + 首点保持,可继续用于后续追加
        轨迹点的相对时间基准.偏差小于阈值时跳过对齐段,只保留保持点.
        """
        if config.reference_grade not in ('q', 'q/v', 'q/v/a'):
            raise ValueError('invalid teach alignment reference grade')
        self.last_report = dict(state='skipped', reference_grade=config.reference_grade,
                                message='未规划对齐段')
        elapsed = max(float(config.start_hold_sec), 0.0)
        hold_point = self._trajectory_point_factory()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions] if config.reference_grade != 'q' else []
        hold_point.accelerations = [0.0 for _ in current_positions] if config.reference_grade == 'q/v/a' else []
        set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        # 逐关节取最大绝对偏差(rad)作为是否需要对齐的判据:只要有一个关节差得多,
        # 就必须规划过渡,否则机械臂会在回放起点发生跳变.
        max_error = max(
            (abs(float(a) - float(b))
                for name, a, b in zip(trajectory.joint_names, current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(config.skip_threshold):
            elapsed = self._append_plan_points(
                trajectory,
                first_positions=first_positions,
                elapsed=elapsed,
                config=config,
            )
        first_hold = max(float(config.first_hold_sec), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = self._trajectory_point_factory()
            first_point.positions = [float(v) for v in first_positions]
            for field in ('velocities', 'accelerations'):
                present = bool(getattr(trajectory.points[-1], field, ()))
                setattr(first_point, field, [0.]*len(first_positions) if present else [])
            set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _append_plan_points(
        self,
        trajectory: Any,
        *,
        first_positions: tuple[float, ...],
        elapsed: float,
        config: MoveItStartAlignmentConfig,
    ) -> float:
        """请求对齐规划并把规划点重映射后追加到轨迹,返回追加后的累计时长.

        规划按回放轨迹的关节顺序发起,目标为 ``first_positions``;返回的规划点按关节
        名重映射(规划器可能给出不同顺序),缺少任一所需关节即视为无效规划并抛
        ``ValueError``.每个规划点的时间戳都加上 ``elapsed`` 偏移(规划自身从 0 计时),
        因此结果与前方保持段无缝衔接.q/v 示教显式取消 a 约束并重建三次曲线,
        中间 v 保留,首末 v 归零;不补造缺失字段,完整曲线重新限值和碰撞检查.
        """
        plan = self._planner.plan_joint_positions(
            joint_names=tuple(trajectory.joint_names),
            target_positions=first_positions,
            tolerance=float(config.joint_goal_tolerance),
            velocity_scaling=float(config.velocity_scaling),
            acceleration_scaling=float(config.acceleration_scaling),
        )
        if not plan.success or plan.trajectory is None:
            raise ValueError(f"moveit start alignment failed: {plan.message}")
        source_names = list(getattr(plan.trajectory, "joint_names", []))
        if len(set(source_names)) != len(source_names):
            raise ValueError('duplicate MoveIt start alignment joints')
        index_by_name = {name: index for index, name in enumerate(source_names)}
        missing = [name for name in trajectory.joint_names if name not in index_by_name]
        if missing:
            raise ValueError(f"moveit start alignment missing joints: {', '.join(missing)}")
        source_points = list(getattr(plan.trajectory, 'points', []))
        if not source_points:
            raise ValueError('empty MoveIt start alignment')
        for p in source_points:
            if len(p.positions) != len(source_names) or any(
                    getattr(p, field, ()) and len(getattr(p, field)) != len(source_names)
                    for field in ('velocities','accelerations')):
                raise ValueError('MoveIt start alignment field dimensions differ')
        from .polynomial_trajectory import TrajectoryPoint
        from .trajectory_time_parameterization import teach_input_grade, prepare_teach_reference
        order = [index_by_name[name] for name in trajectory.joint_names]
        # 先验证完整规划字段,再显式适配;不能靠删除 a 掩盖原规划的混合缺失.
        original = tuple(TrajectoryPoint(
            p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
            tuple(p.positions[i] for i in order),
            tuple(p.velocities[i] for i in order) if p.velocities else (),
            tuple(p.accelerations[i] for i in order) if getattr(p,'accelerations',()) else ()) for p in source_points)
        source_grade = teach_input_grade(original)
        if config.reference_grade != 'q' and not original[0].velocities:
            raise ValueError('MoveIt teach alignment requires planned velocities')
        if config.reference_grade == 'q/v/a' and not original[0].accelerations:
            raise ValueError('MoveIt teach alignment requires planned accelerations')
        aligned = prepare_teach_reference(tuple(TrajectoryPoint(p.time, p.positions,
            p.velocities if config.reference_grade != 'q' else (),
            p.accelerations if config.reference_grade == 'q/v/a' else ()) for p in original))
        self.last_report = dict(state='converted' if source_grade != config.reference_grade else 'preserved',
            input_grade=source_grade, reference_grade=config.reference_grade,
            message=f'对齐段由 {source_grade} 转换为 {config.reference_grade}；重建后重新限值及碰撞检查',
            original_points=[dict(time=p.time,positions=list(p.positions),velocities=list(p.velocities),
                                  accelerations=list(p.accelerations)) for p in original])
        for source_point in aligned:
            # 规划时间戳的 sec/nanosec 分量为整秒 + 纳秒,需合并为浮点秒再加偏移,
            # 不能只取 sec,否则亚秒部分会丢失.
            source_time = source_point.time
            point = self._trajectory_point_factory()
            point.positions = list(source_point.positions)
            point.velocities = list(source_point.velocities)
            point.accelerations = list(source_point.accelerations)
            set_duration(point.time_from_start, elapsed + source_time)
            if source_time == 0.:
                # t=0 已由前一保持点占据;只接受相同位置,避免制造重复时间或零时长跳变.
                if tuple(point.positions) != tuple(trajectory.points[-1].positions):
                    raise ValueError('MoveIt start alignment initial position mismatch')
                continue
            trajectory.points.append(point)
        # 以最后一个规划点的时间戳回算累计时长,保证返回值与轨迹实际时间轴一致.
        if trajectory.points:
            last = trajectory.points[-1].time_from_start
            elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        return elapsed
