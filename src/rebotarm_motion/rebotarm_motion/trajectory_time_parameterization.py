"""示教派生曲线的导数重建、完整曲线重定时及后端策略报告。

职责与位置
    运动层按输入等级生成参考导数、整体时间缩放及拼接后模型限值复核。
    不负责录制文件、界面、执行或碰撞场景；旧局部维度分析继续通过回调注入。

对外接口
    parameterize_teach_samples 返回 TimeParameterizationResult.
    requested_method 与 used_method 分别记录请求方法和实际方法.
    teach_input_grade 校验原始字段；prepare_teach_reference 保持等级生成参考约束；
    parameterize_teach_waypoints 生成六轴候选；
    finalize_teach_curve 检查模型及示教限值，并复核纳秒编码后的实际曲线。

安全与并发约束
    不对中间点一律补零，不静默删除已有 MoveIt 导数。滤波/重建可能改变节点间路径，
    调用者须检查实际生成曲线的碰撞；时间缩放不能修复位置越界。这里只生成数据，不执行。

文件结构(按定义顺序)
    输入分级与参考导数 → 六轴重定时 → 完整曲线复核 → 方法记录。
    normalize_time_parameterization_method: 把外部传入的方法名归一化为三种合法取值之一.
    ruckig_python_available: 探测运行环境是否可导入 ruckig 的 Python 模块(不抛异常, 仅返回布尔值).
    ruckig_waypoint_adapter_available: 探测"保持示教路径点"的 ruckig 适配器是否可用.
    parameterize_teach_samples: 按策略选择时间参数化方法并产出轨迹点.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

from .polynomial_trajectory import TrajectoryPoint


def teach_input_grade(points):
    """在任何滤波/字段变换之前校验结构，返回 q、q/v 或 q/v/a，不补造缺失字段。"""
    points = tuple(points)
    if not points:
        raise ValueError('teach trajectory points required')
    width = len(points[0].positions)
    if not width or any(len(p.positions) != width for p in points):
        raise ValueError('teach point dimensions differ')
    if any(not math.isfinite(p.time) or p.time < 0 for p in points):
        raise ValueError('teach times must be finite and nonnegative')
    if any(b.time <= a.time for a, b in zip(points, points[1:])):
        raise ValueError('teach times must strictly increase')
    for field in ('velocities', 'accelerations'):
        present = [bool(getattr(p, field)) for p in points]
        if any(present) and not all(present):
            raise ValueError(f'mixed missing {field} in teach reconstruction')
        if any(getattr(p, field) and len(getattr(p, field)) != width for p in points):
            raise ValueError(f'teach {field} dimensions differ')
    if any(not math.isfinite(x) for p in points
           for x in (*p.positions, *p.velocities, *p.accelerations)):
        raise ValueError('teach values must be finite')
    if points[0].accelerations and not points[0].velocities:
        raise ValueError('teach accelerations require velocities')
    return 'q/v/a' if points[0].accelerations else 'q/v' if points[0].velocities else 'q'


def prepare_teach_reference(points, *, derive_velocity=False, zero_endpoints=True):
    """保留输入等级；位置处理后仅重建该等级已有的导数约束。

    q/v 用非均匀三点差分生成派生参考 v，绝不补 a。q/v/a 才允许重建 a；
    未处理位置时保留中间原导数。端点已有导数归零用于连接保持，缺项仍为空。
    这只是候选，调用方必须检查重建后实际路径的解析极值和碰撞。
    """
    points = tuple(points)
    grade = teach_input_grade(points)
    width = len(points[0].positions)
    zeros = (0.,)*width

    def differentiate(values):
        result = [zeros]*len(points)
        for i in range(1, len(points)-1):
            left, right = points[i].time-points[i-1].time, points[i+1].time-points[i].time
            result[i] = tuple((right*(b-a)/left + left*(c-b)/right)/(left+right)
                for a,b,c in zip(values[i-1], values[i], values[i+1]))
        return result

    velocities = (differentiate([p.positions for p in points]) if derive_velocity and grade != 'q'
                  else [p.velocities for p in points])
    if zero_endpoints and grade != 'q':
        velocities[0] = velocities[-1] = zeros
    accelerations = (differentiate(velocities) if derive_velocity and grade == 'q/v/a'
                     else [p.accelerations for p in points])
    if zero_endpoints and grade == 'q/v/a':
        accelerations[0] = accelerations[-1] = zeros
    return tuple(TrajectoryPoint(p.time, p.positions, v, a)
                 for p,v,a in zip(points, velocities, accelerations))


def parameterize_teach_waypoints(names, samples, velocity_limits, acceleration_limit, jerk_limit,
                                 *, replay_speed, initial_delay, zero_endpoints=True):
    """按参考节点的原等级构建曲线并整体放慢，避免逐段改时造成导数失真。

    参考导数已由调用方按位置处理策略生成，不能在这里升级输入等级。
    所有位置节点保留；只缩放时间及已有导数。这里检查段内动态极值，完整位置限位和拼接后的
    纳秒曲线检查由回放工作流再做；重采样不额外增加路径控制节点。
    """
    from .polynomial_trajectory import build_curve
    base = prepare_teach_reference(tuple(TrajectoryPoint(
        (s.stamp-samples[0].stamp)/replay_speed, s.positions,
        tuple(v*replay_speed for v in s.velocities),
        tuple(a*replay_speed**2 for a in s.accelerations)) for s in samples),
        zero_endpoints=zero_endpoints)
    curve = build_curve(names, base)
    scale = 1.
    for segment in curve.segments:
        for j, velocity_limit in enumerate(velocity_limits):
            for order, bound in ((1,velocity_limit),(2,acceleration_limit),(3,jerk_limit)):
                if not math.isfinite(bound) or bound <= 0:
                    raise ValueError('positive finite teach dynamic limits required')
                peak = max(abs(segment.at(u,order)[j]) for u in segment.candidates(j,order))
                scale = max(scale,(peak/bound)**(1/order))
    scale *= 1.+1e-6
    return tuple(TrajectoryPoint(initial_delay+p.time*scale, p.positions,
        tuple(v/scale for v in p.velocities), tuple(a/scale**2 for a in p.accelerations)) for p in base)


def finalize_teach_curve(names, points, config_directory, velocity_limits, acceleration_limit, jerk_limit, *, fixed=False):
    """拼接完成后按模型和示教限值的较小值重定时并复核纳秒编码后的整曲线。"""
    from dataclasses import replace
    from .trajectory_preparation import load_reference_limits
    from .coordinated_trajectory import retime_curve
    lower, upper, limits = load_reference_limits(config_directory)
    limits = {n:replace(limits[n],max_velocity=min(limits[n].max_velocity,v),
        max_acceleration=min(limits[n].max_acceleration,acceleration_limit),
        max_jerk=min(limits[n].max_jerk,jerk_limit)) for n,v in zip(names,velocity_limits)}
    return retime_curve(names, points, lower,upper,limits,fixed=fixed)


def teach_curve_analysis(curve):
    """报告实际曲线的段内解析峰值及节点左右加速度差，不把差分尖峰当作 jerk。"""
    peaks = {kind: max((abs(segment.at(u, order)[j])
        for segment in curve.segments for j in range(len(curve.joint_names))
        for u in segment.candidates(j, order)),
        default=max(map(abs,curve.points[0].positions)) if order == 0 else 0.)
        for order, kind in enumerate(('position', 'velocity', 'acceleration', 'jerk'))}
    jumps = [dict(time=right.start,
        left=list(left.at(1., 2)), right=list(right.at(0., 2)),
        delta=[b-a for a, b in zip(left.at(1., 2), right.at(0., 2))])
        for left, right in zip(curve.segments, curve.segments[1:])]
    return dict(degree=curve.degree, continuity=curve.continuity(), segment_peaks=peaks,
        acceleration_jumps=jumps, dynamic_scope='segment_interior',
        derivative_limits_cover_node_jumps=curve.degree == 5,
        note=('C1：段内 jerk 合格不代表全程有界 jerk' if curve.degree == 3
              else '按输入等级承诺连续性；段内极值与节点连续性分别报告'))


@dataclass(frozen=True)
class TimeParameterizationResult:
    """时间参数化结果与"请求/实际"方法记录.

    - ``points``: 重定时后的轨迹点列表, 元素类型由 ``fallback_retime`` 决定(通常带
      ``positions``/``velocities``/``time_from_start``);
    - ``requested_method``: 归一化后的请求方法, 取值 ``auto``/``current_jerk_retime``/
      ``ruckig``;
    - ``used_method``: 实际方法；六轴工作流按等级标为 ``c0/c1/c2_waypoint_retime``，
      旧局部分析保留 ``current_jerk_retime``；不能把缺失的适配器报告成 ``ruckig``；
    - ``message``: 降级/选择原因, 直接透传到状态面板供运维判断.
    """

    points: list
    requested_method: str
    used_method: str
    message: str


def normalize_time_parameterization_method(method: str | None) -> str:
    """把外部传入的方法名归一化为三种合法取值之一.

    规则: ``None``/空串/``"default"`` 以及任何无法识别的取值都回落为 ``"auto"``,
    不抛异常; 参数来自 YAML 或 launch, 非法值不应让节点启动失败. 比较前会去掉首尾
    空白并转小写, 因此 ``"Auto"``, ``" RUCKIG "`` 均能正确识别.
    """
    value = str(method or "auto").strip().lower()
    if value in ("", "default"):
        return "auto"
    if value not in ("auto", "current_jerk_retime", "ruckig"):
        return "auto"
    return value


def ruckig_python_available() -> bool:
    """探测运行环境是否可导入 ruckig 的 Python 模块(不抛异常, 仅返回布尔值)."""
    try:
        import ruckig  # noqa: F401
    except Exception:
        return False
    return True


def ruckig_waypoint_adapter_available() -> bool:
    """探测"保持示教路径点"的 ruckig 适配器是否可用.

    当前恒为 ``False``: 适配器尚未实现, 缺少它时不能用 ruckig 重定时, 否则会改变原
    始路径点, 破坏示教复现精度. 等适配器落地后再返回真实探测结果.
    """
    return False


def parameterize_teach_samples(
    samples: Sequence,
    *,
    method: str | None,
    fallback_retime: Callable[..., list],
    replay_speed: float,
    max_velocity_rad_s: float,
    max_acceleration_rad_s2: float,
    max_jerk_rad_s3: float,
    initial_delay_sec: float = 0.0,
    boundary_zero_velocity: bool = True,
) -> TimeParameterizationResult:
    """按策略选择时间参数化方法并产出轨迹点.

    参数:
    - ``samples``: 已平滑/滤波/重采样后的示教采样点序列, 仅作为数据输入;
    - ``method``: 请求的方法名, 先经 :func:`normalize_time_parameterization_method`
      归一化;
    - ``fallback_retime``: 实际重定时实现, 以关键字参数接收 ``samples``(列表副本),
      ``replay_speed``, 三个上限, ``initial_delay_sec``, ``boundary_zero_velocity``;
    - ``replay_speed``: 回放速度倍率, 时间轴缩放系数(无量纲, 1.0 表示按录制速度);
    - ``max_velocity_rad_s``: 关节速度上限(rad/s);
    - ``max_acceleration_rad_s2``: 关节加速度上限(rad/s²);
    - ``max_jerk_rad_s3``: 关节加加速度上限(rad/s³);
    - ``initial_delay_sec``: 轨迹起点前的静止延时(秒), 用于等待控制器进入跟踪状态;
    - ``boundary_zero_velocity``: 是否强制首末点速度为 0, 保证起停平滑且终点可控.

    无论走哪条分支, 都不会原地修改调用方的 ``samples``(每次都传 ``list(samples)``
    副本). 返回 :class:`TimeParameterizationResult`; 不会因为 ruckig 缺失而失败.
    """
    requested = normalize_time_parameterization_method(method)
    use_ruckig = (
        (requested == "ruckig" or (requested == "auto" and ruckig_python_available()))
        and ruckig_waypoint_adapter_available()
    )
    if use_ruckig:
        points = fallback_retime(
            list(samples),
            replay_speed=replay_speed,
            max_velocity_rad_s=max_velocity_rad_s,
            max_acceleration_rad_s2=max_acceleration_rad_s2,
            max_jerk_rad_s3=max_jerk_rad_s3,
            initial_delay_sec=initial_delay_sec,
            boundary_zero_velocity=boundary_zero_velocity,
        )
        return TimeParameterizationResult(
            points=points,
            requested_method=requested,
            used_method="ruckig",
            message="ruckig python module available; current waypoint-preserving adapter used",
        )

    points = fallback_retime(
        list(samples),
        replay_speed=replay_speed,
        max_velocity_rad_s=max_velocity_rad_s,
        max_acceleration_rad_s2=max_acceleration_rad_s2,
        max_jerk_rad_s3=max_jerk_rad_s3,
        initial_delay_sec=initial_delay_sec,
        boundary_zero_velocity=boundary_zero_velocity,
    )
    # 三条 message 分支按优先级: 显式请求 ruckig 但适配器缺失(区分 ruckig 模块是否
    # 装上, 便于运维定位) -> 其余情况按当前重定时实现执行.
    message = (
        "python ruckig waypoint adapter not implemented; used current jerk-aware retime"
        if requested == "ruckig" and ruckig_python_available()
        else "ruckig python module unavailable; used current jerk-aware retime"
        if requested == "ruckig"
        else "used current jerk-aware retime"
    )
    return TimeParameterizationResult(
        points=points,
        requested_method=requested,
        used_method="current_jerk_retime",
        message=message,
    )
