"""示教采样点的轨迹时间参数化(重定时)策略选择.

职责与位置
    运动层的重定时策略选择与降级报告; 计算实现通过回调注入.

对外接口
    parameterize_teach_samples 返回 TimeParameterizationResult.
    requested_method 与 used_method 分别记录请求方法和实际方法.

安全与并发约束
    模块可导入不代表保路径适配器已实现; 必须报告真实采用的方法. 这里只生成数据, 不执行.

文件结构(按定义顺序)
    TimeParameterizationResult: 时间参数化结果与"请求/实际"方法记录.
    normalize_time_parameterization_method: 把外部传入的方法名归一化为三种合法取值之一.
    ruckig_python_available: 探测运行环境是否可导入 ruckig 的 Python 模块(不抛异常, 仅返回布尔值).
    ruckig_waypoint_adapter_available: 探测"保持示教路径点"的 ruckig 适配器是否可用.
    parameterize_teach_samples: 按策略选择时间参数化方法并产出轨迹点.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence


@dataclass(frozen=True)
class TimeParameterizationResult:
    """时间参数化结果与"请求/实际"方法记录.

    - ``points``: 重定时后的轨迹点列表, 元素类型由 ``fallback_retime`` 决定(通常带
      ``positions``/``velocities``/``time_from_start``);
    - ``requested_method``: 归一化后的请求方法, 取值 ``auto``/``current_jerk_retime``/
      ``ruckig``;
    - ``used_method``: 实际生效的方法, 取值 ``current_jerk_retime`` 或 ``ruckig``;
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
