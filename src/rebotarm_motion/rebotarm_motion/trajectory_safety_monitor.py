"""示教回放的运行期跟踪校验: 无状态纯函数.

职责与位置
    本模块只比较期望轨迹与实测关节位置, 速度, 不持有执行状态或下发控制命令.
    ReplayRuntimeMonitor 在此基础上管理起始宽限, 连续违规和一次停止请求.

对外接口
    expected_positions_at 求指定相对时刻的期望位置.
    evaluate_replay_tracking 返回 ReplayTrackingResult, 描述单次判定及原因.

安全与并发约束
    轨迹可为映射或消息对象; 取数辅助函数兼容两种形态.
    缺少轨迹等情况以结构化原因返回, 不意味着允许继续执行.
    新鲜度, 状态有效性和实际停止由调用链处理; 本模块不构成独立安全监测.

统一约定
    旋转关节位置和误差为 rad, 速度为 rad/s, elapsed_sec 为相对秒.
    期望位置按关节名对齐后比较, 不依赖实测数据的排列顺序.

文件结构(按定义顺序)
    ReplayTrackingResult: 不可变的单次跟踪结论.
    _duration_to_sec: 将数值或 sec/nanosec 时长转换为秒.
    _point_time: 取轨迹点相对起始时间, 缺失时为 0.0.
    _point_positions: 取轨迹点位置序列.
    _trajectory_joint_names: 取轨迹声明的关节顺序.
    _trajectory_points: 取轨迹点列表.
    expected_positions_at: 按相对时间插值期望位置.
    evaluate_replay_tracking: 比较实测位置和速度与配置边界.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class ReplayTrackingResult:
    """一次跟踪校验的判决结果(不可变).

    - `ok`: 是否在全部容差内; False 时 `reason` 给出机器可读原因;
    - `reason`: `ok` / `missing_trajectory` / `missing_joint_state` /
      `tracking_error` / `live_velocity` 之一, 上层据此分类告警;
    - `message`: 面向人的说明文本(英文, 用于日志与报告);
    - `worst_joint`: 误差或速度最大的关节名; 速度违规时指向速度最大的关节;
    - `max_tracking_error_rad`: 本帧最大位置偏差, 单位 rad;
    - `max_live_velocity_rad_s`: 本帧最大实测关节速度, 单位 rad/s.
    """

    ok: bool
    reason: str
    message: str
    worst_joint: str = ""
    max_tracking_error_rad: float = 0.0
    max_live_velocity_rad_s: float = 0.0


def _duration_to_sec(value) -> float:
    """把时长转成秒: 既接受数值(浮点秒), 也接受含 sec/nanosec 的时长字段."""
    if isinstance(value, (int, float)):
        return float(value)
    return float(getattr(value, "sec", 0)) + float(getattr(value, "nanosec", 0)) * 1e-9


def _point_time(point) -> float:
    """取轨迹点的起始时刻, 单位 s; 缺失时按 0.0 处理."""
    if isinstance(point, dict):
        return _duration_to_sec(point.get("time_from_start", 0.0))
    return _duration_to_sec(getattr(point, "time_from_start", 0.0))


def _point_positions(point) -> tuple[float, ...]:
    """取轨迹点的关节角序列, 单位 rad."""
    if isinstance(point, dict):
        return tuple(float(v) for v in point.get("positions", ()))
    return tuple(float(v) for v in getattr(point, "positions", ()))


def _trajectory_joint_names(trajectory) -> tuple[str, ...]:
    """取轨迹声明的关节名顺序; 实测数据按名字对齐到它."""
    if isinstance(trajectory, dict):
        return tuple(str(v) for v in trajectory.get("joint_names", ()))
    return tuple(str(v) for v in getattr(trajectory, "joint_names", ()))


def _trajectory_points(trajectory) -> list:
    """取轨迹点列表(含 time_from_start 与 positions 的序列)."""
    if isinstance(trajectory, dict):
        return list(trajectory.get("points", ()))
    return list(getattr(trajectory, "points", ()))


def expected_positions_at(trajectory, elapsed_sec: float) -> tuple[float, ...]:
    """求 elapsed_sec 时刻的期望关节角, 单位 rad.

    在相邻轨迹点之间做**线性插值**: 控制器内部的插值方式未必是线性的, 因此
    容差必须覆盖这个建模误差. 边界处理:

    - 空轨迹返回空元组(上层据此判 `missing_trajectory`);
    - 早于首点时刻直接返回首点(回放起步前按起点保持);
    - 晚于末点时刻返回末点(回放结束按终点保持);
    - `span` 下限 1e-9 s, 避免两个时间相同的点造成除零.
    """
    points = _trajectory_points(trajectory)
    if not points:
        return ()
    elapsed = max(float(elapsed_sec), 0.0)
    first = points[0]
    first_time = _point_time(first)
    if elapsed <= first_time:
        return _point_positions(first)
    previous = first
    for current in points[1:]:
        current_time = _point_time(current)
        if elapsed <= current_time:
            previous_time = _point_time(previous)
            span = max(current_time - previous_time, 1e-9)
            ratio = min(max((elapsed - previous_time) / span, 0.0), 1.0)
            previous_positions = _point_positions(previous)
            current_positions = _point_positions(current)
            return tuple(
                float(a) + (float(b) - float(a)) * ratio
                for a, b in zip(previous_positions, current_positions)
            )
        previous = current
    return _point_positions(points[-1])


def evaluate_replay_tracking(
    trajectory,
    *,
    joint_names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float] = (),
    elapsed_sec: float,
    max_tracking_error_rad: float,
    max_live_velocity_rad_s: float,
) -> ReplayTrackingResult:
    """判定 elapsed_sec 时刻的实测跟踪是否合规.

    参数:
    - `trajectory`: 正在回放的期望轨迹(映射或消息对象);
    - `joint_names` / `positions`: 实测关节名与关节角(rad), 按位对应;
    - `velocities`: 实测关节速度(rad/s), 可选; 空序列表示跳过速度检查;
    - `elapsed_sec`: 自回放开始起算的时间, s(由调用方用同一时间基准计算);
    - `max_tracking_error_rad`: 单轴位置误差上限, rad;
    - `max_live_velocity_rad_s`: 单轴实测速度上限, rad/s.

    判定顺序: 轨迹非空 -> 实测是否覆盖轨迹全部关节 -> 位置误差 -> 速度.
    位置与速度都取绝对值比较, 任一超限立即返回失败并带上最大违规关节.
    注意: 速度检查是"实测速度上限", 与轨迹自身的速度规划无关; 它为 True 时
    即使位置仍在容差内也会判失败(例如机械臂被外力推着跑).
    """
    trajectory_names = _trajectory_joint_names(trajectory)
    expected = expected_positions_at(trajectory, elapsed_sec)
    if not trajectory_names or not expected:
        return ReplayTrackingResult(False, "missing_trajectory", "active replay trajectory is empty")

    # 实测按关节名建索引: 未出现在实测里的轨迹关节会被判缺失, 而不是错位比较.
    actual_by_name = {
        str(name): float(position)
        for name, position in zip(joint_names, positions)
    }
    missing = [name for name in trajectory_names if name not in actual_by_name]
    if missing:
        return ReplayTrackingResult(
            False,
            "missing_joint_state",
            f"joint_states missing replay joints: {', '.join(missing)}",
        )

    worst_joint = ""
    max_error = 0.0
    for name, target in zip(trajectory_names, expected):
        error = abs(float(actual_by_name[name]) - float(target))
        if error > max_error:
            max_error = error
            worst_joint = str(name)
    if max_error > float(max_tracking_error_rad):
        return ReplayTrackingResult(
            False,
            "tracking_error",
            (
                f"{worst_joint} tracking error {max_error:.4f} rad > "
                f"{float(max_tracking_error_rad):.4f} rad"
            ),
            worst_joint=worst_joint,
            max_tracking_error_rad=max_error,
        )

    if velocities:
        velocity_by_name = {
            str(name): abs(float(velocity))
            for name, velocity in zip(joint_names, velocities)
        }
        worst_velocity_joint = ""
        max_velocity = 0.0
        for name in trajectory_names:
            velocity = float(velocity_by_name.get(name, 0.0))
            if velocity > max_velocity:
                max_velocity = velocity
                worst_velocity_joint = str(name)
        if max_velocity > float(max_live_velocity_rad_s):
            return ReplayTrackingResult(
                False,
                "live_velocity",
                (
                    f"{worst_velocity_joint} live velocity {max_velocity:.4f} rad/s > "
                    f"{float(max_live_velocity_rad_s):.4f} rad/s"
                ),
                worst_joint=worst_velocity_joint,
                max_live_velocity_rad_s=max_velocity,
            )

    return ReplayTrackingResult(
        True,
        "ok",
        "replay tracking within limits",
        worst_joint=worst_joint,
        max_tracking_error_rad=max_error,
    )
