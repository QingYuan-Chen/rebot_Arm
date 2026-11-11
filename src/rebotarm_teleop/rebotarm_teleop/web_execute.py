"""Web 遥操作「关节目标 / 夹爪目标」请求的纯校验。

本模块是 Web 面板下发点对点运动前的第一道安全门，把校验逻辑集中在此处，
不导入任何 ROS 运行时，因此可以在没有 ROS 的环境下做完整单元测试。

对外接口（全部为纯函数或不可变数据类）：
- ``validate_web_execute_request``：校验关节目标并产出 ``WebExecuteDecision``；
- ``validate_web_gripper_request``：校验夹爪目标并产出 ``WebGripperDecision``；

校验顺序（任一步失败立即返回 accepted=False，不产生任何下发副作用）：
1. 确认串：关节运动必须携带 ``confirm="EXECUTE"``，夹爪必须携带
   ``confirm="SET_GRIPPER"``，大小写不敏感——防止误触发/重复提交；
2. 数值合法性：目标必须是有限值，且逐关节落在关节限位内，越界或非有限值一律拒绝；
3. 基于实时反馈的增量门：缺少任一关节的当前状态、或单关节增量超过 max_delta_rad 上限
   （可被请求里的 max_delta_rad 收紧，但不能放宽）时拒绝；
4. 时长与速度门：duration 被夹到 [min_duration, max_duration]，再按关节速度上限
   （取「服务端配置」「请求值」「逐关节配置」三者最小值）反查是否超速。

单位约定：关节角用弧度（rad），角速度用 rad/s，时长用秒（s），夹爪开口用米（m），
夹爪力矩上限为无量纲归一化值（量纲由夹爪固件定义）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WebExecuteDecision:
    """关节目标校验结论；``accepted=False`` 时只填 message，其余字段保持默认。"""

    accepted: bool
    message: str
    joint_names: tuple[str, ...] = ()
    positions: tuple[float, ...] = ()
    max_delta: float = 0.0
    max_delta_limit: float = 0.0
    duration: float = 0.0


@dataclass(frozen=True)
class WebGripperDecision:
    """夹爪目标校验结论；``position`` 单位为 m，``max_effort`` 为归一化力矩上限。"""

    accepted: bool
    message: str
    position: float = 0.0
    max_effort: float = 0.0


def validate_web_execute_request(
    payload: dict[str, Any],
    *,
    joint_names: tuple[str, ...],
    current_positions: dict[str, float],
    joint_limits: dict[str, tuple[float, float]],
    max_delta_rad: float,
    min_duration: float,
    max_duration: float,
    joint_velocity_limits: dict[str, float] | None = None,
    max_joint_speed_rad_s: float | None = None,
) -> WebExecuteDecision:
    """校验一次 Web 关节目标请求，返回是否放行及归一化后的目标。

    参数：
    - ``payload``：前端 JSON 字典，需含 ``confirm="EXECUTE"`` 与
      ``joint_positions``（关节名 → 目标角 rad）；可选 ``max_delta_rad``（单关节增量
      上限 rad，只会被收紧）、``duration``（期望时长 s）、``max_joint_speed_rad_s``
      （请求速度上限 rad/s，只会被收紧）。
    - ``current_positions``：实时关节反馈（关节名 → 当前角 rad），缺失即拒绝；
    - ``joint_limits``：完整的关节名 → (下限, 上限) rad 映射；
    - ``max_delta_rad`` / ``min_duration`` / ``max_duration``：服务端安全上限与时长区间；
    - ``joint_velocity_limits`` / ``max_joint_speed_rad_s``：逐关节与服务端速度上限
      （rad/s），二者都存在时逐关节取更小值。

    返回值：放行时带上关节名、目标角、本次实际最大增量、生效的增量上限与夹紧后的时长；
    拒绝时 message 说明原因（中文语义见上，文本本身是对外契约，保持英文原样）。
    本函数不发送任何 ROS 消息，调用方拿到 accepted=True 后才会组轨迹下发。
    """
    if str(payload.get("confirm", "")).strip().upper() != "EXECUTE":
        return WebExecuteDecision(False, "missing EXECUTE confirmation")

    raw_targets = payload.get("joint_positions")
    if not isinstance(raw_targets, dict):
        return WebExecuteDecision(False, "joint_positions must be an object")

    # 请求可以附带更严格的增量上限，但通过 min() 永远不能放宽服务端配置的上限。
    max_delta_limit = float(max_delta_rad)
    requested_max_delta = payload.get("max_delta_rad")
    if requested_max_delta is not None:
        try:
            requested_max_delta_value = float(requested_max_delta)
        except (TypeError, ValueError):
            return WebExecuteDecision(False, "invalid max_delta_rad")
        if not math.isfinite(requested_max_delta_value) or requested_max_delta_value <= 0.0:
            return WebExecuteDecision(False, "invalid max_delta_rad")
        max_delta_limit = min(requested_max_delta_value, max_delta_limit)

    positions: list[float] = []
    max_delta = 0.0
    missing_current: list[str] = []
    for name in joint_names:
        if name not in raw_targets:
            return WebExecuteDecision(False, f"missing target for {name}")
        # JSON null 与键缺失等价处理：都必须显式给出每个关节的目标角。
        if raw_targets[name] is None:
            return WebExecuteDecision(False, f"missing target for {name}")
        try:
            target = float(raw_targets[name])
        except (TypeError, ValueError):
            return WebExecuteDecision(False, f"invalid target for {name}")
        # 拒绝 NaN/Inf：它们能绕过下面的区间比较，必须在比较前挡住。
        if not math.isfinite(target):
            return WebExecuteDecision(False, f"non-finite target for {name}")

        lower, upper = joint_limits[name]
        if upper < lower:
            lower, upper = upper, lower
        if target < lower or target > upper:
            return WebExecuteDecision(False, f"{name} target outside joint limit")

        if name not in current_positions:
            missing_current.append(name)
        else:
            current = float(current_positions[name])
            if not math.isfinite(current):
                return WebExecuteDecision(False, f"non-finite live joint state: {name}")
            max_delta = max(max_delta, abs(target - current))
        positions.append(target)

    # 增量门依赖实时反馈；反馈不全时不做「以 0 为基准」的乐观估算，直接拒绝。
    if missing_current:
        return WebExecuteDecision(False, f"missing live joint state: {', '.join(missing_current)}")
    if max_delta > max_delta_limit:
        return WebExecuteDecision(
            False,
            f"target delta too large: {max_delta:.4f} rad > {max_delta_limit:.4f} rad",
        )

    # 时长缺失时采用配置下限；后面的速度门继续检查该时长是否足够。
    try:
        duration = float(payload.get("duration", min_duration))
    except (TypeError, ValueError):
        return WebExecuteDecision(False, "invalid duration")
    if not math.isfinite(duration):
        return WebExecuteDecision(False, "invalid duration")
    duration = min(max(duration, float(min_duration)), float(max_duration))

    # 速度上限取三者最小值：服务端配置 / 请求值 / 逐关节配置（后两者可缺省）。
    requested_speed_limit = payload.get("max_joint_speed_rad_s", max_joint_speed_rad_s)
    speed_limit = float(max_joint_speed_rad_s) if max_joint_speed_rad_s is not None else None
    if requested_speed_limit is not None:
        try:
            requested_speed_value = float(requested_speed_limit)
        except (TypeError, ValueError):
            return WebExecuteDecision(False, "invalid max_joint_speed_rad_s")
        if not math.isfinite(requested_speed_value) or requested_speed_value <= 0.0:
            return WebExecuteDecision(False, "invalid max_joint_speed_rad_s")
        speed_limit = requested_speed_value if speed_limit is None else min(speed_limit, requested_speed_value)

    if speed_limit is not None:
        for name, target in zip(joint_names, positions):
            current = current_positions[name]
            # 平均速度估算：增量已在上面通过增量门，这里只判「时长是否够走完」。
            required_speed = abs(float(target) - float(current)) / duration
            configured_limit = None
            if joint_velocity_limits is not None and name in joint_velocity_limits:
                configured_limit = float(joint_velocity_limits[name])
            joint_limit = speed_limit if configured_limit is None else min(speed_limit, configured_limit)
            if required_speed > joint_limit:
                # 报错文本里直接给出满足限速所需的最短时长，方便前端提示操作者改参数。
                min_duration_needed = abs(float(target) - float(current)) / max(joint_limit, 1e-9)
                return WebExecuteDecision(
                    False,
                    (
                        f"{name} speed too high: {required_speed:.4f} rad/s > "
                        f"{joint_limit:.4f} rad/s; use duration >= {min_duration_needed:.2f}s"
                    ),
                )

    return WebExecuteDecision(
        True,
        f"web preview execution accepted: max_delta={max_delta:.4f} rad",
        joint_names=joint_names,
        positions=tuple(positions),
        max_delta=max_delta,
        max_delta_limit=max_delta_limit,
        duration=duration,
    )


def validate_web_gripper_request(
    payload: dict[str, Any],
    *,
    gripper_limits: tuple[float, float],
    default_max_effort: float,
    max_effort_limit: float,
) -> WebGripperDecision:
    """校验一次 Web 夹爪目标请求，返回是否放行及归一化后的目标。

    ``payload`` 需含 ``confirm="SET_GRIPPER"`` 与 ``position``（夹爪开口，m）；
    可选 ``max_effort``（归一化力矩上限），缺省用 ``default_max_effort``，
    并且无论请求怎么写都不会超过 ``max_effort_limit``——夹爪力矩上限是抓取安全的关键，
    因此这里只允许收紧不允许放宽。开合上下限写反时先排序再判断，超出范围即拒绝。
    """
    if str(payload.get("confirm", "")).strip().upper() != "SET_GRIPPER":
        return WebGripperDecision(False, "missing SET_GRIPPER confirmation")
    try:
        position = float(payload.get("position"))
    except (TypeError, ValueError):
        return WebGripperDecision(False, "invalid gripper position")
    if not math.isfinite(position):
        return WebGripperDecision(False, "invalid gripper position")

    lower, upper = float(gripper_limits[0]), float(gripper_limits[1])
    if upper < lower:
        lower, upper = upper, lower
    if position < lower or position > upper:
        return WebGripperDecision(False, f"gripper target outside limit: {lower:.4f}..{upper:.4f} m")

    raw_effort = payload.get("max_effort", default_max_effort)
    try:
        max_effort = float(raw_effort)
    except (TypeError, ValueError):
        return WebGripperDecision(False, "invalid gripper max_effort")
    if not math.isfinite(max_effort) or max_effort <= 0.0:
        return WebGripperDecision(False, "invalid gripper max_effort")
    max_effort = min(max_effort, float(max_effort_limit))

    return WebGripperDecision(
        True,
        f"web gripper target accepted: position={position:.4f} m",
        position=position,
        max_effort=max_effort,
    )
