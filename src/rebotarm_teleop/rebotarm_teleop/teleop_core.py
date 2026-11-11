"""遥操作命令的映射与校验核心(纯逻辑层,不依赖 ROS 运行时).

系统位置:
- 本模块把"按键 / Web 键盘请求"翻译成关节目标角,是操作者意图进入运动链路的第一道加工,
  只做映射,限位与限速判断,不发送任何动作,也不访问硬件.
- 键盘节点与 Web 面板共用这里的映射器和规划器,保证两条操作路径的步长,
  限位与限速语义完全一致;因为没有 ROS 依赖,可以直接做单元测试.

主要组成:
- ``KeyboardCommandMapper``:按键 → (关节名, 方向) 映射;
- ``TeleopTargetPlanner``:在关节限位内对单个关节施加一步增量;
- ``validate_web_keyboard_command``:Web 键盘单步命令的完整校验链.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


# 默认键位表:按键 → (关节名, 方向).数字键为正向,紧随其后的字母键为反向;
# 方向只取 ±1.0,具体步长由运行期参数 joint_step_rad 提供,
# 因此换机械臂时只需改配置而不必改这张表.
DEFAULT_KEY_BINDINGS: dict[str, tuple[str, float]] = {
    "1": ("joint1", 1.0),
    "q": ("joint1", -1.0),
    "2": ("joint2", 1.0),
    "w": ("joint2", -1.0),
    "3": ("joint3", 1.0),
    "e": ("joint3", -1.0),
    "4": ("joint4", 1.0),
    "r": ("joint4", -1.0),
    "5": ("joint5", 1.0),
    "t": ("joint5", -1.0),
    "6": ("joint6", 1.0),
    "y": ("joint6", -1.0),
}


@dataclass(frozen=True)
class KeyboardCommand:
    """一次按键的解析结果:目标关节名 + 运动方向(+1.0 正向 / -1.0 反向)."""

    joint_name: str
    direction: float


class KeyboardCommandMapper:
    """把按键映射成关节运动指令.

    ``joint_names`` 用于过滤:键位表中存在,但当前机械臂没有的关节会被判为未映射,
    这样同一张键位表可以适配不同关节数量的机械臂.
    """

    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        key_bindings: dict[str, tuple[str, float]] | None = None,
    ) -> None:
        """``key_bindings`` 为 None(或空字典)时回退到 ``DEFAULT_KEY_BINDINGS``."""
        self._joint_names = set(joint_names)
        self._bindings = key_bindings or DEFAULT_KEY_BINDINGS

    def command_for_key(self, key: str) -> KeyboardCommand | None:
        """返回按键对应的指令;按键未绑定,或绑定关节不在 ``joint_names`` 中时返回 None."""
        binding = self._bindings.get(key)
        if binding is None:
            return None
        joint_name, direction = binding
        if joint_name not in self._joint_names:
            return None
        return KeyboardCommand(joint_name=joint_name, direction=float(direction))


@dataclass(frozen=True)
class TeleopTarget:
    """一次单步遥操作的目标:全量关节顺序及其目标角度(rad).

    ``positions`` 与 ``joint_names`` 等长且同序,可直接作为轨迹位置向量下发.
    即使 ``accepted=False``,``positions`` 也是当前状态的快照,便于调用方回显.
    """

    accepted: bool
    message: str
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class WebKeyboardCommandDecision:
    """Web 键盘单步命令的校验结果.

    字段含义:
    - ``accepted`` / ``message``:是否放行及人类可读说明(拒绝时其余字段保持默认空值);
    - ``key`` / ``joint_name``:触发的按键与其绑定的关节;
    - ``joint_names`` / ``positions``:校验后的全量目标(rad),顺序与关节列表一致;
    - ``step_rad``:本次生效的步长(rad,正值,已夹紧到允许区间);
    - ``duration``:期望运动时长(s),限速校验用它把增量换算成角速度;
    - ``max_joint_speed_rad_s``:最终生效的速度上限(rad/s),0.0 作为"本次未启用"的哨兵值.
    """

    accepted: bool
    message: str
    key: str = ""
    joint_name: str = ""
    joint_names: tuple[str, ...] = ()
    positions: tuple[float, ...] = ()
    step_rad: float = 0.0
    duration: float = 0.0
    max_joint_speed_rad_s: float = 0.0


class TeleopTargetPlanner:
    """把单关节增量叠加到当前关节状态上,并做限位夹紧.

    只会改动被操作的那一个关节,其余关节保持当前值不变-这是单步遥操作的语义;
    调用方负责把结果作为下一步的当前状态,从而形成连续点动.
    """

    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        joint_limits: dict[str, tuple[float, float]],
        joint_step_rad: float,
    ) -> None:
        """``joint_limits`` 为关节名 → (下限, 上限),单位 rad.

        ``joint_step_rad`` 取绝对值:负步长会让方向键语义反转,这里直接按正步长处理.
        """
        self._joint_names = joint_names
        self._joint_limits = joint_limits
        self._joint_step_rad = abs(float(joint_step_rad))

    def apply_delta(
        self,
        *,
        current_positions: dict[str, float],
        joint_name: str,
        direction: float,
    ) -> TeleopTarget:
        """对 ``joint_name`` 施加 ``direction × joint_step_rad`` 的增量.

        ``direction`` 来自键位表,通常为 ±1.0;每个关节都必须有有限的当前位置.
        越界时夹紧到限位:操作者顶到限位后
        继续按键应保持不动,而不是让命令失效或反向弹回.
        """
        if joint_name not in self._joint_names:
            return TeleopTarget(
                accepted=False,
                message=f"unknown joint: {joint_name}",
                joint_names=self._joint_names,
                positions=(),
            )

        missing = [name for name in self._joint_names if name not in current_positions]
        if missing:
            return TeleopTarget(False, f"missing live joint state: {', '.join(missing)}", self._joint_names, ())
        positions = list(self._ordered_positions(current_positions))
        if not all(math.isfinite(value) for value in positions):
            return TeleopTarget(False, "non-finite live joint state", self._joint_names, ())
        index = self._joint_names.index(joint_name)
        lower, upper = self._joint_limits[joint_name]
        proposed = positions[index] + (float(direction) * self._joint_step_rad)
        # 夹紧到硬限位:先取与下限的较大者,再取与上限的较小者.
        # 注意限位必须按 (下限, 上限) 顺序配置;若配反,结果会固定落在 upper 上.
        clamped = min(max(proposed, lower), upper)
        positions[index] = clamped
        # 被夹紧时提示调用方/操作者:本次实际增量小于请求步长
        message = "teleop target ready"
        if clamped != proposed:
            message = f"teleop target clamped for {joint_name}"
        return TeleopTarget(
            accepted=True,
            message=message,
            joint_names=self._joint_names,
            positions=tuple(positions),
        )

    def _ordered_positions(self, current_positions: dict[str, float]) -> tuple[float, ...]:
        """按 ``joint_names`` 的顺序取出当前角度(rad)."""
        return tuple(float(current_positions[name]) for name in self._joint_names)


def _float_from_payload(
    payload: dict[str, Any],
    key: str,
    default: float,
    *,
    minimum: float,
    maximum: float,
) -> float | None:
    """从 payload 读取一个浮点参数并夹紧到 [minimum, maximum].

    返回 None 表示取值不可用:无法转成 float,或不是有限值(NaN / ±Inf).
    NaN 必须用 ``isfinite`` 显式拦截-它与任何数比较都是 False,
    只靠上下界夹紧会把它放过去.调用方需要区分 None 与合法值.
    """
    raw = payload.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return min(max(value, float(minimum)), float(maximum))


def validate_web_keyboard_command(
    payload: dict[str, Any],
    *,
    enabled: bool,
    joint_names: tuple[str, ...],
    current_positions: dict[str, float],
    joint_limits: dict[str, tuple[float, float]],
    default_step_rad: float,
    min_step_rad: float,
    max_step_rad: float,
    default_duration: float,
    min_duration: float,
    max_duration: float,
    joint_velocity_limits: dict[str, float] | None = None,
    max_joint_speed_rad_s: float | None = None,
) -> WebKeyboardCommandDecision:
    """校验 Web 键盘单步命令,并算出目标关节角.

    校验顺序(任一失败立即返回拒绝,且不产生任何副作用):
    1. ``enabled``:Web 键盘遥操作必须已由操作者显式启用;
    2. ``confirm`` 必须等于 ``KEYBOARD_TELEOP``(防止误触或页面自动重放触发运动);
    3. 按键必须能映射到当前机械臂存在的关节;
    4. 每个关节都必须有实时反馈(没有反馈就无法计算增量,也无法限速).

    参数单位:
    - ``default_step_rad`` / ``min_step_rad`` / ``max_step_rad``:步长(rad),
      请求中的 ``step_rad`` 先被夹紧到 [min, max] 再使用;
    - ``default_duration`` / ``min_duration`` / ``max_duration``:单步时长(s),同样夹紧;
    - ``joint_velocity_limits``:关节名 → 速度上限(rad/s),可选;
    - ``max_joint_speed_rad_s``:全局速度上限(rad/s),可选;
      若它与请求都未给出,则跳过限速校验.

    返回值见 ``WebKeyboardCommandDecision``.
    """
    if not enabled:
        return WebKeyboardCommandDecision(False, "web keyboard teleop disabled")
    # 确认串忽略大小写与首尾空白,但必须完全匹配,避免误触触发真实运动
    if str(payload.get("confirm", "")).strip().upper() != "KEYBOARD_TELEOP":
        return WebKeyboardCommandDecision(False, "missing KEYBOARD_TELEOP confirmation")

    key = str(payload.get("key", ""))
    mapper = KeyboardCommandMapper(joint_names=joint_names)
    command = mapper.command_for_key(key)
    if command is None:
        return WebKeyboardCommandDecision(False, f"unmapped keyboard key: {key}")

    # 缺任一关节的实时反馈就拒绝:增量与限速都必须基于真实当前位置
    missing_current = [name for name in joint_names if name not in current_positions]
    if missing_current:
        return WebKeyboardCommandDecision(False, f"missing live joint state: {', '.join(missing_current)}")

    step_rad = _float_from_payload(
        payload,
        "step_rad",
        default_step_rad,
        minimum=min_step_rad,
        maximum=max_step_rad,
    )
    # step_rad / duration 已由 _float_from_payload 夹紧到各自区间,这里只需排除 None 与非正值
    if step_rad is None or step_rad <= 0.0:
        return WebKeyboardCommandDecision(False, "invalid step_rad")
    duration = _float_from_payload(
        payload,
        "duration",
        default_duration,
        minimum=min_duration,
        maximum=max_duration,
    )
    if duration is None or duration <= 0.0:
        return WebKeyboardCommandDecision(False, "invalid duration")

    # 请求只能收紧不能放宽:最终上限取"服务端配置上限"与"请求上限"中的较小值
    requested_speed_limit = payload.get("max_joint_speed_rad_s", max_joint_speed_rad_s)
    speed_limit = float(max_joint_speed_rad_s) if max_joint_speed_rad_s is not None else None
    if requested_speed_limit is not None:
        try:
            requested_speed_value = float(requested_speed_limit)
        except (TypeError, ValueError):
            return WebKeyboardCommandDecision(False, "invalid max_joint_speed_rad_s")
        if not math.isfinite(requested_speed_value) or requested_speed_value <= 0.0:
            # 非有限值或非正值都判为非法:0 不代表"不限速",要取消限制必须不传该字段
            return WebKeyboardCommandDecision(False, "invalid max_joint_speed_rad_s")
        speed_limit = requested_speed_value if speed_limit is None else min(speed_limit, requested_speed_value)

    planner = TeleopTargetPlanner(
        joint_names=joint_names,
        joint_limits=joint_limits,
        joint_step_rad=step_rad,
    )
    target = planner.apply_delta(
        current_positions=current_positions,
        joint_name=command.joint_name,
        direction=command.direction,
    )
    if not target.accepted:
        return WebKeyboardCommandDecision(False, target.message)

    index = joint_names.index(command.joint_name)
    # 用夹紧后的实际增量而非请求步长算速度:顶到限位时实际增量为 0,不应被判成超速
    actual_delta = abs(float(target.positions[index]) - float(current_positions[command.joint_name]))
    if speed_limit is not None:
        # 单关节速度上限与全局上限取小,两者都不存在时才不限制
        configured_limit = None
        if joint_velocity_limits is not None and command.joint_name in joint_velocity_limits:
            configured_limit = float(joint_velocity_limits[command.joint_name])
        joint_limit = speed_limit if configured_limit is None else min(speed_limit, configured_limit)
        # 平均角速度 = 实际增量 / 时长;max(..., 1e-9) 仅为除零保护(duration 已保证 > 0)
        required_speed = actual_delta / max(duration, 1e-9)
        if required_speed > joint_limit:
            # 反解出满足限速所需的最短时长(s),直接写进消息供操作者调整
            min_duration_needed = actual_delta / max(joint_limit, 1e-9)
            return WebKeyboardCommandDecision(
                False,
                (
                    f"{command.joint_name} speed too high: {required_speed:.4f} rad/s > "
                    f"{joint_limit:.4f} rad/s; use duration >= {min_duration_needed:.2f}s"
                ),
            )

    return WebKeyboardCommandDecision(
        True,
        f"web keyboard target accepted: {key} -> {command.joint_name}",
        key=key,
        joint_name=command.joint_name,
        joint_names=target.joint_names,
        positions=target.positions,
        step_rad=step_rad,
        duration=duration,
        max_joint_speed_rad_s=float(speed_limit) if speed_limit is not None else 0.0,
    )
