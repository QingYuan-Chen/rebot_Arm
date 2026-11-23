"""终端键盘力控分模式路由,单关节阻抗草稿与参数隔离校验(纯逻辑层).

职责与位置
    作为 `rebotarm_teleop` 的纯逻辑核心模块(零 `rclpy`/硬件/仿真依赖),提供:
    1. 终端键盘阻抗配置 `KeyboardImpedanceProfile` 与仿真/真机参数隔离强校验;
    2. 依据后端 `/arm_status` 状态机的按键分类与互斥路由 `KeyboardModeRouter`;
    3. 六轴单关节 `kp` / `kd` / `q_ref` / 对齐调参草稿 `ImpedanceTuningDraft` 候选计算;
    4. 终端六轴实时遥测格式化 `format_telemetry`.

对外接口
    PROTOCOL_KP_MAX / PROTOCOL_KD_MAX / KeyboardImpedanceProfile /
    ImpedanceTuningDraft / KeyDispatchAction / KeyboardModeRouter / format_telemetry.

安全与并发约束
    - 本模块绝不内置默认刚度/阻尼值;缺省配置只能保持未配置状态(`None`),严禁兜底生成;
    - 强制隔离仿真与真机 profile: `use_hardware=True` 禁止加载 `simulation` profile,
      `use_hardware=False` 禁止加载 `hardware` profile,越限(Kp>500/Kd>5)立即抛错;
    - 所有草稿更新均为不可变候选计算,节点须等待后端服务返回 `success=True` 后方可提交;
    - 任何时刻均不得绕过 `STOP` (`h`/`空格`) 语义,选轴(`1..6`)不触发后端动作.

文件结构(按定义顺序)
    协议上限与配置对象 → 不可变调参草稿 → 路由动作与分模式路由器 → 遥测格式化.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

from .teleop_core import KeyboardCommandMapper


# 达妙 DM 系列 MIT 协议硬件上限(纯协议常数,避免操作交互层依赖力控算法包内部).
PROTOCOL_KP_MAX = 500.0
PROTOCOL_KD_MAX = 5.0
ALLOWED_PROFILES = ("simulation", "hardware")
IMPEDANCE_SELECT_KEYS = ("1", "2", "3", "4", "5", "6")
IMPEDANCE_TUNE_KEYS = ("[", "]", ",", ".", "-", "=", "0", "l")


def _six_finite_floats(values: Sequence[float], label: str) -> tuple[float, ...]:
    items = tuple(values)
    if len(items) != 6:
        raise ValueError(f"{label} must contain exactly 6 values")
    result = tuple(float(v) for v in items)
    if any(isinstance(v, bool) or not math.isfinite(v) for v in result):
        raise ValueError(f"{label} must contain 6 finite values")
    return result


def _positive_finite(value: float, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be positive and finite")
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} must be positive and finite")
    return number


def _clamp_axis(
    values: tuple[float, ...],
    index: int,
    new_value: float,
    lower: float,
    upper: float,
) -> tuple[tuple[float, ...], float, bool]:
    clamped = min(max(float(new_value), float(lower)), float(upper))
    updated = list(values)
    updated[index] = clamped
    return tuple(updated), clamped, clamped != float(new_value)


@dataclass(frozen=True)
class KeyboardImpedanceProfile:
    """终端键盘使用的显式阻抗初始参数与单步调参幅值(区分 simulation 与 hardware)."""

    profile_name: str
    initial_stiffness: tuple[float, ...]
    initial_damping: tuple[float, ...]
    max_stiffness: tuple[float, ...]
    max_damping: tuple[float, ...]
    kp_step: float
    kd_step: float
    q_step_rad: float

    def __post_init__(self) -> None:
        name = str(self.profile_name).strip().lower()
        if name not in ALLOWED_PROFILES:
            raise ValueError(f"impedance_profile must be one of {ALLOWED_PROFILES}")
        max_kp = _six_finite_floats(self.max_stiffness, "impedance_max_stiffness")
        max_kd = _six_finite_floats(self.max_damping, "impedance_max_damping")
        init_kp = _six_finite_floats(self.initial_stiffness, "impedance_initial_stiffness")
        init_kd = _six_finite_floats(self.initial_damping, "impedance_initial_damping")
        if any(v <= 0.0 or v > PROTOCOL_KP_MAX for v in max_kp):
            raise ValueError(f"impedance_max_stiffness must be within (0, {PROTOCOL_KP_MAX}]")
        if any(v <= 0.0 or v > PROTOCOL_KD_MAX for v in max_kd):
            raise ValueError(f"impedance_max_damping must be within (0, {PROTOCOL_KD_MAX}]")
        for index in range(6):
            if not 0.0 <= init_kp[index] <= max_kp[index]:
                raise ValueError(
                    f"impedance_initial_stiffness[{index}]={init_kp[index]} outside [0, {max_kp[index]}]"
                )
            if not 0.0 <= init_kd[index] <= max_kd[index]:
                raise ValueError(
                    f"impedance_initial_damping[{index}]={init_kd[index]} outside [0, {max_kd[index]}]"
                )
        object.__setattr__(self, "profile_name", name)
        object.__setattr__(self, "initial_stiffness", init_kp)
        object.__setattr__(self, "initial_damping", init_kd)
        object.__setattr__(self, "max_stiffness", max_kp)
        object.__setattr__(self, "max_damping", max_kd)
        object.__setattr__(self, "kp_step", _positive_finite(self.kp_step, "impedance_kp_step"))
        object.__setattr__(self, "kd_step", _positive_finite(self.kd_step, "impedance_kd_step"))
        object.__setattr__(self, "q_step_rad", _positive_finite(self.q_step_rad, "impedance_q_step_rad"))

    @classmethod
    def from_parameters(
        cls,
        *,
        use_hardware: bool,
        profile_name: str,
        initial_stiffness: Sequence[float],
        initial_damping: Sequence[float],
        max_stiffness: Sequence[float],
        max_damping: Sequence[float],
        kp_step: float,
        kd_step: float,
        q_step_rad: float,
    ) -> KeyboardImpedanceProfile | None:
        """校验 ROS 参数并按 use_hardware 强隔离仿真/真机阻抗 profile.

        全部参数都处于显式未配置空值时返回 None;只要出现任一非空字段即执行全套强校验.
        """
        name = str(profile_name).strip().lower()
        init_kp = tuple(initial_stiffness)
        init_kd = tuple(initial_damping)
        max_kp = tuple(max_stiffness)
        max_kd = tuple(max_damping)
        kp_delta = float(kp_step)
        kd_delta = float(kd_step)
        q_delta = float(q_step_rad)
        all_unset = (
            not name
            and not init_kp
            and not init_kd
            and not max_kp
            and not max_kd
            and kp_delta == 0.0
            and kd_delta == 0.0
            and q_delta == 0.0
        )
        if all_unset:
            return None
        expected = "hardware" if bool(use_hardware) else "simulation"
        if name != expected:
            raise ValueError(
                f"impedance_profile mismatch: use_hardware={bool(use_hardware)} requires "
                f"{expected!r}, got {name or '<unset>'!r}"
            )
        return cls(
            profile_name=name,
            initial_stiffness=init_kp,
            initial_damping=init_kd,
            max_stiffness=max_kp,
            max_damping=max_kd,
            kp_step=kp_delta,
            kd_step=kd_delta,
            q_step_rad=q_delta,
        )


@dataclass(frozen=True)
class ImpedanceTuningDraft:
    """不可变的六轴阻抗调参草稿(每次调参产生新候选,服务确认后提交)."""

    joint_names: tuple[str, ...]
    selected_index: int
    positions: tuple[float, ...]
    stiffness: tuple[float, ...]
    damping: tuple[float, ...]

    def __post_init__(self) -> None:
        names = tuple(str(v) for v in self.joint_names)
        if len(names) != 6 or len(set(names)) != 6:
            raise ValueError("joint_names must contain 6 unique names")
        if isinstance(self.selected_index, bool) or not 0 <= int(self.selected_index) < 6:
            raise ValueError("selected_index must be in [0, 5]")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "selected_index", int(self.selected_index))
        object.__setattr__(self, "positions", _six_finite_floats(self.positions, "positions"))
        kp = _six_finite_floats(self.stiffness, "stiffness")
        kd = _six_finite_floats(self.damping, "damping")
        if any(v < 0.0 for v in kp) or any(v < 0.0 for v in kd):
            raise ValueError("stiffness and damping must be nonnegative")
        object.__setattr__(self, "stiffness", kp)
        object.__setattr__(self, "damping", kd)

    @classmethod
    def create_initial(
        cls,
        *,
        joint_names: tuple[str, ...],
        current_positions: dict[str, float],
        joint_limits: dict[str, tuple[float, float]],
        profile: KeyboardImpedanceProfile,
    ) -> ImpedanceTuningDraft:
        """按当前实时关节角与显式配置 profile 构造进入阻抗模式的初始六轴草稿."""
        missing = [name for name in joint_names if name not in current_positions]
        if missing:
            raise ValueError(f"missing live joint state: {', '.join(missing)}")
        positions = []
        for name in joint_names:
            value = float(current_positions[name])
            if not math.isfinite(value):
                raise ValueError(f"non-finite live joint position: {name}")
            lower, upper = joint_limits[name]
            positions.append(min(max(value, float(lower)), float(upper)))
        return cls(
            joint_names=joint_names,
            selected_index=0,
            positions=tuple(positions),
            stiffness=profile.initial_stiffness,
            damping=profile.initial_damping,
        )

    @property
    def selected_joint_name(self) -> str:
        return self.joint_names[self.selected_index]

    def select_joint(self, index: int) -> ImpedanceTuningDraft:
        return replace(self, selected_index=int(index))

    def adjust_kp(
        self, delta: float, *, max_stiffness: tuple[float, ...]
    ) -> tuple[ImpedanceTuningDraft, str]:
        idx = self.selected_index
        upper = float(max_stiffness[idx])
        updated, clamped, was_clamped = _clamp_axis(
            self.stiffness, idx, self.stiffness[idx] + float(delta), 0.0, upper
        )
        suffix = " (clamped)" if was_clamped else ""
        msg = (
            f"{self.selected_joint_name} kp -> {clamped:.2f}{suffix} "
            f"[Kp={clamped:.2f}, Kd={self.damping[idx]:.2f}, q_ref={self.positions[idx]:+.4f} rad]"
        )
        return replace(self, stiffness=updated), msg

    def adjust_kd(
        self, delta: float, *, max_damping: tuple[float, ...]
    ) -> tuple[ImpedanceTuningDraft, str]:
        idx = self.selected_index
        upper = float(max_damping[idx])
        updated, clamped, was_clamped = _clamp_axis(
            self.damping, idx, self.damping[idx] + float(delta), 0.0, upper
        )
        suffix = " (clamped)" if was_clamped else ""
        msg = (
            f"{self.selected_joint_name} kd -> {clamped:.2f}{suffix} "
            f"[Kp={self.stiffness[idx]:.2f}, Kd={clamped:.2f}, q_ref={self.positions[idx]:+.4f} rad]"
        )
        return replace(self, damping=updated), msg

    def adjust_position(
        self, delta: float, *, joint_limits: dict[str, tuple[float, float]]
    ) -> tuple[ImpedanceTuningDraft, str]:
        idx = self.selected_index
        name = self.selected_joint_name
        lower, upper = joint_limits[name]
        updated, clamped, was_clamped = _clamp_axis(
            self.positions, idx, self.positions[idx] + float(delta), float(lower), float(upper)
        )
        suffix = " (clamped)" if was_clamped else ""
        msg = (
            f"{name} q_ref -> {clamped:.4f} rad{suffix} "
            f"[Kp={self.stiffness[idx]:.2f}, Kd={self.damping[idx]:.2f}]"
        )
        return replace(self, positions=updated), msg

    def align_positions(
        self,
        *,
        current_positions: dict[str, float],
        joint_limits: dict[str, tuple[float, float]],
    ) -> tuple[ImpedanceTuningDraft, str]:
        missing = [name for name in self.joint_names if name not in current_positions]
        if missing:
            raise ValueError(f"missing live joint state: {', '.join(missing)}")
        aligned = []
        for name in self.joint_names:
            val = float(current_positions[name])
            if not math.isfinite(val):
                raise ValueError(f"non-finite live joint position: {name}")
            lower, upper = joint_limits[name]
            aligned.append(min(max(val, float(lower)), float(upper)))
        idx = self.selected_index
        return (
            replace(self, positions=tuple(aligned)),
            (
                f"impedance q_ref aligned to current joint positions "
                f"[sel={self.selected_joint_name}: Kp={self.stiffness[idx]:.2f}, "
                f"Kd={self.damping[idx]:.2f}, q_ref={aligned[idx]:+.4f} rad]"
            ),
        )


@dataclass(frozen=True)
class KeyDispatchAction:
    """模式路由器产出的单键意图."""

    kind: str
    message: str = ""
    joint_name: str = ""
    direction: float = 0.0
    joint_index: int = 0
    tune_op: str = ""
    sign: float = 0.0
    requires_deadman: bool = False


class KeyboardModeRouter:
    """依据后端 `arm_status.state_machine` 对终端按键做严格分模式路由."""

    def __init__(
        self,
        *,
        joint_names: tuple[str, ...],
        key_bindings: dict[str, tuple[str, float]] | None = None,
    ) -> None:
        self._joint_names = tuple(joint_names)
        self._mapper = KeyboardCommandMapper(
            joint_names=self._joint_names,
            key_bindings=key_bindings,
        )

    def classify_key(
        self,
        key: str,
        *,
        state_machine: str | None,
        has_impedance_profile: bool,
    ) -> KeyDispatchAction:
        """将单键映射为分模式执行动作或显式拒绝原因."""
        if key in ("h", " "):
            return KeyDispatchAction(kind="stop", message="stop requested")
        if key == "p":
            return KeyDispatchAction(kind="print_telemetry", message="telemetry printed")

        if state_machine is None or not str(state_machine).strip():
            if (
                key in ("g", "i")
                or key in IMPEDANCE_TUNE_KEYS
                or self._mapper.command_for_key(key) is not None
            ):
                return KeyDispatchAction(
                    kind="reject",
                    message="arm_status unavailable; waiting for controller status",
                )
            return KeyDispatchAction(kind="ignore")

        mode = str(state_machine).strip().upper()
        if key == "g":
            if mode == "IDLE":
                return KeyDispatchAction(
                    kind="start_gravity_comp",
                    message="starting gravity compensation",
                    requires_deadman=True,
                )
            return KeyDispatchAction(
                kind="reject",
                message=f"gravity compensation requires IDLE mode (current={mode}); press h/space first",
            )

        if key == "i":
            if mode == "JOINT_IMPEDANCE":
                return KeyDispatchAction(
                    kind="reject",
                    message="already in JOINT_IMPEDANCE; use 0/l to align q_ref or h/space to stop",
                )
            if mode != "IDLE":
                return KeyDispatchAction(
                    kind="reject",
                    message=f"joint impedance requires IDLE mode (current={mode}); press h/space first",
                )
            if not has_impedance_profile:
                return KeyDispatchAction(
                    kind="reject",
                    message="impedance profile not configured; load simulation or hardware impedance YAML",
                )
            return KeyDispatchAction(
                kind="start_impedance",
                message="starting joint impedance",
                requires_deadman=True,
            )

        if mode == "JOINT_IMPEDANCE":
            if key in IMPEDANCE_SELECT_KEYS:
                index = int(key) - 1
                if index >= len(self._joint_names):
                    return KeyDispatchAction(kind="ignore")
                return KeyDispatchAction(
                    kind="select_joint",
                    joint_index=index,
                    joint_name=self._joint_names[index],
                    message=f"selected {self._joint_names[index]} for impedance tuning",
                )
            tune_map = {
                "[": ("kp", -1.0),
                "]": ("kp", 1.0),
                ",": ("kd", -1.0),
                ".": ("kd", 1.0),
                "-": ("q_ref", -1.0),
                "=": ("q_ref", 1.0),
                "0": ("align", 0.0),
                "l": ("align", 0.0),
            }
            if key in tune_map:
                tune_op, sign = tune_map[key]
                return KeyDispatchAction(
                    kind="tune_impedance",
                    tune_op=tune_op,
                    sign=sign,
                    requires_deadman=True,
                )
            if self._mapper.command_for_key(key) is not None:
                return KeyDispatchAction(
                    kind="reject",
                    message="FJT jog disabled in JOINT_IMPEDANCE mode; use 1..6/[]/,. /-=/0/l or h/space to stop",
                )
            return KeyDispatchAction(kind="ignore")

        if mode == "GRAVITY_COMP":
            if key in IMPEDANCE_TUNE_KEYS or self._mapper.command_for_key(key) is not None:
                return KeyDispatchAction(
                    kind="reject",
                    message="motion and tuning disabled in GRAVITY_COMP mode; press h/space to stop",
                )
            return KeyDispatchAction(kind="ignore")

        if mode == "IDLE":
            command = self._mapper.command_for_key(key)
            if command is not None:
                return KeyDispatchAction(
                    kind="jog_fjt",
                    joint_name=command.joint_name,
                    direction=command.direction,
                    requires_deadman=True,
                )
            if key in IMPEDANCE_TUNE_KEYS:
                return KeyDispatchAction(
                    kind="reject",
                    message="impedance tuning keys require JOINT_IMPEDANCE mode (press i first)",
                )
            return KeyDispatchAction(kind="ignore")

        # TRAJ_RUNNING / TRAJ_PREPARING / LOWLEVEL_STREAMING 或未知态一律拦截运动按键.
        if key in IMPEDANCE_TUNE_KEYS or self._mapper.command_for_key(key) is not None:
            return KeyDispatchAction(
                kind="reject",
                message=f"keyboard motion blocked in {mode} mode; press h/space to stop",
            )
        return KeyDispatchAction(kind="ignore")


def apply_draft_tuning(
    draft: ImpedanceTuningDraft,
    *,
    tune_op: str,
    sign: float,
    profile: KeyboardImpedanceProfile,
    current_positions: dict[str, float],
    joint_limits: dict[str, tuple[float, float]],
) -> tuple[ImpedanceTuningDraft, str]:
    """根据调参操作码生成不可变新草稿与回显消息."""
    if tune_op == "kp":
        return draft.adjust_kp(sign * profile.kp_step, max_stiffness=profile.max_stiffness)
    if tune_op == "kd":
        return draft.adjust_kd(sign * profile.kd_step, max_damping=profile.max_damping)
    if tune_op == "q_ref":
        return draft.adjust_position(sign * profile.q_step_rad, joint_limits=joint_limits)
    if tune_op == "align":
        return draft.align_positions(
            current_positions=current_positions,
            joint_limits=joint_limits,
        )
    raise ValueError(f"unsupported tune_op: {tune_op}")


def format_telemetry(
    *,
    joint_names: Sequence[str],
    state_machine: str | None,
    arm_mode: str | None,
    enabled: bool | None,
    current_positions: dict[str, float],
    current_velocities: dict[str, float],
    current_efforts: dict[str, float],
    draft: ImpedanceTuningDraft | None,
) -> str:
    """格式化终端六轴状态与阻抗参数快照,供 `p` 键手动核对."""
    header = (
        f"[teleop] state_machine={state_machine or 'UNKNOWN'} "
        f"mode={arm_mode or 'unknown'} enabled={enabled}"
    )
    lines = [header]
    for index, name in enumerate(joint_names):
        q = float(current_positions.get(name, float("nan")))
        qd = float(current_velocities.get(name, 0.0))
        tau = float(current_efforts.get(name, 0.0))
        if draft is not None and index < len(draft.positions):
            marker = "*" if index == draft.selected_index else " "
            q_ref = draft.positions[index]
            err = q_ref - q if math.isfinite(q) else float("nan")
            kp = draft.stiffness[index]
            kd = draft.damping[index]
            lines.append(
                f"  {marker}{name}: q={q:+.4f} q_ref={q_ref:+.4f} err={err:+.4f} "
                f"qd={qd:+.4f} tau={tau:+.3f} kp={kp:6.2f} kd={kd:5.2f}"
            )
        else:
            lines.append(
                f"   {name}: q={q:+.4f} qd={qd:+.4f} tau={tau:+.3f}"
            )
    return "\n".join(lines)
