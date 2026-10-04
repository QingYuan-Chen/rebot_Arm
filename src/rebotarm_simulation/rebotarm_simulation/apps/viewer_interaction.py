"""viewer interaction; ownership and behavior unchanged."""
from __future__ import annotations
import time
import math
from typing import Callable, Sequence
from dataclasses import dataclass, replace
from queue import Empty, SimpleQueue
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES


@dataclass(frozen=True)
class ViewerControlState:
    """查看器控制状态的不可变快照。

    每个仿真周期都基于旧状态生成新状态（`dataclasses.replace`），因此状态迁移
    是纯函数式的，便于回放与测试；时间型的点动状态（`jog_time_remaining`、
    `*_jog_direction`）用于实现"按住某键持续点动"的手感。

    字段含义：
    - `selected_joint`：当前选中的关节下标 0..5，对应 `joint1`..`joint6`；
    - `joint_targets`：六个关节的位置目标（单位 rad，顺序同 `joint1`..`joint6`）；
    - `joint_positions`：六个关节的当前实际角度（单位 rad，仅用于显示）；
    - `gripper_width`：夹爪目标开口宽度（单位 m，范围为夹爪机械行程）；
    - `paused`：暂停时不再推进物理步，只处理按键；
    - `joint_delta`/`gripper_delta`：本周期待执行的单步增量次数（按键累积，执行后清零）；
    - `joint_jog_direction`/`gripper_jog_direction`：持续点动方向（-1/0/+1）；
    - `jog_time_remaining`：持续点动剩余时间（单位 s），归零后方向自动失效；
    - `single_step`：暂停状态下按 `.` 触发的单步执行请求；
    - `reset`/`home`：本周期请求的复位/回零动作标志；
    - `mode`：本周期请求切换的控制模式（None 表示不切换）；
    - `active_mode`：仿真当前实际生效的控制模式；
    - `quit`：请求退出主循环。
    """

    selected_joint: int = 0
    joint_targets: tuple[float, ...] = (0.0,) * 6
    joint_positions: tuple[float, ...] = (0.0,) * 6
    gripper_width: float = 0.09
    paused: bool = False
    joint_delta: int = 0
    gripper_delta: int = 0
    joint_jog_direction: int = 0
    gripper_jog_direction: int = 0
    jog_time_remaining: float = 0.0
    single_step: bool = False
    reset: bool = False
    home: bool = False
    mode: str | None = None
    active_mode: str = "pos_vel"
    quit: bool = False


def reduce_key(state: ViewerControlState, key: str) -> ViewerControlState:
    """把一个按键归约为新的控制状态（纯函数，不触碰仿真）。

    键位与 `HELP` 一一对应；无法识别的按键原样返回旧状态。注意空格切换暂停，
    而 `.` 仅在暂停状态下触发单步，避免运行中误触导致额外步进。
    """
    key = key.lower()
    if key == "]":
        return replace(state, selected_joint=(state.selected_joint + 1) % 6)
    if key == "[":
        return replace(state, selected_joint=(state.selected_joint - 1) % 6)
    if key in "123456":
        return replace(state, selected_joint=int(key) - 1)
    if key == "j":
        return replace(state, joint_delta=state.joint_delta - 1, joint_jog_direction=-1)
    if key == "k":
        return replace(state, joint_delta=state.joint_delta + 1, joint_jog_direction=1)
    if key == "c":
        return replace(state, gripper_delta=state.gripper_delta - 1, gripper_jog_direction=-1)
    if key == "o":
        return replace(state, gripper_delta=state.gripper_delta + 1, gripper_jog_direction=1)
    if key == " ":
        return replace(state, paused=not state.paused, single_step=False)
    if key == "." and state.paused:
        return replace(state, single_step=True)
    if key == "r":
        return replace(state, reset=True)
    if key == "t":
        return replace(state, home=True)
    if key == "g":
        return replace(state, mode="gravity_comp")
    if key == "h":
        return replace(state, mode="hold")
    if key == "p":
        return replace(state, mode="pos_vel")
    if key in ("q", "\x1b"):
        return replace(state, quit=True)
    return state


def _take_key_snapshot(events: SimpleQueue) -> tuple[int, ...]:
    # 只取当前队列中已到达的事件（按 qsize 限量），保证处理的是"有限快照"：
    # 新到达的按键留给下一个周期，避免持续按键时本周期被无限拖长。
    # `Empty` 兜底是因为 qsize 与实际取出之间可能被其它线程改变。
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def drain_key_events(events: SimpleQueue, state: ViewerControlState) -> ViewerControlState:
    """消费一个有限 FIFO 快照，把新事件留给下一个周期。

    仅做状态归约、不推进仿真，用于测试或需要先观察按键效果的场景。
    """
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
    return state


def process_key_events(
    sim,
    events: SimpleQueue,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    """在仿真线程上顺序执行一个有限事件快照。

    逐个事件依次"归约状态 -> 执行待办命令"，因此同一快照内的多次按键会按到达
    顺序累加（例如连按 K 叠加多次关节增量）。`single_step` 在暂停状态下立即
    推进一个物理步，实现暂停时的逐帧观察。检测到退出请求时提前结束本批次。
    """
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
        state = apply_pending_commands(
            sim, state, joint_step, gripper_step, jog_hold_time=jog_hold_time
        )
        if state.quit:
            break
        if state.single_step:
            sim.step()
            state = replace(state, single_step=False)
    return state


def _state_from_sim(sim, *, paused: bool = False, selected_joint: int = 0) -> ViewerControlState:
    # 从仿真对象重新同步一份状态快照。仿真侧每个关节角都是弧度，夹爪宽度是米；
    # 只取前 6 个自由度作为机械臂关节（其余为夹爪手指关节）。
    state = sim.get_state()
    targets = tuple(float(value) for value in sim.control_targets[:6])
    positions = tuple(float(value) for value in state.joint_positions[:6])
    return ViewerControlState(
        selected_joint=selected_joint,
        joint_targets=targets,
        joint_positions=positions,
        gripper_width=float(state.gripper_width),
        paused=paused,
        # 兼容没有控制模式概念的仿真对象：缺失时按默认位置/速度模式处理。
        active_mode=str(getattr(sim, "control_mode", "pos_vel")),
    )


def apply_pending_commands(
    sim,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    *,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    """执行一次性待办命令（单步点动、复位/回零、模式切换），并回读仿真状态。

    `joint_step` 为每次关节点动的角度增量（rad），`gripper_step` 为每次夹爪
    点动的宽度增量（m），两者都必须为正数。执行完成后所有一次性标志位清零，
    但未执行的增量、退出请求以及 `jog_hold_time` 启动的持续点动计时会被保留。
    """
    if state.reset or state.home:
        # 复位会重建仿真内部状态，因此先保存与"人机意图"相关的字段，事后恢复，
        # 否则按下的增量或退出请求会被复位流程吞掉。
        pending_joint_delta = state.joint_delta
        pending_gripper_delta = state.gripper_delta
        quit_requested = state.quit
        if state.home:
            sim.reset_home()
        else:
            sim.reset()
        # 复位后统一进入重力补偿模式：此时控制器不主动保持目标，便于手动拖动。
        if hasattr(sim, "set_control_mode"):
            sim.set_control_mode("gravity_comp")
        state = _state_from_sim(
            sim, paused=state.paused, selected_joint=state.selected_joint
        )
        state = replace(
            state,
            joint_delta=pending_joint_delta,
            gripper_delta=pending_gripper_delta,
            quit=quit_requested,
        )

    # 模式切换只在请求时下发；否则沿用仿真当前模式。
    if state.mode is not None and hasattr(sim, "set_control_mode"):
        active_mode = sim.set_control_mode(state.mode)
    else:
        active_mode = getattr(sim, "control_mode", state.active_mode)

    targets = state.joint_targets
    jog_time_remaining = state.jog_time_remaining
    if state.joint_delta:
        # 单步点动：以旧目标为基准叠加增量，由仿真侧做关节限位钳制。
        # 仿真在设置关节目标时会切回位置/速度模式，故这里重新读取实际模式。
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_delta * joint_step
        targets = tuple(sim.set_joint_position_targets({name: requested}))
        active_mode = getattr(sim, "control_mode", "pos_vel")
        jog_time_remaining = jog_hold_time

    width = state.gripper_width
    if state.gripper_delta:
        # 夹爪宽度同样由仿真侧按机械行程钳制，返回实际生效宽度。
        width = float(sim.set_gripper_width(width + state.gripper_delta * gripper_step))
        jog_time_remaining = jog_hold_time

    sim_state = sim.get_state()
    return replace(
        state,
        joint_targets=targets,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        gripper_width=width,
        joint_delta=0,
        gripper_delta=0,
        jog_time_remaining=jog_time_remaining,
        reset=False,
        home=False,
        mode=None,
        active_mode=str(active_mode),
    )


def apply_continuous_jog(
    sim,
    state: ViewerControlState,
    *,
    dt: float,
    joint_rate: float,
    gripper_rate: float,
) -> ViewerControlState:
    """按速率推进"按住按键"的持续点动，每个仿真周期调用一次。

    `dt` 为本次物理步时长（通常等于仿真 timestep，单位 s），`joint_rate` 为
    关节点动速率（rad/s），`gripper_rate` 为夹爪点动速率（m/s）。剩余保持时间
    归零后方向标志清零，等价于松开按键，避免键盘自动重复结束后仍继续运动。
    """
    if state.jog_time_remaining <= 0.0:
        return replace(state, joint_jog_direction=0, gripper_jog_direction=0)

    targets = state.joint_targets
    if state.joint_jog_direction:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_jog_direction * joint_rate * dt
        targets = tuple(sim.set_joint_position_targets({name: requested}))

    width = state.gripper_width
    if state.gripper_jog_direction:
        width = float(sim.set_gripper_width(width + state.gripper_jog_direction * gripper_rate * dt))

    sim_state = sim.get_state()
    # 剩余时间不会变成负数，便于上层直接判断 <= 0 判定停止。
    remaining = max(0.0, state.jog_time_remaining - dt)
    return replace(
        state,
        joint_targets=targets,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        gripper_width=width,
        jog_time_remaining=remaining,
        joint_jog_direction=state.joint_jog_direction if remaining > 0.0 else 0,
        gripper_jog_direction=state.gripper_jog_direction if remaining > 0.0 else 0,
        active_mode=str(getattr(sim, "control_mode", state.active_mode)),
    )


def overlay_text(state: ViewerControlState) -> str:
    """生成叠加/打印用的状态文本（角度与宽度保留 3 位小数）。

    第一段是当前选中关节的模式、角度与目标（rad），第二段是夹爪目标宽度（m）
    与运行/暂停状态；并特别提示 MuJoCo 自带控制面板显示的是力矩/力而非位置，
    以免误读。文本变化时才输出，避免刷屏。
    """
    name = ARM_JOINT_NAMES[state.selected_joint]
    run_state = "paused" if state.paused else "running"
    return (
        f"mode: {state.active_mode}  selected: {name}  "
        f"q: {state.joint_positions[state.selected_joint]:.3f} rad  "
        f"target: {state.joint_targets[state.selected_joint]:.3f} rad\n"
        f"gripper target: {state.gripper_width:.3f} m  state: {run_state}\n"
        "MuJoCo control panel shows torque/force, not joint position.\n"
        f"{HELP}"
    )


def _decode_key(keycode: int) -> str:
    # GLFW 的 ESC 键码为 256，转换为控制字符 "\x1b"，与 `reduce_key` 中的退出分支一致；
    # 其它键码按 Unicode 码位转换成字符，无法转换时返回空串（等价于无操作）。
    if keycode == 256:
        return "\x1b"
    try:
        return chr(keycode)
    except (TypeError, ValueError):
        return ""


HELP = (
    "[ / ] select | 1-6 select | hold J/K joint | hold C/O gripper | "
    "G gravity | H hold | P pos | R zero | T home | Q quit"
)
