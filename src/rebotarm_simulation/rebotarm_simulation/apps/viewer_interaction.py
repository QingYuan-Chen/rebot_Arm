from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass, replace
import importlib
import json
import math
from queue import Empty, SimpleQueue
import sys
import threading
import time
from typing import Callable, Sequence
import warnings

from rebotarm_simulation.control.mujoco_cartesian import CartesianDelta, MujocoCartesianController
from rebotarm_simulation.apps.mujoco_commands import dispatch_sim_command
from rebotarm_simulation.apps.mujoco_dashboard import (
    DASHBOARD_PAGES,
    PLOT_PAGES,
    compose_dashboard,
)
from rebotarm_simulation.control.mujoco_jog import JOG_SPEED_LEVELS
from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco
from rebotarm_simulation.diagnostics.mujoco_telemetry import MujocoTelemetryHistory
from rebotarm_simulation.apps.mujoco_visualization import GhostArmOverlay, TelemetryFigures


HELP = (
    "M JOINT/XYZ/RPY | Z/X select | J/K start jog | C/O gripper | S stop\n"
    "-/+ speed | G gravity | H hold | P position | V collision | F6 page | F7 help\n"
    "F8 world/tool | F9 plots | T home | R reset | Q quit\n"
    "Terminal: joints J1..J6 | joint NAME VALUE | gripper WIDTH | state"
)
TELEMETRY_SAMPLE_HZ = 50.0
TELEMETRY_WINDOW_S = 10.0
VISUAL_UPDATE_HZ = 30.0
DASHBOARD_UPDATE_HZ = 10.0



@dataclass(frozen=True)
class ViewerControlState:
    selected_joint: int = 0
    joint_targets: tuple[float, ...] = (0.0,) * 6
    joint_positions: tuple[float, ...] = (0.0,) * 6
    joint_velocities: tuple[float, ...] = (0.0,) * 6
    gripper_width: float = 0.09
    gripper_actual_width: float = 0.09
    max_contact_force: float = 0.0
    contact_count: int = 0
    requested_torques: tuple[float, ...] = (0.0,) * 6
    applied_torques: tuple[float, ...] = (0.0,) * 6
    saturated: tuple[bool, ...] = (False,) * 6
    watchdog_remaining_s: float = 0.0
    collision_visible: bool = False
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
    active_mode: str = "hold"
    jog_speed_index: int = 1
    joint_jog_rate: float = 0.20
    gripper_jog_rate: float = 0.02
    stop_jog: bool = False
    interaction_mode: str = "joint"
    selected_cartesian_axis: int = 0
    cartesian_frame: str = "world"
    cartesian_accumulator_s: float = 0.0
    ik_status: str = "idle"
    ik_error: float = 0.0
    dashboard_page: str = "overview"
    plot_page: str = "off"
    help_visible: bool = False
    ee_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    ee_rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    target_position: tuple[float, ...] = (0.0, 0.0, 0.0)
    target_rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    cartesian_target_reset: bool = False
    quit: bool = False


def reduce_key(state: ViewerControlState, key: str) -> ViewerControlState:
    key = key.lower()
    if key == "m":
        modes = ("joint", "xyz", "rpy")
        mode = modes[(modes.index(state.interaction_mode) + 1) % len(modes)]
        return replace(
            state,
            interaction_mode=mode,
            joint_jog_direction=0,
            gripper_jog_direction=0,
            cartesian_accumulator_s=0.0,
        )
    if key == "x":
        if state.interaction_mode != "joint":
            return replace(
                state,
                selected_cartesian_axis=(state.selected_cartesian_axis + 1) % 3,
                joint_jog_direction=0,
                stop_jog=bool(state.joint_jog_direction),
            )
        return replace(
            state,
            selected_joint=(state.selected_joint + 1) % 6,
            joint_jog_direction=0,
            stop_jog=bool(state.joint_jog_direction),
        )
    if key == "z":
        if state.interaction_mode != "joint":
            return replace(
                state,
                selected_cartesian_axis=(state.selected_cartesian_axis - 1) % 3,
                joint_jog_direction=0,
                stop_jog=bool(state.joint_jog_direction),
            )
        return replace(
            state,
            selected_joint=(state.selected_joint - 1) % 6,
            joint_jog_direction=0,
            stop_jog=bool(state.joint_jog_direction),
        )
    if key == "j":
        return replace(state, joint_jog_direction=-1, gripper_jog_direction=0)
    if key == "k":
        return replace(state, joint_jog_direction=1, gripper_jog_direction=0)
    if key == "c":
        return replace(state, joint_jog_direction=0, gripper_jog_direction=-1)
    if key == "o":
        return replace(state, joint_jog_direction=0, gripper_jog_direction=1)
    if key == "-":
        return replace(state, jog_speed_index=max(0, state.jog_speed_index - 1))
    if key in ("=", "+"):
        return replace(
            state,
            jog_speed_index=min(len(JOG_SPEED_LEVELS) - 1, state.jog_speed_index + 1),
        )
    if key == "s":
        return replace(
            state,
            joint_jog_direction=0,
            gripper_jog_direction=0,
            jog_time_remaining=0.0,
            stop_jog=True,
        )
    if key == " ":
        return replace(state, paused=not state.paused, single_step=False)
    if key == "." and state.paused:
        return replace(state, single_step=True)
    if key == "r":
        return replace(state, reset=True)
    if key == "t":
        return replace(state, home=True)
    if key == "g":
        return replace(
            state, mode="gravity_comp", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "h":
        return replace(
            state, mode="hold", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "p":
        return replace(
            state, mode="position", joint_jog_direction=0, gripper_jog_direction=0
        )
    if key == "v":
        return replace(state, collision_visible=not state.collision_visible)
    if key == "f9":
        page = PLOT_PAGES[(PLOT_PAGES.index(state.plot_page) + 1) % len(PLOT_PAGES)]
        return replace(state, plot_page=page, help_visible=False)
    if key == "f8":
        return replace(
            state,
            cartesian_frame="tool" if state.cartesian_frame == "world" else "world",
            joint_jog_direction=0,
        )
    if key == "f6":
        page = DASHBOARD_PAGES[
            (DASHBOARD_PAGES.index(state.dashboard_page) + 1) % len(DASHBOARD_PAGES)
        ]
        return replace(state, dashboard_page=page, help_visible=False)
    if key == "f7":
        return replace(state, help_visible=not state.help_visible, plot_page="off")
    if key in ("q", "\x1b"):
        return replace(state, quit=True)
    return state


def _take_key_snapshot(events: SimpleQueue) -> tuple[int, ...]:
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def _take_line_snapshot(events: SimpleQueue) -> tuple[str, ...]:
    snapshot = []
    for _ in range(events.qsize()):
        try:
            snapshot.append(events.get_nowait())
        except Empty:
            break
    return tuple(snapshot)


def start_command_reader(command_stream, command_events: SimpleQueue) -> threading.Thread:
    def read_lines() -> None:
        for line in command_stream:
            command_events.put(str(line).strip())

    thread = threading.Thread(target=read_lines, name="mujoco-viewer-command-input", daemon=True)
    thread.start()
    return thread


def drain_key_events(events: SimpleQueue, state: ViewerControlState) -> ViewerControlState:
    """Consume a finite FIFO snapshot, leaving new events for the next cycle."""
    for keycode in _take_key_snapshot(events):
        state = reduce_key(state, _decode_key(keycode))
    return state


def process_command_events(
    sim, events: SimpleQueue, state: ViewerControlState, status_stream
) -> ViewerControlState:
    for line in _take_line_snapshot(events):
        if not line:
            continue
        try:
            command_name = line.split(maxsplit=1)[0].lower()
            result = dispatch_sim_command(sim, line, paused=state.paused)
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            print(f"command error: {exc}", file=status_stream, flush=True)
            continue
        state = replace(state, paused=result.paused, quit=state.quit or result.should_quit)
        if result.value is not None:
            print(f"command result: {_command_value_text(result.value)}", file=status_stream, flush=True)
        state = _state_from_sim(
            sim,
            paused=state.paused,
            selected_joint=state.selected_joint,
            previous=state,
        )
        state = replace(state, quit=state.quit)
        if command_name in {"home", "reset"}:
            state = replace(state, cartesian_target_reset=True)
        if state.quit:
            break
    return state


def _command_value_text(value) -> str:
    try:
        return json.dumps(_jsonable(value), ensure_ascii=False, sort_keys=True)
    except TypeError:
        return repr(value)


def _jsonable(value):
    if hasattr(value, "__dataclass_fields__"):
        return {name: _jsonable(getattr(value, name)) for name in value.__dataclass_fields__}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "__dict__"):
        return {key: _jsonable(item) for key, item in vars(value).items()}
    return value


def process_key_events(
    sim,
    events: SimpleQueue,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    """Execute one finite event snapshot sequentially on the simulation thread."""
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


def _state_from_sim(
    sim,
    *,
    paused: bool = False,
    selected_joint: int = 0,
    previous: ViewerControlState | None = None,
) -> ViewerControlState:
    state = sim.get_state()
    targets = tuple(float(value) for value in sim.control_targets[:6])
    positions = tuple(float(value) for value in state.joint_positions[:6])
    velocities = tuple(float(value) for value in state.joint_velocities[:6])
    ee_position = tuple(
        float(value) for value in getattr(state, "end_effector_position", (0.0, 0.0, 0.0))
    )
    ee_rpy = _quaternion_xyzw_to_rpy(
        getattr(state, "end_effector_orientation", (0.0, 0.0, 0.0, 1.0))
    )
    contacts = _contact_summary(sim)
    control = _control_status(sim)
    return ViewerControlState(
        selected_joint=selected_joint,
        joint_targets=targets,
        joint_positions=positions,
        joint_velocities=velocities,
        gripper_width=float(state.gripper_width),
        gripper_actual_width=float(state.gripper_width),
        max_contact_force=contacts[0],
        contact_count=contacts[1],
        paused=paused,
        requested_torques=control[0],
        applied_torques=control[1],
        saturated=control[2],
        watchdog_remaining_s=control[3],
        active_mode=control[4],
        collision_visible=False if previous is None else previous.collision_visible,
        jog_speed_index=1 if previous is None else previous.jog_speed_index,
        joint_jog_rate=0.20 if previous is None else previous.joint_jog_rate,
        gripper_jog_rate=0.02 if previous is None else previous.gripper_jog_rate,
        interaction_mode="joint" if previous is None else previous.interaction_mode,
        selected_cartesian_axis=0 if previous is None else previous.selected_cartesian_axis,
        cartesian_frame="world" if previous is None else previous.cartesian_frame,
        dashboard_page="overview" if previous is None else previous.dashboard_page,
        plot_page="off" if previous is None else previous.plot_page,
        help_visible=False if previous is None else previous.help_visible,
        ee_position=ee_position,
        ee_rpy=ee_rpy,
        target_position=ee_position if previous is None else previous.target_position,
        target_rpy=ee_rpy if previous is None else previous.target_rpy,
    )


def _quaternion_xyzw_to_rpy(quaternion: Sequence[float]) -> tuple[float, float, float]:
    x, y, z, w = (float(value) for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _quaternion_wxyz_to_rpy(quaternion: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = (float(value) for value in quaternion)
    return _quaternion_xyzw_to_rpy((x, y, z, w))


def _control_status(sim) -> tuple[tuple[float, ...], tuple[float, ...], tuple[bool, ...], float, str]:
    if hasattr(sim, "get_control_status"):
        status = sim.get_control_status()
        remaining = status.watchdog_remaining_s
        return (
            tuple(float(value) for value in status.requested_torques[:6]),
            tuple(float(value) for value in status.applied_torques[:6]),
            tuple(bool(value) for value in status.saturated[:6]),
            0.0 if remaining is None else float(remaining),
            str(status.mode),
        )
    mode = str(getattr(sim, "control_mode", "hold"))
    return (0.0,) * 6, (0.0,) * 6, (False,) * 6, 0.0, mode


def _contact_summary(sim) -> tuple[float, int]:
    if not hasattr(sim, "get_contacts"):
        return 0.0, 0
    contacts = tuple(sim.get_contacts())
    if not contacts:
        return 0.0, 0
    return max(float(contact.force) for contact in contacts), len(contacts)


def _refresh_observed_state(sim, state: ViewerControlState) -> ViewerControlState:
    sim_state = sim.get_state()
    max_contact_force, contact_count = _contact_summary(sim)
    control = _control_status(sim)
    return replace(
        state,
        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
        joint_velocities=tuple(float(value) for value in sim_state.joint_velocities[:6]),
        gripper_actual_width=float(sim_state.gripper_width),
        max_contact_force=max_contact_force,
        contact_count=contact_count,
        requested_torques=control[0],
        applied_torques=control[1],
        saturated=control[2],
        watchdog_remaining_s=control[3],
        active_mode=control[4],
        ee_position=tuple(
            float(value)
            for value in getattr(
                sim_state, "end_effector_position", state.ee_position
            )
        ),
        ee_rpy=(
            _quaternion_xyzw_to_rpy(sim_state.end_effector_orientation)
            if hasattr(sim_state, "end_effector_orientation")
            else state.ee_rpy
        ),
    )


def _set_mode(sim, mode: str) -> str:
    setter = getattr(sim, "set_mode", None) or getattr(sim, "set_control_mode")
    return str(setter(mode))


def _command_positions(sim, targets):
    command = getattr(sim, "command_joint_positions", None) or getattr(
        sim, "set_joint_position_targets"
    )
    return command(targets)


def _command_gripper(sim, width: float) -> float:
    command = getattr(sim, "command_gripper_width", None) or getattr(sim, "set_gripper_width")
    return float(command(width))


def apply_pending_commands(
    sim,
    state: ViewerControlState,
    joint_step: float,
    gripper_step: float,
    *,
    jog_hold_time: float = 0.18,
) -> ViewerControlState:
    if state.reset or state.home:
        pending_joint_delta = state.joint_delta
        pending_gripper_delta = state.gripper_delta
        quit_requested = state.quit
        if state.home:
            sim.reset_home()
        else:
            sim.reset()
        _set_mode(sim, "hold")
        state = _state_from_sim(
            sim,
            paused=state.paused,
            selected_joint=state.selected_joint,
            previous=state,
        )
        state = replace(
            state,
            joint_delta=pending_joint_delta,
            gripper_delta=pending_gripper_delta,
            quit=quit_requested,
            cartesian_target_reset=True,
            dashboard_page="overview",
            plot_page="off",
            help_visible=False,
        )

    if state.mode is not None:
        active_mode = _set_mode(sim, state.mode)
    else:
        active_mode = getattr(sim, "control_mode", state.active_mode)

    targets = state.joint_targets
    if state.joint_delta:
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = targets[state.selected_joint] + state.joint_delta * joint_step
        targets = tuple(_command_positions(sim, {name: requested}))
        active_mode = getattr(sim, "control_mode", "position")

    width = state.gripper_width
    if state.gripper_delta:
        width = _command_gripper(sim, width + state.gripper_delta * gripper_step)

    if state.stop_jog:
        active_mode = _set_mode(sim, "hold")

    state = replace(
        state,
        joint_targets=targets,
        gripper_width=width,
        joint_delta=0,
        gripper_delta=0,
        jog_time_remaining=0.0,
        reset=False,
        home=False,
        mode=None,
        stop_jog=False,
        active_mode=str(active_mode),
    )
    return _refresh_observed_state(sim, state)


def apply_continuous_jog(
    sim,
    state: ViewerControlState,
    *,
    dt: float,
    joint_rate: float,
    gripper_rate: float,
    cartesian_controller=None,
    cartesian_update_period_s: float = 0.02,
) -> ViewerControlState:
    if not state.joint_jog_direction and not state.gripper_jog_direction:
        return state

    targets = state.joint_targets
    speed_scale = JOG_SPEED_LEVELS[state.jog_speed_index][1]
    cartesian_accumulator = state.cartesian_accumulator_s
    ik_status = state.ik_status
    ik_error = state.ik_error
    target_position = state.target_position
    target_rpy = state.target_rpy
    if state.joint_jog_direction and state.interaction_mode == "joint":
        name = ARM_JOINT_NAMES[state.selected_joint]
        requested = (
            targets[state.selected_joint]
            + state.joint_jog_direction * joint_rate * speed_scale * dt
        )
        targets = tuple(_command_positions(sim, {name: requested}))
    elif state.joint_jog_direction and cartesian_controller is not None:
        cartesian_accumulator += dt
        rate = gripper_rate if state.interaction_mode == "xyz" else joint_rate
        distance_or_angle = rate * speed_scale * cartesian_accumulator
        options = getattr(cartesian_controller, "options", None)
        tolerance = float(
            getattr(
                options,
                "position_tolerance_m"
                if state.interaction_mode == "xyz"
                else "orientation_tolerance_rad",
                0.0,
            )
        )
        # An increment inside the IK convergence tolerance succeeds at
        # iteration zero without changing a joint. Accumulate it instead of
        # repeatedly reporting a no-op as CONVERGED.
        command_due = (
            cartesian_accumulator >= cartesian_update_period_s
            and distance_or_angle > tolerance * 1.05
        )
        if command_due:
            direction = state.joint_jog_direction
            values = [0.0, 0.0, 0.0]
            values[state.selected_cartesian_axis] = direction * distance_or_angle
            delta = CartesianDelta(
                xyz_m=values if state.interaction_mode == "xyz" else (0.0, 0.0, 0.0),
                rpy_rad=values if state.interaction_mode == "rpy" else (0.0, 0.0, 0.0),
                frame=state.cartesian_frame,
            )
            result = cartesian_controller.command_delta(delta)
            ik_status = result.status
            ik_error = max(result.position_error_m, result.orientation_error_rad)
            if result.success:
                targets = tuple(result.joint_positions)
                _set_mode(sim, "position")
                target_position = tuple(
                    getattr(result, "target_position_m", state.target_position)
                )
                target_rpy = tuple(
                    getattr(result, "target_rpy_rad", state.target_rpy)
                )
            else:
                target_position = state.target_position
                target_rpy = state.target_rpy
            cartesian_accumulator = 0.0
        else:
            target_position = state.target_position
            target_rpy = state.target_rpy
    else:
        target_position = state.target_position
        target_rpy = state.target_rpy

    width = state.gripper_width
    if state.gripper_jog_direction:
        width = _command_gripper(
            sim, width + state.gripper_jog_direction * gripper_rate * speed_scale * dt
        )

    state = replace(
        state,
        joint_targets=targets,
        gripper_width=width,
        jog_time_remaining=0.0,
        active_mode=str(getattr(sim, "control_mode", state.active_mode)),
        cartesian_accumulator_s=cartesian_accumulator,
        ik_status=ik_status,
        ik_error=ik_error,
        target_position=target_position,
        target_rpy=target_rpy,
    )
    return _refresh_observed_state(sim, state)


def overlay_text(state: ViewerControlState) -> str:
    """Return the currently visible dashboard content for terminal diagnostics."""
    panels = compose_dashboard(state)
    blocks = []
    for panel in (
        panels.top_left,
        panels.top_right,
        panels.bottom_left,
        panels.bottom_right,
    ):
        if panel is not None:
            blocks.append(f"{panel.left}\n{panel.right}")
    return "\n\n".join(blocks)


def configure_viewer_rendering(
    viewer, *, collision_visible: bool, target_visible: bool = False
) -> None:
    option = getattr(viewer, "opt", None)
    groups = getattr(option, "geomgroup", None)
    if option is None or groups is None or len(groups) < 4:
        return
    lock = getattr(viewer, "lock", None)
    with lock() if lock is not None else nullcontext():
        groups[1] = int(target_visible)
        groups[2] = 1
        groups[3] = int(collision_visible)
        # The stock Simulate UI handles the same key event after our callback.
        # Restore its visualization flags so T/H/P/Z/X/J/C/O/V cannot make the
        # robot transparent, hide textures/lights, or add debug overlays while
        # those keys are being used as robot controls.
        flags = getattr(option, "flags", None)
        if flags is not None:
            mujoco = importlib.import_module("mujoco")
            for index, (_name, default, _shortcut) in enumerate(mujoco.mjVISSTRING):
                flags[index] = int(default)
            # Some stock shortcuts select labels or RGB coordinate frames via
            # fields outside ``flags``.  Reset those too; otherwise controls
            # such as Tab/F6/F7/V can leave large debug axes over the robot even
            # when the dashboard says the collision layer is hidden.
            option.frame = mujoco.mjtFrame.mjFRAME_NONE
            option.label = mujoco.mjtLabel.mjLABEL_NONE


def update_viewer_overlay(viewer, state: ViewerControlState) -> None:
    setter = getattr(viewer, "set_texts", None)
    if setter is not None:
        mujoco = importlib.import_module("mujoco")
        viewport = getattr(viewer, "viewport", None)
        compact = bool(
            viewport is not None
            and (
                int(getattr(viewport, "width", 1280)) < 1100
                or int(getattr(viewport, "height", 720)) < 700
            )
        )
        panels = compose_dashboard(state, compact=compact)
        positions = (
            (mujoco.mjtGridPos.mjGRID_TOPLEFT, panels.top_left),
            (mujoco.mjtGridPos.mjGRID_TOPRIGHT, panels.top_right),
            (mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, panels.bottom_left),
            (mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, panels.bottom_right),
        )
        setter(
            [
                (
                    mujoco.mjtFontScale.mjFONTSCALE_100,
                    position,
                    panel.left,
                    panel.right,
                )
                for position, panel in positions
                if panel is not None
            ]
        )


def update_ghost_overlay(ghost, viewer, joint_targets: Sequence[float]) -> bool:
    lock = getattr(viewer, "lock", None)
    with lock() if lock is not None else nullcontext():
        return bool(ghost.update(viewer, joint_targets))


def clear_ghost_overlay(ghost, viewer) -> bool:
    lock = getattr(viewer, "lock", None)
    with lock() if lock is not None else nullcontext():
        return bool(ghost.clear(viewer))


def align_cartesian_target(model, data) -> tuple[int, tuple[float, ...], tuple[float, ...]]:
    """Place the draggable mocap target on the current end-effector pose."""
    mujoco = importlib.import_module("mujoco")
    body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ee_target"))
    site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site"))
    if body_id < 0 or site_id < 0:
        raise ValueError("scene must define ee_target mocap body and ee_site")
    mocap_id = int(model.body_mocapid[body_id])
    if mocap_id < 0:
        raise ValueError("ee_target must be a mocap body")
    data.mocap_pos[mocap_id] = data.site_xpos[site_id]
    mujoco.mju_mat2Quat(data.mocap_quat[mocap_id], data.site_xmat[site_id])
    return (
        mocap_id,
        tuple(float(value) for value in data.mocap_pos[mocap_id]),
        tuple(float(value) for value in data.mocap_quat[mocap_id]),
    )


def process_cartesian_target(
    controller,
    data,
    mocap_id: int,
    previous_pose: tuple[tuple[float, ...], tuple[float, ...]],
    state: ViewerControlState,
) -> tuple[tuple[tuple[float, ...], tuple[float, ...]], ViewerControlState]:
    """Submit a dragged mocap pose once, leaving failed targets visible for diagnosis."""
    position = tuple(float(value) for value in data.mocap_pos[mocap_id])
    quaternion = tuple(float(value) for value in data.mocap_quat[mocap_id])
    pose = (position, quaternion)
    changed = any(
        abs(current - old) > 1e-8
        for current, old in zip(position + quaternion, previous_pose[0] + previous_pose[1])
    )
    if not changed:
        return previous_pose, state
    result = controller.command_pose(position, quaternion)
    return pose, replace(
        state,
        joint_targets=(tuple(result.joint_positions) if result.success else state.joint_targets),
        joint_jog_direction=0,
        gripper_jog_direction=0,
        ik_status=result.status,
        ik_error=max(result.position_error_m, result.orientation_error_rad),
        target_position=position,
        target_rpy=_quaternion_wxyz_to_rpy(quaternion),
    )


def _decode_key(keycode: int) -> str:
    if keycode == 256:
        return "\x1b"
    if keycode == 333:  # GLFW_KEY_KP_SUBTRACT
        return "-"
    if keycode == 334:  # GLFW_KEY_KP_ADD
        return "+"
    special = {
        258: "tab",
        295: "f6",
        296: "f7",
        297: "f8",
        298: "f9",
    }
    if keycode in special:
        return special[keycode]
    try:
        return chr(keycode)
    except (TypeError, ValueError):
        return ""
