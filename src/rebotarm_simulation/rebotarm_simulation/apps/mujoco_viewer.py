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



from rebotarm_simulation.apps.viewer_interaction import (
    ViewerControlState,
    reduce_key,
    _take_key_snapshot,
    _take_line_snapshot,
    start_command_reader,
    drain_key_events,
    process_command_events,
    _command_value_text,
    _jsonable,
    process_key_events,
    _state_from_sim,
    _quaternion_xyzw_to_rpy,
    _quaternion_wxyz_to_rpy,
    _control_status,
    _contact_summary,
    _refresh_observed_state,
    _set_mode,
    _command_positions,
    _command_gripper,
    apply_pending_commands,
    apply_continuous_jog,
    overlay_text,
    configure_viewer_rendering,
    update_viewer_overlay,
    update_ghost_overlay,
    clear_ghost_overlay,
    align_cartesian_target,
    process_cartesian_target,
    _decode_key,
)
from rebotarm_simulation.apps.viewer_lifecycle import _close_viewer_then_sim

def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control reBotArm in the MuJoCo viewer")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--joint-step", type=_positive_float, default=0.01, help="joint jog in radians")
    parser.add_argument("--gripper-step", type=_positive_float, default=0.001, help="gripper jog in metres")
    parser.add_argument(
        "--joint-rate",
        type=_positive_float,
        default=0.20,
        help="normal-gear joint jog rate in rad/s",
    )
    parser.add_argument(
        "--gripper-rate",
        type=_positive_float,
        default=0.02,
        help="normal-gear gripper jog rate in m/s",
    )
    parser.add_argument(
        "--jog-hold-time",
        type=_positive_float,
        default=0.18,
        help="seconds to keep jogging after the latest key-repeat event",
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=None,
        help="exit after this many seconds of simulated time",
    )
    parser.add_argument(
        "--no-command-input",
        action="store_true",
        help="disable terminal line commands while the viewer is running",
    )
    parser.add_argument(
        "--verbose-status",
        action="store_true",
        help="also print changing real-time state to the terminal (off by default)",
    )
    return parser


def _launch_passive_viewer(launch_passive: Callable, model, data, on_key):
    """Launch while hiding one harmless GLFW/Wayland capability warning."""
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message=r".*Wayland: The platform does not provide the window position.*",
            category=Warning,
        )
        return launch_passive(
            model,
            data,
            key_callback=on_key,
            show_left_ui=False,
            show_right_ui=False,
        )


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    launch_passive: Callable | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    status_stream=None,
    command_stream=None,
) -> int:
    args = build_parser().parse_args(argv)
    if status_stream is None:
        status_stream = sys.stderr
    sim = sim_factory(args.model)
    viewer = None
    model = data = None
    try:
        sim.reset_home()
        _set_mode(sim, "hold")
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        state = replace(
            _state_from_sim(sim),
            joint_jog_rate=args.joint_rate,
            gripper_jog_rate=args.gripper_rate,
        )
        cartesian_controller = (
            MujocoCartesianController(sim) if isinstance(sim, RebotArmMujoco) else None
        )
        telemetry = MujocoTelemetryHistory(
            capacity=round(TELEMETRY_SAMPLE_HZ * TELEMETRY_WINDOW_S)
        )
        start_simulation_time = float(sim.get_state().simulation_time)
        events = SimpleQueue()
        command_events = SimpleQueue()
        if command_stream is None:
            command_stream = sys.stdin
            command_input_enabled = (
                not args.no_command_input
                and hasattr(command_stream, "isatty")
                and command_stream.isatty()
            )
        else:
            command_input_enabled = not args.no_command_input
        if command_input_enabled:
            command_thread = start_command_reader(command_stream, command_events)
            if command_stream is not sys.stdin and (
                not hasattr(command_stream, "isatty") or not command_stream.isatty()
            ):
                command_thread.join(timeout=0.05)
        previous_status = overlay_text(state)
        print(
            "reBotArm MuJoCo viewer ready: Home + Hold. "
            "Real-time state is shown in the viewer; press Q to quit.",
            file=status_stream,
            flush=True,
        )

        def on_key(keycode: int) -> None:
            events.put(keycode)

        model, data = sim._unsafe_viewer_handles()
        ghost = GhostArmOverlay(model)
        figures = TelemetryFigures(max_points=500)
        target_mocap_id = None
        target_pose = None
        if cartesian_controller is not None:
            target_mocap_id, target_position, target_quaternion = align_cartesian_target(
                model, data
            )
            target_pose = (target_position, target_quaternion)
            state = replace(
                state,
                target_position=target_position,
                target_rpy=_quaternion_wxyz_to_rpy(target_quaternion),
            )
        viewer = _launch_passive_viewer(launch_passive, model, data, on_key)
        configure_viewer_rendering(
            viewer, collision_visible=False, target_visible=False
        )
        update_viewer_overlay(viewer, state)
        displayed_dashboard = overlay_text(state)
        last_visual_update_sim_time = float("-inf")
        last_telemetry_sample_sim_time = float("-inf")
        last_dashboard_update_sim_time = float("-inf")
        plots_attached = False
        ghost_visible = False
        try:
            while viewer.is_running():
                state = process_command_events(sim, command_events, state, status_stream)
                if state.quit:
                    break
                state = process_key_events(
                    sim,
                    events,
                    state,
                    args.joint_step,
                    args.gripper_step,
                    args.jog_hold_time,
                )
                if state.quit:
                    break
                if cartesian_controller is not None and state.cartesian_target_reset:
                    target_mocap_id, target_position, target_quaternion = align_cartesian_target(
                        model, data
                    )
                    target_pose = (target_position, target_quaternion)
                    state = replace(
                        state,
                        cartesian_target_reset=False,
                        ik_status="idle",
                        target_position=target_position,
                        target_rpy=_quaternion_wxyz_to_rpy(target_quaternion),
                    )
                if cartesian_controller is not None and target_pose is not None:
                    target_pose, state = process_cartesian_target(
                        cartesian_controller,
                        data,
                        target_mocap_id,
                        target_pose,
                        state,
                    )
                cycle_start = clock()
                if not state.paused or state.single_step:
                    state = apply_continuous_jog(
                        sim,
                        state,
                        dt=sim.timestep,
                        joint_rate=args.joint_rate,
                        gripper_rate=args.gripper_rate,
                        cartesian_controller=cartesian_controller,
                    )
                    sim.step()
                    state = _refresh_observed_state(sim, replace(state, single_step=False))
                    simulation_time = float(sim.get_state().simulation_time)
                    telemetry_due = (
                        simulation_time < last_telemetry_sample_sim_time
                        or simulation_time - last_telemetry_sample_sim_time
                        >= 1.0 / TELEMETRY_SAMPLE_HZ
                    )
                    if telemetry_due and hasattr(sim, "get_control_status"):
                        try:
                            telemetry.append(
                                float(sim.get_state().simulation_time),
                                sim.get_control_status(),
                                sim.get_contacts() if hasattr(sim, "get_contacts") else (),
                            )
                        except (AttributeError, TypeError, ValueError):
                            # Test doubles and third-party adapters may expose only
                            # the older, smaller status record.
                            pass
                        last_telemetry_sample_sim_time = simulation_time
                current_status = overlay_text(state)
                if args.verbose_status and current_status != previous_status:
                    print(current_status, file=status_stream, flush=True)
                    previous_status = current_status
                configure_viewer_rendering(
                    viewer,
                    collision_visible=state.collision_visible,
                    target_visible=state.interaction_mode != "joint",
                )
                simulation_time = float(sim.get_state().simulation_time)
                visual_update_due = (
                    simulation_time < last_visual_update_sim_time
                    or simulation_time - last_visual_update_sim_time
                    >= 1.0 / VISUAL_UPDATE_HZ
                )
                if visual_update_due:
                    if state.interaction_mode != "joint":
                        ghost_visible = update_ghost_overlay(
                            ghost, viewer, state.joint_targets
                        )
                    elif ghost_visible:
                        clear_ghost_overlay(ghost, viewer)
                        ghost_visible = False
                    if state.plot_page != "off":
                        figures.select(
                            "tracking" if state.plot_page == "tracking" else "torque"
                        )
                        figures.update(
                            telemetry.snapshot(), joint_index=state.selected_joint
                        )
                        plots_attached = figures.attach_active(viewer)
                    last_visual_update_sim_time = simulation_time
                if state.plot_page == "off" and plots_attached:
                    figures.clear(viewer)
                    plots_attached = False
                dashboard_due = (
                    simulation_time < last_dashboard_update_sim_time
                    or simulation_time - last_dashboard_update_sim_time
                    >= 1.0 / DASHBOARD_UPDATE_HZ
                    or current_status != displayed_dashboard
                )
                if dashboard_due:
                    update_viewer_overlay(viewer, state)
                    displayed_dashboard = current_status
                    last_dashboard_update_sim_time = simulation_time
                viewer.sync()
                elapsed = float(sim.get_state().simulation_time) - start_simulation_time
                if args.duration is not None and elapsed >= args.duration:
                    break
                sleep(max(0.0, sim.timestep - (clock() - cycle_start)))
            return 0
        except KeyboardInterrupt:
            return 130
    finally:
        _close_viewer_then_sim(
            viewer,
            sim,
            model,
            data,
            clock=clock,
            sleep=sleep,
        )



if __name__ == "__main__":
    raise SystemExit(main())
