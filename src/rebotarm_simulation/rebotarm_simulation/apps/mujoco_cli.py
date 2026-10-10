from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import json
import math
from pathlib import Path
import sys

from rebotarm_simulation.apps.mujoco_commands import dispatch_sim_command
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco


def _nonnegative_finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative finite number")
    return number


def _positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def _positive_finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run or diagnose the reBotArm MuJoCo backend")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="run a headless simulation")
    run.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    run.add_argument("--duration", type=_nonnegative_finite, help="simulation seconds to run")
    run.add_argument("--steps", type=_positive_int, help="number of physics steps to run")

    shell = subparsers.add_parser("shell", help="read simulation commands from standard input")
    shell.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")

    torque = subparsers.add_parser("torque", help="apply a watchdog-limited diagnostic torque")
    torque.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    torque.add_argument("--values", nargs=6, type=float, required=True, metavar="NM")
    torque.add_argument("--timeout", type=_positive_finite, default=0.1)
    torque.add_argument(
        "--observe", type=_nonnegative_finite, default=0.2,
        help="simulation seconds to observe, including watchdog fallback",
    )
    return parser


def _plain(value):
    if is_dataclass(value):
        return {field.name: _plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "__dict__"):
        return {key: _plain(item) for key, item in vars(value).items()}
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _emit(value, stdout) -> None:
    print(json.dumps(_plain(value), ensure_ascii=False, sort_keys=True), file=stdout)


def dispatch_command(sim, line: str, *, paused: bool = False):
    result = dispatch_sim_command(sim, line, paused=paused)
    return result.paused, result.value, result.should_quit


def _run_headless(sim, duration: float | None, steps: int | None):
    if steps is not None:
        sim.step(steps)
    requested_duration = duration
    duration_start = float(sim.get_state().simulation_time)
    if requested_duration is not None:
        target = duration_start + requested_duration
        while float(sim.get_state().simulation_time) + 1e-15 < target:
            sim.step()
    elif steps is None:
        sim.step()
    state = _plain(sim.get_state())
    state["requested_duration"] = requested_duration
    state["achieved_duration"] = (
        float(state["simulation_time"]) - duration_start if requested_duration is not None else None
    )
    return state


def _prepare_simulation(sim) -> None:
    sim.reset_home()
    sim.set_mode("hold")


def _run_torque(sim, values, *, timeout: float, observe: float):
    sim.command_joint_torques(values, timeout_s=timeout)
    start = float(sim.get_state().simulation_time)
    while float(sim.get_state().simulation_time) - start + 1e-15 < observe:
        sim.step()
    return {
        "state": sim.get_state(),
        "control": sim.get_control_status(),
        "requested_timeout_s": timeout,
        "observed_duration_s": float(sim.get_state().simulation_time) - start,
    }


def _interactive(sim, stdin, stdout) -> int:
    paused = False
    for line in stdin:
        try:
            paused, result, should_quit = dispatch_command(sim, line, paused=paused)
            if result is not None:
                _emit(result, stdout)
            if should_quit:
                return 0
        except (argparse.ArgumentTypeError, RuntimeError, TypeError, ValueError) as exc:
            print(f"error: {exc}", file=stdout)
    return 0


def main(argv=None, *, sim_factory=RebotArmMujoco, stdin=None, stdout=None, stderr=None) -> int:
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    args = build_parser().parse_args(argv)
    sim = None
    try:
        sim = sim_factory(args.model)
        _prepare_simulation(sim)
        if args.command == "run":
            _emit(_run_headless(sim, args.duration, args.steps), stdout)
            return 0
        if args.command == "torque":
            _emit(
                _run_torque(
                    sim, args.values, timeout=args.timeout, observe=args.observe
                ),
                stdout,
            )
            return 0
        return _interactive(sim, stdin, stdout)
    except Exception as exc:
        print(f"MuJoCo CLI error: {exc}", file=stderr)
        return 1
    finally:
        if sim is not None:
            sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
