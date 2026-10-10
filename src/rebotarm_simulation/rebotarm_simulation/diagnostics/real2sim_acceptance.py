from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco
from rebotarm_simulation.core.real2sim import (
    JointMappingConfig,
    Real2SimMapper,
    Real2SimSynchronizer,
    RobotStateSample,
    default_real2sim_mapping_path,
)


HOME = np.asarray((0.0, -0.8, -1.0, 0.3, 0.0, 0.0), dtype=float)
AMPLITUDE = np.asarray((0.12, 0.08, 0.08, 0.06, 0.05, 0.08), dtype=float)
PHASE = np.asarray((0.0, 0.5, 1.0, 1.5, 2.0, 2.5), dtype=float)


def _positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def _positive_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError("must be positive and finite")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run no-hardware reBotArm Real2Sim synchronization acceptance"
    )
    parser.add_argument("--model", default=None)
    parser.add_argument("--mapping", default=None)
    parser.add_argument("--profile", default="rebotarm")
    parser.add_argument("--mode", choices=("mirror", "physics"), default="mirror")
    parser.add_argument("--steps", type=_positive_int, default=200)
    parser.add_argument("--rate-hz", type=_positive_float, default=100.0)
    parser.add_argument("--physics-steps-per-update", type=_positive_int, default=5)
    parser.add_argument("--max-tracking-error", type=_positive_float, default=None)
    return parser


def run_acceptance(
    *,
    model: str | None,
    mapping_path: str | None,
    profile: str,
    mode: str,
    steps: int,
    rate_hz: float,
    physics_steps_per_update: int,
    max_tracking_error: float | None,
    sim_factory: Callable = RebotArmMujoco,
) -> dict:
    config = JointMappingConfig.from_yaml(
        mapping_path or default_real2sim_mapping_path(), profile=profile
    )
    errors = []
    gripper_errors = []
    finite = True
    with sim_factory(model) as simulation:
        simulation.reset_home()
        synchronizer = Real2SimSynchronizer(
            simulation,
            Real2SimMapper(config),
            mode=mode,
            physics_steps_per_update=physics_steps_per_update,
        )
        dt = 1.0 / float(rate_hz)
        for index in range(steps):
            timestamp = (index + 1) * dt
            phase = 2.0 * math.pi * 0.15 * timestamp
            positions = HOME + AMPLITUDE * np.sin(phase + PHASE)
            velocities = AMPLITUDE * 2.0 * math.pi * 0.15 * np.cos(phase + PHASE)
            width = 0.055 + 0.01 * math.sin(phase)
            result = synchronizer.apply(
                RobotStateSample(
                    timestamp=timestamp,
                    joint_names=ARM_JOINT_NAMES,
                    positions=tuple(positions),
                    velocities=tuple(velocities),
                    gripper_width=width,
                )
            )
            errors.append(result.max_tracking_error_rad)
            gripper_errors.append(abs(result.gripper_width - width))
            finite = finite and all(
                math.isfinite(value)
                for value in (
                    *result.simulated_positions,
                    result.gripper_width,
                    result.simulation_time,
                )
            )
        final_state = simulation.get_state()
    default_limit = 1e-8 if mode == "mirror" else 0.25
    error_limit = default_limit if max_tracking_error is None else float(max_tracking_error)
    max_error = max(errors, default=math.inf)
    ok = (
        finite
        and len(errors) == steps
        and max_error <= error_limit
        and float(final_state.simulation_time) > 0.0
    )
    return {
        "ok": ok,
        "hardware_connected": False,
        "source": "synthetic_joint_state",
        "mode": mode,
        "steps": steps,
        "rate_hz": float(rate_hz),
        "mapping_profile": profile,
        "physics_steps_per_update": physics_steps_per_update,
        "max_tracking_error_rad": max_error,
        "tracking_error_limit_rad": error_limit,
        "mean_tracking_error_rad": float(np.mean(errors)),
        "max_gripper_error_m": max(gripper_errors, default=math.inf),
        "simulation_time": float(final_state.simulation_time),
        "finite_state": finite,
    }


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    stdout=None,
    stderr=None,
) -> int:
    args = build_parser().parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        payload = run_acceptance(
            model=args.model,
            mapping_path=args.mapping,
            profile=args.profile,
            mode=args.mode,
            steps=args.steps,
            rate_hz=args.rate_hz,
            physics_steps_per_update=args.physics_steps_per_update,
            max_tracking_error=args.max_tracking_error,
            sim_factory=sim_factory,
        )
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
