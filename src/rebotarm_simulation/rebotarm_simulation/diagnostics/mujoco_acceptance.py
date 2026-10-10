from __future__ import annotations

import argparse
import json
import sys
from typing import Callable, Sequence

from rebotarm_simulation.diagnostics.mujoco_batch import run_batch
from rebotarm_simulation.diagnostics.mujoco_contact_check import run_contact_check
from rebotarm_simulation.diagnostics.mujoco_health import collect_health
from rebotarm_simulation.diagnostics.mujoco_pick_batch import run_pick_batch


def _run_step(name: str, operation: Callable[[], dict]) -> dict:
    try:
        payload = operation()
        return {
            "name": name,
            "ok": bool(payload.get("ok")),
            "payload": payload,
        }
    except Exception as exc:
        return {
            "name": name,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the reBotArm MuJoCo acceptance suite")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--skip-renderer", action="store_true")
    parser.add_argument("--include-ros", action="store_true", help="also run in-process ROS 2 interface acceptance")
    parser.add_argument(
        "--include-moveit",
        action="store_true",
        help="also probe an already running MoveIt + MuJoCo launch pair",
    )
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser


def run_acceptance_suite(
    *,
    model: str | None = None,
    skip_renderer: bool = False,
    include_ros: bool = False,
    include_moveit: bool = False,
    timeout: float = 30.0,
) -> dict:
    steps = [
        _run_step(
            "health",
            lambda: collect_health(
                model,
                renderer_check=(
                    (lambda _sim: (False, "renderer check skipped"))
                    if skip_renderer
                    else None
                ),
            ),
        ),
        _run_step(
            "headless_reach_batch",
            lambda: run_batch(model=model, episodes=1, steps=5, seed=0),
        ),
        _run_step(
            "headless_pick_environment",
            lambda: run_pick_batch(
                model=model,
                episodes=1,
                steps=5,
                seed=0,
                action_magnitude=0.0,
                config_path=None,
                profile="default",
            ),
        ),
        _run_step(
            "cube_contact",
            lambda: run_contact_check(
                model=model,
                settle_steps=3000,
                contact_steps=600,
                min_finger_contact_steps=100,
                max_contact_force=20.0,
                max_cube_jump=0.005,
                min_cube_z=0.018,
            ),
        ),
    ]
    if include_ros:
        from rebotarm_simulation.diagnostics.mujoco_ros_acceptance import run_acceptance as run_ros_acceptance

        steps.append(
            _run_step("ros2_interfaces", lambda: run_ros_acceptance(timeout=timeout))
        )
    if include_moveit:
        from rebotarm_simulation.diagnostics.mujoco_moveit_acceptance import run_acceptance as run_moveit_acceptance

        steps.append(
            _run_step("moveit_execution", lambda: run_moveit_acceptance(timeout=timeout))
        )
    return {
        "ok": all(step["ok"] for step in steps),
        "steps": steps,
    }


def main(argv: Sequence[str] | None = None, *, stdout=None) -> int:
    args = build_parser().parse_args(argv)
    if stdout is None:
        stdout = sys.stdout
    payload = run_acceptance_suite(
        model=args.model,
        skip_renderer=args.skip_renderer,
        include_ros=args.include_ros,
        include_moveit=args.include_moveit,
        timeout=args.timeout,
    )
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
