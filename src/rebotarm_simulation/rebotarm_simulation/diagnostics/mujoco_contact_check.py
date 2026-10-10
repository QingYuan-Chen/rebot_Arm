from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco


CONTACT_TEST_ARM_TARGET = (
    -0.35102474,
    -2.01621516,
    -0.39537157,
    -1.06428339,
    -1.27931398,
    0.38051148,
)
CONTACT_TEST_CUBE_POSE = (0.31, 0.04, 0.04)


def _positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the headless MuJoCo cube contact check")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--settle-steps", type=_positive_int, default=3000)
    parser.add_argument("--contact-steps", type=_positive_int, default=600)
    parser.add_argument("--min-finger-contact-steps", type=_positive_int, default=100)
    parser.add_argument("--max-contact-force", type=float, default=20.0)
    parser.add_argument("--max-cube-jump", type=float, default=0.005)
    parser.add_argument("--min-cube-z", type=float, default=0.018)
    return parser


def _cube_finger_contact(contact) -> bool:
    bodies = (contact.body1, contact.body2)
    return "test_cube" in bodies and (
        "finger" in contact.body1 or "finger" in contact.body2
    )


def run_contact_check(
    *,
    model: str | None,
    settle_steps: int,
    contact_steps: int,
    min_finger_contact_steps: int,
    max_contact_force: float,
    max_cube_jump: float,
    min_cube_z: float,
    sim_factory: Callable = RebotArmMujoco,
) -> dict:
    with sim_factory(model) as sim:
        sim.reset_home()
        sim.set_gripper_width(0.09)
        sim.set_joint_position_targets(CONTACT_TEST_ARM_TARGET)
        sim.step(settle_steps)
        sim.set_object_pose("test_cube", CONTACT_TEST_CUBE_POSE, (0.0, 0.0, 0.0, 1.0))
        sim.step(50)
        sim.set_gripper_width(0.025)

        cube_positions = []
        finger_contact_steps = 0
        max_finger_force = 0.0
        max_cube_contact_force = 0.0
        for _ in range(contact_steps):
            state = sim.step()
            cube_positions.append(state.object_poses["test_cube"][:3])
            contacts = sim.get_contacts()
            has_finger_contact = False
            for contact in contacts:
                if "test_cube" not in (contact.body1, contact.body2):
                    continue
                max_cube_contact_force = max(max_cube_contact_force, float(contact.force))
                if _cube_finger_contact(contact):
                    has_finger_contact = True
                    max_finger_force = max(max_finger_force, float(contact.force))
            if has_finger_contact:
                finger_contact_steps += 1

        positions = np.asarray(cube_positions, dtype=float)
        deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1) if len(positions) > 1 else np.zeros(1)
        final_state = sim.get_state()
        min_observed_z = float(np.min(positions[:, 2])) if len(positions) else math.inf
        max_step_jump = float(np.max(deltas)) if len(deltas) else 0.0
        finite = bool(np.isfinite(positions).all())
        ok = (
            finite
            and finger_contact_steps >= min_finger_contact_steps
            and max_cube_contact_force <= max_contact_force
            and max_step_jump <= max_cube_jump
            and min_observed_z >= min_cube_z
        )
        return {
            "ok": ok,
            "settle_steps": settle_steps,
            "contact_steps": contact_steps,
            "finger_contact_steps": finger_contact_steps,
            "max_finger_force_n": max_finger_force,
            "max_cube_contact_force_n": max_cube_contact_force,
            "max_cube_step_jump_m": max_step_jump,
            "min_cube_z_m": min_observed_z,
            "final_cube_pose": final_state.object_poses["test_cube"],
            "final_gripper_width_m": final_state.gripper_width,
        }


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    stdout=None,
    stderr=None,
) -> int:
    args = build_parser().parse_args(argv)
    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    try:
        payload = run_contact_check(
            model=args.model,
            settle_steps=args.settle_steps,
            contact_steps=args.contact_steps,
            min_finger_contact_steps=args.min_finger_contact_steps,
            max_contact_force=args.max_contact_force,
            max_cube_jump=args.max_cube_jump,
            min_cube_z=args.min_cube_z,
            sim_factory=sim_factory,
        )
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
