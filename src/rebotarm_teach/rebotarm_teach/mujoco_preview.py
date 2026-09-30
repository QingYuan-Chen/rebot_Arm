"""Preview a recorded teach path using the existing preparation pipeline and MuJoCo."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from .teach_recording import load_teach_samples, prepare_teach_replay_samples


JOINTS = tuple(f"joint{index}" for index in range(1, 7))


def preview_record(record_path: str | Path, *, viewer: bool = False) -> dict:
    samples = load_teach_samples(record_path)
    if len(samples) < 2:
        raise ValueError("teach record needs at least two samples")
    if any(tuple(sample.joint_names) != JOINTS for sample in samples):
        raise ValueError("teach record must use joint1..joint6 in canonical order")
    if any(len(sample.positions) != 6 or not all(math.isfinite(float(v)) for v in sample.positions) for sample in samples):
        raise ValueError("teach record positions must be six finite values")
    if any(right.stamp <= left.stamp for left, right in zip(samples, samples[1:])):
        raise ValueError("teach timestamps must increase")
    prepared = prepare_teach_replay_samples(samples, retime_enabled=True)
    if prepared.raw_quality.risk_level == "red" or prepared.after_quality.risk_level == "red":
        raise ValueError("teach record has red quality and cannot be previewed as an executable path")
    if not prepared.retimed_points:
        raise ValueError("teach preparation produced no retimed path")

    from rebotarm_simulation.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.offline_trajectory import play_path
    from rebotarm_simulation.resource_paths import package_resource

    sim = RebotArmMujoco(package_resource(
        "rebotarm_simulation", "models/rebotarm/scene_bottle.xml"
    ))
    window = None
    release_sim = True
    try:
        sim.reset_joint_positions(samples[0].positions)
        initial = sim.get_state()
        contact_counts = {"table_arm": 0, "bottle_arm": 0, "bottle_table": 0, "arm_self": 0}
        max_bottle_force = 0.0

        def observe(_state, contacts):
            nonlocal max_bottle_force
            for contact in contacts:
                bodies = {contact.body1, contact.body2}
                robot = any(body.startswith("link") or body == "end_link" or "finger_link" in body for body in bodies)
                if "table" in bodies and robot:
                    contact_counts["table_arm"] += 1
                if "bottle" in bodies:
                    if robot:
                        contact_counts["bottle_arm"] += 1
                        max_bottle_force = max(max_bottle_force, contact.force)
                    if "table" in bodies:
                        contact_counts["bottle_table"] += 1
                if all(body.startswith("link") or body == "end_link" for body in bodies):
                    contact_counts["arm_self"] += 1

        try:
            if viewer:
                import mujoco.viewer
                window = mujoco.viewer.launch_passive(sim._model, sim._data)
            def on_step(_sim):
                if window is not None:
                    window.sync()
                    time.sleep(sim.timestep)
            metrics = play_path(
                sim, prepared.retimed_points, JOINTS,
                observe=observe, on_step=on_step if viewer else None,
            )
            final = sim.get_state()
        finally:
            if window is not None:
                try:
                    window.close()
                except BaseException:
                    from rebotarm_simulation.mujoco_viewer import _RETAINED_UNSAFE_VIEWERS
                    _RETAINED_UNSAFE_VIEWERS.append((window, sim, sim._model, sim._data))
                    release_sim = False
                    raise
                deadline = time.monotonic() + 5.0
                while getattr(window, "m", None) is not None and time.monotonic() < deadline:
                    time.sleep(0.01)
                if getattr(window, "m", None) is not None:
                    # Keep native model/data alive if the passive viewer has not released them.
                    from rebotarm_simulation.mujoco_viewer import _RETAINED_UNSAFE_VIEWERS
                    _RETAINED_UNSAFE_VIEWERS.append((window, sim, sim._model, sim._data))
                    release_sim = False
                    raise RuntimeError("MuJoCo viewer did not finish closing")
        return {
            "record_path": str(Path(record_path).resolve()),
            "raw_samples": len(samples),
            "prepared_points": len(prepared.retimed_points),
            "raw_quality": prepared.raw_quality.risk_level,
            "prepared_quality": prepared.after_quality.risk_level,
            "time_parameterization": prepared.time_parameterization_used_method,
            "trajectory": metrics,
            "contacts": contact_counts,
            "max_bottle_contact_force_n": max_bottle_force,
            "initial_bottle_xyz_m": initial.object_poses["bottle"][:3],
            "final_bottle_xyz_m": final.object_poses["bottle"][:3],
            "final_arm_positions_rad": final.joint_positions[:6],
            "simulation_only": True,
        }
    finally:
        if release_sim:
            sim.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", type=Path, help="existing teach JSONL record")
    parser.add_argument("--viewer", action="store_true", help="show playback in a MuJoCo window")
    args = parser.parse_args(argv)
    try:
        report = preview_record(args.record, viewer=args.viewer)
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}))
        return 1
    print(json.dumps({"ok": True, **report}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
