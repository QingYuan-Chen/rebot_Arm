"""Compare one saved Reach policy on identical CPU MuJoCo and MJX goals."""
import argparse
import json
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from .gym_reach import RebotArmReachEnv
from .mjx_reach import MJXReachVecEnv


def _summary(records):
    def mean(key):
        return float(np.mean([record[key] for record in records]))
    return {"episodes": len(records), "success_rate": mean("success"),
            "mean_final_distance_m": mean("final_distance_m"),
            "mean_final_orientation_error_deg": mean("final_orientation_error_deg"),
            "position_gate_rate": mean("position_gate"),
            "orientation_gate_rate": mean("orientation_gate"),
            "pose_gate_rate": mean("pose_gate"),
            "pose_and_speed_gate_rate": mean("pose_and_speed_gate")}


def _record(index, start, final, steps):
    distance = final["distance_m"]
    angle = float(np.rad2deg(final["orientation_error_rad"]))
    speed = final["max_joint_speed_rad_s"]
    return {"goal_index": index, "initial_distance_m": start["distance_m"],
            "initial_orientation_error_deg": float(np.rad2deg(start["orientation_error_rad"])),
            "success": bool(final["is_success"]), "steps": steps,
            "final_distance_m": distance, "final_orientation_error_deg": angle,
            "final_max_joint_speed_rad_s": speed, "final_hold_steps": final["hold_steps"],
            "position_gate": distance < 0.01, "orientation_gate": angle < 3.0,
            "pose_gate": distance < 0.01 and angle < 3.0,
            "pose_and_speed_gate": distance < 0.01 and angle < 3.0 and speed < 0.1}


def run(model, episodes, seed, backend, stage="local"):
    with RebotArmReachEnv(curriculum_stage=stage) as cpu:
        goals = []
        for index in range(episodes):
            cpu.reset(seed=seed + index)
            goals.append(cpu._goal_q.copy())
        goals = np.asarray(goals)
        cpu_records = []
        if backend in ("cpu", "both"):
            for index, goal in enumerate(goals):
                observation, start = cpu.reset(options={"goal_q": goal})
                for step in range(1, cpu.max_episode_steps + 1):
                    action, _ = model.predict(observation, deterministic=True)
                    observation, _, terminated, truncated, info = cpu.step(action)
                    if terminated or truncated:
                        cpu_records.append(_record(index, start, info, step))
                        break
    mjx_records = []
    if backend in ("mjx", "both"):
        # One goal per lane; retain each lane's first completed episode only.
        env = MJXReachVecEnv(episodes, curriculum_stage=stage, seed=seed)
        try:
            observations = env.reset_to_goals(goals)
            starts = [{"distance_m": float(np.linalg.norm(row[12:15])),
                       "orientation_error_rad": float(np.linalg.norm(row[15:18]))}
                      for row in observations]
            completed = np.zeros(episodes, dtype=bool)
            for step in range(1, env.max_episode_steps + 1):
                action, _ = model.predict(observations, deterministic=True)
                observations, _, dones, infos = env.step(action)
                for index in np.flatnonzero(dones & ~completed):
                    mjx_records.append(_record(int(index), starts[index], infos[index], step))
                    completed[index] = True
                if np.all(completed):
                    break
        finally:
            env.close()
    return {"seed": seed, "goals_joint_rad": goals.tolist(),
            "cpu": {"summary": _summary(cpu_records), "per_goal": cpu_records} if cpu_records else None,
            "mjx": {"summary": _summary(mjx_records), "per_goal": mjx_records} if mjx_records else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=20000)
    parser.add_argument("--backend", choices=("cpu", "mjx", "both"), default="both")
    parser.add_argument("--curriculum-stage", choices=("local_easy", "local"), default="local")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.episodes <= 1024 or args.seed < 0:
        parser.error("episodes must be in [1, 1024] and seed nonnegative")
    model = PPO.load(args.model, device="cuda")
    report = run(model, args.episodes, args.seed, args.backend, args.curriculum_stage)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({name: value["summary"] for name, value in report.items()
                      if name in ("cpu", "mjx") and value}, indent=2))


if __name__ == "__main__":
    main()
