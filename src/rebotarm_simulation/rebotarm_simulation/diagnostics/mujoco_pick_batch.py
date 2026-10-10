from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_pick_env import RebotArmPickEnv
from rebotarm_simulation.sim2real import (
    RandomizationConfig,
    TrajectoryRecorder,
    default_randomization_config_path,
    safety_limits_from_env,
    validate_trajectory,
)


def _positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def _non_negative_int(value: str) -> int:
    result = int(value)
    if result < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run headless MuJoCo Pick environment acceptance rollouts"
    )
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--episodes", type=_positive_int, default=3)
    parser.add_argument("--steps", type=_positive_int, default=100)
    parser.add_argument("--seed", type=_non_negative_int, default=0)
    parser.add_argument("--action-magnitude", type=float, default=0.25)
    parser.add_argument("--randomization-config", default=None)
    parser.add_argument("--randomization-profile", default="training_profile")
    parser.add_argument("--log-dir", default=None)
    return parser


def run_pick_batch(
    *,
    model: str | None,
    episodes: int,
    steps: int,
    seed: int,
    action_magnitude: float,
    config_path: str | None,
    profile: str,
    log_dir: str | Path | None = None,
    env_factory: Callable = RebotArmPickEnv,
) -> dict:
    action_magnitude = float(action_magnitude)
    if not math.isfinite(action_magnitude) or not 0.0 <= action_magnitude <= 1.0:
        raise ValueError("action_magnitude must be between 0 and 1")
    randomization_config = RandomizationConfig.from_yaml(
        config_path or default_randomization_config_path(), profile=profile
    )
    results = []
    failure_counts: Counter[str] = Counter()
    for episode in range(episodes):
        episode_seed = seed + episode
        sample = randomization_config.sample(episode_seed)
        rng = np.random.default_rng(episode_seed)
        episode_id = f"pick-seed-{episode_seed}"
        recorder = TrajectoryRecorder(episode_id=episode_id, source="sim")
        total_reward = 0.0
        stage_counts: Counter[str] = Counter()
        episode_max_force = 0.0
        episode_max_penetration = 0.0
        terminated = truncated = False
        with env_factory(model) as env:
            limits = safety_limits_from_env(
                env,
                max_contact_force=env.config.task.max_contact_force_n,
                max_contact_penetration=env.config.task.max_contact_penetration_m,
            )
            _obs, info = env.reset(seed=episode_seed, randomization=sample)
            for step_index in range(steps):
                action = rng.uniform(-action_magnitude, action_magnitude, size=7)
                _obs, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                stage_counts[info["stage"]] += 1
                episode_max_force = max(
                    episode_max_force, float(info["max_cube_contact_force_n"])
                )
                episode_max_penetration = max(
                    episode_max_penetration,
                    float(info["max_contact_penetration_m"]),
                )
                recorder.append(
                    env.sample_from_last_step(
                        action, episode_id=episode_id, step_index=step_index
                    )
                )
                if terminated or truncated:
                    break
        safety = validate_trajectory(recorder.samples, limits)
        if info["failure_reason"] != "none":
            failure_counts[info["failure_reason"]] += 1
        if log_dir is not None:
            recorder.to_jsonl(Path(log_dir) / f"{episode_id}.jsonl")
        results.append(
            {
                "ok": safety["ok"] and math.isfinite(total_reward),
                "episode": episode,
                "seed": episode_seed,
                "steps": int(info["step_count"]),
                "stage": info["stage"],
                "is_success": bool(info["is_success"]),
                "failure_reason": info["failure_reason"],
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "total_reward": total_reward,
                "stage_counts": dict(sorted(stage_counts.items())),
                "lift_height_m": float(info["lift_height_m"]),
                "bilateral_finger_contact": bool(info["bilateral_finger_contact"]),
                "force_closure_candidate": bool(info["force_closure_candidate"]),
                "finger_normal_dot": float(info["finger_normal_dot"]),
                "max_cube_contact_force_n": episode_max_force,
                "max_contact_penetration_m": episode_max_penetration,
                "randomization": asdict(sample),
                "safety": safety,
            }
        )
    success_count = sum(result["is_success"] for result in results)
    return {
        "ok": all(result["ok"] for result in results),
        "task": "Pick",
        "simulator": "MuJoCo",
        "policy": "bounded_random_acceptance",
        "episodes": episodes,
        "requested_steps": steps,
        "seed": seed,
        "randomization_profile": profile,
        "action_magnitude": action_magnitude,
        "success_count": success_count,
        "success_rate": success_count / episodes,
        "failure_counts": dict(sorted(failure_counts.items())),
        "results": results,
    }


def main(
    argv: Sequence[str] | None = None,
    *,
    env_factory: Callable = RebotArmPickEnv,
    stdout=None,
    stderr=None,
) -> int:
    args = build_parser().parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        payload = run_pick_batch(
            model=args.model,
            episodes=args.episodes,
            steps=args.steps,
            seed=args.seed,
            action_magnitude=args.action_magnitude,
            config_path=args.randomization_config,
            profile=args.randomization_profile,
            log_dir=args.log_dir,
            env_factory=env_factory,
        )
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
