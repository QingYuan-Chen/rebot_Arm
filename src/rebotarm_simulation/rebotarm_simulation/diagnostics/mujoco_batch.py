from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_env import RebotArmReachEnv


def _positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def _non_negative_int(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run headless MuJoCo Reach rollouts")
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--episodes", type=_positive_int, default=3)
    parser.add_argument("--steps", type=_positive_int, default=100)
    parser.add_argument("--seed", type=_non_negative_int, default=0)
    return parser


def run_batch(
    *,
    model: str | None,
    episodes: int,
    steps: int,
    seed: int,
    env_factory: Callable = RebotArmReachEnv,
) -> dict:
    rng = np.random.default_rng(seed)
    results = []
    with env_factory(model) as env:
        for episode in range(episodes):
            obs, info = env.reset(seed=seed + episode)
            total_reward = 0.0
            terminated = truncated = False
            for _ in range(steps):
                action = rng.uniform(-1.0, 1.0, size=7)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += float(reward)
                if terminated or truncated:
                    break
            results.append({
                "episode": episode,
                "steps": int(info["step_count"]),
                "total_reward": total_reward,
                "distance_to_target_m": float(info["distance_to_target_m"]),
                "is_success": bool(info["is_success"]),
                "terminated": bool(terminated),
                "truncated": bool(truncated),
            })
    distances = [item["distance_to_target_m"] for item in results]
    return {
        "ok": all(math.isfinite(value) for value in distances),
        "episodes": episodes,
        "requested_steps": steps,
        "seed": seed,
        "mean_final_distance_m": float(np.mean(distances)) if distances else 0.0,
        "success_count": sum(1 for item in results if item["is_success"]),
        "results": results,
    }


def main(
    argv: Sequence[str] | None = None,
    *,
    env_factory: Callable = RebotArmReachEnv,
    stdout=None,
    stderr=None,
) -> int:
    args = build_parser().parse_args(argv)
    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    try:
        payload = run_batch(
            model=args.model,
            episodes=args.episodes,
            steps=args.steps,
            seed=args.seed,
            env_factory=env_factory,
        )
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
