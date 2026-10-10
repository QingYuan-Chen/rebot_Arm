from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_env import RebotArmReachEnv, ReachEnvConfig
from rebotarm_simulation.sim2real import (
    ComparisonThresholds,
    RandomizationConfig,
    TrajectoryRecorder,
    compare_trajectories,
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


def _non_negative_float(value: str) -> float:
    result = float(value)
    if math.isnan(result) or result < 0.0:
        raise argparse.ArgumentTypeError("must be non-negative")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run simulation-only reBotArm Sim2Real workflows"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    rollout = subparsers.add_parser("rollout", help="run and record one randomized rollout")
    _add_environment_arguments(rollout)
    rollout.add_argument("--steps", type=_positive_int, default=100)
    rollout.add_argument("--record", required=True, help="output trajectory JSONL")
    rollout.add_argument("--report", help="optional JSON summary path")

    replay = subparsers.add_parser("replay", help="replay actions from a trajectory log")
    _add_environment_arguments(replay)
    replay.add_argument("input", help="reference trajectory JSONL")
    replay.add_argument("--record", required=True, help="replayed trajectory JSONL")
    replay.add_argument("--report", help="optional comparison JSON path")
    _add_comparison_thresholds(replay)

    compare = subparsers.add_parser("compare", help="compare two trajectory logs")
    compare.add_argument("reference")
    compare.add_argument("candidate")
    compare.add_argument("--report", help="optional comparison JSON path")
    _add_comparison_thresholds(compare)

    batch = subparsers.add_parser(
        "batch-check", help="run randomized safety and seed reproducibility checks"
    )
    _add_environment_arguments(batch)
    batch.add_argument("--episodes", type=_positive_int, default=3)
    batch.add_argument("--steps", type=_positive_int, default=100)
    batch.add_argument("--log-dir", help="optional directory for reference/replay JSONL logs")
    batch.add_argument("--report", help="optional batch JSON report path")
    return parser


def _add_environment_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    parser.add_argument("--randomization-config", default=None, help="randomization YAML path")
    parser.add_argument("--randomization-profile", default="training_profile")
    parser.add_argument("--seed", type=_non_negative_int, default=0)
    parser.add_argument(
        "--max-contact-force", type=_non_negative_float, default=math.inf
    )
    parser.add_argument(
        "--max-contact-penetration", type=_non_negative_float, default=0.01
    )


def _add_comparison_thresholds(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--joint-position-max", type=_non_negative_float, default=math.inf)
    parser.add_argument("--joint-velocity-max", type=_non_negative_float, default=math.inf)
    parser.add_argument("--ee-position-max", type=_non_negative_float, default=math.inf)
    parser.add_argument("--gripper-width-max", type=_non_negative_float, default=math.inf)
    parser.add_argument("--actuator-torque-max", type=_non_negative_float, default=math.inf)
    parser.add_argument("--contact-force-max", type=_non_negative_float, default=math.inf)


def run_rollout(
    *,
    model: str | None,
    steps: int,
    seed: int,
    config_path: str | None,
    profile: str,
    record_path: str | Path,
    max_contact_force: float,
    max_contact_penetration: float,
    env_factory: Callable = RebotArmReachEnv,
) -> dict:
    config = RandomizationConfig.from_yaml(
        config_path or default_randomization_config_path(), profile=profile
    )
    randomization = config.sample(seed)
    rng = np.random.default_rng(seed)
    actions = rng.uniform(-1.0, 1.0, size=(steps, 7))
    recorder, safety = _run_actions(
        model=model,
        actions=actions,
        seed=seed,
        randomization=randomization,
        episode_id=f"sim-seed-{seed}",
        max_contact_force=max_contact_force,
        max_contact_penetration=max_contact_penetration,
        env_factory=env_factory,
    )
    recorder.to_jsonl(record_path)
    return {
        "ok": safety["ok"],
        "command": "rollout",
        "record": str(record_path),
        "profile": profile,
        "seed": seed,
        "randomization": asdict(randomization),
        "trajectory": recorder.summary(),
        "safety": safety,
    }


def run_replay(
    *,
    input_path: str | Path,
    record_path: str | Path,
    model: str | None,
    seed: int,
    config_path: str | None,
    profile: str,
    thresholds: ComparisonThresholds,
    max_contact_force: float,
    max_contact_penetration: float,
    env_factory: Callable = RebotArmReachEnv,
) -> dict:
    reference = TrajectoryRecorder.from_jsonl(input_path)
    randomization = RandomizationConfig.from_yaml(
        config_path or default_randomization_config_path(), profile=profile
    ).sample(seed)
    candidate, safety = _run_actions(
        model=model,
        actions=[sample.action for sample in reference.samples],
        seed=seed,
        randomization=randomization,
        episode_id=reference.episode_id,
        max_contact_force=max_contact_force,
        max_contact_penetration=max_contact_penetration,
        env_factory=env_factory,
    )
    candidate.to_jsonl(record_path)
    comparison = compare_trajectories(
        reference.samples, candidate.samples, thresholds=thresholds
    ).to_dict()
    return {
        "ok": safety["ok"] and comparison["ok"],
        "command": "replay",
        "input": str(input_path),
        "record": str(record_path),
        "profile": profile,
        "seed": seed,
        "trajectory": candidate.summary(),
        "safety": safety,
        "comparison": comparison,
    }


def run_compare(
    reference_path: str | Path,
    candidate_path: str | Path,
    *,
    thresholds: ComparisonThresholds,
) -> dict:
    reference = TrajectoryRecorder.from_jsonl(reference_path)
    candidate = TrajectoryRecorder.from_jsonl(candidate_path)
    report = compare_trajectories(
        reference.samples, candidate.samples, thresholds=thresholds
    ).to_dict()
    return {
        "command": "compare",
        "reference": str(reference_path),
        "candidate": str(candidate_path),
        **report,
    }


def run_batch_check(
    *,
    model: str | None,
    episodes: int,
    steps: int,
    seed: int,
    config_path: str | None,
    profile: str,
    log_dir: str | Path | None,
    max_contact_force: float,
    max_contact_penetration: float,
    env_factory: Callable = RebotArmReachEnv,
) -> dict:
    config = RandomizationConfig.from_yaml(
        config_path or default_randomization_config_path(), profile=profile
    )
    results = []
    for episode in range(episodes):
        episode_seed = seed + episode
        randomization = config.sample(episode_seed)
        actions = np.random.default_rng(episode_seed).uniform(
            -1.0, 1.0, size=(steps, 7)
        )
        episode_id = f"sim-seed-{episode_seed}"
        first, safety = _run_actions(
            model=model,
            actions=actions,
            seed=episode_seed,
            randomization=randomization,
            episode_id=episode_id,
            max_contact_force=max_contact_force,
            max_contact_penetration=max_contact_penetration,
            env_factory=env_factory,
        )
        second, replay_safety = _run_actions(
            model=model,
            actions=actions,
            seed=episode_seed,
            randomization=randomization,
            episode_id=episode_id,
            max_contact_force=max_contact_force,
            max_contact_penetration=max_contact_penetration,
            env_factory=env_factory,
        )
        reproducible = first.samples == second.samples
        if log_dir is not None:
            directory = Path(log_dir)
            first.to_jsonl(directory / f"seed-{episode_seed}-reference.jsonl")
            second.to_jsonl(directory / f"seed-{episode_seed}-replay.jsonl")
        results.append(
            {
                "ok": safety["ok"] and replay_safety["ok"] and reproducible,
                "seed": episode_seed,
                "randomization": asdict(randomization),
                "sample_count": len(first),
                "safety": safety,
                "replay_safety": replay_safety,
                "seed_reproducible": reproducible,
            }
        )
    return {
        "ok": all(result["ok"] for result in results),
        "command": "batch-check",
        "episodes": episodes,
        "requested_steps": steps,
        "profile": profile,
        "seed": seed,
        "results": results,
    }


def _run_actions(
    *,
    model: str | None,
    actions,
    seed: int,
    randomization,
    episode_id: str,
    max_contact_force: float,
    max_contact_penetration: float,
    env_factory: Callable,
) -> tuple[TrajectoryRecorder, dict]:
    recorder = TrajectoryRecorder(episode_id=episode_id, source="sim")
    with env_factory(model) as env:
        limits = safety_limits_from_env(
            env,
            max_contact_force=max_contact_force,
            max_contact_penetration=max_contact_penetration,
        )
        env.reset(seed=seed, randomization=randomization)
        for step_index, action in enumerate(actions):
            _obs, _reward, terminated, truncated, _info = env.step(action)
            recorder.append(
                env.sample_from_last_step(
                    action, episode_id=episode_id, step_index=step_index
                )
            )
            if terminated or truncated:
                break
    return recorder, validate_trajectory(recorder.samples, limits)


def _thresholds(args) -> ComparisonThresholds:
    return ComparisonThresholds(
        joint_position_max=args.joint_position_max,
        joint_velocity_max=args.joint_velocity_max,
        end_effector_position_max=args.ee_position_max,
        gripper_width_max=args.gripper_width_max,
        actuator_torque_max=args.actuator_torque_max,
        contact_force_max=args.contact_force_max,
    )


def _write_report(path: str | None, payload: dict) -> None:
    if path is None:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main(
    argv: Sequence[str] | None = None,
    *,
    env_factory: Callable = RebotArmReachEnv,
    stdout=None,
    stderr=None,
) -> int:
    args = build_parser().parse_args(argv)
    stdout = stdout or sys.stdout
    stderr = stderr or sys.stderr
    try:
        if args.command == "rollout":
            payload = run_rollout(
                model=args.model,
                steps=args.steps,
                seed=args.seed,
                config_path=args.randomization_config,
                profile=args.randomization_profile,
                record_path=args.record,
                max_contact_force=args.max_contact_force,
                max_contact_penetration=args.max_contact_penetration,
                env_factory=env_factory,
            )
        elif args.command == "replay":
            payload = run_replay(
                input_path=args.input,
                record_path=args.record,
                model=args.model,
                seed=args.seed,
                config_path=args.randomization_config,
                profile=args.randomization_profile,
                thresholds=_thresholds(args),
                max_contact_force=args.max_contact_force,
                max_contact_penetration=args.max_contact_penetration,
                env_factory=env_factory,
            )
        elif args.command == "compare":
            payload = run_compare(
                args.reference, args.candidate, thresholds=_thresholds(args)
            )
        else:
            payload = run_batch_check(
                model=args.model,
                episodes=args.episodes,
                steps=args.steps,
                seed=args.seed,
                config_path=args.randomization_config,
                profile=args.randomization_profile,
                log_dir=args.log_dir,
                max_contact_force=args.max_contact_force,
                max_contact_penetration=args.max_contact_penetration,
                env_factory=env_factory,
            )
        _write_report(args.report, payload)
    except Exception as exc:
        print(f"error: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
