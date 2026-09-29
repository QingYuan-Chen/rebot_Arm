"""Batched GPU MJX Reach training and evaluation (simulation only)."""
import argparse
import json
import os
from pathlib import Path

import numpy as np
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
import jax
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

from .mjx_reach import MJXReachVecEnv


def evaluate(model, stage, episodes, seed, num_envs, env=None):
    own_env = env is None
    if own_env:
        env = MJXReachVecEnv(min(num_envs, episodes), curriculum_stage=stage, seed=seed)
    try:
        env._initial_key = jax.random.PRNGKey(seed)
        obs = env.reset()
        # Assign a fixed number of episodes to each lane before stepping.
        # Taking the first N completed episodes would favor easier/shorter goals.
        quotas = np.full(env.num_envs, episodes // env.num_envs, dtype=np.int32)
        quotas[:episodes % env.num_envs] += 1
        counts = np.zeros(env.num_envs, dtype=np.int32)
        records = []
        while np.any(counts < quotas):
            action, _ = model.predict(obs, deterministic=True)
            obs, _, dones, infos = env.step(action)
            for index, (done, info) in enumerate(zip(dones, infos)):
                if done and counts[index] < quotas[index]:
                    records.append(info)
                    counts[index] += 1
        return {
            "episodes": len(records),
            "success_rate": float(np.mean([r["is_success"] for r in records])),
            "mean_final_distance_m": float(np.mean([r["distance_m"] for r in records])),
            "mean_final_orientation_error_deg": float(np.rad2deg(np.mean([
                r["orientation_error_rad"] for r in records]))),
        }
    finally:
        if own_env:
            env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "train", "eval"))
    parser.add_argument("--curriculum-stage", choices=("fixed", "local_easy", "local"), default="fixed")
    parser.add_argument("--reward-profile", choices=("baseline", "pose_v2", "pose_v3", "adaptive_position"), default="baseline")
    parser.add_argument("--num-envs", type=int, default=128)
    parser.add_argument("--steps", type=int, default=500000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--eval-freq", type=int, default=25000)
    parser.add_argument("--model", type=Path, default=Path("runs/reach/mjx_fixed_seed7.zip"))
    parser.add_argument("--init-model", type=Path,
                        help="Initialize a new training run from an existing PPO model")
    args = parser.parse_args()
    if args.num_envs < 1 or args.steps < 1 or args.episodes < 1 or args.eval_freq < 1:
        parser.error("num-envs, steps, episodes and eval-freq must be positive")
    if args.model.suffix != ".zip":
        parser.error("--model must end in .zip")
    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch CUDA GPU is unavailable")
    if args.mode == "check":
        env = MJXReachVecEnv(args.num_envs, curriculum_stage=args.curriculum_stage, seed=args.seed)
        try:
            obs = env.reset()
            next_obs, reward, done, _ = env.step(np.zeros((args.num_envs, 6), np.float32))
            print(json.dumps({"gpu": True, "num_envs": args.num_envs,
                              "observation_shape": list(obs.shape),
                              "next_shape": list(next_obs.shape),
                              "finite": bool(np.all(np.isfinite(next_obs)) and np.all(np.isfinite(reward))),
                              "done_count": int(done.sum())}))
        finally:
            env.close()
        return
    if args.mode == "eval":
        model = PPO.load(args.model, device="cuda")
        print(json.dumps(evaluate(model, args.curriculum_stage, args.episodes, args.seed,
                                  args.num_envs), indent=2))
        return
    checkpoint_dir = args.model.with_name(args.model.stem + "_checkpoints")
    best_path = args.model.with_name(args.model.stem + "_best.zip")
    if args.model.exists() or checkpoint_dir.exists() or best_path.exists():
        raise FileExistsError(f"Choose a new --model path: {args.model}")
    args.model.parent.mkdir(parents=True, exist_ok=True)
    checkpoint_dir.mkdir()
    env = MJXReachVecEnv(args.num_envs, curriculum_stage=args.curriculum_stage, seed=args.seed,
                         reward_profile=args.reward_profile)
    eval_env = MJXReachVecEnv(min(args.num_envs, 20), curriculum_stage=args.curriculum_stage, seed=10000,
                              reward_profile=args.reward_profile)
    class EvalAndSave(BaseCallback):
        def __init__(self):
            super().__init__()
            self.next_eval = args.eval_freq
            self.best = (-1.0, -float("inf"))
        def _on_step(self):
            if self.num_timesteps < self.next_eval:
                return True
            self.next_eval += args.eval_freq
            self.model.save(checkpoint_dir / f"step_{self.num_timesteps}.zip")
            result = evaluate(self.model, args.curriculum_stage, 20, 10000, args.num_envs, eval_env)
            score = (result["success_rate"], -result["mean_final_distance_m"])
            if score > self.best:
                self.best = score
                self.model.save(best_path)
            print(json.dumps({"step": self.num_timesteps, **result, "best_model": str(best_path)}), flush=True)
            return True
    try:
        if args.init_model:
            model = PPO.load(args.init_model, env=env, device="cuda", verbose=1)
            model.set_random_seed(args.seed)
        else:
            model = PPO("MlpPolicy", env, n_steps=256, batch_size=256, seed=args.seed,
                        device="cuda", verbose=1)
        model.learn(total_timesteps=args.steps, callback=EvalAndSave())
        model.save(args.model)
        if not best_path.exists():
            model.save(best_path)
        print(json.dumps({"model": str(args.model), "best_model": str(best_path),
                          "timesteps": model.num_timesteps}))
    finally:
        env.close()
        eval_env.close()


if __name__ == "__main__":
    main()
