"""Reach API 检查、CUDA PPO 训练和独立种子评估入口。"""
import argparse
import json
from pathlib import Path

import numpy as np
from gymnasium.utils.env_checker import check_env

from .gym_reach import RebotArmReachEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "train", "eval"))
    parser.add_argument("--steps", type=int, default=100000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--model", type=Path, default=Path("runs/reach/ppo.zip"))
    args = parser.parse_args()
    if args.steps < 1 or args.episodes < 1 or args.seed < 0:
        parser.error("steps/episodes must be positive; seed must be nonnegative")
    if args.model.suffix != ".zip":
        parser.error("--model must end in .zip")
    env = RebotArmReachEnv()
    try:
        if args.mode == "check":
            from stable_baselines3.common.env_checker import check_env as sb3_check
            check_env(env, skip_render_check=True)
            sb3_check(env)
            print(json.dumps({"gymnasium_check": True, "sb3_check": True}))
            return
        import torch
        from stable_baselines3 import PPO
        from stable_baselines3.common.monitor import Monitor
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA GPU is unavailable; check NVIDIA driver and CUDA PyTorch wheel")
        torch.empty(1, device="cuda")  # fail before training if CUDA context cannot start
        if args.mode == "train":
            if args.model.exists():
                raise FileExistsError(f"Choose a new --model path: {args.model}")
            args.model.parent.mkdir(parents=True, exist_ok=True)
            model = PPO("MlpPolicy", Monitor(env), n_steps=256, batch_size=64,
                        seed=args.seed, device="cuda", verbose=1)
            model.learn(total_timesteps=args.steps)
            model.save(args.model)
            print(json.dumps({"model": str(args.model), "timesteps": model.num_timesteps}))
        else:
            model = PPO.load(args.model, device="cuda")
            records = []
            for episode in range(args.episodes):
                obs, _ = env.reset(seed=args.seed + episode)
                while True:
                    action, _ = model.predict(obs, deterministic=True)
                    obs, _, terminated, truncated, info = env.step(action)
                    if terminated or truncated:
                        records.append(info)
                        break
            print(json.dumps({"episodes": len(records),
                              "success_rate": float(np.mean([r["is_success"] for r in records])),
                              "collision_rate": float(np.mean([r["collision"] for r in records])),
                              "mean_final_distance_m": float(np.mean([r["distance_m"] for r in records])),
                              "seed": args.seed}, indent=2))
    finally:
        env.close()


if __name__ == "__main__":
    main()
