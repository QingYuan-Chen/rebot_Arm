"""Reach API 检查、CUDA PPO 训练和独立种子评估入口。"""
import argparse
import json
from pathlib import Path

import numpy as np
from gymnasium.utils.env_checker import check_env

from rebotarm_simulation.core.gym_reach import RebotArmReachEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("check", "train", "eval"))
    parser.add_argument("--steps", type=int, default=100000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--model", type=Path, default=Path("runs/reach/ppo.zip"))
    parser.add_argument("--curriculum-stage", choices=("fixed", "local"), default="local")
    parser.add_argument("--eval-freq", type=int, default=25000,
                        help="Training steps between checkpoints and independent evaluations")
    parser.add_argument("--eval-episodes", type=int, default=20)
    args = parser.parse_args()
    if args.steps < 1 or args.episodes < 1 or args.seed < 0 or args.eval_freq < 1 or args.eval_episodes < 1:
        parser.error("steps/episodes/eval-freq/eval-episodes must be positive; seed must be nonnegative")
    if args.model.suffix != ".zip":
        parser.error("--model must end in .zip")
    env = RebotArmReachEnv(curriculum_stage=args.curriculum_stage)
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
            checkpoint_dir = args.model.with_suffix("").with_name(args.model.stem + "_checkpoints")
            best_path = args.model.with_name(args.model.stem + "_best.zip")
            if args.model.exists() or best_path.exists() or checkpoint_dir.exists():
                raise FileExistsError(f"Choose a new --model path: {args.model}")
            args.model.parent.mkdir(parents=True, exist_ok=True)
            checkpoint_dir.mkdir()
            from stable_baselines3.common.callbacks import BaseCallback

            class ReachEvaluationCallback(BaseCallback):
                def __init__(self):
                    super().__init__()
                    self.best_score = (-1.0, -float("inf"))
                    self.eval_env = RebotArmReachEnv(curriculum_stage=args.curriculum_stage)

                def _on_step(self) -> bool:
                    if self.num_timesteps % args.eval_freq:
                        return True
                    self.model.save(checkpoint_dir / f"step_{self.num_timesteps}.zip")
                    records = []
                    for episode in range(args.eval_episodes):
                        obs, _ = self.eval_env.reset(seed=10000 + episode)
                        while True:
                            action, _ = self.model.predict(obs, deterministic=True)
                            obs, _, terminated, truncated, info = self.eval_env.step(action)
                            if terminated or truncated:
                                records.append(info)
                                break
                    success_rate = float(np.mean([r["is_success"] for r in records]))
                    mean_distance = float(np.mean([r["distance_m"] for r in records]))
                    score = (success_rate, -mean_distance)
                    if score > self.best_score:
                        self.best_score = score
                        self.model.save(best_path)
                    print(json.dumps({"eval_step": self.num_timesteps,
                                      "success_rate": success_rate,
                                      "mean_final_distance_m": mean_distance,
                                      "best_model": str(best_path)}, flush=True))
                    return True

                def _on_training_end(self) -> None:
                    self.eval_env.close()
                    if self.best_score[0] < 0.0:
                        self.model.save(best_path)

            model = PPO("MlpPolicy", Monitor(env), n_steps=256, batch_size=64,
                        seed=args.seed, device="cuda", verbose=1)
            model.learn(total_timesteps=args.steps, callback=ReachEvaluationCallback())
            model.save(args.model)
            print(json.dumps({"model": str(args.model), "best_model": str(best_path),
                              "timesteps": model.num_timesteps}))
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
                              "mean_final_orientation_error_deg": float(np.rad2deg(np.mean([
                                  r["orientation_error_rad"] for r in records
                              ]))),
                              "seed": args.seed}, indent=2))
    finally:
        env.close()


if __name__ == "__main__":
    main()
