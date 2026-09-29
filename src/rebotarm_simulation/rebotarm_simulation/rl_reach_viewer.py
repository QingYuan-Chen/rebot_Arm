"""Watch a saved Reach policy in the native MuJoCo Viewer (simulation only)."""
import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from .gym_reach import RebotArmReachEnv, _rotate_points
from .mujoco_viewer import _close_viewer_then_sim


def _draw_goal(viewer, env):
    """Overlay target TCP and XYZ axes without adding physical geometry."""
    with viewer.lock():
        scene = viewer.user_scn
        scene.ngeom = 0
        geom = scene.geoms[0]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.full(3, 0.004), env._goal, np.eye(3).ravel(),
                           np.array((1, 0.8, 0, 0.8), dtype=np.float32))
        scene.ngeom = 1
        axes = _rotate_points(env._goal_orientation, np.eye(3) * 0.04)
        for axis, color in zip(axes, ((1, 0, 0, 1), (0, 1, 0, 1), (0, 0.4, 1, 1))):
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                               np.zeros(3), env._goal, np.eye(3).ravel(),
                               np.asarray(color, dtype=np.float32))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3,
                                env._goal, env._goal + axis)
            scene.ngeom += 1


def _wait(viewer, seconds):
    deadline = time.monotonic() + seconds
    while viewer.is_running() and time.monotonic() < deadline:
        viewer.sync()
        time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--curriculum-stage", choices=("fixed", "local"), default="fixed")
    parser.add_argument("--episodes", type=int, default=5, help="Number of episodes to watch")
    parser.add_argument("--seed", type=int, default=20000)
    parser.add_argument("--speed", type=float, default=0.25, help="Playback speed: 1=real time, 0.25=quarter speed")
    parser.add_argument("--pause", type=float, default=1.0, help="Seconds to show the start and end of each episode")
    args = parser.parse_args()
    if args.episodes < 1 or args.seed < 0 or not np.isfinite(args.speed) or args.speed <= 0 or not np.isfinite(args.pause) or args.pause < 0:
        parser.error("episodes/speed must be positive; seed/pause must be nonnegative and speed/pause finite")
    import torch
    from stable_baselines3 import PPO
    import mujoco.viewer
    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch CUDA GPU is unavailable")
    policy = PPO.load(args.model, device="cuda")
    env = RebotArmReachEnv(curriculum_stage=args.curriculum_stage)
    viewer = None
    model, data = env.sim._unsafe_viewer_handles()
    records = []
    try:
        obs, _ = env.reset(seed=args.seed)
        viewer = mujoco.viewer.launch_passive(model, data)
        with viewer.lock():
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = 0.8
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -22
            viewer.opt.geomgroup[3] = 0
            viewer.opt.sitegroup[:] = 0
        for episode in range(args.episodes):
            if not viewer.is_running():
                break
            obs, _ = env.reset(seed=args.seed + episode)
            _draw_goal(viewer, env)
            _wait(viewer, args.pause)
            steps = 0
            while viewer.is_running():
                started = time.monotonic()
                action, _ = policy.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                steps += 1
                viewer.sync()
                _wait(viewer, max(0.0, env.frame_skip * env.sim.timestep / args.speed - (time.monotonic() - started)))
                if terminated or truncated:
                    record = {"episode": episode + 1, "is_success": info["is_success"],
                              "steps": steps, "distance_mm": info["distance_m"] * 1000,
                              "orientation_error_deg": float(np.rad2deg(info["orientation_error_rad"]))}
                    records.append(record)
                    print(json.dumps(record), flush=True)
                    _wait(viewer, args.pause)
                    break
        print(json.dumps({"completed_episodes": len(records),
                          "success_rate": float(np.mean([r["is_success"] for r in records])) if records else None}))
    except KeyboardInterrupt:
        pass
    finally:
        _close_viewer_then_sim(viewer, env.sim, model, data)


if __name__ == "__main__":
    main()
