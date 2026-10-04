"""Offline CPU MuJoCo / mjlab Warp evaluation of the same Reach checkpoint.

Uses mjlab's compiled model for the CPU reference (including scene attachment
and solver settings). Never imports ROS or hardware drivers. This evaluates the
current task observation encoding; it does not certify sim-to-real readiness.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import torch
from tensordict import TensorDict

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
from rebotarm_rl.tasks import reach  # Registers task; no hardware imports.


def pose(model, data, site):
    mujoco.mj_forward(model, data)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site])
    return data.site_xpos[site].copy(), quat


def errors(pos, quat, target, target_quat):
    cosine = np.clip(abs(np.dot(quat, target_quat)), 0.0, 1.0)
    return float(np.linalg.norm(pos - target)), float(2 * np.arccos(cosine))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episodes', type=int, default=10)
    parser.add_argument('--steps', type=int, default=250)
    parser.add_argument('--seed', type=int, default=20000)
    args = parser.parse_args()
    if args.episodes < 1 or args.steps < 1:
        parser.error('episodes and steps must be positive')
    checkpoint = args.checkpoint.resolve(strict=True)
    task = 'RebotArm-Reach-Mjlab'
    cfg = load_env_cfg(task, play=False)
    cfg.scene.num_envs = 1
    cfg.seed = args.seed
    # Fixed length trajectory comparison: suppress automatic reset on success
    # and timeout; compute first-success and final-success ourselves.
    cfg.terminations = {}
    cfg.commands['reach'].position_radius = 0.0
    cfg.commands['reach'].resampling_time_range = (1e9, 1e9)
    rl_cfg = load_rl_cfg(task)
    env = RslRlVecEnvWrapper(ManagerBasedRlEnv(cfg, device='cuda:0'),
                            clip_actions=rl_cfg.clip_actions)
    try:
        runner = MjlabOnPolicyRunner(env, asdict(rl_cfg), device='cuda:0')
        runner.load(str(checkpoint), load_cfg={'actor': True}, strict=True,
                    map_location='cuda:0')
        policy = runner.get_inference_policy(device='cuda:0')
        base = env.unwrapped
        robot = base.scene['robot']
        command = base.command_manager.get_term('reach')
        # Exactly the compiled training model, run through CPU mj_step.
        model = base.sim.mj_model
        data = mujoco.MjData(model)
        site = int(robot.indexing.site_ids[command.site_id])
        qadr = robot.indexing.joint_q_adr.cpu().numpy()
        vadr = robot.indexing.joint_v_adr.cpu().numpy()
        actuator_ids = [i for i in range(model.nu)
                        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                            or '').split('/')[-1] in [f'joint{j}_torque' for j in range(1, 7)]]
        if len(actuator_ids) != 6:
            raise ValueError('Expected six ordered torque actuators')
        default_q = robot.data.default_joint_pos[0].cpu().numpy()
        rng = np.random.default_rng(args.seed)
        rows = []
        max_initial_obs_delta = 0.0
        for episode in range(args.episodes):
            env.reset()
            data.qpos[qadr] = robot.data.joint_pos[0].cpu().numpy()
            data.qvel[vadr] = robot.data.joint_vel[0].cpu().numpy()
            home_pos, home_quat = pose(model, data, site)
            direction = rng.normal(size=3)
            target = home_pos + direction / np.linalg.norm(direction) * rng.uniform(0, .06)
            target_quat = home_quat.copy()
            command.target_pos[:] = torch.as_tensor(target, device='cuda:0', dtype=torch.float32)
            command.target_quat[:] = torch.as_tensor(target_quat, device='cuda:0', dtype=torch.float32)
            results = {'cpu': [], 'gpu': []}
            q_deltas = []
            for step in range(args.steps + 1):
                cpos, cquat = pose(model, data, site)
                gpose = robot.data.site_pose_w[0, command.site_id].cpu().numpy()
                for backend, p, q in [('cpu', cpos, cquat), ('gpu', gpose[:3], gpose[3:])]:
                    pe, oe = errors(p, q, target, target_quat)
                    results[backend].append([pe, oe, bool(pe < .01 and oe < .05236)])
                q_deltas.append(float(np.max(np.abs(data.qpos[qadr] - robot.data.joint_pos[0].cpu().numpy()))))
                if step == args.steps:
                    break
                # Match the current task's exact public observation encoding.
                corient = 2 * (cquat[:3] * target_quat[3] - target_quat[:3] * cquat[3])
                cpu_obs = np.concatenate([data.qpos[qadr[:6]] - default_q[:6],
                                          data.qvel[vadr[:6]], cpos - target, corient])
                gpu_obs = base.observation_manager.compute_group('actor')
                if step == 0:
                    delta = float(np.max(np.abs(cpu_obs - gpu_obs[0].cpu().numpy())))
                    max_initial_obs_delta = max(max_initial_obs_delta, delta)
                    if delta > 1e-4:
                        raise ValueError(f'CPU/GPU initial observation mismatch: {delta}')
                with torch.inference_mode():
                    ca = policy(TensorDict({'actor': torch.tensor(cpu_obs[None], dtype=torch.float32,
                                                                 device='cuda:0')}, batch_size=[1]))
                    ga = policy(TensorDict({'actor': gpu_obs}, batch_size=[1]))
                if rl_cfg.clip_actions is not None:
                    ca = ca.clamp(-rl_cfg.clip_actions, rl_cfg.clip_actions)
                data.ctrl[:] = 0
                data.ctrl[actuator_ids] = ca[0].cpu().numpy()
                for _ in range(cfg.decimation):
                    mujoco.mj_step(model, data)
                env.step(ga)
                if not np.isfinite(data.qpos).all() or not torch.isfinite(robot.data.joint_pos).all():
                    raise ValueError('Non-finite trajectory')
            row = {'episode': episode, 'target_position_m': target.tolist(),
                   'target_quaternion_wxyz': target_quat.tolist(),
                   'max_joint_trajectory_delta_rad': max(q_deltas)}
            for backend in results:
                trajectory = results[backend]
                row[backend] = {'final_position_error_m': trajectory[-1][0],
                                'final_orientation_error_rad': trajectory[-1][1],
                                'final_success': trajectory[-1][2],
                                'first_success_step': next((i for i, v in enumerate(trajectory) if v[2]), None),
                                'error_trajectory': trajectory}
            rows.append(row)
        summary = {}
        for backend in ('cpu', 'gpu'):
            summary[backend] = {
                'final_success_count': sum(r[backend]['final_success'] for r in rows),
                'mean_final_position_error_m': float(np.mean([r[backend]['final_position_error_m'] for r in rows])),
                'mean_final_orientation_error_rad': float(np.mean([r[backend]['final_orientation_error_rad'] for r in rows]))}
        report = {'checkpoint': str(checkpoint), 'checkpoint_sha256': hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                  'seed': args.seed, 'episodes': args.episodes, 'steps': args.steps,
                  'control_dt_s': cfg.decimation * model.opt.timestep,
                  'integrator': int(model.opt.integrator), 'max_initial_obs_delta': max_initial_obs_delta,
                  'scope': 'Closed-loop deterministic smoke checkpoint evaluation; not convergence or hardware acceptance',
                  'summary': summary, 'results': rows}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(summary, indent=2))
        print(f'Report: {args.output}')
    finally:
        env.close()


if __name__ == '__main__':
    main()
