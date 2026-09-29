"""Batched GPU MJX Reach physics for Stable-Baselines3.

The Reach scene has only the arm and gripper. All contacts are disabled in a
private MjModel copy; this is a contact-free pose task, not the grasp/ROS backend.
"""
from __future__ import annotations

import os
from typing import NamedTuple

import mujoco
import numpy as np
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
import jax
import jax.numpy as jnp
from jax import lax
from mujoco import mjx
from gymnasium import spaces
from stable_baselines3.common.vec_env import VecEnv

from .motor_control import load_motor_control_parameters
from .mujoco_sim import _default_scene_path


class ReachState(NamedTuple):
    data: object
    targets: object
    goal_pos: object
    goal_mat: object
    initial_distance: object
    pos_integral: object
    vel_integral: object
    applied: object
    previous_action: object
    previous_error: object
    hold_steps: object
    steps: object
    key: object


def _orientation_vector(current, target):
    relative = target @ current.T
    cos_angle = jnp.clip((jnp.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    angle = jnp.arccos(cos_angle)
    skew = jnp.array((relative[2, 1] - relative[1, 2],
                      relative[0, 2] - relative[2, 0],
                      relative[1, 0] - relative[0, 1]))
    return jnp.where(angle < 1e-5, skew * 0.5,
                     skew * angle / jnp.maximum(2.0 * jnp.sin(angle), 1e-6))


class MJXReachVecEnv(VecEnv):
    """GPU-batched Reach environment with the same 24-D observation and action contract."""

    def __init__(self, num_envs=64, *, curriculum_stage="fixed", seed=7, max_episode_steps=250,
                 reward_profile="baseline"):
        if not 1 <= num_envs <= 1024:
            raise ValueError("num_envs must be in [1, 1024]")
        if curriculum_stage not in ("fixed", "local_easy", "local"):
            raise ValueError("curriculum_stage must be fixed, local_easy or local")
        if reward_profile not in ("baseline", "pose_v2", "pose_v3", "adaptive_position"):
            raise ValueError("reward_profile must be baseline, pose_v2, pose_v3 or adaptive_position")
        if not any(device.platform == "gpu" for device in jax.devices()):
            raise RuntimeError("MJX Reach requires a JAX CUDA GPU")
        self.curriculum_stage = curriculum_stage
        self.reward_profile = reward_profile
        self.max_episode_steps = max_episode_steps
        self.render_mode = None
        self._mj_model = mujoco.MjModel.from_xml_path(str(_default_scene_path().with_name("reach_scene.xml")))
        self._mj_model.geom_contype[:] = 0
        self._mj_model.geom_conaffinity[:] = 0
        self._mjx_model = mjx.put_model(self._mj_model)
        mj_data = mujoco.MjData(self._mj_model)
        home_id = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(self._mj_model, mj_data, home_id)
        mujoco.mj_forward(self._mj_model, mj_data)
        self._arm_qpos = np.array([self._mj_model.jnt_qposadr[mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)])
        self._arm_qvel = np.array([self._mj_model.jnt_dofadr[mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")] for i in range(1, 7)])
        self._actuators = np.array([mujoco.mj_name2id(
            self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"joint{i}_torque") for i in range(1, 7)])
        # Match RebotArmMujoco.reset_home(): seed the torque filter and ctrl
        # with the gravity/Coriolis bias before the first physical step.
        self._initial_torque = jnp.asarray(mj_data.qfrc_bias[self._arm_qvel] *
                                           load_motor_control_parameters().arm.gravity_compensation_scale)
        mj_data.ctrl[self._actuators] = np.asarray(self._initial_torque)
        self._home_data = mjx.put_data(self._mj_model, mj_data)
        self._home_qpos = jnp.asarray(mj_data.qpos.copy())
        self._site = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self._home_q = jnp.asarray(mj_data.qpos[self._arm_qpos].copy())
        self._home_pos = jnp.asarray(mj_data.site_xpos[self._site].copy())
        self._home_mat = jnp.asarray(mj_data.site_xmat[self._site].copy().reshape(3, 3))
        self._joint_lower = jnp.asarray(self._mj_model.jnt_range[:6, 0])
        self._joint_upper = jnp.asarray(self._mj_model.jnt_range[:6, 1])
        params = load_motor_control_parameters().arm
        self._p = {name: jnp.asarray(getattr(params, name)) for name in (
            "pos_kp", "pos_ki", "vel_kp", "vel_ki", "velocity_limit", "effort_limit",
            "firmware_to_torque_scale", "torque_rate_limit_nm_s", "torque_lowpass_alpha")}
        self._pos_limit = params.position_integral_limit
        self._vel_limit = params.velocity_integral_limit
        self._gravity_scale = params.gravity_compensation_scale
        self._dt = 0.01
        self._offsets = jnp.asarray(((0.0, 0.0, 0.0), (0.04, 0.0, 0.0),
                                     (0.0, 0.04, 0.0), (0.0, 0.0, 0.04)))
        self._fixed_q = self._home_q + jnp.asarray((0.04, -0.03, 0.03, 0.02, -0.02, 0.03))
        # The fixed candidate passes the CPU Reach goal's 2-10 cm constraint.
        self._initial_key = jax.random.PRNGKey(seed)
        self._reset_one = jax.jit(self._build_reset())
        self._reset_batch = jax.jit(jax.vmap(self._reset_one))
        self._step_batch = jax.jit(jax.vmap(self._build_step()))
        self._state = None
        self._pending_actions = None
        super().__init__(num_envs, spaces.Box(-jnp.inf, jnp.inf, shape=(24,), dtype=np.float32),
                         spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32))

    def _observe(self, state):
        data = state.data
        q = data.qpos[self._arm_qpos]
        dq = data.qvel[self._arm_qvel]
        pos = data.site_xpos[self._site]
        mat = data.site_xmat[self._site].reshape(3, 3)
        return jnp.concatenate((q - self._home_q, dq, state.goal_pos - pos,
                                _orientation_vector(mat, state.goal_mat), state.targets - self._home_q))

    def _keypoint_error(self, state):
        pos = state.data.site_xpos[self._site]
        mat = state.data.site_xmat[self._site].reshape(3, 3)
        current = pos + self._offsets @ mat.T
        target = state.goal_pos + self._offsets @ state.goal_mat.T
        return jnp.mean(jnp.linalg.norm(current - target, axis=1))

    def _build_reset(self):
        def reset(key):
            if self.curriculum_stage == "fixed":
                goal_q = self._fixed_q
                goal_data = self._home_data.replace(qpos=self._home_qpos.at[self._arm_qpos].set(goal_q))
                goal_data = mjx.forward(self._mjx_model, goal_data)
                key, _ = jax.random.split(key)
            else:
                # Match CPU Reach: first of at most 100 uniform joint samples
                # whose FK target is 2-10 cm from home. Do not clip a sampled
                # joint before testing it; out-of-range candidates are rejected.
                def draw(carry):
                    rng, _, _, _, count = carry
                    rng, subkey = jax.random.split(rng)
                    radius = 0.06 if self.curriculum_stage == "local_easy" else 0.12
                    candidate = self._home_q + jax.random.uniform(
                        subkey, (6,), minval=-radius, maxval=radius)
                    candidate_data = self._home_data.replace(
                        qpos=self._home_qpos.at[self._arm_qpos].set(candidate))
                    candidate_data = mjx.forward(self._mjx_model, candidate_data)
                    distance = jnp.linalg.norm(
                        candidate_data.site_xpos[self._site] - self._home_pos)
                    angle = jnp.linalg.norm(_orientation_vector(
                        self._home_mat, candidate_data.site_xmat[self._site].reshape(3, 3)))
                    in_range = jnp.all((candidate >= self._joint_lower) &
                                       (candidate <= self._joint_upper))
                    max_distance = 0.05 if self.curriculum_stage == "local_easy" else 0.10
                    accepted = in_range & (distance >= 0.02) & (distance <= max_distance)
                    if self.curriculum_stage == "local_easy":
                        accepted = accepted & (angle <= jnp.deg2rad(8.0))
                    return rng, candidate, candidate_data, accepted, count + 1

                def needs_goal(carry):
                    return (~carry[3]) & (carry[4] < 100)

                key, goal_q, goal_data, accepted, _attempts = lax.while_loop(
                    needs_goal, draw,
                    (key, self._home_q, self._home_data, jnp.array(False), jnp.array(0)))
                # The fixed FK target is a valid deterministic fallback for the
                # exceptionally unlikely case where all 100 samples are rejected.
                goal_q = jnp.where(accepted, goal_q, self._fixed_q)
                goal_data = lax.cond(
                    accepted, lambda _: goal_data,
                    lambda _: mjx.forward(self._mjx_model, self._home_data.replace(
                        qpos=self._home_qpos.at[self._arm_qpos].set(self._fixed_q))),
                    operand=None)
            goal_pos = goal_data.site_xpos[self._site]
            goal_mat = goal_data.site_xmat[self._site].reshape(3, 3)
            initial_distance = jnp.linalg.norm(goal_pos - self._home_pos)
            state = ReachState(self._home_data, self._home_q, goal_pos, goal_mat, initial_distance,
                               jnp.zeros(6), jnp.zeros(6), self._initial_torque, jnp.zeros(6),
                               jnp.array(0.0), jnp.array(0), jnp.array(0), key)
            return state._replace(previous_error=self._keypoint_error(state))
        return reset

    def _control(self, state):
        p = self._p
        q = state.data.qpos[self._arm_qpos]
        dq = state.data.qvel[self._arm_qvel]
        position_error = state.targets - q
        pc = jnp.clip(state.pos_integral + position_error * self._dt, -self._pos_limit, self._pos_limit)
        raw_v = p["pos_kp"] * position_error + p["pos_ki"] * pc
        target_v = jnp.clip(raw_v, -p["velocity_limit"], p["velocity_limit"])
        pos_accept = (raw_v == target_v) | (jnp.sign(position_error) != jnp.sign(raw_v))
        pi = jnp.where(pos_accept, pc, state.pos_integral)
        velocity_error = target_v - dq
        vc = jnp.clip(state.vel_integral + velocity_error * self._dt, -self._vel_limit, self._vel_limit)
        raw_feedback = (p["vel_kp"] * velocity_error + p["vel_ki"] * vc) * p["firmware_to_torque_scale"]
        raw_torque = raw_feedback + state.data.qfrc_bias[self._arm_qvel] * self._gravity_scale
        clipped = jnp.clip(raw_torque, -p["effort_limit"], p["effort_limit"])
        vel_accept = (raw_torque == clipped) | (jnp.sign(velocity_error) != jnp.sign(raw_feedback))
        vi = jnp.where(vel_accept, vc, state.vel_integral)
        delta = p["torque_rate_limit_nm_s"] * self._dt
        limited = state.applied + jnp.clip(clipped - state.applied, -delta, delta)
        torque = jnp.clip(state.applied + p["torque_lowpass_alpha"] * (limited - state.applied),
                          -p["effort_limit"], p["effort_limit"])
        data = state.data.replace(ctrl=state.data.ctrl.at[self._actuators].set(torque))
        return state._replace(data=data, pos_integral=pi, vel_integral=vi, applied=torque)

    def _build_step(self):
        def step(state, action):
            action = jnp.clip(action, -1.0, 1.0)
            targets = jnp.clip(state.targets + 0.01 * action, self._home_q - 0.25, self._home_q + 0.25)
            targets = jnp.clip(targets, self._joint_lower, self._joint_upper)
            state = state._replace(targets=targets)
            def physics(i, current):
                current = lax.cond(i % 5 == 0, self._control, lambda x: x, current)
                return current._replace(data=mjx.step(self._mjx_model, current.data))
            state = lax.fori_loop(0, 10, physics, state)
            state = state._replace(data=mjx.forward(self._mjx_model, state.data))
            pos = state.data.site_xpos[self._site]
            mat = state.data.site_xmat[self._site].reshape(3, 3)
            distance = jnp.linalg.norm(pos - state.goal_pos)
            angle = jnp.linalg.norm(_orientation_vector(mat, state.goal_mat))
            slow = jnp.max(jnp.abs(state.data.qvel[self._arm_qvel])) < 0.1
            hold = jnp.where((distance < 0.01) & (angle < jnp.deg2rad(3.0)) & slow,
                             state.hold_steps + 1, 0)
            steps = state.steps + 1
            success = hold >= 10
            timeout = steps >= self.max_episode_steps
            error = self._keypoint_error(state)
            progress = state.previous_error - error
            if self.reward_profile == "adaptive_position":
                # Freeze the distance scale at reset. A broad term helps far
                # goals, while the fixed 15 mm term still rewards final accuracy.
                scale = jnp.clip(0.4 * state.initial_distance, 0.015, 0.04)
                position_reward = (0.5 * jnp.exp(-distance / scale)
                                   + 0.5 * jnp.exp(-distance / 0.015))
                reward = (position_reward + 0.5 * jnp.exp(-angle / 0.1)
                          + progress - 0.005 * jnp.sum(action ** 2)
                          - 0.005 * jnp.sum((action - state.previous_action) ** 2)
                          + 5.0 * success)
            elif self.reward_profile == "pose_v3":
                # Increase precision pressure around the 1 cm position gate
                # while retaining a distinct orientation term.
                reward = (jnp.exp(-distance / 0.015) + 0.5 * jnp.exp(-angle / 0.1)
                          + progress - 0.005 * jnp.sum(action ** 2)
                          - 0.005 * jnp.sum((action - state.previous_action) ** 2)
                          + 5.0 * success)
            elif self.reward_profile == "pose_v2":
                # Independent, normalized precision signals: the old four-point
                # mean can hide a large orientation error behind small position.
                reward = (jnp.exp(-distance / 0.03) + 0.5 * jnp.exp(-angle / 0.1)
                          + progress - 0.005 * jnp.sum(action ** 2)
                          - 0.005 * jnp.sum((action - state.previous_action) ** 2)
                          + 5.0 * success)
            else:
                reward = (-2.0 * error + jnp.exp(-error / 0.03) + progress
                          - 0.005 * jnp.sum(action ** 2)
                          - 0.005 * jnp.sum((action - state.previous_action) ** 2)
                          + 5.0 * success)
            state = state._replace(previous_action=action, previous_error=error,
                                   hold_steps=hold, steps=steps)
            observation = self._observe(state)
            done = success | timeout
            next_state = lax.cond(done, lambda s: self._reset_one(s.key), lambda s: s, state)
            next_observation = lax.cond(done, self._observe, lambda s: observation, next_state)
            speed = jnp.max(jnp.abs(state.data.qvel[self._arm_qvel]))
            return next_state, next_observation, reward, done, success, distance, angle, speed, hold, observation
        return step

    def reset(self):
        keys = jax.random.split(self._initial_key, self.num_envs + 1)
        self._initial_key = keys[0]
        self._state = self._reset_batch(keys[1:])
        return np.array(jax.device_get(jax.vmap(self._observe)(self._state)), dtype=np.float32, copy=True)

    def reset_to_goals(self, goal_q):
        """Reset lanes to supplied reachable joint goals for paired evaluations."""
        goal_q = np.asarray(goal_q, dtype=np.float32)
        if goal_q.shape != (self.num_envs, 6) or not np.all(np.isfinite(goal_q)):
            raise ValueError("goal_q must be finite [num_envs, 6]")
        lower = np.asarray(self._joint_lower)
        upper = np.asarray(self._joint_upper)
        if np.any(goal_q < lower) or np.any(goal_q > upper):
            raise ValueError("goal_q exceeds model joint limits")
        goals = jnp.asarray(goal_q)
        def one(q):
            data = self._home_data.replace(qpos=self._home_qpos.at[self._arm_qpos].set(q))
            data = mjx.forward(self._mjx_model, data)
            return data.site_xpos[self._site], data.site_xmat[self._site].reshape(3, 3)
        positions, matrices = jax.jit(jax.vmap(one))(goals)
        distance = np.asarray(jnp.linalg.norm(positions - self._home_pos, axis=1))
        if np.any((distance < 0.02 - 1e-5) | (distance > 0.10 + 1e-5)):
            raise ValueError("goal_q must place TCP 2-10 cm from home")
        self.reset()
        initial_distance = jnp.linalg.norm(positions - self._home_pos, axis=1)
        self._state = self._state._replace(goal_pos=positions, goal_mat=matrices,
                                          initial_distance=initial_distance)
        self._state = self._state._replace(
            previous_error=jax.vmap(self._keypoint_error)(self._state))
        return np.array(jax.device_get(jax.vmap(self._observe)(self._state)), dtype=np.float32, copy=True)

    def step_async(self, actions):
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (self.num_envs, 6) or not np.all(np.isfinite(actions)):
            raise ValueError("actions must be finite [num_envs, 6]")
        self._pending_actions = jnp.asarray(actions)

    def step_wait(self):
        if self._pending_actions is None:
            raise RuntimeError("step_async must precede step_wait")
        result = self._step_batch(self._state, self._pending_actions)
        self._state, observations, rewards, dones, successes, distances, angles, speeds, holds, terminal_obs = result
        self._pending_actions = None
        observations, rewards, dones, successes, distances, angles, speeds, holds, terminal_obs = jax.device_get(
            (observations, rewards, dones, successes, distances, angles, speeds, holds, terminal_obs))
        infos = []
        for index in range(self.num_envs):
            info = {"is_success": bool(successes[index]), "distance_m": float(distances[index]),
                    "orientation_error_rad": float(angles[index]),
                    "max_joint_speed_rad_s": float(speeds[index]), "hold_steps": int(holds[index])}
            if dones[index]:
                info["terminal_observation"] = np.asarray(terminal_obs[index], dtype=np.float32)
                info["TimeLimit.truncated"] = not bool(successes[index])
            infos.append(info)
        return (np.array(observations, dtype=np.float32, copy=True),
                np.array(rewards, dtype=np.float32, copy=True),
                np.array(dones, dtype=bool, copy=True), infos)

    def close(self):
        self._state = None

    def get_attr(self, attr_name, indices=None):
        return [getattr(self, attr_name)] * len(self._get_indices(indices))

    def set_attr(self, attr_name, value, indices=None):
        raise NotImplementedError("MJX batch attributes are immutable during training")

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError("MJX batch methods are immutable during training")

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * len(self._get_indices(indices))
