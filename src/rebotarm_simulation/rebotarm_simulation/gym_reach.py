"""纯仿真 Reach 任务；不导入 ROS，不接触真实硬件。"""
from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .mujoco_sim import RebotArmMujoco, _default_scene_path


class RebotArmReachEnv(gym.Env):
    """六轴从 home 到附近末端位姿的 headless Gymnasium 任务。"""
    metadata = {"render_modes": []}

    def __init__(self, *, max_episode_steps: int = 250, render_mode=None,
                 curriculum_stage: str = "local"):
        if render_mode is not None:
            raise ValueError("Only headless render_mode=None is supported")
        if isinstance(max_episode_steps, bool) or not isinstance(max_episode_steps, int) or max_episode_steps < 1:
            raise ValueError("max_episode_steps must be a positive integer")
        if curriculum_stage not in {"fixed", "local_easy", "local"}:
            raise ValueError("curriculum_stage must be fixed, local_easy or local")
        self.render_mode = None
        self.max_episode_steps = max_episode_steps
        self.curriculum_stage = curriculum_stage
        self.action_space = spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)
        # q-home(6), dq(6), TCP position error(3), orientation error vector(3), target-home(6)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(24,), dtype=np.float32)
        self.sim = RebotArmMujoco(model_path=_default_scene_path().with_name("reach_scene.xml"),
                                  collisionless=True)
        self.frame_skip = 10
        self.action_scale = 0.01
        self._done = True
        self._closed = False
        self._goal = np.zeros(3)
        self._goal_orientation = np.array((0.0, 0.0, 0.0, 1.0))
        self._steps = 0
        self._hold_steps = 0
        self._previous_action = np.zeros(6, dtype=np.float64)
        self._previous_keypoint_error = 0.0
        self._fixed_goal = None
        self._goal_q = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _observe(self, state):
        observation = np.asarray([
            *(np.asarray(state.joint_positions[:6]) - self._home),
            *state.joint_velocities[:6],
            *(self._goal - np.asarray(state.end_effector_position)),
            *_quaternion_error_vector(np.asarray(state.end_effector_orientation), self._goal_orientation),
            *(np.asarray(self.sim.control_targets[:6]) - self._home),
        ], dtype=np.float32)
        if not np.all(np.isfinite(observation)):
            self._done = True
            raise FloatingPointError("Non-finite simulation observation")
        return observation

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError("Environment is closed")
        if options and set(options) != {"goal_q"}:
            raise ValueError("Only goal_q reset option is supported")
        super().reset(seed=seed)
        self._done = True
        state = self.sim.reset_home(seed=seed)
        self._home = np.asarray(state.joint_positions[:6])
        saved = self.sim.save_state()
        candidates = []
        if options:
            candidate = np.asarray(options["goal_q"], dtype=np.float64)
            if candidate.shape != (6,) or not np.all(np.isfinite(candidate)):
                raise ValueError("goal_q must be six finite joint positions")
            candidates = [candidate]
        elif self.curriculum_stage == "fixed" and self._fixed_goal is not None:
            candidates = [self._fixed_goal]
        else:
            if self.curriculum_stage == "fixed":
                candidates.append(self._home + np.array((0.04, -0.03, 0.03, 0.02, -0.02, 0.03)))
            radius = 0.06 if self.curriculum_stage == "local_easy" else 0.12
            candidates.extend(self._home + self.np_random.uniform(-radius, radius, 6) for _ in range(100))
        for candidate in candidates:
            try:
                goal_state = self.sim.reset_joint_positions(candidate)
            except ValueError:
                if options:
                    self.sim.restore_state(saved)
                    raise
                continue
            goal = np.asarray(goal_state.end_effector_position)
            distance = np.linalg.norm(goal - state.end_effector_position)
            orientation = _quaternion_distance(
                np.asarray(state.end_effector_orientation),
                np.asarray(goal_state.end_effector_orientation))
            valid_goal = (0.02 <= distance <= (0.05 if self.curriculum_stage == "local_easy" else 0.10)
                          and (self.curriculum_stage != "local_easy" or orientation <= np.deg2rad(8.0)))
            if valid_goal:
                self._goal = goal
                self._goal_orientation = np.asarray(goal_state.end_effector_orientation)
                self._goal_q = np.asarray(candidate).copy()
                if self.curriculum_stage == "fixed" and self._fixed_goal is None:
                    self._fixed_goal = np.asarray(candidate).copy()
                break
        else:
            self.sim.restore_state(saved)
            if options:
                raise ValueError("goal_q must place the TCP 2-10 cm from home")
            raise RuntimeError("No valid local Reach goal after 100 attempts")
        self.sim.restore_state(saved)
        self._steps = self._hold_steps = 0
        self._previous_action.fill(0.0)
        self._previous_keypoint_error = self._keypoint_error(state)
        self._done = False
        return self._observe(state), {
            "distance_m": float(distance),
            "orientation_error_rad": _quaternion_distance(np.asarray(state.end_effector_orientation), self._goal_orientation),
            "is_success": False,
        }

    def step(self, action):
        if self._closed or self._done:
            raise RuntimeError("Call reset before step and after an episode ends")
        action = np.asarray(action, dtype=np.float64)
        if action.shape != (6,) or not np.all(np.isfinite(action)) or np.any(np.abs(action) > 1):
            raise ValueError("action must be six finite values in [-1, 1]")
        targets = np.asarray(self.sim.control_targets[:6]) + self.action_scale * action
        targets = np.clip(targets, self._home - 0.25, self._home + 0.25)
        self.sim.set_joint_position_targets(targets)
        for _ in range(self.frame_skip):
            state = self.sim.step()
        self._steps += 1
        distance = float(np.linalg.norm(np.asarray(state.end_effector_position) - self._goal))
        orientation_error = _quaternion_distance(np.asarray(state.end_effector_orientation), self._goal_orientation)
        slow = np.max(np.abs(state.joint_velocities[:6])) < 0.1
        self._hold_steps = self._hold_steps + 1 if (distance < 0.01 and orientation_error < np.deg2rad(3.0) and slow) else 0
        success = self._hold_steps >= 10
        terminated = bool(success)
        truncated = bool(self._steps >= self.max_episode_steps and not terminated)
        keypoint_error = self._keypoint_error(state)
        progress = self._previous_keypoint_error - keypoint_error
        action_rate = action - self._previous_action
        reward_terms = {
            "keypoint": -2.0 * keypoint_error,
            "precision": float(np.exp(-keypoint_error / 0.03)),
            "progress": float(progress),
            "action": -0.005 * float(np.sum(action ** 2)),
            "action_rate": -0.005 * float(np.sum(action_rate ** 2)),
            "success": 5.0 if success else 0.0,
        }
        reward = float(sum(reward_terms.values()))
        self._previous_action = action.copy()
        self._previous_keypoint_error = keypoint_error
        self._done = terminated or truncated
        return self._observe(state), reward, terminated, truncated, {
            "distance_m": distance, "orientation_error_rad": orientation_error,
            "max_joint_speed_rad_s": float(np.max(np.abs(state.joint_velocities[:6]))),
            "hold_steps": self._hold_steps,
            "is_success": success, "collision": False,
            "simulation_time": state.simulation_time, "keypoint_error_m": keypoint_error,
            "progress_m": progress, "reward_terms": reward_terms,
        }

    def _keypoint_error(self, state):
        offsets = np.asarray(((0.0, 0.0, 0.0), (0.04, 0.0, 0.0), (0.0, 0.04, 0.0), (0.0, 0.0, 0.04)))
        current = _rotate_points(np.asarray(state.end_effector_orientation), offsets) + np.asarray(state.end_effector_position)
        target = _rotate_points(self._goal_orientation, offsets) + self._goal
        return float(np.mean(np.linalg.norm(current - target, axis=1)))

    def close(self):
        if not self._closed:
            self.sim.close()
            self._closed = True


def _quaternion_distance(first: np.ndarray, second: np.ndarray) -> float:
    first = np.asarray(first, dtype=np.float64); second = np.asarray(second, dtype=np.float64)
    first /= np.linalg.norm(first); second /= np.linalg.norm(second)
    return float(2.0 * np.arccos(np.clip(abs(np.dot(first, second)), -1.0, 1.0)))


def _quaternion_error_vector(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    current = np.asarray(current, dtype=np.float64); target = np.asarray(target, dtype=np.float64)
    current /= np.linalg.norm(current); target /= np.linalg.norm(target)
    relative = _quaternion_multiply(target, _quaternion_conjugate(current))
    if relative[3] < 0.0: relative = -relative
    norm = np.linalg.norm(relative[:3])
    if norm < 1e-10: return np.zeros(3)
    return relative[:3] / norm * (2.0 * np.arctan2(norm, np.clip(relative[3], -1.0, 1.0)))


def _quaternion_conjugate(quaternion):
    return np.array((-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]))


def _quaternion_multiply(first, second):
    x1, y1, z1, w1 = first; x2, y2, z2, w2 = second
    return np.array((w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2))


def _rotate_points(quaternion, points):
    x, y, z, w = np.asarray(quaternion, dtype=np.float64) / np.linalg.norm(quaternion)
    matrix = np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                       (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                       (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))
    return np.asarray(points) @ matrix.T
