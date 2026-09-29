"""纯仿真局部 Reach 任务；不导入 ROS，不接触真实硬件。

观测为 q(6), dq(6), control_targets(6), TCP位置/姿态(3+4), goal位置/姿态(3+4)。
保留目标角使增量动作的目标累积状态可观测；底层控制器积分状态仍不可观测。
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .mujoco_sim import RebotArmMujoco


class RebotArmReachEnv(gym.Env):
    """固定 home 起步、在 home 附近采样 FK 可达目标位姿，暂不控制夹爪。

    50 Hz 策略、500 Hz 物理；仅支持 headless。失败为模型报告的机器人接触。
    这不是 MoveIt 碰撞有效性检查，也不构成真实机械臂安全控制器。
    """
    metadata = {"render_modes": []}

    def __init__(self, *, max_episode_steps: int = 250, render_mode=None):
        if render_mode is not None:
            raise ValueError("Only headless render_mode=None is supported")
        if isinstance(max_episode_steps, bool) or not isinstance(max_episode_steps, int) or max_episode_steps < 1:
            raise ValueError("max_episode_steps must be a positive integer")
        self.render_mode = None
        self.max_episode_steps = max_episode_steps
        self.action_space = spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(32,), dtype=np.float32)
        self.sim = RebotArmMujoco()
        self.frame_skip = 10
        self.action_scale = 0.01  # rad / decision; local target envelope adds a second bound
        self._done = True
        self._closed = False
        self._goal = np.zeros(3)
        self._goal_orientation = np.array((0.0, 0.0, 0.0, 1.0))
        self._steps = 0
        self._hold_steps = 0

    def _collision(self):
        # 标准场景允许瓶子与桌面/地面接触；其它接触包含机器人。
        passive = {"world", "table", "bottle"}
        return any(not ({c.body1, c.body2} <= passive) for c in self.sim.get_contacts())

    def _observe(self, state):
        observation = np.asarray([
            *state.joint_positions[:6], *state.joint_velocities[:6],
            *self.sim.control_targets[:6], *state.end_effector_position,
            *state.end_effector_orientation, *self._goal, *self._goal_orientation,
        ], dtype=np.float32)
        if not np.all(np.isfinite(observation)):
            self._done = True
            raise FloatingPointError("Non-finite simulation observation")
        return observation

    def reset(self, *, seed=None, options=None):
        if self._closed:
            raise RuntimeError("Environment is closed")
        if options:
            raise ValueError("This baseline has no reset options")
        super().reset(seed=seed)
        self._done = True
        state = self.sim.reset_home(seed=seed)
        self._home = np.asarray(state.joint_positions[:6])
        saved = self.sim.save_state()
        # 从关节构型生成目标，避免任意 xyz 采样生成不可达点。
        # 只检查目标构型接触，不宣称起点到目标之间有无碰撞路径。
        for _ in range(100):
            candidate = self._home + self.np_random.uniform(-0.12, 0.12, 6)
            goal_state = self.sim.reset_joint_positions(candidate)
            goal = np.asarray(goal_state.end_effector_position)
            distance = np.linalg.norm(goal - state.end_effector_position)
            if not self._collision() and 0.02 <= distance <= 0.10:
                self._goal = goal
                self._goal_orientation = np.asarray(goal_state.end_effector_orientation)
                break
        else:
            self.sim.restore_state(saved)
            raise RuntimeError("No valid local Reach goal after 100 attempts")
        self.sim.restore_state(saved)
        self._steps = self._hold_steps = 0
        self._done = False
        return self._observe(state), {
            "distance_m": float(distance),
            "orientation_error_rad": _quaternion_distance(
                np.asarray(state.end_effector_orientation), self._goal_orientation
            ),
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
        collision = False
        # 每个物理步检查，不能漏掉两个策略决策之间的短暂接触。
        for _ in range(self.frame_skip):
            state = self.sim.step()
            if self._collision():
                collision = True
                break
        self._steps += 1
        distance = float(np.linalg.norm(np.asarray(state.end_effector_position) - self._goal))
        orientation_error = _quaternion_distance(
            np.asarray(state.end_effector_orientation), self._goal_orientation
        )
        slow = np.max(np.abs(state.joint_velocities[:6])) < 0.1
        self._hold_steps = self._hold_steps + 1 if (
            distance < 0.01 and orientation_error < np.deg2rad(3.0)
            and slow and not collision
        ) else 0
        success = self._hold_steps >= 10  # 连续 0.2 s 到位且低速
        terminated = bool(success or collision)
        truncated = bool(self._steps >= self.max_episode_steps and not terminated)
        reward = float(
            -10.0 * distance - 0.25 * orientation_error
            - 0.001 * np.sum(action**2) + 5.0 * success - 5.0 * collision
        )
        self._done = terminated or truncated
        return self._observe(state), reward, terminated, truncated, {
            "distance_m": distance, "orientation_error_rad": orientation_error,
            "is_success": success, "collision": collision,
            "simulation_time": state.simulation_time,
        }

    def close(self):
        if not self._closed:
            self.sim.close()
            self._closed = True


def _quaternion_distance(first: np.ndarray, second: np.ndarray) -> float:
    """返回两个 XYZW 单位四元数代表姿态之间的最短角距离（rad）。"""
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    dot = float(np.clip(abs(np.dot(first, second)), -1.0, 1.0))
    return float(2.0 * np.arccos(dot))
