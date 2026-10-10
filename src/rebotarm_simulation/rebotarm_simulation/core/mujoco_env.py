from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.sim2real.randomization import RandomizationSample
from rebotarm_simulation.sim2real.schemas import TrajectorySample


@dataclass(frozen=True)
class ReachEnvConfig:
    max_steps: int = 200
    physics_steps_per_action: int = 10
    action_scale_rad: float = 0.02
    gripper_action_scale_m: float = 0.002
    target_tolerance_m: float = 0.03
    home_on_reset: bool = True


class RebotArmReachEnv:
    """Small Gym-style Reach task wrapper around the MuJoCo simulator.

    It intentionally avoids depending on gymnasium so the ROS 2 simulation
    package remains lightweight. The default method signatures mirror
    Gymnasium: reset() -> (obs, info), step(action) -> (obs, reward,
    terminated, truncated, info). step_done(action) returns the older Gym
    shape: (obs, reward, done, info).
    """

    def __init__(
        self,
        model_path: str | None = None,
        *,
        config: ReachEnvConfig | None = None,
        sim_factory: Callable = RebotArmMujoco,
    ) -> None:
        self.config = config or ReachEnvConfig()
        if self.config.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.config.physics_steps_per_action <= 0:
            raise ValueError("physics_steps_per_action must be positive")
        self._sim = sim_factory(model_path)
        self._target = np.zeros(3, dtype=float)
        self._step_count = 0
        self._randomization_sample: RandomizationSample | None = None
        self._randomization_rng = np.random.default_rng()
        self._action_history: list[np.ndarray] = []

    @property
    def sim(self):
        return self._sim

    @property
    def target_position(self) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._target)

    @property
    def randomization_sample(self) -> RandomizationSample | None:
        return self._randomization_sample

    def set_randomization_sample(self, sample: RandomizationSample | None) -> None:
        if sample is not None and not isinstance(sample, RandomizationSample):
            raise TypeError("sample must be a RandomizationSample or None")
        if sample is None:
            if hasattr(self._sim, "restore_randomization"):
                self._sim.restore_randomization()
        elif hasattr(self._sim, "apply_randomization"):
            self._sim.apply_randomization(sample)
        self._randomization_sample = sample

    def reset(
        self,
        *,
        seed: int | None = None,
        randomize: bool = True,
        randomization: RandomizationSample | None = None,
    ):
        self._step_count = 0
        self._action_history = []
        self._randomization_rng = np.random.default_rng(seed)
        self.set_randomization_sample(None)
        if self.config.home_on_reset and hasattr(self._sim, "reset_home"):
            self._sim.reset_home(seed=seed)
        else:
            self._sim.reset(seed=seed)
        self.set_randomization_sample(randomization)
        if randomize:
            scene = self._sim.randomize_scene(seed=seed)
            self._target[:] = np.asarray(scene.reach_target_position, dtype=float)
        else:
            self._target[:] = self._sim.get_state().end_effector_position
        obs = self._observation()
        return obs, self._info(obs)

    def step(self, action: Sequence[float]):
        self._advance_action(action)
        obs = self._observation()
        distance = float(np.linalg.norm(obs["ee_position"] - obs["target_position"]))
        reward = -distance
        terminated = distance <= self.config.target_tolerance_m
        truncated = self._step_count >= self.config.max_steps
        info = self._info(obs)
        info["done"] = terminated or truncated
        info["terminated"] = terminated
        info["truncated"] = truncated
        return obs, reward, terminated, truncated, info

    def _advance_action(self, action: Sequence[float]) -> np.ndarray:
        action_vector = _finite_action(action)
        applied_action = self._apply_randomized_action(action_vector)
        current_targets = np.asarray(self._sim.control_targets[:6], dtype=float)
        target = current_targets + applied_action[:6] * self.config.action_scale_rad
        self._sim.set_joint_position_targets(target)
        if len(applied_action) >= 7:
            current_width = float(self._sim.control_targets[-2] - self._sim.control_targets[-1])
            self._sim.set_gripper_width(
                current_width + applied_action[6] * self.config.gripper_action_scale_m
            )
        self._sim.step(self.config.physics_steps_per_action)
        self._step_count += 1
        return applied_action

    def _apply_randomized_action(self, action_vector: np.ndarray) -> np.ndarray:
        sample = self._randomization_sample
        if sample is None:
            return action_vector
        noisy = action_vector.copy()
        if sample.action_noise_std:
            noisy += self._randomization_rng.normal(0.0, sample.action_noise_std, size=noisy.shape)
        self._action_history.append(noisy.copy())
        latency = sample.control_latency_steps
        if latency and len(self._action_history) <= latency:
            return np.zeros_like(noisy)
        if latency:
            return self._action_history[-latency - 1].copy()
        return noisy

    def sample_from_last_step(
        self,
        action: Sequence[float],
        *,
        episode_id: str,
        step_index: int,
    ) -> TrajectorySample:
        state = self._sim.get_state()
        contacts = self._sim.get_contacts()
        targets = tuple(float(value) for value in self._sim.control_targets)
        return TrajectorySample(
            schema_version=1,
            episode_id=episode_id,
            step_index=step_index,
            simulation_time=float(state.simulation_time),
            joint_positions=tuple(float(value) for value in state.joint_positions[:6]),
            joint_velocities=tuple(float(value) for value in state.joint_velocities[:6]),
            joint_targets=targets[:6],
            actuator_torques=tuple(float(value) for value in state.actuator_forces[:6]),
            gripper_width=float(state.gripper_width),
            gripper_target_width=max(0.0, targets[-2] - targets[-1]),
            end_effector_position=tuple(float(value) for value in state.end_effector_position),
            end_effector_orientation_xyzw=tuple(float(value) for value in state.end_effector_orientation),
            action=tuple(float(value) for value in _finite_action(action)),
            max_contact_force=max((float(contact.force) for contact in contacts), default=0.0),
            contact_count=len(contacts),
            source="sim",
            max_contact_penetration=max(
                (float(getattr(contact, "penetration_depth", 0.0)) for contact in contacts),
                default=0.0,
            ),
        )

    def step_done(self, action: Sequence[float]):
        obs, reward, terminated, truncated, info = self.step(action)
        return obs, reward, bool(terminated or truncated), info

    def close(self) -> None:
        if self._randomization_sample is not None:
            self.set_randomization_sample(None)
        self._sim.close()

    def _observation(self) -> dict[str, np.ndarray | float]:
        state = self._sim.get_state()
        contacts = self._sim.get_contacts()
        max_contact_force = max((contact.force for contact in contacts), default=0.0)
        joint_positions = np.asarray(state.joint_positions[:6], dtype=np.float32)
        joint_velocities = np.asarray(state.joint_velocities[:6], dtype=np.float32)
        if self._randomization_sample is not None:
            sample = self._randomization_sample
            if sample.position_noise_std:
                joint_positions += self._randomization_rng.normal(
                    0.0, sample.position_noise_std, size=joint_positions.shape
                ).astype(np.float32)
            if sample.velocity_noise_std:
                joint_velocities += self._randomization_rng.normal(
                    0.0, sample.velocity_noise_std, size=joint_velocities.shape
                ).astype(np.float32)
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "gripper_width": float(state.gripper_width),
            "ee_position": np.asarray(state.end_effector_position, dtype=np.float32),
            "target_position": self._target.astype(np.float32).copy(),
            "cube_pose": np.asarray(state.object_poses["test_cube"], dtype=np.float32),
            "max_contact_force": float(max_contact_force),
        }

    def _info(self, obs: dict[str, np.ndarray | float]) -> dict[str, float | int | bool]:
        distance = float(np.linalg.norm(obs["ee_position"] - obs["target_position"]))
        return {
            "step_count": self._step_count,
            "distance_to_target_m": distance,
            "is_success": distance <= self.config.target_tolerance_m,
        }

    def __enter__(self) -> "RebotArmReachEnv":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def _finite_action(action: Sequence[float]) -> np.ndarray:
    if isinstance(action, (str, bytes)):
        raise TypeError("action must be a numeric sequence")
    result = np.asarray(tuple(action), dtype=float)
    if result.shape not in {(6,), (7,)}:
        raise ValueError(f"action must have shape (6,) or (7,), got {result.shape}")
    if not np.isfinite(result).all():
        raise ValueError("action values must be finite")
    return np.clip(result, -1.0, 1.0)
