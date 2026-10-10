from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

from rebotarm_simulation.core.mujoco_env import ReachEnvConfig, RebotArmReachEnv
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.core.pick_task import PickTaskConfig, pick_failure_reason, summarize_cube_contacts


@dataclass(frozen=True)
class PickEnvConfig(ReachEnvConfig):
    max_steps: int = 400
    settle_steps: int = 200
    task: PickTaskConfig = field(default_factory=PickTaskConfig)

    def __post_init__(self) -> None:
        if self.max_steps <= 0:
            raise ValueError("max_steps must be positive")
        if self.physics_steps_per_action <= 0:
            raise ValueError("physics_steps_per_action must be positive")
        if self.settle_steps < 0:
            raise ValueError("settle_steps must be non-negative")


class RebotArmPickEnv(RebotArmReachEnv):
    """Gym-style cube Pick task with explicit contact and failure semantics."""

    def __init__(
        self,
        model_path: str | None = None,
        *,
        config: PickEnvConfig | None = None,
        sim_factory: Callable = RebotArmMujoco,
    ) -> None:
        super().__init__(model_path, config=config or PickEnvConfig(), sim_factory=sim_factory)
        self.config: PickEnvConfig
        self._initial_cube_z = 0.0
        self._grasp_stable_steps = 0
        self._lift_stable_steps = 0
        self._lost_grasp_steps = 0
        self._ever_grasped = False
        self._stage = "approach"
        self._failure_reason = "none"

    @property
    def stage(self) -> str:
        return self._stage

    @property
    def failure_reason(self) -> str:
        return self._failure_reason

    def reset(
        self,
        *,
        seed: int | None = None,
        randomize: bool = True,
        randomization=None,
    ):
        super().reset(seed=seed, randomize=randomize, randomization=randomization)
        if self.config.settle_steps:
            self._sim.step(self.config.settle_steps)
        state = self._sim.get_state()
        cube_position = np.asarray(state.object_poses["test_cube"][:3], dtype=float)
        self._initial_cube_z = float(cube_position[2])
        self._target[:] = cube_position + np.array([0.0, 0.0, self.config.task.lift_height_m])
        self._grasp_stable_steps = 0
        self._lift_stable_steps = 0
        self._lost_grasp_steps = 0
        self._ever_grasped = False
        self._stage = "approach"
        self._failure_reason = "none"
        obs = self._observation()
        return obs, self._pick_info(obs)

    def step(self, action: Sequence[float]):
        self._advance_action(action)
        obs = self._observation()
        contacts = summarize_cube_contacts(self._sim.get_contacts())
        bilateral = contacts.bilateral_finger_contact
        force_closure = contacts.force_closure_candidate(
            self.config.task.max_finger_normal_dot
        )
        grasp_candidate = force_closure and float(obs["gripper_width"]) <= self.config.task.max_grasp_width_m
        self._grasp_stable_steps = self._grasp_stable_steps + 1 if grasp_candidate else 0
        grasped = self._grasp_stable_steps >= self.config.task.grasp_stability_steps
        self._ever_grasped = self._ever_grasped or grasped
        if self._ever_grasped and not bilateral:
            self._lost_grasp_steps += 1
        else:
            self._lost_grasp_steps = 0

        lift_height = float(obs["cube_pose"][2]) - self._initial_cube_z
        lifted = lift_height >= self.config.task.lift_height_m
        self._lift_stable_steps = self._lift_stable_steps + 1 if lifted and force_closure else 0
        success = self._lift_stable_steps >= self.config.task.success_stability_steps
        self._failure_reason = pick_failure_reason(
            config=self.config.task,
            cube_position=obs["cube_pose"][:3],
            contacts=contacts,
            ever_grasped=self._ever_grasped,
            lost_grasp_steps=self._lost_grasp_steps,
        )
        failed = self._failure_reason != "none"
        self._stage = self._resolve_stage(obs, contacts, success, failed)
        terminated = success or failed
        truncated = self._step_count >= self.config.max_steps
        reward = self._reward(obs, contacts, lift_height, success, failed)
        info = self._pick_info(obs, contacts=contacts)
        info.update(
            done=terminated or truncated,
            terminated=terminated,
            truncated=truncated,
            is_success=success,
            lift_height_m=lift_height,
        )
        return obs, reward, terminated, truncated, info

    def _observation(self) -> dict:
        obs = super()._observation()
        contacts = summarize_cube_contacts(self._sim.get_contacts())
        cube_position = np.asarray(obs["cube_pose"][:3], dtype=np.float32)
        ee_position = np.asarray(obs["ee_position"], dtype=np.float32)
        obs.update(
            cube_position=cube_position,
            cube_to_ee=cube_position - ee_position,
            lift_target_position=self._target.astype(np.float32).copy(),
            left_finger_contact=float(contacts.left_finger_contact),
            right_finger_contact=float(contacts.right_finger_contact),
            bilateral_finger_contact=float(contacts.bilateral_finger_contact),
            force_closure_candidate=float(
                contacts.force_closure_candidate(self.config.task.max_finger_normal_dot)
            ),
            finger_normal_dot=contacts.finger_normal_dot,
            left_finger_force_n=contacts.left_finger_force_n,
            right_finger_force_n=contacts.right_finger_force_n,
            cube_contact_count=contacts.cube_contact_count,
            max_contact_penetration_m=contacts.max_penetration_m,
        )
        return obs

    def _resolve_stage(self, obs, contacts, success: bool, failed: bool) -> str:
        if success:
            return "success"
        if failed:
            return "failure"
        if self._ever_grasped:
            return "lift"
        if contacts.force_closure_candidate(self.config.task.max_finger_normal_dot):
            return "grasp"
        if contacts.left_finger_contact or contacts.right_finger_contact:
            return "contact"
        distance = float(np.linalg.norm(obs["cube_to_ee"]))
        return "contact" if distance <= self.config.task.approach_tolerance_m else "approach"

    def _reward(self, obs, contacts, lift_height: float, success: bool, failed: bool) -> float:
        distance = float(np.linalg.norm(obs["cube_to_ee"]))
        reward = -distance + 2.0 * max(0.0, lift_height)
        reward += 0.1 * float(contacts.left_finger_contact)
        reward += 0.1 * float(contacts.right_finger_contact)
        reward += 0.25 * float(self._ever_grasped)
        if success:
            reward += 10.0
        if failed:
            reward -= 5.0
        return float(reward)

    def _pick_info(self, obs, *, contacts=None) -> dict:
        contacts = contacts or summarize_cube_contacts(self._sim.get_contacts())
        return {
            "step_count": self._step_count,
            "stage": self._stage,
            "failure_reason": self._failure_reason,
            "is_success": self._stage == "success",
            "initial_cube_z_m": self._initial_cube_z,
            "lift_height_m": float(obs["cube_pose"][2]) - self._initial_cube_z,
            "grasp_stable_steps": self._grasp_stable_steps,
            "lift_stable_steps": self._lift_stable_steps,
            "lost_grasp_steps": self._lost_grasp_steps,
            "left_finger_contact": contacts.left_finger_contact,
            "right_finger_contact": contacts.right_finger_contact,
            "bilateral_finger_contact": contacts.bilateral_finger_contact,
            "force_closure_candidate": contacts.force_closure_candidate(
                self.config.task.max_finger_normal_dot
            ),
            "finger_normal_dot": contacts.finger_normal_dot,
            "left_finger_force_n": contacts.left_finger_force_n,
            "right_finger_force_n": contacts.right_finger_force_n,
            "max_cube_contact_force_n": contacts.max_contact_force_n,
            "max_contact_penetration_m": contacts.max_penetration_m,
            "cube_contact_count": contacts.cube_contact_count,
        }
