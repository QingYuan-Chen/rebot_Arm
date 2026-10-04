"""Optional mjlab GPU Reach task for the reBotArm.

This module is intentionally isolated from the ROS/MuJoCo CPU runtime.  It is
loaded by mjlab's ``mjlab.tasks`` entry-point only when the optional mjlab
environment is installed.  CPU/GPU paired evaluation uses the same compiled model and control contract.
The six torque actions are not ROS position-trajectory commands.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import torch

from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import Entity, EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnv, ManagerBasedRlEnvCfg
from mjlab.envs.mdp import joint_pos_rel, joint_vel_rel, time_out
from mjlab.envs.mdp.actions import JointEffortActionCfg
from mjlab.managers.action_manager import ActionTermCfg
from mjlab.managers.command_manager import CommandTerm, CommandTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from rebotarm_rl.agents.reach_ppo import _runner_cfg
from rebotarm_rl.resources import reach_scene_path
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.registry import register_mjlab_task
from mjlab.utils.lab_api.math import quat_error_magnitude

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


def _scene_path() -> Path:
    return reach_scene_path()


def _get_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(_scene_path()))


@dataclass(kw_only=True)
class ReachCommandCfg(CommandTermCfg):
    """Fixed or locally randomized TCP target around the home pose."""

    position_radius: float = 0.06
    orientation_radius: float = 0.0
    asset_name: str = "robot"
    site_name: str = "ee_site"

    def build(self, env: ManagerBasedRlEnv) -> ReachCommand:
        return ReachCommand(self, env)


class ReachCommand(CommandTerm):
    cfg: ReachCommandCfg

    def __init__(self, cfg: ReachCommandCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self.robot: Entity = env.scene[cfg.asset_name]
        site_ids, _ = self.robot.find_sites((cfg.site_name,), preserve_order=True)
        if len(site_ids) != 1:
            raise ValueError(f"Expected one site named {cfg.site_name!r}, got {site_ids}")
        self.site_id = site_ids[0]
        self.target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_quat = torch.zeros(self.num_envs, 4, device=self.device)
        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["success"] = torch.zeros(self.num_envs, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        return torch.cat((self.target_pos, self.target_quat), dim=-1)

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        home_pose = self.robot.data.site_pose_w[:, self.site_id]
        self.target_pos[env_ids] = home_pose[env_ids, :3]
        if self.cfg.position_radius > 0.0:
            direction = torch.randn(len(env_ids), 3, device=self.device)
            direction = direction / direction.norm(dim=-1, keepdim=True).clamp_min(1.0e-6)
            radius = torch.rand(len(env_ids), 1, device=self.device) * self.cfg.position_radius
            self.target_pos[env_ids] += direction * radius
        self.target_quat[env_ids] = home_pose[env_ids, 3:7]

    def _update_metrics(self) -> None:
        current = self.robot.data.site_pose_w[:, self.site_id]
        position_error = torch.linalg.vector_norm(current[:, :3] - self.target_pos, dim=-1)
        orientation_error = quat_error_magnitude(current[:, 3:7], self.target_quat)
        success = (position_error < 0.01) & (orientation_error < 0.05236)
        self.metrics["position_error"] = position_error
        self.metrics["orientation_error"] = orientation_error
        self.metrics["success"] = success.float()

    def _update_command(self, env_ids: torch.Tensor | None) -> None:
        del env_ids


_ROBOT_CFG = EntityCfg(
    spec_fn=_get_spec,
    articulation=EntityArticulationInfoCfg(
        # Wrap the six XML torque actuators; gripper actuators are intentionally
        # excluded from this first Reach task.
        actuators=(XmlActuatorCfg(target_names_expr=("joint[1-6]",)),),
    ),
    init_state=EntityCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": -0.8,
            "joint3": -1.0,
            "joint4": 0.3,
            "joint5": 0.0,
            "joint6": 0.0,
            "left_finger_joint": 0.03,
            "right_finger_joint": -0.03,
        },
        joint_vel={".*": 0.0},
    ),
)

_ROBOT = SceneEntityCfg("robot", joint_names=("joint[1-6]",))


def _position_error(env: ManagerBasedRlEnv) -> torch.Tensor:
    command = env.command_manager.get_term("reach")
    assert isinstance(command, ReachCommand)
    ee = env.scene["robot"].data.site_pos_w[:, command.site_id]
    return ee - command.target_pos


def _orientation_error(env: ManagerBasedRlEnv) -> torch.Tensor:
    command = env.command_manager.get_term("reach")
    assert isinstance(command, ReachCommand)
    ee = env.scene["robot"].data.site_quat_w[:, command.site_id]
    # Axis-angle vector is more suitable for the policy than a raw quaternion.
    current = ee / ee.norm(dim=-1, keepdim=True).clamp_min(1.0e-6)
    target = command.target_quat / command.target_quat.norm(dim=-1, keepdim=True).clamp_min(1.0e-6)
    xyz = current[:, :3] * target[:, 3:4] - target[:, :3] * current[:, 3:4]
    return 2.0 * xyz


def _reach_reward(env: ManagerBasedRlEnv) -> torch.Tensor:
    command = env.command_manager.get_term("reach")
    assert isinstance(command, ReachCommand)
    current = env.scene["robot"].data.site_pose_w[:, command.site_id]
    pos = torch.linalg.vector_norm(current[:, :3] - command.target_pos, dim=-1)
    ori = quat_error_magnitude(current[:, 3:7], command.target_quat)
    effort = torch.square(env.action_manager.action).sum(dim=-1)
    return torch.exp(-pos / 0.03) + 0.5 * torch.exp(-ori / 0.20) - 0.001 * effort


def _success(env: ManagerBasedRlEnv) -> torch.Tensor:
    command = env.command_manager.get_term("reach")
    assert isinstance(command, ReachCommand)
    return command.metrics["success"] > 0.5


def _make_env_cfg(*, play: bool = False, num_envs: int = 256) -> ManagerBasedRlEnvCfg:
    observations = {
        "actor": ObservationGroupCfg(
            terms={
                "joint_pos": ObservationTermCfg(func=joint_pos_rel, params={"asset_cfg": _ROBOT}),
                "joint_vel": ObservationTermCfg(func=joint_vel_rel, params={"asset_cfg": _ROBOT}),
                "position_error": ObservationTermCfg(func=_position_error),
                "orientation_error": ObservationTermCfg(func=_orientation_error),
            },
            enable_corruption=False,
        ),
    }
    actions: dict[str, ActionTermCfg] = {
        "joint_effort": JointEffortActionCfg(
            entity_name="robot",
            actuator_names=("joint[1-6]",),
            scale=1.0,
        ),
    }
    rewards = {"reach": RewardTermCfg(func=_reach_reward, weight=1.0)}
    terminations = {
        "success": TerminationTermCfg(func=_success, time_out=False),
        "time_out": TerminationTermCfg(func=time_out, time_out=True),
    }
    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(entities={"robot": _ROBOT_CFG}, num_envs=num_envs, env_spacing=1.5),
        observations=observations,
        actions=actions,
        commands={"reach": ReachCommandCfg(resampling_time_range=(1.0e9, 1.0e9))},
        rewards=rewards,
        terminations=terminations,
        sim=SimulationCfg(mujoco=MujocoCfg(timestep=0.002)),
        decimation=10,
        episode_length_s=5.0 if not play else 1.0e9,
        seed=7,
    )




register_mjlab_task(
    task_id="RebotArm-Reach-Mjlab",
    env_cfg=_make_env_cfg(),
    play_env_cfg=_make_env_cfg(play=True, num_envs=1),
    rl_cfg=_runner_cfg(),
)
