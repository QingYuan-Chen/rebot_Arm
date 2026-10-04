"""MJLab task: bounded position increments, torque-limited PD, continuous hold."""
from dataclasses import dataclass

import mujoco
import torch
from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import time_out
from mjlab.managers.action_manager import ActionTerm, ActionTermCfg
from mjlab.managers.metrics_manager import MetricsTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sensor.contact_sensor import ContactMatch, ContactSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.viewer import ViewerConfig

from .reach_model import (DELTA, DT, EPISODE_STEPS, HIGH, HOLD_STEPS, HOME, JOINT_SPEED_TOL,
    KD, KP, LOW, MARGIN, PHYSICS_DT, POSITION_TOL, SPEED_TOL, TORQUE, load_catalogue, model_spec)


def advance_hold(count, qualified):
    return torch.where(qualified, count + 1, torch.zeros_like(count))


def integrate_command(command, actions, low, high):
    if not torch.isfinite(actions).all():
        raise ValueError("Reach actions must be finite")
    return torch.clamp(command + actions.clamp(-1., 1.) * DELTA, low, high)


@dataclass(kw_only=True)
class ReachActionCfg(ActionTermCfg):
    split: str = "train"

    def build(self, env):
        return ReachAction(self, env)


class ReachAction(ActionTerm):
    """Own command/history/task state; counters advance only at termination check."""
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.bank = load_catalogue()[cfg.split]
        self.starts = self.tensor([r["start_rad"] for r in self.bank])
        self.targets = self.tensor([r["target_m"] for r in self.bank])
        self.low, self.high = self.tensor(LOW+MARGIN), self.tensor(HIGH-MARGIN)
        self.kp, self.kd, self.cap = self.tensor(KP), self.tensor(KD), self.tensor(TORQUE)
        self.command = self.tensor(HOME).repeat(self.num_envs, 1)
        self._raw = torch.zeros_like(self.command)
        self.previous = torch.zeros_like(self.command)
        self.action_change = torch.zeros(self.num_envs, device=self.device)
        self.target = self.targets[0].repeat(self.num_envs, 1)
        self.trial_index = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.hold = torch.zeros_like(self.trial_index)
        self.unsafe = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        self.success = self.unsafe.clone()
        self.error = torch.zeros(self.num_envs, device=self.device)
        self.speed = self.error.clone()
        self.tcp = torch.zeros_like(self.target)
        self.velocity = torch.zeros_like(self.target)
        self.last_update = -1
        self.site = self._entity.find_sites("ee_site")[0][0]
        self.mocap_id = env.sim.mj_model.body("reach_goal").mocapid[0]

    def tensor(self, value):
        return torch.as_tensor(value, dtype=torch.float32, device=self.device)

    @property
    def action_dim(self):
        return 6

    @property
    def raw_action(self):
        return self._raw

    def reset(self, env_ids=None):
        if env_ids is None or isinstance(env_ids, slice):
            env_ids = torch.arange(self.num_envs, device=self.device)[env_ids or slice(None)]
        indices = (torch.randint(len(self.bank), (len(env_ids),), device=self.device)
                   if self.cfg.split == "train" else env_ids % len(self.bank))
        self.trial_index[env_ids] = indices
        self.command[env_ids] = self.starts[indices]
        self.target[env_ids] = self.targets[indices]
        self._entity.write_joint_state_to_sim(self.command[env_ids], torch.zeros_like(self.command[env_ids]), env_ids=env_ids)
        self._env.sim.data.mocap_pos[env_ids, self.mocap_id] = self.target[env_ids] + self._env.scene.env_origins[env_ids]
        self._raw[env_ids] = 0
        self.previous[env_ids] = 0
        self.action_change[env_ids] = 0
        self.hold[env_ids] = 0
        self.unsafe[env_ids] = False
        self.success[env_ids] = False

    def process_actions(self, actions):
        self.command = integrate_command(self.command, actions, self.low, self.high)
        self.previous[:] = self._raw
        self._raw[:] = actions.clamp(-1., 1.)
        self.action_change[:] = (self._raw-self.previous).square().sum(-1)
        self.unsafe[:] = False

    def apply_actions(self):
        data = self._entity.data
        bias = self._env.sim.data.qfrc_bias[:, self._entity.indexing.joint_v_adr]
        effort = self.kp*(self.command-data.joint_pos)-self.kd*data.joint_vel+bias
        self._entity.set_joint_effort_target(effort.clamp(-self.cap, self.cap))

    def check_substep(self):
        data = self._entity.data
        contacts = self._env.scene["contacts"].data
        self.unsafe |= ((data.joint_pos < self.tensor(LOW)-.005) | (data.joint_pos > self.tensor(HIGH)+.005)).any(-1)
        self.unsafe |= (data.joint_vel.abs() > 2.).any(-1)
        self.unsafe |= ~torch.isfinite(data.joint_pos).all(-1) | ~torch.isfinite(data.joint_vel).all(-1)
        self.unsafe |= (contacts.found > 0).any(-1)
        return self.unsafe.float()

    def update(self):
        if self.last_update == self._env.common_step_counter:
            return
        self.last_update = self._env.common_step_counter
        # Exact current kinematics for hold adjudication; MJLab normally checks
        # termination on derived state from one physics substep earlier.
        self._env.sim.forward()
        self._env.sim.sense()
        self.check_substep()
        self.tcp[:] = self._entity.data.site_pos_w[:, self.site] - self._env.scene.env_origins
        self.velocity[:] = self._entity.data.site_lin_vel_w[:, self.site]
        self.error[:] = (self.target-self.tcp).norm(dim=-1)
        self.speed[:] = self.velocity.norm(dim=-1)
        qualified = ((self.error <= POSITION_TOL) & (self.speed <= SPEED_TOL)
                     & (self._entity.data.joint_vel.abs().amax(-1) <= JOINT_SPEED_TOL) & ~self.unsafe)
        self.hold[:] = advance_hold(self.hold, qualified)
        self.success[:] = (self.hold >= HOLD_STEPS) & ~self.unsafe


def state(env):
    return env.action_manager.get_term("reach")


def observation(env):
    s = state(env)
    data = env.scene["robot"].data
    tcp = data.site_pos_w[:, s.site] - env.scene.env_origins
    vel = data.site_lin_vel_w[:, s.site]
    return torch.cat((data.joint_pos, data.joint_vel, s.target-tcp, vel,
        s.command-data.joint_pos, s.raw_action, s.target, (s.hold.float()/HOLD_STEPS).unsqueeze(-1)), dim=-1)


def failure(env):
    s = state(env)
    s.update()
    return s.unsafe


def success(env):
    s = state(env)
    s.update()
    return s.success


def timeout(env):
    # A genuine terminal success/failure must not bootstrap as a timeout.
    s = state(env)
    return time_out(env) & ~s.success & ~s.unsafe


def reward(env):
    s = state(env)
    # Manager multiplies by dt; terminal bonuses compensate for that scaling.
    return (2*torch.exp(-s.error/.10) + 3*torch.exp(-s.error/.02)
        + (s.hold > 0).float()*2 - .01*env.scene["robot"].data.joint_vel.square().sum(-1)
        - .02*s.action_change + s.success.float()*5/DT - s.unsafe.float()*5/DT)


def substep_violation(env):
    return state(env).check_substep()


def error_metric(env):
    return state(env).error


def success_metric(env):
    return state(env).success.float()


def add_goal(spec):
    goal = spec.worldbody.add_body(name="reach_goal", mocap=True, pos=[0., 0., 1.])
    goal.add_geom(name="goal_marker", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[.01, 0., 0.],
                  contype=0, conaffinity=0, rgba=[0., 1., 0., .5])


def make_env_cfg(play=False):
    robot = EntityCfg(spec_fn=model_spec,
        articulation=EntityArticulationInfoCfg(actuators=(XmlActuatorCfg(target_names_expr=("joint[1-6]",)),)),
        init_state=EntityCfg.InitialStateCfg(joint_pos={f"joint{i+1}": float(q) for i, q in enumerate(HOME)}, joint_vel={".*": 0.}))
    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(num_envs=1 if play else 256, env_spacing=2., entities={"robot": robot}, spec_fn=add_goal,
            sensors=(ContactSensorCfg(name="contacts", primary=ContactMatch(mode="body", pattern=".*", entity="robot", exclude=("reach_goal",)), fields=("found",)),)),
        observations={name: ObservationGroupCfg({"state": ObservationTermCfg(func=observation)}, enable_corruption=False) for name in ("actor", "critic")},
        actions={"reach": ReachActionCfg(entity_name="robot", split="eval" if play else "train")},
        rewards={"reach_hold": RewardTermCfg(func=reward, weight=1.)},
        terminations={"violation": TerminationTermCfg(func=failure),
                      "success": TerminationTermCfg(func=success),
                      "time_out": TerminationTermCfg(func=timeout, time_out=True)},
        metrics={"success": MetricsTermCfg(func=success_metric, reduce="last"),
                 "tcp_error_m": MetricsTermCfg(func=error_metric),
                 "violation": MetricsTermCfg(func=substep_violation, per_substep=True, reduce="max")},
        sim=SimulationCfg(mujoco=MujocoCfg(timestep=PHYSICS_DT, integrator="implicitfast")),
        viewer=ViewerConfig(origin_type=ViewerConfig.OriginType.ASSET_BODY, entity_name="robot", body_name="base_link", distance=1.5),
        decimation=10, episode_length_s=EPISODE_STEPS*DT)
