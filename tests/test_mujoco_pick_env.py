from __future__ import annotations

import io
import json
from pathlib import Path
from types import MappingProxyType, SimpleNamespace

import numpy as np
import pytest

from rebotarm_simulation.diagnostics import mujoco_pick_batch
from rebotarm_simulation.core.mujoco_pick_env import PickEnvConfig, RebotArmPickEnv
from rebotarm_simulation.core.mujoco_types import ContactInfo, RandomizedScene
from rebotarm_simulation.core.pick_task import (
    PickTaskConfig,
    pick_failure_reason,
    summarize_cube_contacts,
)


def _contact(body2, force=1.0, penetration=0.001, normal=None):
    if normal is None:
        normal = (0.0, -1.0, 0.0) if "left_finger" in body2 else (0.0, 1.0, 0.0)
    return ContactInfo(
        "test_cube",
        body2,
        "test_cube_geom",
        f"{body2}_geom",
        (0.0, 0.0, 0.02),
        force,
        penetration,
        normal,
    )


class FakePickSim:
    def __init__(self, model_path=None, *, scenarios=None):
        self.model_path = model_path
        self.arm_joint_limits = ((-2.0, 2.0),) * 6
        self.arm_actuator_force_limits = (20.0,) * 6
        self.targets = np.zeros(8, dtype=float)
        self.positions = np.zeros(8, dtype=float)
        self.width = 0.08
        self.ee = np.array([0.2, 0.0, 0.1], dtype=float)
        self.cube_pose = [0.28, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0]
        self.scenarios = list(scenarios or [])
        self.scenario_index = -1
        self.contacts = []
        self.time = 0.0
        self.randomization_sample = None

    @property
    def control_targets(self):
        return tuple(self.targets[:6]) + (self.width / 2.0, -self.width / 2.0)

    def reset_home(self, seed=None):
        self.positions[:] = 0.0
        self.targets[:] = 0.0
        self.width = 0.08
        self.time = 0.0
        self.scenario_index = -1
        self.contacts = []
        return self.get_state()

    def reset(self, seed=None):
        return self.reset_home(seed)

    def randomize_scene(self, seed=None):
        self.cube_pose[:] = [0.28, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0]
        return RandomizedScene(tuple(self.cube_pose), (0.28, 0.0, 0.12), seed)

    def set_joint_position_targets(self, targets):
        self.targets[:6] = np.asarray(targets)

    def set_gripper_width(self, width):
        self.width = min(0.09, max(0.0, float(width)))

    def step(self, n_steps=1):
        self.positions[:6] = self.targets[:6]
        self.time += 0.002 * int(n_steps)
        if self.scenario_index + 1 < len(self.scenarios):
            self.scenario_index += 1
            scenario = self.scenarios[self.scenario_index]
            self.cube_pose[2] = scenario.get("cube_z", self.cube_pose[2])
            self.contacts = list(scenario.get("contacts", []))
        return self.get_state()

    def get_contacts(self):
        return tuple(self.contacts)

    def get_state(self):
        self.positions[-2:] = (self.width / 2.0, -self.width / 2.0)
        return SimpleNamespace(
            joint_positions=tuple(self.positions),
            joint_velocities=(0.0,) * 8,
            actuator_forces=(0.0,) * 8,
            end_effector_position=tuple(self.ee),
            end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
            gripper_width=self.width,
            object_poses=MappingProxyType({"test_cube": tuple(self.cube_pose)}),
            simulation_time=self.time,
        )

    def apply_randomization(self, sample):
        self.randomization_sample = sample

    def restore_randomization(self):
        self.randomization_sample = None

    def close(self):
        pass


def test_cube_contact_summary_distinguishes_both_fingers_and_table():
    summary = summarize_cube_contacts(
        [_contact("left_finger_link", 2.0), _contact("right_finger_link", 3.0), _contact("table", 4.0)]
    )

    assert summary.bilateral_finger_contact is True
    assert summary.left_finger_force_n == 2.0
    assert summary.right_finger_force_n == 3.0
    assert summary.table_contact is True
    assert summary.max_contact_force_n == 4.0
    assert summary.finger_normal_dot == pytest.approx(-1.0)
    assert summary.force_closure_candidate(-0.3) is True


def test_same_direction_finger_contacts_are_not_force_closure():
    summary = summarize_cube_contacts(
        [
            _contact("left_finger_link", normal=(0.0, 1.0, 0.0)),
            _contact("right_finger_link", normal=(0.0, 1.0, 0.0)),
        ]
    )

    assert summary.bilateral_finger_contact is True
    assert summary.force_closure_candidate(-0.3) is False


@pytest.mark.parametrize(
    ("changes", "expected"),
    [
        ({"contacts": [_contact("left_finger_link", 21.0)]}, "excessive_contact_force"),
        ({"contacts": [_contact("left_finger_link", penetration=0.02)]}, "excessive_penetration"),
        ({"cube_position": (0.2, 0.0, -0.03)}, "cube_fell"),
        ({"cube_position": (0.8, 0.0, 0.02)}, "cube_out_of_workspace"),
        ({"ever_grasped": True, "lost_grasp_steps": 10}, "dropped_after_grasp"),
    ],
)
def test_pick_failure_taxonomy(changes, expected):
    values = {
        "config": PickTaskConfig(),
        "cube_position": (0.2, 0.0, 0.02),
        "contacts": summarize_cube_contacts([]),
        "ever_grasped": False,
        "lost_grasp_steps": 0,
    }
    values.update(changes)
    if isinstance(values["contacts"], list):
        values["contacts"] = summarize_cube_contacts(values["contacts"])

    assert pick_failure_reason(**values) == expected


def test_pick_env_requires_stable_bilateral_grasp_and_lift_for_success():
    both = [_contact("left_finger_link"), _contact("right_finger_link")]
    sim = FakePickSim(
        scenarios=[
            {"cube_z": 0.02, "contacts": both},
            {"cube_z": 0.02, "contacts": both},
            {"cube_z": 0.08, "contacts": both},
            {"cube_z": 0.08, "contacts": both},
        ]
    )
    config = PickEnvConfig(
        settle_steps=0,
        task=PickTaskConfig(
            grasp_stability_steps=2,
            success_stability_steps=2,
            max_grasp_width_m=0.09,
        ),
    )
    with RebotArmPickEnv(config=config, sim_factory=lambda _model: sim) as env:
        obs, info = env.reset(seed=7)
        assert obs["cube_to_ee"].shape == (3,)
        assert obs["lift_target_position"] == pytest.approx((0.28, 0.0, 0.07))
        assert info["stage"] == "approach"

        outcomes = [env.step([0.0] * 7) for _ in range(4)]

    assert outcomes[0][4]["stage"] == "grasp"
    assert outcomes[1][4]["stage"] == "lift"
    assert outcomes[2][2] is False
    assert outcomes[3][2] is True
    assert outcomes[3][4]["is_success"] is True
    assert outcomes[3][4]["stage"] == "success"


def test_pick_env_terminates_after_grasp_is_lost():
    both = [_contact("left_finger_link"), _contact("right_finger_link")]
    sim = FakePickSim(scenarios=[{"contacts": both}, {}, {}])
    config = PickEnvConfig(
        settle_steps=0,
        task=PickTaskConfig(
            grasp_stability_steps=1,
            drop_patience_steps=2,
            max_grasp_width_m=0.09,
        ),
    )
    with RebotArmPickEnv(config=config, sim_factory=lambda _model: sim) as env:
        env.reset(seed=0)
        env.step([0.0] * 7)
        env.step([0.0] * 7)
        _obs, _reward, terminated, _truncated, info = env.step([0.0] * 7)

    assert terminated is True
    assert info["failure_reason"] == "dropped_after_grasp"
    assert info["stage"] == "failure"


def test_pick_batch_reports_runtime_safety_separately_from_success(tmp_path):
    output = io.StringIO()
    config_path = Path(__file__).parents[1] / "src/rebotarm_simulation/config/sim2real_randomization.yaml"
    env_factory = lambda model: RebotArmPickEnv(
        model,
        config=PickEnvConfig(settle_steps=0, max_steps=3),
        sim_factory=FakePickSim,
    )
    code = mujoco_pick_batch.main(
        [
            "--episodes", "2",
            "--steps", "3",
            "--seed", "4",
            "--randomization-config", str(config_path),
            "--log-dir", str(tmp_path),
        ],
        env_factory=env_factory,
        stdout=output,
    )

    payload = json.loads(output.getvalue())
    assert code == 0
    assert payload["ok"] is True
    assert payload["success_rate"] == 0.0
    assert payload["policy"] == "bounded_random_acceptance"
    assert payload["results"][0]["stage_counts"] == {"approach": 3}
    assert len(list(tmp_path.glob("*.jsonl"))) == 2
