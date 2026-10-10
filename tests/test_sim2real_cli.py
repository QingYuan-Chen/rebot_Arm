from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np

from rebotarm_simulation.apps import sim2real_cli
from rebotarm_simulation.sim2real.schemas import TrajectorySample


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/rebotarm_simulation/config/sim2real_randomization.yaml"


class FakeEnv:
    def __init__(self, model=None):
        self.model = model
        self.sim = self
        self.arm_joint_limits = ((-1.0, 1.0),) * 6
        self.arm_actuator_force_limits = (10.0,) * 6
        self.positions = np.zeros(6)
        self.step_index = 0
        self.seed = 0

    def reset(self, *, seed, randomization):
        self.positions[:] = 0.0
        self.step_index = 0
        self.seed = int(seed)
        self.randomization = randomization
        return {}, {}

    def step(self, action):
        self.positions += np.asarray(action[:6]) * 0.01
        self.step_index += 1
        return {}, 0.0, False, False, {}

    def sample_from_last_step(self, action, *, episode_id, step_index):
        return TrajectorySample(
            schema_version=1,
            episode_id=episode_id,
            step_index=step_index,
            simulation_time=self.step_index * 0.01,
            joint_positions=tuple(self.positions),
            joint_velocities=tuple(np.asarray(action[:6]) * 0.01),
            joint_targets=tuple(self.positions),
            actuator_torques=tuple(np.asarray(action[:6])),
            gripper_width=0.05,
            gripper_target_width=0.05,
            end_effector_position=(0.2 + self.positions[0], 0.0, 0.2),
            end_effector_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            action=tuple(action),
            max_contact_force=1.0,
            contact_count=1,
            source="sim",
            max_contact_penetration=0.001,
        )

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def _run(args):
    stdout = io.StringIO()
    stderr = io.StringIO()
    code = sim2real_cli.main(
        args, env_factory=FakeEnv, stdout=stdout, stderr=stderr
    )
    return code, json.loads(stdout.getvalue()) if stdout.getvalue() else None, stderr.getvalue()


def test_rollout_replay_compare_and_batch_workflow(tmp_path):
    reference = tmp_path / "reference.jsonl"
    candidate = tmp_path / "candidate.jsonl"
    rollout_report = tmp_path / "rollout-report.json"
    common = [
        "--randomization-config",
        str(CONFIG),
        "--randomization-profile",
        "training_profile",
        "--seed",
        "7",
    ]

    code, rollout, error = _run(
        ["rollout", *common, "--steps", "3", "--record", str(reference), "--report", str(rollout_report)]
    )
    assert (code, error) == (0, "")
    assert rollout["ok"] is True
    assert rollout["trajectory"]["sample_count"] == 3
    assert reference.is_file() and rollout_report.is_file()

    code, replay, error = _run(
        ["replay", str(reference), *common, "--record", str(candidate)]
    )
    assert (code, error) == (0, "")
    assert replay["comparison"]["ok"] is True

    code, comparison, error = _run(["compare", str(reference), str(candidate)])
    assert (code, error) == (0, "")
    assert comparison["sample_count"] == 3

    code, batch, error = _run(
        ["batch-check", *common, "--episodes", "2", "--steps", "3", "--log-dir", str(tmp_path / "logs")]
    )
    assert (code, error) == (0, "")
    assert batch["ok"] is True
    assert all(item["seed_reproducible"] for item in batch["results"])


def test_rollout_returns_acceptance_failure_for_contact_penetration(tmp_path):
    code, payload, error = _run(
        [
            "rollout",
            "--randomization-config",
            str(CONFIG),
            "--steps",
            "1",
            "--record",
            str(tmp_path / "trajectory.jsonl"),
            "--max-contact-penetration",
            "0.0001",
        ]
    )

    assert (code, error) == (2, "")
    assert payload["ok"] is False
    assert payload["safety"]["violations"][0]["kind"] == "contact_penetration"
