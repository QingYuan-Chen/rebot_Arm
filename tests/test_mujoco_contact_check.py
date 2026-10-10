from __future__ import annotations

import io
import json
from types import MappingProxyType, SimpleNamespace

import numpy as np
import pytest

from rebotarm_simulation.diagnostics import mujoco_contact_check


class FakeContactSim:
    def __init__(self, model_path=None):
        self.model_path = model_path
        self.closed = False
        self.width = 0.09
        self.cube_pose = (0.31, 0.04, 0.04, 0.0, 0.0, 0.0, 1.0)

    def reset_home(self):
        return self.get_state()

    def set_gripper_width(self, width):
        self.width = float(width)
        return self.width

    def set_joint_position_targets(self, targets):
        self.targets = tuple(targets)
        return self.targets

    def set_object_pose(self, body_name, position, orientation):
        self.cube_pose = tuple(position) + tuple(orientation)
        return self.cube_pose

    def step(self, n_steps=1):
        x, y, z, qx, qy, qz, qw = self.cube_pose
        self.cube_pose = (x, y + 0.00001, max(z - 0.00001, 0.03), qx, qy, qz, qw)
        return self.get_state()

    def get_contacts(self):
        return (
            SimpleNamespace(body1="test_cube", body2="left_finger_link", force=0.6),
            SimpleNamespace(body1="table", body2="test_cube", force=0.2),
        )

    def get_state(self):
        return SimpleNamespace(
            object_poses=MappingProxyType({"test_cube": self.cube_pose}),
            gripper_width=self.width,
        )

    def close(self):
        self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def test_contact_check_outputs_json_summary_for_stable_fake_sim():
    output = io.StringIO()
    code = mujoco_contact_check.main(
        ["--settle-steps", "1", "--contact-steps", "4", "--min-finger-contact-steps", "3"],
        sim_factory=FakeContactSim,
        stdout=output,
    )

    assert code == 0
    payload = json.loads(output.getvalue())
    assert payload["ok"] is True
    assert payload["finger_contact_steps"] == 4
    assert payload["max_finger_force_n"] == pytest.approx(0.6)
    assert payload["max_cube_contact_force_n"] == pytest.approx(0.6)


def test_contact_check_runtime_keeps_cube_contact_stable():
    pytest.importorskip("mujoco")
    payload = mujoco_contact_check.run_contact_check(
        model=None,
        settle_steps=3000,
        contact_steps=600,
        min_finger_contact_steps=100,
        max_contact_force=20.0,
        max_cube_jump=0.005,
        min_cube_z=0.018,
    )

    assert payload["ok"] is True
    assert payload["finger_contact_steps"] >= 100
    assert payload["max_cube_contact_force_n"] <= 20.0
    assert payload["max_cube_step_jump_m"] <= 0.005
    assert payload["min_cube_z_m"] >= 0.018
    assert np.isfinite(payload["final_cube_pose"]).all()
