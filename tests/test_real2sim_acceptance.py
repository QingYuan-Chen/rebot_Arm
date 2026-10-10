from __future__ import annotations

import io
import json
from types import SimpleNamespace

from rebotarm_simulation.diagnostics import real2sim_acceptance


HOME = [0.0, -0.8, -1.0, 0.3, 0.0, 0.0]


class FakeSimulation:
    def __init__(self, model=None):
        self.model = model
        self.positions = HOME.copy()
        self.targets = HOME.copy()
        self.width = 0.06
        self.time = 0.0

    def reset_home(self):
        self.positions = HOME.copy()
        return self.get_state()

    def mirror_joint_state(self, positions, velocities, *, gripper_width=None):
        self.positions[:] = positions
        if gripper_width is not None:
            self.width = gripper_width
        return self.get_state()

    def set_joint_position_targets(self, positions):
        self.targets[:] = positions

    def set_gripper_width(self, width):
        self.width = width

    def step(self, count):
        self.time += int(count) * 0.002
        self.positions[:] = [
            current + 0.8 * (target - current)
            for current, target in zip(self.positions, self.targets)
        ]
        return self.get_state()

    def get_state(self):
        return SimpleNamespace(
            joint_positions=tuple(self.positions) + (self.width / 2.0, -self.width / 2.0),
            gripper_width=self.width,
            simulation_time=self.time,
        )

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def test_no_hardware_acceptance_main_reports_mirror_and_physics_results():
    for mode in ("mirror", "physics"):
        output = io.StringIO()
        code = real2sim_acceptance.main(
            ["--mode", mode, "--steps", "20"],
            sim_factory=FakeSimulation,
            stdout=output,
        )
        payload = json.loads(output.getvalue())

        assert code == 0
        assert payload["ok"] is True
        assert payload["hardware_connected"] is False
        assert payload["mode"] == mode
        assert payload["source"] == "synthetic_joint_state"
