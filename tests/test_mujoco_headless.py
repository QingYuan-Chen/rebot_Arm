from __future__ import annotations

from importlib.util import find_spec
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


pytestmark = pytest.mark.skipif(find_spec("mujoco") is None, reason="mujoco is not installed")


def test_canonical_robot_model_loads_and_steps():
    from rebotarm_simulation.core.resource_paths import package_resource
    from rebotarm_simulation.diagnostics.mujoco_runner import run_smoke

    result = run_smoke(package_resource("rebotarm_simulation", "models/rebotarm/robot.xml"), seconds=1.0)

    assert result.nq == 8
    assert result.nu == 8
    assert result.finite


def test_canonical_step_response_is_force_limited():
    from rebotarm_simulation.core.resource_paths import package_resource
    from rebotarm_simulation.diagnostics.mujoco_runner import run_step_response

    result = run_step_response(package_resource("rebotarm_simulation", "models/rebotarm/robot.xml"), joint="joint1", target=0.05, seconds=1.0)

    assert result.max_abs_actuator_force <= 27.0 + 1e-6
    assert result.final_abs_error < 0.015
    assert result.final_position > 0.03


def test_canonical_grasp_scene_loads_and_steps(bottle_scene):
    from rebotarm_simulation.core.resource_paths import package_resource
    from rebotarm_simulation.diagnostics.mujoco_runner import run_grasp_benchmark

    result = run_grasp_benchmark(bottle_scene, target_body="bottle", gripper_bodies=("left_finger_link", "right_finger_link"), command=lambda sim, elapsed: None, seconds=0.1)

    assert result.finite
    assert result.target_body == "bottle"
    assert not result.grasp_success
    assert result.max_contacts >= result.final_contacts
    assert not result.lift_detected
