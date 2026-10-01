from __future__ import annotations

import ast
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
VISION_ROOT = ROOT / "src/rebotarm_vision"
FEATURE_PATH = VISION_ROOT / "rebotarm_vision/single_bottle_grasp.py"
PROFILE_PATH = VISION_ROOT / "config/single_bottle_grasp.yaml"


def test_single_bottle_grasp_is_an_installed_vision_feature() -> None:
    setup_text = (VISION_ROOT / "setup.py").read_text(encoding="utf-8")

    assert (
        "rebotarm_single_bottle_grasp = "
        "rebotarm_vision.single_bottle_grasp:main"
    ) in setup_text
    assert '"config/single_bottle_grasp.yaml"' in setup_text
    assert FEATURE_PATH.is_file()
    assert PROFILE_PATH.is_file()


def test_single_bottle_grasp_does_not_import_task_tools() -> None:
    tree = ast.parse(FEATURE_PATH.read_text(encoding="utf-8"))
    imported_modules = {
        node.module
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module is not None
    }

    assert "p5_paired_trajectory_runner" not in imported_modules
    assert "p6_single_bottle_grasp_runner" not in imported_modules
    assert "rebotarm_motion.guarded_trajectory_client" in imported_modules


def test_installed_feature_profile_keeps_real_accepted_parameters() -> None:
    payload = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))

    assert payload["schema_version"] == 1
    assert payload["feature_arguments"] == {
        "namespace": "rebotarm",
        "runs": 1,
        "plan_timeout_sec": 45.0,
        "max_plan_age_sec": 1.5,
        "min_grasp_z_m": 0.03,
        "plan_stability_frames": 3,
        "plan_stability_xy_tolerance_m": 0.015,
        "plan_stability_z_tolerance_m": 0.010,
        "plan_stability_jaw_width_tolerance_m": 0.010,
        "pregrasp_sec": 10.0,
        "approach_sec": 3.0,
        "hold_sec": 3.0,
        "return_sec": 10.0,
        "gripper_open_m": 0.080,
        "gripper_open_max_effort": 1.5,
        "grasp_close_force": 1.0,
        "grasp_hold_force": 0.4,
        "grasp_close_timeout_sec": 4.0,
        "controller_grasp_hold_timeout_sec": 30.0,
    }
