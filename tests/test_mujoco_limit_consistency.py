from __future__ import annotations

from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_mujoco_motor_ctrlranges_match_urdf_position_limits():
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import motor_profile_position_limit_mismatches
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import load_canonical_motor_profiles

    mismatches = motor_profile_position_limit_mismatches(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf",
        load_canonical_motor_profiles(),
    )

    assert mismatches == []


def test_limit_check_reports_drifted_motor_ctrlrange():
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import motor_profile_position_limit_mismatches
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import MotorProfile

    profiles = [
        MotorProfile("joint1", "-2.7 2.8", "-27 27", "270", "24"),
    ]

    mismatches = motor_profile_position_limit_mismatches(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf",
        profiles,
    )

    assert len(mismatches) == 1
    assert mismatches[0].joint == "joint1"
    assert mismatches[0].urdf_lower == pytest.approx(-2.8)
    assert mismatches[0].mujoco_lower == pytest.approx(-2.7)


def test_urdf_effort_velocity_limits_are_exposed_for_cross_layer_checks():
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import load_urdf_joint_limits

    limits = load_urdf_joint_limits(ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf")

    assert limits["joint1"].effort == pytest.approx(27.0)
    assert limits["joint1"].velocity == pytest.approx(50.0)
    assert limits["joint4"].effort == pytest.approx(7.0)
    assert limits["joint4"].velocity == pytest.approx(200.0)


def test_moveit_runtime_limits_are_conservative_and_complete_for_arm_joints():
    from rebotarm_motion.trajectory_runtime_limits import load_joint_runtime_limits
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import load_urdf_joint_limits

    joint_names = [f"joint{index}" for index in range(1, 7)]
    urdf_limits = load_urdf_joint_limits(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )
    runtime_limits = load_joint_runtime_limits(
        ROOT / "src/rebotarm_moveit_config/config/joint_limits.yaml",
        joint_names,
    )

    for name in joint_names:
        runtime = runtime_limits[name]
        assert runtime.max_velocity is not None
        assert runtime.max_velocity <= urdf_limits[name].velocity
        assert runtime.max_acceleration is not None and runtime.max_acceleration > 0.0
        assert runtime.max_jerk is not None and runtime.max_jerk > 0.0


def test_mujoco_arm_force_limits_match_urdf_effort_limits(tmp_path: Path):
    pytest.importorskip("mujoco")
    import mujoco
    from rebotarm_simulation.diagnostics.mujoco_limit_checks import load_urdf_joint_limits
    from rebotarm_simulation.core.resource_paths import package_resource

    model = mujoco.MjModel.from_xml_path(str(package_resource("rebotarm_simulation", "models/rebotarm/robot.xml")))
    urdf_limits = load_urdf_joint_limits(
        ROOT / "src/rebotarm_moveit_config/config/rebotarm.urdf"
    )

    assert [max(abs(x) for x in model.actuator(f"joint{i}_torque").forcerange) for i in range(1, 7)] == pytest.approx(
        [urdf_limits[f"joint{index}"].effort for index in range(1, 7)]
    )
