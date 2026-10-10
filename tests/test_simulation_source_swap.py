from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ACTIVE = ROOT / "src" / "rebotarm_simulation"


def test_active_source_contains_upstream_core_without_current_runtime_entrypoint():
    setup_text = (ACTIVE / "setup.py").read_text(encoding="utf-8")
    package_text = (ACTIVE / "package.xml").read_text(encoding="utf-8")

    assert (ACTIVE / "models/rebotarm/robot.xml").is_file()
    assert (ACTIVE / "models/rebotarm/scene.xml").is_file()
    assert (ACTIVE / "rebotarm_simulation/core/mujoco_sim.py").is_file()
    assert (ACTIVE / "rebotarm_simulation/ros/mujoco_ros_node.py").is_file()
    assert "rebotarm_mujoco_node = rebotarm_simulation.ros.mujoco_ros_node:main" in setup_text
    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" not in setup_text
    assert "rebotarm_upstream_mujoco_node = rebotarm_simulation.upstream_backend:main" not in setup_text
    assert "<exec_depend>ament_index_python</exec_depend>" in package_text


def test_moveit_sim_wrapper_is_upstream_only():
    launch_text = (ROOT / "src/rebotarm_bringup/launch/mujoco_moveit_sim.launch.py").read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in launch_text
    assert "simulation_backend" not in launch_text
    assert "rebotarm_mujoco_adapter" not in launch_text
    assert "current_condition" not in launch_text
    assert '"use_fake_joint_states": "false"' in launch_text
