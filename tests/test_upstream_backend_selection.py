from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_active_package_installs_direct_upstream_node_only():
    setup_text = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")

    assert "rebotarm_mujoco_node = rebotarm_simulation.ros.mujoco_ros_node:main" in setup_text
    assert "rebotarm_upstream_mujoco_node = rebotarm_simulation.upstream_backend:main" not in setup_text
    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" not in setup_text


def test_mujoco_moveit_launch_has_no_selectable_current_backend():
    launch_text = (
        ROOT / "src/rebotarm_bringup/launch/mujoco_moveit_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in launch_text
    assert "simulation_backend" not in launch_text
    assert "rebotarm_upstream_mujoco_node" not in launch_text
    assert "rebotarm_mujoco_adapter" not in launch_text
    assert "IfCondition" not in launch_text
    assert '"use_fake_joint_states": "false"' in launch_text


def test_active_launch_uses_package_owned_upstream_model_not_snapshot_process_boundary():
    launch_text = (
        ROOT / "src/rebotarm_bringup/launch/mujoco_moveit_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert 'FindPackageShare("rebotarm_simulation")' in launch_text
    assert '"models", "rebotarm", "scene.xml"' in launch_text
    assert "robotarm_ros2_mujoco_snapshot" not in launch_text
    assert "REBOTARM_UPSTREAM_SNAPSHOT" not in launch_text
