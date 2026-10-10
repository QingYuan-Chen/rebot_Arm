from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_upstream_mujoco_entrypoint_and_launch_are_installed():
    setup_text = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")
    package_text = (ROOT / "src/rebotarm_simulation/package.xml").read_text(encoding="utf-8")

    assert "rebotarm_mujoco_node = rebotarm_simulation.ros.mujoco_ros_node:main" in setup_text
    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" not in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
    assert "<exec_depend>control_msgs</exec_depend>" in package_text
    assert "<exec_depend>rebotarm_msgs</exec_depend>" in package_text


def test_mujoco_has_two_entrypoints_with_explicit_window_controls():
    launch = ROOT / "src/rebotarm_bringup/launch"
    assert not (launch / "mujoco_headless.launch.py").exists()
    assert not (launch / "mujoco_rviz_viewer.launch.py").exists()
    text = (launch / "mujoco_moveit_sim.launch.py").read_text()
    assert 'DeclareLaunchArgument("use_rviz", default_value="true")' in text
    assert '"use_mujoco_viewer"' in text
    assert '"show_viewer": use_mujoco_viewer' in text
    assert '"use_rviz": use_rviz' in text


def test_mujoco_only_launch_accepts_namespace_and_initial_state_overrides():
    launch_text = (
        ROOT / "src/rebotarm_simulation/launch/mujoco_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("mujoco_arm_namespace", default_value="rebotarm")' in launch_text
    assert 'DeclareLaunchArgument(\n                "initial_joint_positions",' in launch_text
    assert '"arm_namespace": mujoco_arm_namespace' in launch_text
    assert '"initial_joint_positions": initial_joint_positions' in launch_text


def test_simulation_has_no_virtual_vision_publishers_or_launch_options():
    node_text = (ROOT / "src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py").read_text()
    launch_text = (ROOT / "src/rebotarm_simulation/launch/mujoco_sim.launch.py").read_text()
    assert "virtual_camera" not in node_text
    assert "virtual_camera" not in launch_text
    assert '"/camera/' not in node_text
    assert "StaticTransformBroadcaster" not in node_text
    assert not (ROOT / "src/rebotarm_simulation/rebotarm_simulation/virtual_camera.py").exists()


def test_mujoco_moveit_launch_starts_upstream_node_and_moveit_without_fake_joint_states():
    launch_text = (
        ROOT / "src/rebotarm_bringup/launch/mujoco_moveit_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in launch_text
    assert "rebotarm_mujoco_adapter" not in launch_text
    assert "simulation_backend" not in launch_text
    assert 'moveit_demo.launch.py' in launch_text
    assert '"use_fake_joint_states": "false"' in launch_text
    assert '"show_viewer": use_mujoco_viewer' in launch_text
    assert 'DeclareLaunchArgument(' in launch_text
    assert '"use_mujoco_viewer"' in launch_text
    assert 'DeclareLaunchArgument(' in launch_text
    assert '"python_executable"' in launch_text
    assert "prefix=python_executable" in launch_text


def test_physics_and_rviz_preview_launches_select_distinct_execution_backends():
    physics_launch = (
        ROOT / "src/rebotarm_bringup/launch/mujoco_moveit_sim.launch.py"
    ).read_text(encoding="utf-8")
    preview_launch = (
        ROOT / "src/rebotarm_bringup/launch/includes/visual_backend.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_node"' in physics_launch
    assert 'executable="rebotarm_sim_trajectory_controller"' not in physics_launch
    assert 'package="rebotarm_preview"' in preview_launch
    assert 'executable="rebotarm_sim_trajectory_controller"' in preview_launch
    assert 'executable="rebotarm_mujoco_node"' not in preview_launch


def test_upstream_mujoco_ros_node_owns_required_ros_interfaces():
    node_text = (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py"
    ).read_text(encoding="utf-8")

    assert "ActionServer(" in node_text
    assert "FollowJointTrajectory" in node_text
    assert '"/{self._arm_namespace}/joint_states"' in node_text
    assert '"/{self._arm_namespace}/follow_joint_trajectory"' in node_text
    assert '"/{self._arm_namespace}/trajectory_stop"' in node_text
    assert '"/{self._arm_namespace}/gripper/set"' in node_text
    assert '"/{self._arm_namespace}/gripper/state"' in node_text
    assert '"max_trajectory_points"' in node_text
    assert '"max_trajectory_duration_sec"' in node_text
    assert '"goal_position_tolerance"' in node_text
    assert '"goal_velocity_tolerance"' in node_text
    assert '"goal_time_tolerance_sec"' in node_text
    assert '"feedback_rate_hz"' in node_text


def test_active_mujoco_cli_does_not_dispatch_to_current_legacy_runtime():
    cli_text = (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/apps/mujoco_cli.py"
    ).read_text(encoding="utf-8")
    setup_text = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")

    assert "_dispatch_legacy_command" not in cli_text
    assert "mujoco_legacy_cli" not in cli_text
    assert "rebotarm_mujoco_legacy_cli" not in setup_text
    assert not (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/mujoco_legacy_cli.py"
    ).exists()
