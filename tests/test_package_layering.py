from __future__ import annotations

from pathlib import Path
import ast
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_simulation_has_no_motion_implementation_or_manifest_dependency() -> None:
    package = ROOT / "src/rebotarm_simulation"
    for source in (package / "rebotarm_simulation").rglob("*.py"):
        for node in ast.walk(ast.parse(source.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                modules = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and not node.level:
                modules = [node.module or ""]
            else:
                continue
            assert all(module.split(".")[0] != "rebotarm_motion" for module in modules), source
    manifest = ET.parse(package / "package.xml").getroot()
    assert not any(child.text == "rebotarm_motion" for child in manifest)


def test_retired_mujoco_ros_backends_are_absent_from_active_package() -> None:
    package = ROOT / "src/rebotarm_simulation/rebotarm_simulation"
    assert not (package / "mujoco_ros_adapter_node.py").exists()
    assert not (package / "upstream_backend.py").exists()


def test_motion_package_exports_core_modules() -> None:
    import rebotarm_motion.collision_precheck as collision_precheck
    import rebotarm_motion.replay_runtime_monitor as replay_runtime_monitor
    import rebotarm_motion.trajectory_safety_monitor as trajectory_safety_monitor
    import rebotarm_motion.trajectory_time_parameterization as trajectory_time_parameterization

    assert hasattr(collision_precheck, "CollisionPrechecker")
    assert hasattr(replay_runtime_monitor, "ReplayRuntimeMonitor")
    assert hasattr(trajectory_safety_monitor, "evaluate_replay_tracking")
    assert hasattr(trajectory_time_parameterization, "parameterize_teach_samples")


def test_teach_package_exports_core_modules() -> None:
    import rebotarm_teach.teach_recording as teach_recording
    import rebotarm_teach.teach_replay_coordinator as teach_replay_coordinator
    import rebotarm_teach.teach_replay_settings as teach_replay_settings

    assert hasattr(teach_recording, "TeachSample")
    assert hasattr(teach_replay_coordinator, "TeachReplayCoordinator")
    assert hasattr(teach_replay_settings, "TeachReplaySettingsProvider")


def test_teleop_package_exports_command_adapters() -> None:
    import rebotarm_teleop.teleop_core as teleop_core
    import rebotarm_teleop.web_execute as web_execute
    import rebotarm_teleop.web_teleop_client as web_teleop_client

    assert hasattr(teleop_core, "validate_web_keyboard_command")
    assert hasattr(web_execute, "validate_web_execute_request")
    assert hasattr(web_teleop_client, "WebTeleopClient")


def test_dashboard_package_exports_status_panel_modules() -> None:
    import rebotarm_dashboard.status_panel_api as status_panel_api
    import rebotarm_dashboard.status_panel_http as status_panel_http
    import rebotarm_dashboard.status_panel_state as status_panel_state

    assert hasattr(status_panel_api, "dispatch_post_request")
    assert hasattr(status_panel_http, "create_status_panel_server")
    assert hasattr(status_panel_state, "TeleopStatusStore")


def test_retired_interactive_control_package_is_absent() -> None:
    assert not (ROOT / "src/rebotarm_interactive_control").exists()


def test_layered_packages_do_not_reference_retired_interactive_control_package() -> None:
    package_roots = [
        ROOT / "src/rebotarm_calibration/rebotarm_calibration",
        ROOT / "src/rebotarm_dashboard/rebotarm_dashboard",
        ROOT / "src/rebotarm_motion/rebotarm_motion",
        ROOT / "src/rebotarm_teach/rebotarm_teach",
        ROOT / "src/rebotarm_teleop/rebotarm_teleop",
    ]
    sources = [
        source
        for package_root in package_roots
        for source in package_root.rglob("*.py")
    ]
    offenders = {
        source.name: source.read_text(encoding="utf-8")
        for source in sources
        if "rebotarm_interactive_control" in source.read_text(encoding="utf-8")
    }

    assert offenders == {}


def test_primary_bringup_launches_dashboard_package_directly() -> None:
    launch_files = [
        ROOT / "src/rebotarm_bringup/launch/rebotarm_app.launch.py",
        ROOT / "src/rebotarm_bringup/launch/teleop_system.launch.py",
    ]

    for launch_file in launch_files:
        text = launch_file.read_text(encoding="utf-8")
        panel_index = text.index('executable="TeleopStatusPanelNode"')
        package_context = text[max(0, panel_index - 120):panel_index]
        assert 'package="rebotarm_dashboard"' in package_context


def test_teach_launches_use_teach_package_directly() -> None:
    launch_expectations = [
        (ROOT / "src/rebotarm_bringup/launch/rebotarm_app.launch.py", "TeachRecorderNode"),
        (ROOT / "src/rebotarm_bringup/launch/teleop_system.launch.py", "TeachRecorderNode"),
    ]

    for launch_file, executable in launch_expectations:
        text = launch_file.read_text(encoding="utf-8")
        node_index = text.index(f'executable="{executable}"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teach"' in package_context


def test_keyboard_launches_use_teleop_package_directly() -> None:
    launch_expectations = [
        ROOT / "src/rebotarm_bringup/launch/teleop_keyboard.launch.py",
    ]

    for launch_file in launch_expectations:
        text = launch_file.read_text(encoding="utf-8")
        node_index = text.index('executable="TeleopKeyboardNode"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teleop"' in package_context


def test_bringup_has_one_real_hardware_controller_owner() -> None:
    launch_dir = ROOT / "src/rebotarm_bringup/launch"
    launch_files = sorted(launch_dir.glob("*.launch.py"))
    direct_owners = [
        path.name
        for path in launch_files
        if 'package="rebotarmcontroller"' in path.read_text(encoding="utf-8")
    ]

    assert direct_owners == ["hardware_controller.launch.py"]

    hardware_consumers = {
        "bringup.launch.py",
        "interactive_system.launch.py",
        "teleop_keyboard.launch.py",
    }
    for name in hardware_consumers:
        assert "hardware_controller.launch.py" in (launch_dir / name).read_text(
            encoding="utf-8"
        )
    moveit_hardware = (launch_dir / "moveit_hardware.launch.py").read_text(
        encoding="utf-8"
    )
    assert "interactive_system.launch.py" in moveit_hardware
    assert "hardware_controller.launch.py" not in moveit_hardware


def test_rviz_drag_launch_does_not_start_legacy_preview_execution_nodes() -> None:
    text = (ROOT / "src/rebotarm_bringup/launch/interactive_system.launch.py").read_text(
        encoding="utf-8"
    )

    assert "PreviewNode" not in text
    assert "ExecutionNode" not in text
    assert "MarkerServerNode" not in text
    assert "interactive_control/execute_preview" not in text


def test_visual_gripper_launches_use_teleop_package_directly() -> None:
    for launch_file in (ROOT / "src/rebotarm_bringup/launch").glob("*.launch.py"):
        text = launch_file.read_text(encoding="utf-8")
        if 'executable="GripperVisualJointStateNode"' not in text:
            continue
        node_index = text.index('executable="GripperVisualJointStateNode"')
        package_context = text[max(0, node_index - 120):node_index]
        assert 'package="rebotarm_teleop"' in package_context


def test_retired_interactive_control_config_is_not_loaded() -> None:
    config = ROOT / "src/rebotarm_bringup/config/interactive_control.yaml"
    launch = (ROOT / "src/rebotarm_bringup/launch/interactive_system.launch.py").read_text(
        encoding="utf-8"
    )

    assert not config.exists()
    assert "interactive_control.yaml" not in launch
    assert "interactive_config" not in launch
    assert 'parameters=[{"arm_namespace": arm_namespace}]' in launch


def test_architecture_document_defines_package_responsibilities_and_rules() -> None:
    architecture = (ROOT / "docs/implemented/architecture.md").read_text(encoding="utf-8")
    context = (ROOT / "docs/reference/context.md").read_text(encoding="utf-8")
    agents = (ROOT / "AGENTS.md").read_text(encoding="utf-8")

    required_terms = [
        "rebotarmcontroller",
        "rebotarm_motion",
        "rebotarm_teach",
        "rebotarm_teleop",
        "rebotarm_dashboard",
        "rebotarm_vision",
        "rebotarm_calibration",
        "Hardware ownership",
        "Motion ownership",
        "Operator interaction ownership",
    ]
    for term in required_terms:
        assert term in architecture

    assert "Teach Replay" in context
    assert "Point-to-Point Execution" in context
    assert "docs/implemented/architecture.md" in agents
    assert "rebotarm_interactive_control" not in architecture
    assert "rebotarm_interactive_control" not in context
    assert "rebotarm_interactive_control" not in agents


def test_rviz_moveit_drag_entrypoints_use_visible_native_goal_marker() -> None:
    feature_doc = (ROOT / "docs/reference/commands/rebotarm_feature_commands.md").read_text(
        encoding="utf-8"
    )
    rviz_config = (
        ROOT / "src/rebotarm_bringup/rviz/interactive_system.rviz"
    ).read_text(encoding="utf-8")
    sim_launch = (
        ROOT / "src/rebotarm_bringup/launch/rviz_ee_drag_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert "rviz_ee_drag_real.launch.py" in feature_doc
    assert "已移除" in feature_doc.split("rviz_ee_drag_real.launch.py", 1)[1][:20]
    assert "moveit_hardware.launch.py" in feature_doc
    assert "rviz_ee_drag_sim.launch.py" in feature_doc
    assert not (ROOT / "src/rebotarm_bringup/launch/rviz.launch.py").exists()
    assert '"use_moveit_preview": "true"' in sim_launch
    assert '"start_passive_joint_state_publisher": "false"' in sim_launch
    assert '"use_moveit_fake_joint_states": "false"' in sim_launch
    assert 'executable="rebotarm_sim_trajectory_controller"' in sim_launch
    assert "start_interaction_nodes" not in sim_launch
    assert "interactive_control/ee_target" not in feature_doc
    assert "EndEffectorTarget" not in rviz_config
    assert "InteractiveMarkers" not in rviz_config
    assert "/rebotarm/interactive_control/ee_target/update" not in rviz_config
    assert "Interactive Marker Size: 0.12" in rviz_config
    assert "  - Class: moveit_rviz_plugin/MotionPlanning" not in rviz_config.split(
        "Visualization Manager:", 1
    )[0]
    assert rviz_config.count("Show Robot Visual: false") >= 2
    assert "Goal State Alpha: 1" in rviz_config
    assert "Robot Alpha: 0" in rviz_config
    assert "MarkerServerNode" not in (
        ROOT / "src/rebotarm_teleop/setup.py"
    ).read_text(encoding="utf-8")
    assert "InteractiveTargetNode" not in (
        ROOT / "src/rebotarm_teleop/setup.py"
    ).read_text(encoding="utf-8")


def test_legacy_custom_interactive_preview_entrypoints_are_removed() -> None:
    motion_setup = (ROOT / "src/rebotarm_motion/setup.py").read_text(encoding="utf-8")
    console_lines = [line.strip().strip('",') for line in motion_setup.splitlines()]
    assert not any(line.startswith("PreviewNode =") for line in console_lines)
    assert not any(line.startswith("ExecutionNode =") for line in console_lines)
    assert "MarkerServerNode" not in motion_setup
    assert "InteractiveTargetNode" not in motion_setup

    assert not (ROOT / "src/rebotarm_motion/rebotarm_motion/preview_node.py").exists()
    assert not (ROOT / "src/rebotarm_motion/rebotarm_motion/execution_node.py").exists()
