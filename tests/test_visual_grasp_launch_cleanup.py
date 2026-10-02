from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _registered_interfaces() -> list[str]:
    cmake_text = (ROOT / "src" / "rebotarm_msgs" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    interfaces: list[str] = []
    for line in cmake_text.splitlines():
        stripped = line.strip()
        if stripped.startswith('"') and stripped.endswith('"'):
            value = stripped.strip('"')
            if value.startswith(("msg/", "srv/", "action/")):
                interfaces.append(value)
    return interfaces


def test_rebotarm_msgs_registered_interfaces_exist():
    missing = [
        interface
        for interface in _registered_interfaces()
        if not (ROOT / "src" / "rebotarm_msgs" / interface).is_file()
    ]

    assert missing == []


def test_visual_grasp_launch_uses_existing_motion_execution_package():
    launch_text = (
        ROOT / "src" / "rebotarm_bringup" / "launch" / "includes" / "motion_execution.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="rebotarm_motion_execution"' not in launch_text
    assert 'package="rebotarm_motion"' in launch_text
    assert 'executable="PoseExecutionNode"' in launch_text


def test_visual_grasp_system_is_a_composition_entry_with_stage_includes():
    launch_dir = ROOT / "src" / "rebotarm_bringup" / "launch"
    main = (launch_dir / "visual_grasp_system.launch.py").read_text(encoding="utf-8")
    assert len(main.splitlines()) < 300
    assert '"visual_backend", "visual_lifecycle"' in main
    for stage in (
        "visual_input",
        "grasp_candidate",
        "candidate_filter",
        "motion_execution",
        "grasp_executor",
    ):
        assert (launch_dir / "includes" / f"{stage}.launch.py").exists()
    assert (ROOT / "src" / "rebotarm_bringup" / "config" / "visual_grasp_interfaces.yaml").exists()
    assert '"visual_interfaces_config"' in main


def test_compact_visual_entry_exposes_only_composition_parameters():
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "visual_grasp_compact.launch.py").read_text(
        encoding="utf-8"
    )
    assert "_system_module" in text
    assert "_prepare_stages" in text
    assert "candidate_min_confidence" not in text
    assert "gripper_grasp_close_force" not in text


def test_interface_profile_is_documented_as_the_authoritative_connection_contract():
    docs = (ROOT / "docs" / "reference" / "parameters" / "current_parameter_sources.md").read_text(
        encoding="utf-8"
    )
    assert "visual_grasp_interfaces.yaml" in docs
    assert "兼容入口只允许转发" in docs


def test_visual_ready_failure_closes_pipeline_without_disabling_hardware():
    node = (ROOT / "src" / "rebotarm_motion" / "rebotarm_motion" / "visual_ready_node.py").read_text(
        encoding="utf-8"
    )
    lifecycle = (ROOT / "src" / "rebotarm_bringup" / "launch" / "includes" / "visual_lifecycle.launch.py").read_text(
        encoding="utf-8"
    )
    assert "sys.exit(1)" in node
    assert "event.returncode == 0" in lifecycle
    assert "pipeline remains closed" in lifecycle
    assert "/rebotarm/disable" not in lifecycle


def test_vision_launch_does_not_wrap_static_tf_in_ros2_run():
    text = (ROOT / "src" / "rebotarm_vision" / "launch" / "vision.launch.py").read_text(encoding="utf-8")

    assert "exec ros2 run tf2_ros static_transform_publisher" not in text
    assert "package=\"tf2_ros\"" in text
    assert "executable=\"static_transform_publisher\"" in text


def test_vision_launch_configures_opencv_qt_font_environment():
    text = (ROOT / "src" / "rebotarm_vision" / "launch" / "vision.launch.py").read_text(encoding="utf-8")

    assert '"QT_QPA_PLATFORM": "xcb"' in text
    assert '"QT_QPA_FONTDIR": "/usr/share/fonts/truetype/dejavu"' in text


def test_vision_launch_has_no_machine_specific_home_paths():
    text = (ROOT / "src" / "rebotarm_vision" / "launch" / "vision.launch.py").read_text(
        encoding="utf-8"
    )

    assert "/home/u24" not in text
    assert "camera_config" in text
    assert "handeye_config" in text
    assert "start_ordinary_grasp" not in text


def test_interactive_launch_removes_legacy_start_interaction_nodes_flag():
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "interactive_system.launch.py").read_text(encoding="utf-8")

    assert "start_interaction_nodes" not in text
    assert "PreviewNode" not in text
    assert "ExecutionNode" not in text


def test_visual_grasp_launch_exposes_adaptive_gripper_and_retreat_params():
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "includes" / "grasp_executor.launch.py").read_text(encoding="utf-8")

    gripper_names = (
        "auto_gripper_effort",
        "min_gripper_effort",
        "max_gripper_effort",
        "max_allowed_grasp_width_m",
        "gripper_grasp_enabled",
        "gripper_grasp_close_force",
        "gripper_grasp_timeout_sec",
        "gripper_grasp_min_close_time_sec",
        "gripper_grasp_velocity_threshold",
        "gripper_grasp_min_closure_distance_m",
    )
    for name in gripper_names:
        assert name in text
    assert "**policy_defaults" in text

    for name in (
        "safe_retreat_enabled",
        "safe_retreat_distance_m",
        "safe_home_after_grasp",
    ):
        assert f'LaunchConfiguration("{name}")' in text
        assert f'LaunchConfiguration("{name}")' in text

    for retired_name in (
        "lift_z_m",
        "candidate_safe_lift_min_z_m",
        "safe_retreat_min_lift_z_m",
        "safe_retreat_axis_xyz",
        "visual_lift_check_enabled",
        "visual_lift_min_delta_m",
    ):
        assert retired_name not in text
