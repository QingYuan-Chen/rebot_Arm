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
        ROOT / "src" / "rebotarm_bringup" / "launch" / "visual_grasp_system.launch.py"
    ).read_text(encoding="utf-8")

    assert 'package="rebotarm_motion_execution"' not in launch_text
    assert 'package="rebotarm_motion"' in launch_text
    assert 'executable="PoseExecutionNode"' in launch_text


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
    text = (ROOT / "src" / "rebotarm_bringup" / "launch" / "visual_grasp_system.launch.py").read_text(encoding="utf-8")

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
        assert f'DeclareLaunchArgument("{name}"' in text
    assert "**policy_defaults" in text

    for name in (
        "safe_retreat_enabled",
        "safe_retreat_distance_m",
        "safe_home_after_grasp",
    ):
        assert f'DeclareLaunchArgument("{name}"' in text
        assert f'"{name}": {name}' in text

    for retired_name in (
        "lift_z_m",
        "candidate_safe_lift_min_z_m",
        "safe_retreat_min_lift_z_m",
        "safe_retreat_axis_xyz",
        "visual_lift_check_enabled",
        "visual_lift_min_delta_m",
    ):
        assert retired_name not in text
