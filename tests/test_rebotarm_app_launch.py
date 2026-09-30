from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_rebotarm_app_launch_exposes_simple_modes_and_profiles() -> None:
    launch_text = _read("src/rebotarm_bringup/launch/rebotarm_app.launch.py")

    assert 'DeclareLaunchArgument("use_hardware", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("web_execute_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("profile"' not in launch_text
    assert 'DeclareLaunchArgument("name", default_value="teach_record")' in launch_text
    assert "moveit_hardware.launch.py" in launch_text
    assert 'executable="TeachRecorderNode"' in launch_text
    assert 'executable="TeachReplayNode"' not in launch_text
    assert "_idle_recorder_node(" not in launch_text
    assert "Teach Trajectory card" in launch_text
    assert 'DeclareLaunchArgument("mode"' not in launch_text
    assert 'LaunchConfiguration("mode")' not in launch_text
    assert 'if mode ==' not in launch_text
    assert "mode must be one of" not in launch_text
    assert "teleop_system.launch.py" not in launch_text
    assert "def _as_bool" in launch_text
    assert '"use_hardware": _as_bool(use_hardware)' in launch_text
    assert '"web_execute_enabled": _as_bool(web_execute_enabled)' in launch_text
    assert '"panel_mode": panel_mode' in launch_text
    assert 'panel_mode="control"' in launch_text
    assert "full MoveIt + web teleop workbench" in launch_text
    assert 'DeclareLaunchArgument("channel", default_value="auto")' in launch_text
    assert 'DeclareLaunchArgument("execution_mode", default_value="execute")' in launch_text
    assert "def _resolve_channel" in launch_text
    assert 'for candidate in ("/dev/ttyACM0", "/dev/ttyACM1")' in launch_text
    assert '"use_rviz": "false"' in launch_text
    assert 'arguments=["-d", web_rviz_config]' in launch_text
    assert '"web_teleop_status.rviz"' in launch_text
    assert 'executable="TeleopKeyboardNode"' not in launch_text
    assert 'safe_name = os.path.basename(safe_name.replace("\\\\", "/")) or "teach_record"' in launch_text
    assert 'return f"teleop_records/{safe_name}"' in launch_text


def test_web_teleop_rviz_is_robot_status_only() -> None:
    rviz_text = _read("src/rebotarm_bringup/rviz/web_teleop_status.rviz")

    assert "rviz_default_plugins/RobotModel" in rviz_text
    assert "rviz_default_plugins/TF" in rviz_text
    assert "rviz_default_plugins/Interact" in rviz_text
    assert "rviz_default_plugins/MoveCamera" not in rviz_text
    assert "rviz_default_plugins/Select" not in rviz_text
    assert "moveit_rviz_plugin/MotionPlanning" not in rviz_text
    assert "InteractiveMarkers" not in rviz_text
    assert "EndEffectorTarget" not in rviz_text


def test_operator_configs_are_split_by_consumer() -> None:
    common = yaml.safe_load(_read("src/rebotarm_bringup/config/operator_common.yaml"))
    keyboard = yaml.safe_load(_read("src/rebotarm_bringup/config/keyboard_control.yaml"))
    web = yaml.safe_load(_read("src/rebotarm_bringup/config/web_teleop.yaml"))
    teach = yaml.safe_load(_read("src/rebotarm_bringup/config/teach_control.yaml"))

    common_params = common["/**"]["ros__parameters"]
    keyboard_params = keyboard["/**"]["ros__parameters"]
    web_params = web["/**"]["ros__parameters"]
    teach_params = teach["/**"]["ros__parameters"]

    assert "deadman_required" in keyboard_params
    assert "deadman_key" in keyboard_params
    assert not (set(keyboard_params) & set(teach_params))
    assert "web_execute_enabled" in web_params
    assert "web_keyboard_default_step_rad" in web_params
    assert "replay_monitor_enabled" in teach_params
    assert "collision_check_enabled" in teach_params
    assert "joint_names" in common_params
    assert "joint_lower_limits" in common_params

    assert "deadman_required" not in web_params
    assert "deadman_key" not in web_params
    assert "web_execute_enabled" not in teach_params

    assert teach_params["sample_rate_hz"] == 150.0
    assert teach_params["filter_sample_rate_hz"] == 150.0
    assert teach_params["resample_rate_hz"] == 150.0

    hardware_launch = _read(
        "src/rebotarm_bringup/launch/hardware_controller.launch.py"
    )
    assert 'DeclareLaunchArgument("joint_state_rate", default_value="100.0")' in hardware_launch

    for launch_path in (
        "src/rebotarm_bringup/launch/interactive_system.launch.py",
        "src/rebotarm_bringup/launch/bringup.launch.py",
        "src/rebotarm_bringup/launch/teleop_keyboard.launch.py",
    ):
        assert "hardware_controller.launch.py" in _read(launch_path)
    moveit_hardware = _read(
        "src/rebotarm_bringup/launch/moveit_hardware.launch.py"
    )
    assert "interactive_system.launch.py" in moveit_hardware
    assert "hardware_controller.launch.py" not in moveit_hardware


def test_launches_reference_consumer_specific_operator_configs() -> None:
    keyboard = _read("src/rebotarm_bringup/launch/teleop_keyboard.launch.py")
    system = _read("src/rebotarm_bringup/launch/teleop_system.launch.py")
    app = _read("src/rebotarm_bringup/launch/rebotarm_app.launch.py")

    assert "keyboard_control.yaml" in keyboard
    assert "operator_common.yaml" in keyboard
    assert "keyboard_control.yaml" in system
    for text in (system, app):
        assert "web_teleop.yaml" in text
        assert "teach_control.yaml" in text
        assert "operator_common.yaml" in text
    for text in (keyboard, system, app):
        assert "teleop_control.yaml" not in text


def test_keyboard_no_hardware_mode_uses_one_simulated_trajectory_backend() -> None:
    keyboard = _read("src/rebotarm_bringup/launch/teleop_keyboard.launch.py")

    assert keyboard.count('executable="rebotarm_sim_trajectory_controller"') == 1
    assert "condition=UnlessCondition(use_hardware)" in keyboard
    assert 'executable="joint_state_publisher"' not in keyboard
    assert 'executable="reBotArmController"' not in keyboard
    assert "hardware_controller.launch.py" in keyboard
    assert 'parameters=[{"arm_namespace": arm_namespace}]' in keyboard


def test_teleop_system_forwards_execution_mode_to_dashboard() -> None:
    system = _read("src/rebotarm_bringup/launch/teleop_system.launch.py")

    assert 'DeclareLaunchArgument("execution_mode", default_value="execute")' in system
    assert 'execution_mode = LaunchConfiguration("execution_mode")' in system
    assert '"execution_mode": execution_mode' in system
    assert '"web_execute_enabled": web_execute_enabled' in system


def test_dashboard_hides_and_blocks_hardware_only_commands_in_simulation() -> None:
    node = _read(
        "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py"
    )
    html = _read(
        "src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html"
    )

    assert '"use_hardware": bool(self.get_parameter("use_hardware").value)' in node
    assert 'if not self._use_hardware:' in node
    assert "hardware arm command unavailable in simulation mode" in node
    assert 'id="hardware-arm-command-row"' in html
    assert "const useHardware = panelConfig.use_hardware === true;" in html
    assert "hardwareArmCommandRow.hidden = !useHardware" in html
    assert "button.disabled = !useHardware ||" in html

def test_common_commands_document_recommends_one_entrypoint() -> None:
    doc = _read("docs/reference/commands/rebotarm_common_commands.md")

    assert "rebotarm_app.launch.py" in doc
    assert "mode:=" not in doc
    assert "channel:=/dev/ttyACM0" in doc
    assert "网页、键盘与示教命令" in doc
    assert "网页工作台" in doc
    assert "键盘遥操作" in doc
    assert "示教录制" in doc
    assert "Ctrl+C" in doc
    assert "Safe Home" in doc


def test_feature_commands_document_web_teleop_next_to_rviz_drag() -> None:
    doc = _read("docs/reference/commands/rebotarm_feature_commands.md")

    assert "## 仿真 RViz 拖动" in doc
    assert "rebotarm_common_commands.md" in doc
    assert "ros2 launch rebotarm_bringup moveit_hardware.launch.py" in doc
    assert "rviz_ee_drag_real.launch.py" in doc
    assert "channel:=/dev/ttyACM0" in doc
    operator_doc = _read("docs/reference/commands/rebotarm_common_commands.md")
    assert "ros2 launch rebotarm_bringup rebotarm_app.launch.py" in operator_doc
    assert "Preview、Execute、Stop" in operator_doc
