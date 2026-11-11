from __future__ import annotations

from pathlib import Path
import re

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
    assert "def _resolve_channel" not in launch_text
    assert 'for candidate in ("/dev/ttyACM0", "/dev/ttyACM1")' not in launch_text
    assert '"channel": channel' in launch_text
    assert '"use_rviz": "false"' in launch_text
    assert 'arguments=["-d", web_rviz_config]' in launch_text
    assert '"web_teleop_status.rviz"' in launch_text
    assert 'executable="TeleopKeyboardNode"' not in launch_text
    assert 'safe_name = os.path.basename(safe_name.replace("\\\\", "/")) or "teach_record"' in launch_text
    assert 'return f"teleop_records/{safe_name}"' in launch_text
    calibration = launch_text.split('executable="rebotarm_handeye_capture"',1)[1].split('return actions',1)[0]
    assert '"arm_namespace": arm_namespace' in calibration


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
    # 随包配置不能覆盖节点已有的分模式默认值；用户自定义 YAML 仍可设置时长。
    assert "trajectory_duration" not in keyboard_params
    assert "web_keyboard_default_duration" not in web_params
    assert keyboard_params["joint_step_rad"] == web_params["web_keyboard_default_step_rad"] == 0.02
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
    ):
        assert "hardware_controller.launch.py" in _read(launch_path)
    moveit_hardware = _read(
        "src/rebotarm_bringup/launch/moveit_hardware.launch.py"
    )
    assert "interactive_system.launch.py" in moveit_hardware
    assert "hardware_controller.launch.py" not in moveit_hardware
    interactive = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")
    assert '"prepare_trajectories": prepare_trajectories' in interactive
    assert "condition=IfCondition(use_moveit_stack)" in interactive
    assert interactive.count("condition=UnlessCondition(use_moveit_stack)") == 2


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

    assert keyboard.count('"mujoco_moveit_sim.launch.py"') == 1
    assert "condition=UnlessCondition(hardware)" in keyboard
    assert "condition=IfCondition(hardware)" in keyboard
    assert "interactive_system.launch.py" in keyboard
    assert 'executable="joint_state_publisher"' not in keyboard
    assert 'executable="reBotArmController"' not in keyboard
    assert '"use_sim_time": use_sim_time' in keyboard
    assert '"use_moveit_fake_joint_states": "false"' in keyboard


def test_teleop_system_forwards_execution_mode_to_dashboard() -> None:
    system = _read("src/rebotarm_bringup/launch/teleop_system.launch.py")

    assert 'DeclareLaunchArgument("execution_mode", default_value="execute")' in system
    assert 'execution_mode = LaunchConfiguration("execution_mode")' in system
    assert '"execution_mode": execution_mode' in system
    assert '"web_execute_enabled": web_execute_enabled' in system
    # 控制器所在 include 与网页参数必须收到同一模式。


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
    assert re.search(r"(?<![A-Za-z_])mode:=", doc) is None
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


def test_launches_have_no_selectable_trajectory_mode():
    import ast
    launch_dir = ROOT / "src/rebotarm_bringup/launch"
    for path in launch_dir.rglob("*.py"):
        source = path.read_text()
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                if node.func.id in ("DeclareLaunchArgument", "LaunchConfiguration") and node.args:
                    assert not (isinstance(node.args[0], ast.Constant)
                        and node.args[0].value == "trajectory_execution_mode"), path
    demo = (launch_dir / "moveit_demo.launch.py").read_text()
    assert "rebotarm_motion/BoundSimpleControllerManager" in demo
    assert "rebotarm_motion/RetimeExecutionCurve" in demo
    assert "TrajectoryRetimingNode" in demo and "TrajectoryPreparationNode" in demo


def test_interactive_stack_condition_evaluates_hardware_and_passive_combinations():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    spec = importlib.util.spec_from_file_location('interactive_contract',
        ROOT / 'src/rebotarm_bringup/launch/interactive_system.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    stack = next(entity for entity in entities if isinstance(entity, IncludeLaunchDescription))
    for hardware in ('false', 'true'):
        for preview in ('false', 'true'):
            context = LaunchContext()
            context.launch_configurations.update(use_hardware=hardware, use_moveit_preview=preview)
            for entity in entities:
                if isinstance(entity, DeclareLaunchArgument):
                    entity.execute(context)
            assert stack.condition.evaluate(context) == (hardware == 'true' or preview == 'true')
