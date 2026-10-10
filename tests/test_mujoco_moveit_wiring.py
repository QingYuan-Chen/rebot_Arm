import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
DEMO = ROOT / "src" / "rebotarm_bringup" / "launch" / "moveit_demo.launch.py"
MOVEIT_SIM = ROOT / "src" / "rebotarm_bringup" / "launch" / "mujoco_moveit_sim.launch.py"
README = ROOT / "src" / "rebotarm_simulation" / "README_mujoco.md"
MUJOCO_NODE = (
    ROOT
    / "src"
    / "rebotarm_simulation"
    / "rebotarm_simulation"
    / "ros"
    / "mujoco_ros_node.py"
)
MUJOCO_LAUNCH = (
    ROOT / "src" / "rebotarm_simulation" / "launch" / "mujoco_sim.launch.py"
)


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8-sig")


def _parse(path: Path) -> ast.AST:
    return ast.parse(_text(path), filename=str(path))


def _node_calls(path: Path) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(_parse(path))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]


def _keyword(call: ast.Call, name: str) -> ast.AST | None:
    return next((item.value for item in call.keywords if item.arg == name), None)


def _literal_keyword(call: ast.Call, name: str) -> str | None:
    value = _keyword(call, name)
    return value.value if isinstance(value, ast.Constant) else None


def _declares_use_sim_time_default_false(path: Path) -> bool:
    for call in ast.walk(_parse(path)):
        if not (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "DeclareLaunchArgument"
            and call.args
            and isinstance(call.args[0], ast.Constant)
            and call.args[0].value == "use_sim_time"
        ):
            continue
        return _literal_keyword(call, "default_value") == "false"
    return False


def _parameters_reference_use_sim_time(call: ast.Call) -> bool:
    parameters = _keyword(call, "parameters")
    if parameters is None:
        return False
    return any(
        isinstance(node, ast.Constant) and node.value == "use_sim_time"
        for node in ast.walk(parameters)
    ) and any(
        isinstance(node, ast.Name) and node.id == "use_sim_time"
        for node in ast.walk(parameters)
    )


def test_moveit_demo_declares_sim_time_safe_default_and_wires_all_consumers():
    assert _declares_use_sim_time_default_false(DEMO)
    expected = {
        ("tf2_ros", "static_transform_publisher"),
        ("rebotarm_teleop", "GripperVisualJointStateNode"),
        ("joint_state_publisher", "joint_state_publisher"),
        ("robot_state_publisher", "robot_state_publisher"),
        ("moveit_ros_move_group", "move_group"),
        ("rviz2", "rviz2"),
    }
    nodes = {
        (_literal_keyword(call, "package"), _literal_keyword(call, "executable")): call
        for call in _node_calls(DEMO)
    }
    assert expected <= nodes.keys()
    assert all(_parameters_reference_use_sim_time(nodes[key]) for key in expected)


def test_mujoco_moveit_launch_selects_one_simulated_state_source():
    text = _text(MOVEIT_SIM)
    assert 'DeclareLaunchArgument("use_sim_time", default_value="true")' in text
    assert 'executable="rebotarm_mujoco_node"' in text
    assert '"use_fake_joint_states": "false"' in text
    assert '"use_sim_time": use_sim_time' in text


def test_documented_moveit_command_uses_sim_time_and_safe_targeted_shutdown():
    text = _text(README)
    command = "ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py use_sim_time:=true"
    assert command in text
    assert "pgrep -af rebotarm_mujoco_node" in text
    assert "ps -fp <confirmed-pid>" in text
    assert "kill -TERM <confirmed-pid>" in text
    assert "pkill" not in text
    assert "物理步进定时器固定使用 steady clock" in text
    assert "use_sim_time:=true" in text
    assert "跟随 `/clock`" in text


def test_mujoco_physics_timer_uses_an_explicit_steady_clock():
    tree = _parse(MUJOCO_NODE)
    imports = {
        alias.name: alias.asname
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module == "rclpy.clock"
        for alias in node.names
    }
    assert imports.get("Clock") == "RclpyClock"
    assert "ClockType" in imports

    clock_assignment = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == "_physics_clock"
            for target in node.targets
        )
    )
    assert isinstance(clock_assignment.value, ast.Call)
    assert isinstance(clock_assignment.value.func, ast.Name)
    assert clock_assignment.value.func.id == "RclpyClock"
    clock_type = _keyword(clock_assignment.value, "clock_type")
    assert isinstance(clock_type, ast.Attribute)
    assert isinstance(clock_type.value, ast.Name)
    assert clock_type.value.id == "ClockType"
    assert clock_type.attr == "STEADY_TIME"

    timers = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_timer"
    ]
    assert timers
    for timer in timers:
        clock = _keyword(timer, "clock")
        assert isinstance(clock, ast.Attribute)
        assert isinstance(clock.value, ast.Name) and clock.value.id == "self"
        assert clock.attr == "_physics_clock"


def test_mujoco_launch_does_not_enable_sim_time_on_clock_publisher():
    text = _text(MUJOCO_LAUNCH)
    assert "use_sim_time" not in text
