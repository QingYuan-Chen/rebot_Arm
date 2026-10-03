"""检查真实 launch 参数展开和配置初始化；不启动节点、相机或硬件。"""

import ast
from pathlib import Path
import re
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/rebotarm_bringup/launch"


@pytest.mark.parametrize("entry", [
    "visual_grasp_system.launch.py",
])
def test_real_public_argument_inventory_is_compact(entry):
    # --show-args expands the real include graph, but never launches a process.
    command = (
        'source /opt/ros/jazzy/setup.bash && source install/setup.bash && '
        f'ros2 launch rebotarm_bringup {entry} --show-args'
    )
    result = subprocess.run(["bash", "-c", command], cwd=ROOT,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    names = re.findall(r"^    '([^']+)':", result.stdout, re.M)
    assert 15 <= len(set(names)) <= 25, names
    assert "candidate_min_confidence" not in names
    assert "gripper_grasp_close_force" not in names
    assert "execution_mode" not in names
    assert "use_sim_time" in names
    assert "visual_interfaces_config" in names


def test_compact_prepares_internal_defaults_and_preserves_overrides_without_launching():
    script = r'''
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

root = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location("compact", root / "src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
context = LaunchContext()
context.launch_configurations.update({"execution_mode": "execute", "candidate_min_confidence": "0.61"})
resolver = lambda pkg: str(root / "src" / pkg)
with patch("launch_ros.substitutions.find_package.get_package_share_directory", side_effect=resolver):
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    stages = module._prepare_stages(context)
assert len(stages) == 2
assert context.launch_configurations["candidate_min_confidence"] == "0.61"
assert context.launch_configurations["max_plan_age_sec"] == "4.0"
assert context.launch_configurations["use_hardware"] == "false"
assert context.launch_configurations["start_visual_grasp_executor"] == "true"
assert context.launch_configurations["auto_retry_enabled"] == "false"
'''
    result = subprocess.run([sys.executable, "-", str(ROOT)], input=script,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_stage_launches_do_not_reintroduce_literal_frame_and_ik_names():
    interface_keys = {"target_frame", "frame_id", "ee_frame_id", "moveit_ik_service"}
    for path in (LAUNCH / "includes").glob("*.launch.py"):
        for node in ast.walk(ast.parse(path.read_text())):
            if not isinstance(node, ast.Dict):
                continue
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value in interface_keys:
                    assert not isinstance(value, ast.Constant), (path, key.value)

@pytest.mark.parametrize('hardware', [False, True])
def test_backend_selects_required_stages_without_launching(hardware):
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('system', LAUNCH / 'visual_grasp_system.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(use_hardware=str(hardware).lower(), execute_gripper='true')
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    stages = module._prepare_stages(context)
    enabled = 'true'
    assert context.launch_configurations['execution_mode'] == 'execute'
    for name in ('start_candidate_ik_filter','start_motion_execution','start_visual_grasp_executor'):
        assert context.launch_configurations[name] == enabled
    assert context.launch_configurations['start_sim_trajectory_controller'] == str(not hardware).lower()
    assert context.launch_configurations['execute_gripper'] == 'true'
    assert len(stages) == 2
    for old_mode in ('plan_only', 'readonly'):
        context.launch_configurations['execution_mode'] = old_mode
        with pytest.raises(RuntimeError, match='execution_mode is retired'):
            module._prepare_stages(context)
