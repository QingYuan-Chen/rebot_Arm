"""视觉筛选入口的参数、上下文、话题连接及退役入口门禁。"""

import ast
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'src/rebotarm_bringup/launch'


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare(**values):
    module = load('visual_grasp_system.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(values)
    resolver = lambda pkg: str(ROOT / 'src' / pkg)
    with patch('launch_ros.substitutions.find_package.get_package_share_directory', side_effect=resolver):
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        stages = module._prepare_stages(context)
    return context, stages


@pytest.mark.parametrize('robot_context', ['offline', 'external'])
def test_entry_prepares_candidate_stages(robot_context):
    context, stages = prepare(robot_context=robot_context, candidate_min_confidence='0.61')
    assert context.launch_configurations['candidate_min_confidence'] == '0.61'
    assert context.launch_configurations['start_candidate_ik_filter'] == 'true'
    assert len(stages) == 2
    assert 'start_motion_execution' not in context.launch_configurations
    assert 'start_visual_grasp_executor' not in context.launch_configurations


@pytest.mark.parametrize('name', ['execution_mode', 'use_hardware', 'trajectory_execution_mode',
                                  'execute_gripper', 'backend_profile', 'motion_profile',
                                  'gripper_grasp_enabled', 'executor_input_topic', 'sim_initial_joint_positions',
                                  'shutdown_safe_home', 'hardware_feedback_rate_hz'])
def test_old_execution_arguments_are_rejected(name):
    with pytest.raises(RuntimeError, match='Retired visual execution arguments.*robot_context'):
        prepare(**{name: 'false'})


def test_old_execution_parameters_in_strategy_profile_are_rejected(tmp_path):
    profile = tmp_path / 'old.yaml'
    profile.write_text('launch:\n  sim_initial_joint_positions: [0, 0, 0, 0, 0, 0]\n')
    with pytest.raises(RuntimeError, match='Retired visual execution parameters in strategy_profile'):
        prepare(strategy_profile=str(profile))


def test_offline_alone_starts_passive_moveit_environment():
    defaults, _ = prepare()
    assert defaults.launch_configurations['robot_context'] == 'offline'
    backend = load('includes/visual_backend.launch.py').generate_launch_description()
    include = next(a for a in backend.entities if isinstance(a, IncludeLaunchDescription))
    context = LaunchContext()
    context.launch_configurations['robot_context'] = 'offline'
    assert include.condition.evaluate(context)
    context.launch_configurations['robot_context'] = 'external'
    assert not include.condition.evaluate(context)
    args = dict(include.launch_arguments)
    assert args['use_fake_joint_states'] == 'true'
    assert args['allow_trajectory_execution'] == 'false'
    assert 'use_sim_time' in args


def test_visual_entry_composes_filtering_stages_and_profiles():
    main = (LAUNCH / 'visual_grasp_system.launch.py').read_text()
    assert '"visual_backend", "visual_lifecycle"' in main
    assert '"visual_interfaces_config"' in main
    text = (LAUNCH / 'includes/visual_lifecycle.launch.py').read_text()
    for stage in ('visual_input', 'grasp_candidate', 'candidate_filter', 'grasp_preview'):
        assert f'"{stage}"' in text
        assert (LAUNCH / 'includes' / f'{stage}.launch.py').is_file()
    for stage in ('motion_execution', 'grasp_executor'):
        assert stage not in text
    assert (ROOT / 'src/rebotarm_vision/config/visual_strategy_profile.yaml').is_file()
    assert (ROOT / 'src/rebotarm_bringup/config/visual_grasp_interfaces.yaml').is_file()


def test_marker_consumes_filtered_plan():
    text = (LAUNCH / 'includes/grasp_preview.launch.py').read_text()
    assert "'input_topic': filtered_plan_topic" in text
    assert 'executor_input_topic' not in text
    assert 'grasp_preview_sender' not in text


def test_moveit_execution_default_remains_enabled():
    text = (LAUNCH / 'moveit_demo.launch.py').read_text()
    tree = ast.parse(text)
    declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                    and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                    and n.args and isinstance(n.args[0], ast.Constant)
                    and n.args[0].value == 'allow_trajectory_execution']
    assert len(declarations) == 1
    assert any(k.arg == 'default_value' and ast.literal_eval(k.value) == 'true'
               for k in declarations[0].keywords)


def test_visual_backend_has_no_execution_nodes():
    text = (LAUNCH / 'includes/visual_backend.launch.py').read_text()
    for name in ('rebotarm_sim_trajectory_controller', 'PoseExecutionNode',
                 'rebotarm_visual_grasp_executor', 'reBotArmController'):
        assert name not in text
    for name in ('motion_execution', 'grasp_executor'):
        assert not (LAUNCH / 'includes' / f'{name}.launch.py').exists()


def test_retired_visual_wrappers_are_removed():
    for name in ('visual_grasp_compact', 'visual_readonly', 'visual_plan_only', 'visual_execute'):
        assert not (LAUNCH / f'{name}.launch.py').exists()


def test_visual_ready_tool_is_retired():
    for relative in ('src/rebotarm_bringup/launch/visual_ready.launch.py',
                     'src/rebotarm_motion/rebotarm_motion/visual_ready_node.py',
                     'src/rebotarm_motion/config/visual_ready.yaml'):
        assert not (ROOT / relative).exists()
    assert 'rebotarm_visual_ready =' not in (ROOT / 'src/rebotarm_motion/setup.py').read_text()


def test_interface_profile_is_documented_as_the_authoritative_connection_contract():
    docs = (ROOT / 'docs/reference/parameters/current_parameter_sources.md').read_text()
    assert 'visual_grasp_interfaces.yaml' in docs
    assert 'visual_grasp_system.launch.py' in docs


def test_graspnet_resources_require_explicit_deployment_paths():
    launch = (LAUNCH / 'visual_grasp_system.launch.py').read_text()
    assert 'for parent in Path(__file__).resolve().parents' not in launch
    assert 'third_party/graspnet-baseline' not in launch
    assert '.local-models/checkpoints' not in launch
    inference = (ROOT / 'src/rebotarm_vision/rebotarm_vision/backends/graspnet_baseline_inference.py').read_text()
    assert 'resolve().parent / "graspnetAPI"' not in inference
    assert 'GRASPNET_API_ROOT' in inference
