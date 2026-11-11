"""视觉筛选入口的参数、上下文、话题连接及退役入口门禁。"""

import ast
import importlib.util
from pathlib import Path
from unittest.mock import patch

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'src/rebotarm_bringup/launch'


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare(**values):
    module = load('visual_perception_system.launch.py')
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
    main = (LAUNCH / 'visual_perception_system.launch.py').read_text()
    assert '"visual_backend", "visual_lifecycle"' in main
    assert '"visual_interfaces_config"' in main
    context, _ = prepare()
    lifecycle = load('includes/visual_lifecycle.launch.py').generate_launch_description()
    resolver = lambda pkg: str(ROOT / 'src' / pkg)
    with patch('launch_ros.substitutions.find_package.get_package_share_directory', side_effect=resolver):
        paths = []
        for action in lifecycle.entities:
            action.launch_description_source.get_launch_description(context)
            paths.append(Path(action.launch_description_source.location))
    assert [path.name for path in paths] == [
        'visual_input.launch.py', 'grasp_candidate.launch.py',
        'candidate_filter.launch.py', 'candidate_markers.launch.py',
    ]
    assert all(path.is_file() for path in paths)
    assert (ROOT / 'src/rebotarm_vision/config/visual_strategy_profile.yaml').is_file()
    assert (ROOT / 'src/rebotarm_bringup/config/visual_grasp_interfaces.yaml').is_file()


@pytest.mark.parametrize('enabled', ['true', 'false'])
def test_marker_stage_only_displays_filtered_plan(enabled):
    context, _ = prepare(start_visual_grasp_markers=enabled, filtered_plan_topic='/test/filtered_plan')
    module = load('includes/candidate_markers.launch.py')
    resolver = lambda pkg: str(ROOT / 'src' / pkg)
    with patch('launch_ros.substitutions.find_package.get_package_share_directory', side_effect=resolver), \
            patch.object(module, 'Node', wraps=Node) as factory:
        nodes = module._build(context)
    assert len(nodes) == 1
    node = nodes[0]
    assert isinstance(node, Node)
    assert node.node_package == 'rebotarm_vision'
    assert node.node_executable == 'rebotarm_visual_grasp_markers'
    assert node.condition.evaluate(context) == (enabled == 'true')
    params, = factory.call_args.kwargs['parameters']
    assert params['input_topic'].perform(context) == '/test/filtered_plan'


def test_moveit_passive_state_disables_execution_but_backend_state_enables_it():
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    import importlib.util
    spec = importlib.util.spec_from_file_location('moveit_demo_contract', LAUNCH / 'moveit_demo.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    declarations = [entity for entity in module.generate_launch_description().entities
                    if isinstance(entity, DeclareLaunchArgument)]
    for passive, expected in [('true', 'false'), ('false', 'true')]:
        context = LaunchContext()
        context.launch_configurations['use_fake_joint_states'] = passive
        for declaration in declarations:
            declaration.execute(context)
        assert context.launch_configurations['allow_trajectory_execution'] == expected
        assert context.launch_configurations['prepare_trajectories'] == expected


def test_visual_backend_has_no_execution_nodes():
    text = (LAUNCH / 'includes/visual_backend.launch.py').read_text()
    for name in ('rebotarm_sim_trajectory_controller', 'PoseExecutionNode',
                 'rebotarm_visual_grasp_executor', 'reBotArmController'):
        assert name not in text
    for name in ('motion_execution', 'grasp_executor'):
        assert not (LAUNCH / 'includes' / f'{name}.launch.py').exists()


def test_retired_visual_wrappers_are_removed():
    for name in ('visual_grasp_system', 'visual_grasp_compact', 'visual_readonly',
                 'visual_plan_only', 'visual_execute', 'includes/grasp_preview'):
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
    assert 'visual_perception_system.launch.py' in docs


def test_graspnet_resources_require_explicit_deployment_paths():
    launch = (LAUNCH / 'visual_perception_system.launch.py').read_text()
    assert 'for parent in Path(__file__).resolve().parents' not in launch
    assert 'third_party/graspnet-baseline' not in launch
    assert '.local-models/checkpoints' not in launch
    inference = (ROOT / 'src/rebotarm_vision/rebotarm_vision/backends/graspnet_baseline_inference.py').read_text()
    assert 'resolve().parent / "graspnetAPI"' not in inference
    assert 'GRASPNET_API_ROOT' in inference
