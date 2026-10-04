from pathlib import Path
from types import SimpleNamespace
import ast
import math

import pytest

from rebotarm_simulation.core import resource_paths
from rebotarm_simulation.core.model_contract import ARM_JOINT_NAMES
from rebotarm_simulation.execution.trajectory_sampler import NamedTrajectoryPoint, TrajectorySampler
from rebotarm_simulation.execution.offline_trajectory import play_path

ROOT = Path(__file__).resolve().parents[1]


def test_offline_execution_samples_same_trajectory_as_ros_execution():
    names = ARM_JOINT_NAMES[::-1]
    initial = (.1,) * 6
    points = [NamedTrajectoryPoint(.2, (.3, .4, .5, .6, .7, .8)),
              NamedTrajectoryPoint(.4, (.9,) * 6)]
    sampler = TrajectorySampler(names, points, initial_positions=initial)
    class Sim:
        timestep = .1
        def __init__(self):
            self.target = initial
            self.commands = []
        def get_state(self):
            return SimpleNamespace(joint_positions=self.target)
        def set_joint_position_targets(self, target):
            self.target = target
            self.commands.append(target)
        def step(self):
            return self.get_state()
        def get_contacts(self):
            return ()
    sim = Sim()
    play_path(sim, points, names)
    assert sim.commands == [sampler.sample(t) for t in (.1, .2, .30000000000000004, .4)]
    assert sim.commands[0] == pytest.approx((.45, .4, .35, .3, .25, .2))


@pytest.mark.parametrize('points', [[], [NamedTrajectoryPoint(-.1, (0,)*6)],
    [NamedTrajectoryPoint(.1, (0,)*6), NamedTrajectoryPoint(.1, (1,)*6)],
    [NamedTrajectoryPoint(.1, (math.nan,)*6)]])
def test_sampler_rejects_invalid_paths_for_both_consumers(points):
    with pytest.raises(ValueError):
        TrajectorySampler(ARM_JOINT_NAMES, points)


def test_resource_resolution_in_prefix_without_source_or_ros(tmp_path, monkeypatch):
    monkeypatch.setattr(resource_paths, '_source_package_root', lambda: None)
    monkeypatch.setattr(resource_paths.sys, 'prefix', str(tmp_path))
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    expected = tmp_path / 'share/rebotarm_simulation/models/rebotarm/scene.xml'
    expected.parent.mkdir(parents=True)
    expected.write_text('<mujoco/>')
    assert resource_paths.model_resource() == expected
    override = tmp_path / 'explicit.xml'
    override.write_text('<mujoco/>')
    assert resource_paths.model_resource(explicit=override) == override
    with pytest.raises(FileNotFoundError):
        resource_paths.model_resource(explicit=tmp_path/'missing.xml')


def test_resource_resolution_uses_ament_prefix_without_source(tmp_path, monkeypatch):
    monkeypatch.setattr(resource_paths, '_source_package_root', lambda: None)
    monkeypatch.setattr(resource_paths.sys, 'prefix', str(tmp_path/'absent'))
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(tmp_path))
    expected = tmp_path/'share/rebotarm_simulation/config/test.yaml'
    expected.parent.mkdir(parents=True)
    expected.write_text('test: true')
    assert resource_paths.package_resource('rebotarm_simulation', 'config/test.yaml') == expected


def test_simulation_layers_do_not_import_frontends_from_core_or_control():
    base = ROOT/'src/rebotarm_simulation/rebotarm_simulation'
    assert not (base/'mujoco_adapter_core.py').exists()
    assert [p.name for p in base.glob('*.py')] == ['__init__.py']
    for group in ('core', 'control', 'execution'):
        for path in (base/group).glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                modules = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module or ''] if isinstance(node, ast.ImportFrom) else []
                assert not any(m.startswith(('rclpy', 'rebotarm_rl', 'rebotarmcontroller',
                    'rebotarm_simulation.apps', 'rebotarm_simulation.ros',
                    'rebotarm_simulation.model_tools', 'rebotarm_simulation.diagnostics')) for m in modules), path


def test_control_parameters_can_be_injected_without_loading_files(monkeypatch):
    pytest.importorskip('mujoco')
    from rebotarm_simulation.core import mujoco_sim
    from rebotarm_simulation.control.control_config import load_motor_control_parameters
    parameters = load_motor_control_parameters()
    def forbidden():
        raise AssertionError('injected configuration must bypass resource loading')
    monkeypatch.setattr(mujoco_sim, 'load_motor_control_parameters', forbidden)
    with mujoco_sim.RebotArmMujoco(motor_parameters=parameters) as sim:
        assert sim.step().simulation_time > 0


def test_grasp_trial_rejects_missing_target_and_finger_before_command(bottle_scene):
    pytest.importorskip('mujoco')
    from rebotarm_simulation.diagnostics.mujoco_runner import run_grasp_benchmark
    calls = []
    for target, fingers in [('missing', ('left_finger_link','right_finger_link')),
                            ('bottle', ('left_finger_link','missing'))]:
        with pytest.raises(ValueError):
            run_grasp_benchmark(bottle_scene, target_body=target,
                gripper_bodies=fingers, command=lambda *args: calls.append(args), seconds=.002)
    assert not calls
