"""关节阻抗与力控生命周期入口回归:持续态,资源互斥,ArmStatus 与服务协议检查."""
from __future__ import annotations

import io
import json
import math
from types import SimpleNamespace

import pytest

from rebotarm_simulation.apps import mujoco_cli
from rebotarm_simulation.diagnostics.impedance_gain_budget import (
    RECOMMENDED_DAMPING, RECOMMENDED_STIFFNESS,
)
from rebotarm_simulation.execution.control_api import SimulationControlApi, normalize_ros_control_mode
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.execution.trajectory_execution import TrajectoryExecution
from rebotarm_simulation.execution.trajectory_state import ActiveTrajectory


TARGET = (0., -.8, -1., .3, 0., 0.)
# 与物理测试和 README 示例共用同一组推荐增益,避免协议层与文档漂移.
STIFFNESS = RECOMMENDED_STIFFNESS
DAMPING = RECOMMENDED_DAMPING


class ImpedanceSim:
    """推进物理时钟的替身;核心数值校验由独立物理测试覆盖."""
    timestep = .01

    def __init__(self):
        self.time = 0.
        self.control_mode = 'hold'
        self.calls = []
        self.closed = False

    def reset(self):
        self.time = 0.
        self.control_mode = 'hold'

    def set_mode(self, mode):
        self.control_mode = mode
        return mode

    def command_joint_impedance(self, positions, stiffness, damping, *, allow_from_position=False):
        if any(not math.isfinite(float(v)) or float(v) < 0. for v in (*stiffness, *damping)):
            raise ValueError('invalid stiffness or damping')
        allowed = ('hold', 'joint_impedance', 'position') if allow_from_position else ('hold', 'joint_impedance')
        if self.control_mode not in allowed:
            raise RuntimeError('enter hold before joint_impedance')
        self.calls.append((self.time, tuple(positions), tuple(stiffness), tuple(damping)))
        self.control_mode = 'joint_impedance'
        return tuple(positions)

    def get_state(self):
        return SimpleNamespace(simulation_time=self.time)

    def get_control_status(self):
        return SimpleNamespace(mode=self.control_mode)

    def step(self, n_steps=1):
        self.time += self.timestep * n_steps

    def close(self):
        self.closed = True


def execution_fixture():
    sim = ImpedanceSim()
    execution = object.__new__(TrajectoryExecution)
    execution.active = ActiveTrajectory()
    execution.lock = execution.active._lock
    execution.access = SerializedSimulationAccess(sim)
    execution.control = SimulationControlApi(sim)
    execution.gripper = SimpleNamespace(busy=False)
    execution.stop_unconfirmed = False
    execution.planning_generation = 4
    execution.preparation = None
    execution.scene_revision = 'scene-a'
    return execution, sim


def command(execution, *, stiffness=STIFFNESS):
    return execution.command_joint_impedance(TARGET, stiffness, DAMPING)


def test_cli_runs_continuously_then_stops_explicitly_to_hold():
    sim = ImpedanceSim()
    output = io.StringIO()
    arguments = ['impedance', '--positions', *map(str, TARGET), '--stiffness', *map(str, STIFFNESS),
        '--damping', *map(str, DAMPING), '--duration', '.05']
    assert mujoco_cli.main(arguments, sim_factory=lambda _: sim, stdout=output) == 0
    payload = json.loads(output.getvalue())
    assert [call[0] for call in sim.calls] == pytest.approx([0.])
    assert payload['active_control']['mode'] == 'joint_impedance'
    assert payload['control']['mode'] == 'hold'
    assert payload['explicit_stop_entered_hold'] is True
    assert sim.closed


@pytest.mark.parametrize('flag,value', [('--duration', 'nan'), ('--duration', '0')])
def test_cli_rejects_invalid_duration(flag, value):
    args = ['impedance', '--positions', *map(str, TARGET), '--stiffness', *map(str, STIFFNESS),
        '--damping', *map(str, DAMPING), flag, value]
    with pytest.raises(SystemExit):
        mujoco_cli.build_parser().parse_args(args)


def test_execution_acceptance_hot_update_and_stop_invalidate_planning_binding():
    execution, sim = execution_fixture()
    assert command(execution) == TARGET
    assert sim.control_mode == 'joint_impedance'
    assert execution.planning_generation == 5
    command(execution)
    assert execution.planning_generation == 6
    assert execution.stop_joint_impedance() is True
    assert sim.control_mode == 'hold'
    assert execution.planning_generation == 7
    assert execution.stop_joint_impedance() is False
    assert execution.planning_generation == 7


def test_execution_admits_impedance_from_idle_position_and_forbids_gravity_jump():
    execution, sim = execution_fixture()
    sim.control_mode = 'position'
    assert command(execution) == TARGET
    assert sim.control_mode == 'joint_impedance'
    status = execution.status_snapshot()
    assert status.phase == 'JOINT_IMPEDANCE' and status.arm_mode == 'mit'

    with pytest.raises(RuntimeError, match='joint impedance active'):
        execution.start_gravity_compensation()
    assert sim.control_mode == 'joint_impedance'

    assert execution.stop_joint_impedance() is True
    assert execution.status_snapshot().phase == 'IDLE'
    assert execution.start_gravity_compensation() == 'gravity_comp'
    assert execution.status_snapshot().phase == 'GRAVITY_COMP'
    assert execution.status_snapshot().arm_mode == 'mit'

    with pytest.raises(RuntimeError, match='enter hold'):
        command(execution)
    assert sim.control_mode == 'gravity_comp'

    assert execution.stop_gravity_compensation() is True
    assert execution.stop_gravity_compensation() is False
    assert execution.status_snapshot().phase == 'IDLE'
    assert execution.status_snapshot().arm_mode == 'pos_vel'


@pytest.mark.parametrize('busy', ['arm', 'gripper', 'stop'])
def test_impedance_rejects_active_resource_or_unconfirmed_stop_without_mutation(busy):
    execution, sim = execution_fixture()
    if busy == 'arm':
        execution.active.try_start(object())
    elif busy == 'gripper':
        execution.gripper.busy = True
    else:
        execution.stop_unconfirmed = True
    with pytest.raises(RuntimeError, match='resources reserved or stop unconfirmed'):
        command(execution)
    assert sim.calls == [] and sim.control_mode == 'hold'
    assert execution.planning_generation == 4


def test_failed_impedance_command_keeps_mode_and_planning_generation():
    execution, sim = execution_fixture()
    with pytest.raises(ValueError, match='stiffness'):
        command(execution, stiffness=(-1.,) * 6)
    assert execution.planning_generation == 4
    assert sim.control_mode == 'hold' and sim.calls == []


def test_mode_change_waits_for_stop_confirmation():
    execution, sim = execution_fixture()
    execution.stop_unconfirmed = True
    with pytest.raises(RuntimeError, match='stop unconfirmed'):
        execution.change_mode('hold')
    assert execution.planning_generation == 4


def test_continuous_reservation_preserves_impedance_owner_then_accepts_after_hold():
    from test_mujoco_ros_adapter import _execution_fixture, _reserve_simulation

    execution, sim, curve = _execution_fixture()
    sim.control_mode = 'joint_impedance'
    generation = execution.planning_generation
    with pytest.raises(RuntimeError, match='mode=joint_impedance'):
        _reserve_simulation(execution, curve)
    assert not execution.active.busy
    assert sim.control_mode == 'joint_impedance' and execution.planning_generation == generation
    execution.change_mode('hold')
    token, _ = _reserve_simulation(execution, curve)
    assert execution.active.token is token and execution.preparation.token is token


def test_bare_ros_mode_requires_complete_impedance_service_request():
    with pytest.raises(ValueError):
        normalize_ros_control_mode('joint_impedance')


def test_ros_services_forward_force_modes_and_publish_status():
    from rebotarm_msgs.srv import SetJointImpedance
    from std_srvs.srv import Trigger
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class
    node_class = create_node_class()
    execution, sim = execution_fixture()
    published = []
    node = SimpleNamespace(_execution=execution, _publish_arm_status=lambda: published.append(execution.status_snapshot().phase))
    request = SetJointImpedance.Request(positions=TARGET, stiffness=STIFFNESS, damping=DAMPING)
    response = node_class._impedance_start_service(node, request, SetJointImpedance.Response())
    assert response.success and response.message == 'simulation mode: joint_impedance'
    assert sim.calls[-1][1:] == (TARGET, STIFFNESS, DAMPING)
    assert published[-1] == 'JOINT_IMPEDANCE'

    stop_res = node_class._impedance_stop_service(node, Trigger.Request(), Trigger.Response())
    assert stop_res.success and published[-1] == 'IDLE'

    grav_res = node_class._gravity_start_service(node, Trigger.Request(), Trigger.Response())
    assert grav_res.success and published[-1] == 'GRAVITY_COMP'
    grav_stop = node_class._gravity_stop_service(node, Trigger.Request(), Trigger.Response())
    assert grav_stop.success and published[-1] == 'IDLE'

    execution.stop_unconfirmed = True
    response = node_class._impedance_start_service(node, request, SetJointImpedance.Response())
    assert not response.success and 'stop unconfirmed' in response.message
    assert len(sim.calls) == 1


def test_mujoco_node_registers_unified_force_control_service_names():
    source = (
        __import__('pathlib').Path(__file__).resolve().parents[1]
        / 'src/rebotarm_simulation/rebotarm_simulation/ros/mujoco_ros_node.py'
    ).read_text(encoding='utf-8')
    assert '/{self._arm_namespace}/joint_impedance/start' in source
    assert '/{self._arm_namespace}/joint_impedance/stop' in source
    assert '/{self._arm_namespace}/gravity_compensation/start' in source
    assert '/{self._arm_namespace}/gravity_compensation/stop' in source
    assert 'joint_impedance/set' not in source
    assert 'sim/set_joint_impedance' not in source


@pytest.mark.parametrize('entry', ['reserve', 'start'])
def test_gripper_entry_rejects_impedance_before_reserving_resources(entry):
    from rebotarm_simulation.execution.gripper_execution import GripperExecution
    execution, sim = execution_fixture()
    sim.get_state = lambda: SimpleNamespace(simulation_time=0., gripper_width=.06)
    execution.gripper = GripperExecution(sim, execution.active,
        {'gripper_motor_speed_rad_s': .5, 'gripper_torque_cap_nm': 1.})
    command(execution)
    owner = object()
    invoke = (lambda: execution.gripper.reserve(owner)) if entry == 'reserve' else (
        lambda: execution.gripper.start(owner, .05, 1.))
    with pytest.raises(RuntimeError, match='joint impedance'):
        invoke()
    assert not execution.gripper.busy
    assert execution.gripper.task is None and execution.gripper.reservation is None
    command(execution)  # 夹爪拒绝后仍允许热更新.
    execution.change_mode('hold')
    invoke()
    assert execution.gripper.busy
