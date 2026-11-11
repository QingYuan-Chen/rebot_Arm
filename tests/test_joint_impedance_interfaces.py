"""关节阻抗入口回归：假仿真核对期限、资源互斥和协议错误，零硬件访问。"""
from __future__ import annotations

import io
import json
import math
from types import SimpleNamespace

import pytest

from rebotarm_simulation.apps import mujoco_cli
from rebotarm_simulation.execution.control_api import SimulationControlApi, normalize_ros_control_mode
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.execution.trajectory_execution import TrajectoryExecution
from rebotarm_simulation.execution.trajectory_state import ActiveTrajectory


TARGET = (0., -.8, -1., .3, 0., 0.)
STIFFNESS = (80., 80., 80., 20., 10., 5.)
DAMPING = (8., 8., 8., 1., .5, .1)


class ImpedanceSim:
    """推进物理时钟和看门狗的替身；核心数值校验由独立物理测试覆盖。"""
    timestep = .01

    def __init__(self):
        self.time = 0.
        self.control_mode = 'hold'
        self.calls = []
        self.deadline = None
        self.closed = False

    def reset_home(self):
        self.time = 0.

    def set_mode(self, mode):
        self.control_mode = mode
        return mode

    def command_joint_impedance(self, positions, stiffness, damping, timeout_s=.1):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('invalid timeout_s')
        if self.control_mode not in ('hold', 'joint_impedance'):
            raise RuntimeError('enter hold before joint_impedance')
        self.calls.append((self.time, tuple(positions), tuple(stiffness), tuple(damping), timeout_s))
        self.control_mode = 'joint_impedance'
        self.deadline = self.time + timeout_s
        return tuple(positions)

    def get_state(self):
        return SimpleNamespace(simulation_time=self.time)

    def get_control_status(self):
        return SimpleNamespace(mode=self.control_mode)

    def step(self):
        self.time += self.timestep
        if self.control_mode == 'joint_impedance' and self.time + 1e-15 >= self.deadline:
            self.control_mode = 'hold'

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
    return execution, sim


def command(execution, **kwargs):
    return execution.command_joint_impedance(TARGET, STIFFNESS, DAMPING,
        timeout_s=kwargs.get('timeout_s', .1))


def test_cli_renews_during_duration_and_observes_unassisted_watchdog_hold():
    sim = ImpedanceSim()
    output = io.StringIO()
    arguments = ['impedance', '--positions', *map(str, TARGET), '--stiffness', *map(str, STIFFNESS),
        '--damping', *map(str, DAMPING), '--duration', '.22', '--timeout', '.1']
    assert mujoco_cli.main(arguments, sim_factory=lambda _: sim, stdout=output) == 0
    payload = json.loads(output.getvalue())
    assert [call[0] for call in sim.calls] == pytest.approx([0., .05, .1, .15, .2])
    assert payload['active_control']['mode'] == 'joint_impedance'
    assert payload['control']['mode'] == 'hold'
    assert payload['watchdog_entered_hold']
    assert sim.closed


def test_cli_rejects_timeout_shorter_than_renewal_resolution():
    sim = ImpedanceSim()
    with pytest.raises(ValueError, match='two physics steps'):
        mujoco_cli._run_impedance(sim, TARGET, STIFFNESS, DAMPING, duration=.1, timeout=.001)
    assert sim.calls == []


@pytest.mark.parametrize('flag,value', [('--duration', 'nan'), ('--duration', '0'),
    ('--timeout', 'inf'), ('--timeout', '-1')])
def test_cli_rejects_invalid_duration_or_timeout(flag, value):
    args = ['impedance', '--positions', *map(str, TARGET), '--stiffness', *map(str, STIFFNESS),
        '--damping', *map(str, DAMPING), flag, value]
    with pytest.raises(SystemExit):
        mujoco_cli.build_parser().parse_args(args)


def test_execution_acceptance_and_renewal_invalidate_previous_planning_binding():
    execution, sim = execution_fixture()
    assert command(execution) == TARGET
    assert sim.control_mode == 'joint_impedance'
    assert execution.planning_generation == 5
    command(execution)
    assert execution.planning_generation == 6
    execution.change_mode('hold')
    assert sim.control_mode == 'hold'
    assert execution.planning_generation == 7


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
    with pytest.raises(ValueError, match='timeout_s'):
        command(execution, timeout_s=float('nan'))
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


def test_ros_service_forwards_complete_request_and_reports_failure():
    from rebotarm_msgs.srv import SetJointImpedance
    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class
    node_class = create_node_class()
    execution, sim = execution_fixture()
    node = SimpleNamespace(_execution=execution)
    request = SetJointImpedance.Request(positions=TARGET, stiffness=STIFFNESS,
        damping=DAMPING, timeout_s=.1)
    response = node_class._impedance_service(node, request, SetJointImpedance.Response())
    assert response.success and response.message == 'simulation mode: joint_impedance'
    assert sim.calls[-1][1:] == (TARGET, STIFFNESS, DAMPING, .1)
    execution.stop_unconfirmed = True
    response = node_class._impedance_service(node, request, SetJointImpedance.Response())
    assert not response.success and 'stop unconfirmed' in response.message
    assert len(sim.calls) == 1


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
    command(execution)  # 夹爪拒绝后仍允许正常续期。
    execution.change_mode('hold')
    invoke()
    assert execution.gripper.busy
