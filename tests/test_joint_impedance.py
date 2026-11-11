"""关节阻抗控制律的解析样本,参数边界与限矩限速检查."""

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from rebotarm_force_control.joint_impedance import (
    JointImpedanceCommand, compute_joint_impedance,
)


def compute(command, **kwargs):
    inputs = dict(position=(0.,)*6, velocity=(0.,)*6, gravity=(0.,)*6,
                  effort_limits=(27.,)*6, rate_limits=(100.,)*6,
                  previous=(0.,)*6, dt=1.)
    inputs.update(kwargs)
    return compute_joint_impedance(command, **inputs)


def test_spring_damping_and_gravity_have_correct_sign_and_units():
    command = JointImpedanceCommand((.2, -.2, .1, -.1, 0., 0.), (10.,)*6, (2.,)*6)
    output = compute(command, position=(.1, -.1, 0., 0., 0., 0.),
                     velocity=(.3, -.3, 0., 0., .5, -.5), gravity=(1.,)*6)
    assert output.requested_torques == pytest.approx((1.4, .6, 2., 0., 0., 2.))
    assert output.applied_torques == output.requested_torques
    assert not any(output.saturated)


def test_zero_gains_leave_only_gravity():
    command = JointImpedanceCommand((1.,)*6, (0.,)*6, (0.,)*6)
    assert compute(command, gravity=(1., -2., 3., -4., 5., -6.)).applied_torques == (
        1., -2., 3., -4., 5., -6.)


def test_total_torque_and_slew_are_bounded_from_actual_previous_output():
    command = JointImpedanceCommand((1., -1., 1., -1., 1., -1.), (100.,)*6, (0.,)*6)
    result = compute(command, effort_limits=(5.,)*6, rate_limits=(2.,)*6,
                     previous=(4.9, -4.9, 0., 0., -4.9, 4.9), dt=.1)
    assert result.applied_torques == pytest.approx((5., -5., .2, -.2, -4.7, 4.7))
    assert all(result.saturated)
    assert max(abs(v) for v in result.applied_torques) <= 5.


def test_command_copies_mutable_input_and_is_immutable():
    target, stiffness = [0.]*6, np.ones(6)
    command = JointImpedanceCommand(target, stiffness, (2.,)*6)
    target[0] = 9.
    stiffness[0] = 9.
    assert command.target == (0.,)*6
    assert command.stiffness == (1.,)*6
    with pytest.raises(FrozenInstanceError):
        command.target = (1.,)*6


@pytest.mark.parametrize('field', ['target', 'stiffness', 'damping'])
@pytest.mark.parametrize('value', [(0.,)*5, [[0.]*6], (float('nan'),)*6, (float('inf'),)*6])
def test_command_rejects_invalid_vectors(field, value):
    fields = dict(target=(0.,)*6, stiffness=(1.,)*6, damping=(1.,)*6)
    fields[field] = value
    with pytest.raises(ValueError):
        JointImpedanceCommand(**fields)


@pytest.mark.parametrize('field', ['stiffness', 'damping'])
def test_command_rejects_negative_gains(field):
    fields = dict(target=(0.,)*6, stiffness=(1.,)*6, damping=(1.,)*6)
    fields[field] = (-.1,)*6
    with pytest.raises(ValueError):
        JointImpedanceCommand(**fields)


@pytest.mark.parametrize('field', ['position', 'velocity', 'gravity', 'effort_limits', 'rate_limits', 'previous'])
def test_calculation_rejects_nonfinite_inputs(field):
    command = JointImpedanceCommand((0.,)*6, (1.,)*6, (1.,)*6)
    with pytest.raises(ValueError):
        compute(command, **{field: (float('nan'),)*6})


@pytest.mark.parametrize('kwargs', [
    {'dt': 0.}, {'dt': -1.}, {'dt': float('nan')}, {'dt': float('inf')},
    {'effort_limits': (0.,)*6}, {'rate_limits': (-1.,)*6}, {'previous': (28.,)*6},
])
def test_calculation_rejects_invalid_constraints(kwargs):
    with pytest.raises(ValueError):
        compute(JointImpedanceCommand((0.,)*6, (1.,)*6, (1.,)*6), **kwargs)


def test_overflow_is_rejected_before_clamping():
    command = JointImpedanceCommand((1e200,)*6, (1e200,)*6, (0.,)*6)
    with pytest.raises(ValueError, match='overflowed'):
        compute(command)
