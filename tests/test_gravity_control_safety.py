from pathlib import Path

import numpy as np
import pytest

from rebotarmcontroller.gravity_control import (
    GravitySettings, LoopTiming, bounded_mit_command, load_gravity_model,
)

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'src/rebotarm_moveit_config/config/rebotarm.urdf'


def test_integrated_pd_and_feedforward_obey_total_torque_and_slew_bounds():
    q = np.zeros(6)
    target = np.ones(6)
    previous = np.zeros(6)
    kp, kd, tau, total = bounded_mit_command(
        target, q, q, np.full(6, 7.), np.full(6, .8), np.full(6, 100.),
        np.array([27., 27., 27., 7., 7., 7.]), np.full(6, 10.), previous, .02,
    )
    np.testing.assert_allclose(total, .2)
    np.testing.assert_allclose(kp * (target - q) + tau, total)
    assert np.all(np.abs(tau) <= np.array([27., 27., 27., 7., 7., 7.]))


@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_torque_is_rejected(value):
    with pytest.raises(ValueError, match='finite'):
        bounded_mit_command(*[np.full(6, value)] * 8, np.zeros(6), .01)


def test_loop_timing_uses_elapsed_time_and_recovers_after_on_time_cycle():
    timing = LoopTiming(100.)
    assert timing.begin(1.) == pytest.approx(.01)
    timing.finish(1.002)
    assert timing.begin(1.025) == pytest.approx(.025)
    timing.finish(1.026)
    assert timing.consecutive_misses == 1
    timing.begin(1.035)
    timing.finish(1.037)
    assert timing.consecutive_misses == 0
    assert timing.snapshot()['max_period_sec'] == pytest.approx(.025)


def test_loop_timing_detects_execution_overrun_and_three_consecutive_misses():
    timing = LoopTiming(100.)
    for index in range(3):
        start = 1. + index * .03
        timing.begin(start)
        timing.finish(start + .02)
    assert timing.faulted


@pytest.mark.parametrize('rate', [0, -1, float('nan')])
def test_invalid_loop_rate_rejected(rate):
    with pytest.raises(ValueError):
        LoopTiming(rate)


def test_gravity_model_is_six_axis_reduction_of_canonical_urdf():
    pin = pytest.importorskip('pinocchio')
    model = load_gravity_model(URDF, GravitySettings())
    assert model.model.nq == model.model.nv == 6
    assert list(model.model.names)[1:] == [f'joint{i}' for i in range(1, 7)]
    q = np.array([.2, -.6, -.8, .2, .3, -.2])
    full = pin.buildModelFromUrdf(str(URDF))
    expected = pin.computeGeneralizedGravity(full, full.createData(), np.r_[q, 0., 0.])
    np.testing.assert_allclose(model.gravity(q), expected[:6], atol=1e-10)
    assert model.jacobian(q).shape == (6, 6)


def test_payload_and_install_orientation_have_explicit_effect():
    pytest.importorskip('pinocchio')
    q = np.array([.2, -.6, -.8, .2, .3, -.2])
    base = load_gravity_model(URDF, GravitySettings())
    loaded = load_gravity_model(URDF, GravitySettings(payload_mass_kg=.3,
                                                  payload_com_xyz_m=(.02, 0., .05)))
    J = base.jacobian(q)
    # COM force expressed in base axes, with its moment about end_link origin.
    R = base.data.oMf[base.frame_id].rotation
    force = np.array([0., 0., -.3 * 9.81])
    external = J.T @ np.r_[force, np.cross(R @ np.array([.02, 0., .05]), force)]
    np.testing.assert_allclose(loaded.gravity(q) - base.gravity(q), -external, atol=1e-8)
    inverted = load_gravity_model(URDF, GravitySettings(gravity_xyz_m_s2=(0., 0., 9.81)))
    np.testing.assert_allclose(inverted.gravity(q), -base.gravity(q), atol=1e-10)


def test_model_rejects_wrong_joint_coordinates_before_hardware(tmp_path):
    pytest.importorskip('pinocchio')
    bad = tmp_path / 'bad.urdf'
    bad.write_text(URDF.read_text().replace('lower="-2.8"', 'lower="-2.7"'))
    with pytest.raises(ValueError, match='joint1.*limit'):
        load_gravity_model(bad, GravitySettings())


def test_settings_reject_unknown_fields_and_invalid_payload(tmp_path):
    bad = tmp_path / 'settings.yaml'
    bad.write_text('payload_mass_kg: -1\n')
    with pytest.raises(ValueError):
        GravitySettings.from_yaml(bad)
    bad.write_text('payload_mass: 1\n')
    with pytest.raises(ValueError, match='unknown'):
        GravitySettings.from_yaml(bad)


def test_tool_jacobian_translation_matches_finite_difference():
    pin = pytest.importorskip('pinocchio')
    gravity = load_gravity_model(URDF, GravitySettings())
    q = np.array([.2, -.6, -.8, .2, .3, -.2])
    J = gravity.jacobian(q)
    origin = gravity.data.oMf[gravity.frame_id].translation.copy()
    numeric = np.zeros((3, 6))
    for axis in range(6):
        shifted = q.copy()
        shifted[axis] += 1e-7
        pin.framesForwardKinematics(gravity.model, gravity.data, shifted)
        numeric[:, axis] = (gravity.data.oMf[gravity.frame_id].translation - origin) / 1e-7
    np.testing.assert_allclose(J[:3], numeric, atol=1e-7)
