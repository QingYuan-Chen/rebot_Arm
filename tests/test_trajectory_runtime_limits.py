from __future__ import annotations

import pytest

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('document', ['', '[]', 'joint_limits: null', 'joint_limits: []',
    'joint_limits: {}', 'joint_limits: {joint1: null}', 'joint_limits: {joint1: []}',
    'joint_limits: {joint1: {has_velocity_limits: true}}'])
def test_malformed_limit_config_fails_explicitly(tmp_path, document):
    from rebotarm_motion.trajectory_runtime_limits import load_joint_runtime_limits
    path = tmp_path/'limits.yaml'
    path.write_text(document)
    with pytest.raises(ValueError):
        load_joint_runtime_limits(path, ['joint1'])


def test_declared_optional_limits_remain_disabled(tmp_path):
    from rebotarm_motion.trajectory_runtime_limits import load_joint_runtime_limits
    path = tmp_path/'limits.yaml'
    path.write_text('joint_limits: {joint1: {}}')
    limit = load_joint_runtime_limits(path, ['joint1'])['joint1']
    assert limit.max_velocity is limit.max_acceleration is limit.max_jerk is None


@pytest.mark.parametrize('method,expected', [(None, 'auto'), ('', 'auto'), (' default ', 'auto'),
    ('Auto', 'auto'), (' RUCKIG ', 'ruckig'), ('current_jerk_retime', 'current_jerk_retime')])
def test_retiming_method_aliases(method, expected):
    from rebotarm_motion.trajectory_time_parameterization import normalize_time_parameterization_method
    assert normalize_time_parameterization_method(method) == expected


def test_unknown_retiming_method_fails_explicitly():
    from rebotarm_motion.trajectory_time_parameterization import normalize_time_parameterization_method
    with pytest.raises(ValueError, match='unknown time parameterization method'):
        normalize_time_parameterization_method('ruckgi')


def _limits(**overrides):
    from rebotarm_motion.trajectory_runtime_limits import JointRuntimeLimit

    defaults = dict(
        max_effort=10.0,
        max_velocity=3.0,
        max_acceleration=5.0,
        max_jerk=20.0,
    )
    defaults.update(overrides)
    return {"joint1": JointRuntimeLimit(**defaults)}


def test_runtime_guard_reports_effort_and_velocity_violations() -> None:
    from rebotarm_motion.trajectory_runtime_limits import TrajectoryRuntimeLimitGuard

    guard = TrajectoryRuntimeLimitGuard(_limits(max_effort=2.0, max_velocity=1.0))

    violation = guard.observe(
        joint_names=["joint1"],
        velocities=[1.2],
        efforts=[0.5],
        elapsed=0.1,
    )

    assert violation is not None
    assert violation.kind == "velocity"
    assert violation.joint == "joint1"
    assert violation.value == pytest.approx(1.2)
    assert violation.limit == pytest.approx(1.0)

    guard.reset()
    violation = guard.observe(
        joint_names=["joint1"],
        velocities=[0.0],
        efforts=[2.1],
        elapsed=0.1,
    )
    assert violation is not None
    assert violation.kind == "effort"


def test_runtime_guard_derives_acceleration_from_velocity_samples() -> None:
    from rebotarm_motion.trajectory_runtime_limits import TrajectoryRuntimeLimitGuard

    guard = TrajectoryRuntimeLimitGuard(_limits(max_acceleration=2.0))
    assert guard.observe(joint_names=["joint1"], velocities=[0.0], efforts=[0.0], elapsed=0.0) is None
    violation = guard.observe(
        joint_names=["joint1"],
        velocities=[0.3],
        efforts=[0.0],
        elapsed=0.1,
    )

    assert violation is not None
    assert violation.kind == "acceleration"
    assert violation.value == pytest.approx(3.0)


def test_runtime_guard_derives_jerk_from_acceleration_samples() -> None:
    from rebotarm_motion.trajectory_runtime_limits import TrajectoryRuntimeLimitGuard

    guard = TrajectoryRuntimeLimitGuard(_limits(max_acceleration=10.0, max_jerk=4.0))
    assert guard.observe(joint_names=["joint1"], velocities=[0.0], efforts=[0.0], elapsed=0.0) is None
    assert guard.observe(joint_names=["joint1"], velocities=[0.1], efforts=[0.0], elapsed=0.1) is None
    violation = guard.observe(
        joint_names=["joint1"],
        velocities=[0.5],
        efforts=[0.0],
        elapsed=0.2,
    )

    assert violation is not None
    assert violation.kind == "jerk"
    assert violation.value == pytest.approx(30.0)


def test_runtime_guard_rejects_unknown_joint_and_non_monotonic_time() -> None:
    from rebotarm_motion.trajectory_runtime_limits import TrajectoryRuntimeLimitGuard

    guard = TrajectoryRuntimeLimitGuard(_limits())
    with pytest.raises(ValueError, match="unknown joint"):
        guard.observe(joint_names=["joint2"], velocities=[0.0], efforts=[0.0], elapsed=0.0)

    guard.reset()
    guard.observe(joint_names=["joint1"], velocities=[0.0], efforts=[0.0], elapsed=1.0)
    with pytest.raises(ValueError, match="monotonic"):
        guard.observe(joint_names=["joint1"], velocities=[0.0], efforts=[0.0], elapsed=0.5)


def test_load_moveit_runtime_limits_reads_velocity_acceleration_and_jerk() -> None:
    from rebotarm_motion.trajectory_runtime_limits import load_joint_runtime_limits

    limits = load_joint_runtime_limits(
        ROOT / "src/rebotarm_moveit_config/config/joint_limits.yaml",
        ["joint1", "joint4", "left_finger_joint"],
    )

    assert limits["joint1"].max_velocity == pytest.approx(3.0)
    assert limits["joint1"].max_acceleration == pytest.approx(5.0)
    assert limits["joint1"].max_jerk == pytest.approx(20.0)
    assert limits["joint4"].max_velocity == pytest.approx(1.8)
    assert limits["left_finger_joint"].max_jerk is None
