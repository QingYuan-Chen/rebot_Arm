"""关节阻抗增益的力矩预算:额定/限幅/协议范围与文档示例一致性."""
from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from rebotarm_simulation.control.control_config import load_motor_control_parameters
from rebotarm_simulation.diagnostics import impedance_gain_budget as budget


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / 'src' / 'rebotarm_simulation' / 'README_mujoco.md'
# 旧增益只作为反例保留,用于证明预算判据确实会拦住超额定配置.
LEGACY_STIFFNESS = (80., 80., 80., 20., 10., 5.)
LEGACY_DAMPING = (8., 8., 8., 1., .5, .1)


@pytest.fixture(scope='module')
def limits():
    parameters = load_motor_control_parameters(ROOT)
    return (np.asarray(parameters.arm.rated_torque, dtype=float),
            np.asarray(parameters.arm.effort_limit, dtype=float))


@pytest.fixture(scope='module')
def dynamics():
    return budget.reference_dynamics()


@pytest.fixture(scope='module')
def report(limits, dynamics):
    rated, effort = limits
    gravity, inertia = dynamics
    result = budget.evaluate_gains(budget.RECOMMENDED_STIFFNESS, budget.RECOMMENDED_DAMPING,
                                   rated, effort, gravity, inertia)
    result['violations'] = budget.budget_violations(result, rated, effort)
    return result


def test_recommended_gains_satisfy_spring_rated_limit(report):
    assert report['violations'] == []
    assert np.all(np.asarray(report['spring_over_rated']) <= 1.0 + 1e-9)


def test_recommended_gains_stay_within_urdf_effort(report):
    assert np.all(np.asarray(report['spring_plus_gravity_over_effort']) <= 1.0 + 1e-9)


def test_recommended_gains_stay_within_mit_protocol_range(report):
    # 达妙 MIT 帧 Kp<=500 / Kd<=5;旧配置 joint1-3 的 kd=8 在此之外.
    assert np.all(np.asarray(report['stiffness']) <= budget.PROTOCOL_KP_MAX)
    assert np.all(np.asarray(report['damping']) <= budget.PROTOCOL_KD_MAX + 1e-9)


def test_legacy_gains_are_rejected_by_the_budget(limits, dynamics):
    """反例:旧默认增益必须被判据拦住,否则判据本身失效."""
    rated, effort = limits
    gravity, inertia = dynamics
    legacy = budget.evaluate_gains(LEGACY_STIFFNESS, LEGACY_DAMPING, rated, effort,
                                   gravity, inertia)
    violations = budget.budget_violations(legacy, rated, effort)
    assert violations
    assert any('exceeds rated' in item for item in violations)
    assert any('protocol limit' in item for item in violations)


def test_derived_profile_matches_documented_constants(report):
    """推导结果与常量一致,避免模型或配置变化后文档悄悄过期."""
    derived = budget.derive()
    assert derived['matches_documented_profile']
    assert derived['recommended_stiffness'] == pytest.approx(budget.RECOMMENDED_STIFFNESS)
    assert derived['recommended_damping'] == pytest.approx(budget.RECOMMENDED_DAMPING)
    assert derived['recommended_stiffness'] == pytest.approx(report['stiffness'])
    assert derived['recommended_damping'] == pytest.approx(report['damping'])


def test_gravity_is_part_of_the_effort_budget_only(report):
    """纯重力只用于 B2 边界计算,不单独判违规;现场未观察到 joint3 超负载."""
    assert report['worst_gravity_nm']
    assert report['violations'] == []
    assert 'gravity_warnings' not in report


def test_mujoco_readme_examples_use_recommended_gains(report):
    """README 的 CLI、Python 与 ROS 三处示例必须与常量一致."""
    text = README.read_text(encoding='utf-8')
    stiffness = ' '.join(f'{value:g}' for value in budget.RECOMMENDED_STIFFNESS)
    damping = ' '.join(f'{value:g}' for value in budget.RECOMMENDED_DAMPING)
    assert f'--stiffness {stiffness} --damping {damping}' in text
    python_stiffness = ', '.join(f'{value:g}' for value in budget.RECOMMENDED_STIFFNESS)
    python_damping = ', '.join(f'{value:g}' for value in budget.RECOMMENDED_DAMPING)
    assert python_stiffness in text and python_damping in text
    # ROS service 示例用 YAML 风格浮点排版:整数值带 .0,其余保留原精度.
    service_stiffness = ', '.join(f'{value:.1f}' for value in budget.RECOMMENDED_STIFFNESS)
    service_damping = ', '.join(f'{value:.1f}' if value == int(value) else f'{value:g}'
                                for value in budget.RECOMMENDED_DAMPING)
    assert f'stiffness: [{service_stiffness}]' in text
    assert f'damping: [{service_damping}]' in text
    assert 'stiffness 80 80 80' not in text


def test_cli_reports_success_for_documented_profile(tmp_path):
    output = tmp_path / 'budget.json'
    result = subprocess.run(
        [sys.executable, '-m', 'rebotarm_simulation.diagnostics.impedance_gain_budget',
         '--output', str(output)],
        cwd=ROOT, capture_output=True, text=True,
        env={'PYTHONPATH': f'src/rebotarm_simulation:src/rebotarm_force_control',
             'PATH': '/usr/bin:/bin', 'MUJOCO_GL': 'egl'},
    )
    assert result.returncode == 0, result.stderr
    assert output.is_file()


@pytest.mark.parametrize('stiffness,damping,message', [
    ((1000.,) * 6, (1.,) * 6, 'exceeds effort'),
    ((60.,) * 6, (6.,) * 6, 'protocol limit'),
])
def test_budget_rejects_out_of_bound_gains(limits, dynamics, stiffness, damping, message):
    rated, effort = limits
    gravity, inertia = dynamics
    result = budget.evaluate_gains(stiffness, damping, rated, effort, gravity, inertia)
    violations = budget.budget_violations(result, rated, effort)
    assert any(message in item for item in violations)


def test_extreme_stiffness_exceeds_effort_and_is_not_silently_clamped(limits, dynamics):
    """1000 N·m/rad 的弹簧项远超 URDF 限幅;预算必须报出来而不是当作正常."""
    rated, effort = limits
    gravity, inertia = dynamics
    result = budget.evaluate_gains((1000.,) * 6, (1.,) * 6, rated, effort, gravity, inertia)
    assert np.all(np.asarray(result['spring_plus_gravity_nm']) > effort)
    assert any('exceeds effort' in item
               for item in budget.budget_violations(result, rated, effort))
