"""位姿纯计算及 SDK 退出：规范限位、不可达、采样失败和隔离导入。"""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pinocchio as pin
import pytest
from rebotarm_motion.pose_kinematics import JOINT_NAMES, PoseKinematics
from rebotarm_motion.pose_preview_solver import PosePreviewSolver
from rebotarm_motion.cartesian_trajectory import plan_cartesian, retime_path

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('q', [(0.,-1.,-1.,0.,.3,0.), (.2,.019,-1.,.2,.1,-.2), (0.,0.,0.,0.,0.,0.)])
def test_fk_ik_roundtrip_and_canonical_upper_limits(q):
    solver = PoseKinematics()
    result = solver.solve(solver.pose(q), q)
    assert result.success and result.error <= 1e-4
    assert result.q == pytest.approx(q)
    assert solver.upper[1:3] == pytest.approx((.02,.02))
    preview = PosePreviewSolver(ROOT)
    target = preview.compute_pose_target(q)
    assert preview.solve_pose(target, q, JOINT_NAMES).success
    with pytest.raises(ValueError, match='order'):
        preview.solve_pose(target, q, tuple(reversed(JOINT_NAMES)))


def test_unreachable_ik_and_fixed_retry_source():
    solver = PoseKinematics()
    target = pin.SE3(np.eye(3), np.array((4.,4.,4.)))
    q = (0.,-1.,-1.,0.,.3,0.)
    first = solver.solve(target, q, retries=2, rng=np.random.default_rng(7))
    second = solver.solve(target, q, retries=2, rng=np.random.default_rng(7))
    assert not first.success and first.error > 1.
    assert first.q == pytest.approx(second.q)


def test_cartesian_auto_duration_endpoints_and_retiming():
    solver = PoseKinematics()
    q = np.array((0.,-1.,-1.,0.,.3,0.))
    target = solver.pose(q+np.array((.05,0.,0.,0.,0.,0.)))
    points, duration = plan_cartesian(solver, q, target)
    assert duration >= 1. and points[0] == pytest.approx(q)
    assert np.linalg.norm(pin.log6(solver.pose(points[-1]).actInv(target)).vector) <= 1e-4
    command = retime_path(solver, points, duration)
    assert command['points'][0]['elapsed_sec'] == 0.
    assert command['points'][-1]['elapsed_sec'] == command['duration_sec']
    assert command['duration_sec'] >= duration
    assert command['points'][-1]['positions'] == pytest.approx(points[-1])


def test_any_cartesian_sample_failure_rejects_whole_path(monkeypatch):
    solver = PoseKinematics()
    seed = (0.,-1.,-1.,0.,.3,0.)
    solve = solver.solve
    calls = []
    def injected(*args, **kwargs):
        calls.append(1)
        result = solve(*args, **kwargs)
        return type(result)(len(calls)<3, result.q, result.error, result.iterations)
    monkeypatch.setattr(solver, 'solve', injected)
    with pytest.raises(ValueError, match='sample 3'):
        plan_cartesian(solver, seed, solver.pose(seed), .1)


def test_no_sdk_imports_or_path_injection_in_controller_and_motion():
    for package in ('rebotarmcontroller', 'rebotarm_motion'):
        for path in (ROOT/'src'/package/package).glob('*.py'):
            source = path.read_text()
            assert 'reBotArm_control_py' not in source, path
            tree = ast.parse(source)
            assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                and ast.unparse(n.func) in ('sys.path.insert', 'sys.path.append') for n in ast.walk(tree)), path
    assert not (ROOT/'src/rebotarmcontroller/rebotarmcontroller/legacy_backend.py').exists()


def test_source_imports_and_computation_with_sdk_blocked_in_fresh_process():
    script = '''
import importlib.abc, sys
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, name, *args):
        if name.split('.')[0] == 'reBotArm_control_py':
            raise AssertionError('SDK import attempted: '+name)
sys.meta_path.insert(0, Block())
from rebotarmcontroller.hardware_manager import HardwareManager
from rebotarm_motion.pose_preview_solver import PosePreviewSolver
from rebotarm_motion.cartesian_trajectory import plan_cartesian
from rebotarm_motion.pose_compat_node import PoseCompatNode
manager = HardwareManager()
assert not manager.connected
solver = PosePreviewSolver()
q = (0., -1., -1., 0., .3, 0.)
target = solver.kinematics.pose(q)
assert solver.kinematics.solve(target, q).success
assert plan_cartesian(solver.kinematics, q, target, .1)[0]
'''
    env = dict(os.environ)
    env['PYTHONPATH'] = ':'.join(str(ROOT/'src'/p) for p in ('rebotarmcontroller', 'rebotarm_motion')) + ':' + env.get('PYTHONPATH','')
    subprocess.run([sys.executable, '-c', script], cwd='/tmp', env=env, check=True, timeout=20)


@pytest.mark.parametrize('case', json.loads((ROOT/'tests/fixtures/controller_pose_vectors.json').read_text())['cases'])
def test_fixed_sdk_comparison_vectors(case):
    solver = PoseKinematics()
    target = pin.SE3(np.asarray(case['target']))
    result = solver.solve(target, case['seed'])
    assert result.success == case['new_success'] == case['old_success']
    assert result.error == pytest.approx(case['new_error'], abs=1e-8)
    if result.success:
        assert result.q == pytest.approx(case['old_q'], abs=1e-7)
