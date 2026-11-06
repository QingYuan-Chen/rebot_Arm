"""位姿纯计算及 SDK 退出：规范限位、不可达、采样失败和隔离导入。"""
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import numpy as np
import pinocchio as pin
import pytest
from rebotarm_motion.pose_kinematics import JOINT_NAMES, PoseKinematics
from rebotarm_motion.cartesian_trajectory import plan_cartesian, retime_path

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('q', [(0.,-1.,-1.,0.,.3,0.), (.2,.019,-1.,.2,.1,-.2), (0.,0.,0.,0.,0.,0.)])
def test_fk_ik_roundtrip_and_canonical_upper_limits(q):
    solver = PoseKinematics()
    result = solver.solve(solver.pose(q), q)
    assert result.success and result.error <= 1e-4
    assert result.q == pytest.approx(q)
    assert solver.upper[1:3] == pytest.approx((.02,.02))


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
        for path in (ROOT/'src'/package).rglob('*.py'):
            tree = ast.parse(path.read_text())
            docstrings = {ast.get_docstring(node, clean=False) for node in ast.walk(tree)
                          if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))}
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    assert all(alias.name.split('.')[0] != 'reBotArm_control_py' for alias in node.names), path
                if isinstance(node, ast.ImportFrom):
                    assert (node.module or '').split('.')[0] != 'reBotArm_control_py', path
                if isinstance(node, ast.Constant) and isinstance(node.value, str):
                    if node.value not in docstrings:
                        assert 'reBotArm_control_py' not in node.value, path
                if isinstance(node, ast.Call):
                    assert ast.unparse(node.func) not in ('sys.path.insert', 'sys.path.append'), path
    assert not (ROOT/'src/rebotarmcontroller/rebotarmcontroller/legacy_backend.py').exists()


def test_active_scripts_and_manifests_have_no_implicit_legacy_resource_path():
    # 来源文档和固定数值元数据不参与执行依赖检查; 禁止名单中的纯模块名也不是资源读取.
    paths = [p for folder in ('src', 'scripts') for p in (ROOT/folder).rglob('*')
             if p.suffix in {'.py', '.sh', '.xml', '.yaml', '.cfg', '.toml', '.repos'}]
    for path in paths:
        text = path.read_text()
        if path.suffix == '.py':
            tree = ast.parse(text)
            docstrings = {ast.get_docstring(node, clean=False) for node in ast.walk(tree)
                          if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))}
            strings = [node.value for node in ast.walk(tree)
                       if isinstance(node, ast.Constant) and isinstance(node.value, str)
                       and node.value not in docstrings]
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    assert all(alias.name.split('.')[0] != 'reBotArm_control_py' for alias in node.names), path
                if isinstance(node, ast.ImportFrom):
                    assert (node.module or '').split('.')[0] != 'reBotArm_control_py', path
        else:
            strings = [text]
        assert not any(re.search(r'(?:[/\\]reBotArm_control_py|reBotArm_control_py[/\\])',
                                 value) for value in strings), path


def test_source_imports_and_computation_with_sdk_blocked_in_fresh_process():
    script = (ROOT/'tests/fixtures/controller_sdk_exit_probe.py').read_text()
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


@pytest.mark.parametrize('package,relative', [
    ('rebotarmcontroller', 'config/arm.yaml'),
    ('rebotarmcontroller', 'config/gripper.yaml'),
    ('rebotarmcontroller', 'config/gravity_compensation.yaml'),
    ('rebotarm_moveit_config', 'config/rebotarm.urdf'),
])
def test_owned_resource_uses_source_when_install_index_is_missing(monkeypatch, package, relative):
    """只隔离安装索引，确保目录迁移后的源码回退深度仍能找到实际资源。"""
    import ament_index_python.packages
    from rebotarmcontroller.gravity.gravity_control import package_file

    def missing_index(name):
        raise LookupError(name)

    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', missing_index)
    assert package_file(package, relative) == ROOT/'src'/package/relative


@pytest.mark.parametrize('package,relative', [
    ('rebotarmcontroller', 'config/arm.yaml'),
    ('rebotarmcontroller', 'config/gripper.yaml'),
    ('rebotarmcontroller', 'config/gravity_compensation.yaml'),
    ('rebotarm_moveit_config', 'config/rebotarm.urdf'),
])
def test_owned_resource_prefers_install_index_to_existing_source(tmp_path, monkeypatch, package, relative):
    """让源码与安装资源同时存在，必须选安装索引，避免本机源码掩盖安装错误。"""
    import ament_index_python.packages
    from rebotarmcontroller.gravity.gravity_control import package_file

    share = tmp_path/'share'/package
    installed = share/relative
    installed.parent.mkdir(parents=True)
    installed.write_text('installed resource')
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(tmp_path/'share'/name))
    assert package_file(package, relative) == installed


@pytest.mark.parametrize('package,relative', [
    ('rebotarmcontroller', 'config/arm.yaml'),
    ('rebotarmcontroller', 'config/gripper.yaml'),
    ('rebotarmcontroller', 'config/gravity_compensation.yaml'),
    ('rebotarm_moveit_config', 'config/rebotarm.urdf'),
])
def test_missing_owned_resource_fails_without_legacy_fallback(tmp_path, monkeypatch, package, relative):
    import ament_index_python.packages
    from rebotarmcontroller.gravity import gravity_control
    from rebotarm_motion import pose_kinematics
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory',
                        lambda name: str(tmp_path/'install/share'/name))
    # 同时隔离源码回退, 否则当前机器的正常资源会掩盖缺失安装文件.
    monkeypatch.setattr(gravity_control, '__file__',
                        str(tmp_path/'missing/src/rebotarmcontroller/rebotarmcontroller/gravity/gravity_control.py'))
    monkeypatch.setattr(pose_kinematics, '__file__', str(tmp_path/'missing/src/pkg/module.py'))
    with pytest.raises(FileNotFoundError):
        gravity_control.package_file(package, relative)
    if package == 'rebotarm_moveit_config':
        with pytest.raises(FileNotFoundError):
            pose_kinematics.PoseKinematics()
