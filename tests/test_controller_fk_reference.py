"""FK 固定样本及显式模型对照的离线失败用例；不需要旧 SDK 或设备。"""
import copy
import importlib.util
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def probe():
    spec = importlib.util.spec_from_file_location('fk_reference_probe', ROOT / 'scripts/check_controller_fk.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_reference(tmp_path, probe, edit):
    value = copy.deepcopy(probe.load_reference())
    edit(value)
    path = tmp_path / 'reference.json'
    path.write_text(json.dumps(value))
    return path


def test_default_cli_checks_frozen_values(probe, capsys):
    assert probe.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['reference']['kind'] == 'fixed_reference'
    assert len(report['samples']) == 4


@pytest.mark.parametrize('change', [
    lambda value: value.update(schema_version=2),
    lambda value: value['samples'].pop(),
    lambda value: value['samples'][0].update(q=[0.] * 5),
    lambda value: value['samples'][0].update(translation=[float('nan'), 0., 0.]),
    lambda value: value['samples'][0].update(rotation=np.zeros((3, 3)).tolist()),
    lambda value: value.update(legacy_joints=value['canonical_joints']),
    lambda value: value.pop('reference_urdf_sha256'),
])
def test_damaged_reference_is_rejected(probe, tmp_path, change):
    path = write_reference(tmp_path, probe, change)
    with pytest.raises(ValueError, match='invalid FK reference'):
        probe.compare(reference_path=path)


def test_missing_or_invalid_json_has_no_legacy_fallback(probe, tmp_path):
    path = tmp_path / 'missing.json'
    with pytest.raises(FileNotFoundError):
        probe.compare(reference_path=path)
    path.write_text('{broken')
    with pytest.raises(ValueError):
        probe.compare(reference_path=path)


def test_numerical_mismatch_makes_command_fail(probe, tmp_path, monkeypatch, capsys):
    def shift(value):
        value['samples'][0]['translation'][0] += 1e-6
    path = write_reference(tmp_path, probe, shift)
    compare = probe.compare
    monkeypatch.setattr(probe, 'compare', lambda _: compare(reference_path=path))
    assert probe.main([]) == 1
    assert json.loads(capsys.readouterr().out)['max_position_error_m'] > 1e-7


def test_explicit_missing_model_is_not_replaced(probe, tmp_path):
    with pytest.raises(SystemExit) as error:
        probe.main(['--legacy-urdf', str(tmp_path / 'missing.urdf')])
    assert error.value.code == 2


def test_explicit_model_reports_actual_source_and_failure(probe, tmp_path, capsys):
    # 简单六轴链作为不同模型的反例，避免测试重新依赖厂商参考树。
    names = probe.load_reference()['legacy_joints']
    xml = ['<robot name="different"><link name="base_link"/>']
    parent = 'base_link'
    for name in names:
        child = 'end_link' if name == names[-1] else name + '_link'
        xml.append(f'<link name="{child}"/><joint name="{name}" type="revolute">'
                   f'<parent link="{parent}"/><child link="{child}"/>'
                   '<axis xyz="0 0 1"/><limit lower="-6" upper="6" effort="1" velocity="1"/>'
                   '</joint>')
        parent = child
    path = tmp_path / 'different.urdf'
    path.write_text(''.join([*xml, '</robot>']))
    assert probe.main(['--legacy-urdf', str(path)]) == 1
    report = json.loads(capsys.readouterr().out)
    assert report['reference']['kind'] == 'explicit_urdf'
    assert report['reference']['path'] == str(path.resolve())
    assert report['reference']['urdf_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert report['max_position_error_m'] > 1e-3


def test_explicit_incompatible_model_is_rejected(probe, tmp_path):
    path = tmp_path / 'incompatible.urdf'
    path.write_text('<robot name="empty"><link name="base_link"/></robot>')
    with pytest.raises(SystemExit) as error:
        probe.main(['--legacy-urdf', str(path)])
    assert error.value.code == 2
