#!/usr/bin/env python3
"""离线核对规范模型 FK, 默认读取固定参考数值, 不连接设备.

职责与位置
    本脚本是控制器退出旧 SDK 后的数值回归工具, 不导入旧 SDK 的 Python 实现.

对外接口与文件结构
    load_reference 校验固定样本; compare 返回逐输入误差及来源; main 提供命令行入口.
    无参数时只读规范 URDF 和版本化 JSON; --legacy-urdf 显式选择实时模型对照.

约束
    单位为 rad 和 m, 旋转矩阵表示 end_link 在 base_link 中的方向.
    不生成或修复期望值, 不搜索历史目录. 样本错误与模型缺失均失败;
    两种误差均须严格小于 1e-12. 数值一致不构成实机运动验收.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np
import pinocchio as pin

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rebotarmcontroller'))
sys.path.insert(0, str(ROOT / 'src/rebotarm_force_control'))
from rebotarm_force_control.gravity_control import load_gravity_model, GravitySettings
from rebotarmcontroller.arm.controller_kinematics import ArmKinematics

REFERENCE = ROOT / 'tests/fixtures/controller_fk_reference_v1.json'
TOLERANCE = 1e-12
SAMPLES = ((0., 0., 0., 0., 0., 0.), (0., 0., -.017453292519943295, 0., 0., 0.),
           (.2, -.8, -1.1, .3, -.2, .4), (-.4, -1.4, -.5, -.5, .5, -.8))


def load_reference(path=REFERENCE):
    """读取并验证版本, 来源及四组有限数值; 损坏时拒绝, 不访问历史模型补救."""
    value = json.loads(Path(path).read_text())

    def require(condition):
        if not condition:
            raise ValueError('reference contract mismatch')

    try:
        require(value['schema_version'] == 1)
        require(value['canonical_joints'] == [f'joint{i}' for i in range(1, 7)])
        require(value['legacy_joints'] == ['joint1', 'joint2', 'join3', 'joint4', 'joint5', 'joint6'])
        require(value['joint_name_mapping'] == {'join3': 'joint3'})
        require(value['base_frame'] == 'base_link' and value['end_frame'] == 'end_link')
        require(value['units'] == {'q': 'rad', 'translation': 'm', 'rotation': 'dimensionless'})
        for key, length in (('reference_sdk_commit', 40), ('reference_urdf_sha256', 64)):
            require(len(value[key]) == length and all(c in '0123456789abcdef' for c in value[key]))
        require(value['reference_urdf'] and value['pinocchio_version'])
        require(len(value['samples']) == len(SAMPLES))
        for sample, q in zip(value['samples'], SAMPLES):
            require(sample['q'] == list(q))
            translation, rotation = np.asarray(sample['translation']), np.asarray(sample['rotation'])
            require(translation.shape == (3,) and rotation.shape == (3, 3))
            require(np.isfinite(translation).all() and np.isfinite(rotation).all())
            require(np.allclose(rotation.T @ rotation, np.eye(3), atol=TOLERANCE, rtol=0))
            require(abs(np.linalg.det(rotation) - 1.) < TOLERANCE)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'invalid FK reference: {path}') from exc
    return value


def compare(legacy_urdf=None, *, reference_path=REFERENCE):
    """比较规范 FK 与固定期望值, 或与显式给定的六轴历史模型比较; 不改样本."""
    reference = load_reference(reference_path)
    canonical = ArmKinematics(load_gravity_model(
        ROOT / 'src/rebotarm_moveit_config/config/rebotarm.urdf', GravitySettings()).model)
    source = dict(kind='fixed_reference', path=str(Path(reference_path).resolve()),
                  urdf_sha256=reference['reference_urdf_sha256'])
    legacy = None
    if legacy_urdf is not None:
        path = Path(legacy_urdf).expanduser().resolve(strict=True)
        source = dict(kind='explicit_urdf', path=str(path),
                      urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        legacy = pin.buildModelFromUrdf(str(path))
        if (legacy.nq != 6 or legacy.nv != 6 or list(legacy.names)[1:] != reference['legacy_joints']
                or not all(legacy.existFrame(reference[key]) for key in ('base_frame', 'end_frame'))):
            raise ValueError('explicit legacy model must match the recorded six-joint order and end frame')
    results = []
    for sample in reference['samples']:
        q = sample['q']
        old_position, old_rotation = np.asarray(sample['translation']), np.asarray(sample['rotation'])
        if legacy is not None:
            data = legacy.createData()
            pin.framesForwardKinematics(legacy, data, np.asarray(q))
            old = data.oMf[legacy.getFrameId(reference['base_frame'])].actInv(
                data.oMf[legacy.getFrameId(reference['end_frame'])])
            old_position, old_rotation = old.translation, old.rotation
        position, rotation = canonical.pose(q)
        results.append(dict(q=q, translation_delta_m=(position - old_position).tolist(),
                            position_error_m=float(np.linalg.norm(position - old_position)),
                            rotation_error_rad=float(np.linalg.norm(pin.log3(old_rotation.T @ rotation)))))
    return dict(canonical_joints=list(canonical.model.names)[1:], legacy_joints=reference['legacy_joints'],
                reference=source, samples=results,
                max_position_error_m=max(r['position_error_m'] for r in results),
                max_rotation_error_rad=max(r['rotation_error_rad'] for r in results))


def main(argv=None):
    """打印误差及来源; 输入错误返回 2, 超差返回 1, 数值通过返回 0."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--legacy-urdf', type=Path, help='显式提供历史 URDF; 默认只校验固定样本')
    args = parser.parse_args(argv)
    try:
        result = compare(args.legacy_urdf)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if (result['max_position_error_m'] < TOLERANCE
                 and result['max_rotation_error_rad'] < TOLERANCE) else 1


if __name__ == '__main__':
    sys.exit(main())
