"""关节阻抗增益的力矩预算推导与检查(离线数值诊断).

职责
    用权威 URDF 的 effort 限幅、标定文件的电机额定力矩和 MuJoCo 模型的
    实际动力学,把六轴关节阻抗的刚度/阻尼换算成可检查的力矩预算:
    每个轴的弹簧项、重力项、阻尼项各占额定与限幅的多少,并判断建议增益是否
    落在这些边界内.它是纯数值工具,不参与运行期控制,也不改变任何状态.

对外接口
    REFERENCE_DEFLECTION_RAD / DAMPING_RATIO / PROTOCOL_KP_MAX / PROTOCOL_KD_MAX
        整定规则常量;刚度按"额定力矩 / 参考偏角"取,阻尼按目标阻尼比换算,
        并受达妙 DM 系列 MIT 协议 Kp<=500 / Kd<=5 约束.
    HOME_POSE / REFERENCE_POSES
        求惯量和重力时使用的参考姿态,单位为 rad,顺序 joint1..joint6.
    RECOMMENDED_STIFFNESS / RECOMMENDED_DAMPING
        由 derive() 得出的建议增益;示例、测试与文档一律引用这两个常量.
    evaluate_gains(...) -> dict
        用给定动力学数据检查一组增益,返回逐轴预算与违规项.
    derive(...) -> dict
        从模型与配置推导建议增益,并给出对应预算报告.
    main(argv=None) -> int
        打印 JSON 报告;出现违规或建议增益与常量不一致时返回 1.

安全约束
    本模块不创建 ROS 客户端、不打开串口、不下发轨迹或力矩,只读模型与配置.
    报告只是仿真力矩预算;它不构成真机验收,真机摩擦、回差与温升未建模.
    参考姿态外的构型未覆盖,换末端负载后必须重跑.

文件结构
    常量 → 动力学采样 → 增益换算 → 预算检查 → 报告与 CLI 入口.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from rebotarm_simulation.control.control_config import load_motor_control_parameters


# 参考偏角:额定力矩恰好在偏差达到该角度时用满.取值越大刚度的"软"程度越高.
REFERENCE_DEFLECTION_RAD = 0.2
# 目标阻尼比.低于 1 允许轻微超调,换取更短的回稳时间.
DAMPING_RATIO = 0.7
# 达妙 DM 系列 MIT 帧的增益范围,见官方手册 "Kp range: [0,500], Kd range: [0,5]".
PROTOCOL_KP_MAX = 500.0
PROTOCOL_KD_MAX = 5.0

# Home 姿态与现有阻抗测试的姿态一致;求阻尼用的对角惯量只取 Home 姿态.
HOME_POSE = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)
# 重力项取多个姿态中的最不利值,覆盖零位、Home 与腕部朝下三种构型.
REFERENCE_POSES = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    HOME_POSE,
    (0.0, -1.2, -0.6, 0.6, 0.0, 0.0),
)

# 推导结果;K 取额定 9/3.5 N·m 除以 0.2 rad,D 取目标阻尼比并截断到协议上限.
RECOMMENDED_STIFFNESS = (45.0, 45.0, 45.0, 17.5, 17.5, 17.5)
RECOMMENDED_DAMPING = (3.32, 5.0, 5.0, 1.28, 0.78, 0.34)

ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def reference_dynamics(model_path=None):
    """返回各参考姿态的纯重力力矩与 Home 姿态对角惯量.

    model_path 为 None 时按仿真包的模型搜索顺序取默认场景;接触体被关闭,
    因为这里只读运动学与质量属性,不需要碰撞.
    """
    # 延迟导入:MuJoCo 只是诊断依赖,不在模块导入期要求.
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    gravity = []
    inertia = None
    with RebotArmMujoco(model_path, collisionless=True) as sim:
        model, data = sim.borrow_viewer_handles()
        mj = sim._mj
        dof_addresses = [int(model.jnt_dofadr[sim._joint_ids[index]]) for index in range(6)]
        for pose in REFERENCE_POSES:
            for index, joint_id in enumerate(sim._joint_ids[:6]):
                data.qpos[int(model.jnt_qposadr[joint_id])] = pose[index]
            data.qvel[:] = 0.0
            mj.mj_forward(model, data)
            gravity.append(np.asarray(data.qfrc_bias[dof_addresses], dtype=float).copy())
            if pose == HOME_POSE:
                mass = np.zeros((model.nv, model.nv))
                mj.mj_fullM(model, mass, data.qM)
                inertia = np.diag(mass)[dof_addresses].copy()
    return np.abs(np.vstack(gravity)), inertia


def evaluate_gains(stiffness, damping, rated, effort, gravity, inertia,
                   deflection=REFERENCE_DEFLECTION_RAD):
    """按给定增益和动力学数据算出力矩预算;输入维度不符时直接报错.

    rated/effort/gravity/inertia 的顺序必须与 ARM_JOINT_NAMES 一致.
    gravity 可以是单个姿态的六轴向量,也可以是 (姿态数, 6) 的矩阵,按轴取最大值.
    """
    stiffness = np.asarray(stiffness, dtype=float)
    damping = np.asarray(damping, dtype=float)
    rated = np.asarray(rated, dtype=float)
    effort = np.asarray(effort, dtype=float)
    gravity = np.asarray(gravity, dtype=float)
    inertia = np.asarray(inertia, dtype=float)
    for value, label in ((stiffness, 'stiffness'), (damping, 'damping'),
                         (rated, 'rated torque'), (effort, 'effort'),
                         (inertia, 'inertia')):
        if value.shape != (6,) or not np.isfinite(value).all():
            raise ValueError(f'{label} must contain six finite values')
    if gravity.ndim == 1:
        gravity = gravity.reshape(1, 6)
    if gravity.ndim != 2 or gravity.shape[1] != 6 or not np.isfinite(gravity).all():
        raise ValueError('gravity must be one or more six-axis rows')
    if (rated <= 0).any() or (effort <= 0).any() or (inertia <= 0).any():
        raise ValueError('rated torque, effort and inertia must be positive')
    if deflection <= 0:
        raise ValueError('reference deflection must be positive')

    # 按轴取重力最不利值,其余项逐轴计算.
    worst_gravity = np.abs(gravity).max(axis=0)
    spring = stiffness * deflection
    natural = np.sqrt(stiffness / inertia)
    zeta = damping / (2.0 * np.sqrt(stiffness * inertia))
    gravity_parts = [dict(joint=name, gravity_nm=float(g))
                     for name, g in zip(ARM_JOINT_NAMES, worst_gravity)]
    return {
        'deflection_rad': float(deflection),
        'stiffness': stiffness.tolist(),
        'damping': damping.tolist(),
        'spring_nm': spring.tolist(),
        'spring_over_rated': (spring / rated).tolist(),
        'spring_plus_gravity_nm': (spring + worst_gravity).tolist(),
        'spring_plus_gravity_over_effort': ((spring + worst_gravity) / effort).tolist(),
        'damping_at_1_rad_s_over_rated': (damping / rated).tolist(),
        'natural_frequency_hz': (natural / (2.0 * np.pi)).tolist(),
        'damping_ratio': zeta.tolist(),
        'worst_gravity_nm': worst_gravity.tolist(),
        'gravity_over_rated': (worst_gravity / rated).tolist(),
        'gravity_per_pose': gravity_parts,
        'violations': [],  # 下方按三条边界逐轴汇总.
    }


def budget_violations(report, rated, effort):
    """列出违反边界的具体项;空列表表示全部通过.

    边界:弹簧项不超过额定;弹簧加重力不超过 URDF 限幅;增益不超过协议范围.
    纯重力接近额定只作为报告项,不在这里判失败.
    """
    rated = np.asarray(rated, dtype=float)
    effort = np.asarray(effort, dtype=float)
    violations = []
    for index, name in enumerate(ARM_JOINT_NAMES):
        if report['spring_nm'][index] > rated[index] + 1e-9:
            violations.append(f'{name}: spring {report["spring_nm"][index]:.2f} N·m '
                              f'exceeds rated {rated[index]:.2f} N·m')
        if report['spring_plus_gravity_nm'][index] > effort[index] + 1e-9:
            violations.append(f'{name}: spring+gravity '
                              f'{report["spring_plus_gravity_nm"][index]:.2f} N·m '
                              f'exceeds effort {effort[index]:.2f} N·m')
        if report['stiffness'][index] > PROTOCOL_KP_MAX:
            violations.append(f'{name}: stiffness exceeds MIT protocol limit '
                              f'{PROTOCOL_KP_MAX:.0f}')
        if report['damping'][index] > PROTOCOL_KD_MAX + 1e-9:
            violations.append(f'{name}: damping exceeds MIT protocol limit '
                              f'{PROTOCOL_KD_MAX:.1f}')
    return violations


def derive(model_path=None):
    """推导建议增益并给出完整预算报告.

    刚度 = 额定力矩 / 参考偏角;阻尼 = min(协议上限, 2ζ·sqrt(K·M)),M 取 Home 姿态
    对角惯量.刚度本身由额定力矩决定,因此弹簧项恰好等于额定力矩.
    """
    parameters = load_motor_control_parameters()
    rated = np.asarray(parameters.arm.rated_torque, dtype=float)
    effort = np.asarray(parameters.arm.effort_limit, dtype=float)
    gravity, inertia = reference_dynamics(model_path)
    stiffness = rated / REFERENCE_DEFLECTION_RAD
    damping = np.minimum(PROTOCOL_KD_MAX,
                         np.round(2.0 * DAMPING_RATIO * np.sqrt(stiffness * inertia), 2))
    report = evaluate_gains(stiffness, damping, rated, effort, gravity, inertia)
    report['rated_torque_nm'] = rated.tolist()
    report['effort_limit_nm'] = effort.tolist()
    report['recommended_stiffness'] = stiffness.tolist()
    report['recommended_damping'] = damping.tolist()
    report['violations'] = budget_violations(report, rated, effort)
    report['matches_documented_profile'] = (
        bool(np.allclose(stiffness, RECOMMENDED_STIFFNESS, atol=1e-9))
        and bool(np.allclose(damping, RECOMMENDED_DAMPING, atol=1e-9)))
    return report


def main(argv=None):
    """打印预算报告;有违规或文档常量与实际推导不一致时返回 1."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', help='MJCF 场景路径,默认使用仿真包内置模型')
    parser.add_argument('--output', type=Path, help='可选的 JSON 报告输出路径')
    args = parser.parse_args(argv)
    report = derive(args.model)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is None:
        print(text)
    else:
        args.output.write_text(text + '\n', encoding='utf-8')
        print(f'wrote {args.output}')
    for item in report['violations']:
        print(f'budget violation: {item}', file=sys.stderr)
    if not report['matches_documented_profile']:
        print('recommended gains drifted from RECOMMENDED_STIFFNESS/RECOMMENDED_DAMPING: '
              f'{report["recommended_stiffness"]} / {report["recommended_damping"]}',
              file=sys.stderr)
        return 1
    return 1 if report['violations'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
