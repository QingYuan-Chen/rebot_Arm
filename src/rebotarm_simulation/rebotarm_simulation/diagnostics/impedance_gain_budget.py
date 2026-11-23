"""仿真关节阻抗增益的力矩预算推导与检查(离线数值诊断).

职责
    用权威 URDF 的 effort 限幅、标定文件的电机额定力矩和 MuJoCo 模型的
    实际动力学,调用 `rebotarm_force_control` 的纯预算计算与边界判定,
    推导并检查仿真示例推荐增益.它是纯数值工具,不参与运行期控制,也不改变任何状态.
    注意:RECOMMENDED_STIFFNESS / RECOMMENDED_DAMPING 仅适用于当前仿真模型,
    后续真机阻抗刚度/阻尼参数与之完全分开.

对外接口
    PROTOCOL_KP_MAX / PROTOCOL_KD_MAX / REFERENCE_DEFLECTION_RAD / DAMPING_RATIO
    evaluate_gains / budget_violations
        透传自 `rebotarm_force_control`（`mit_command` / `joint_impedance_budget`）的协议上限与通用预算函数.
    HOME_POSE / REFERENCE_POSES
        仿真求惯量和重力时使用的参考姿态,单位为 rad,顺序 joint1..joint6.
    RECOMMENDED_STIFFNESS / RECOMMENDED_DAMPING
        由仿真 derive() 得出的仿真建议增益;仿真示例、测试与文档引用这两个常量.
    reference_dynamics(model_path=None) -> tuple[ndarray, ndarray]
        从 MuJoCo 采参考姿态重力与 Home 对角惯量.
    derive(model_path=None) -> dict
        从模型与仿真配置推导建议增益,并给出对应预算报告.
    main(argv=None) -> int
        打印 JSON 报告;出现违规或建议增益与常量不一致时返回 1.

安全约束
    本模块不创建 ROS 客户端、不打开串口、不下发轨迹或力矩,只读模型与配置.
    报告只是仿真力矩预算;它不构成真机验收,真机摩擦、回差与温升未建模.
    参考姿态外的构型未覆盖,换末端负载后必须重跑.

文件结构
    仿真常量 → MuJoCo 动力学采样 → 增益推导 → CLI 入口.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from rebotarm_force_control.joint_impedance_budget import (
    DAMPING_RATIO,
    REFERENCE_DEFLECTION_RAD,
    budget_violations,
    evaluate_gains,
)
from rebotarm_force_control.mit_command import PROTOCOL_KD_MAX, PROTOCOL_KP_MAX
from rebotarm_simulation.control.control_config import load_motor_control_parameters


# Home 姿态与现有阻抗测试的姿态一致;求阻尼用的对角惯量只取 Home 姿态.
HOME_POSE = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)
# 重力项取多个姿态中的最不利值,覆盖零位、Home 与腕部朝下三种构型.
REFERENCE_POSES = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    HOME_POSE,
    (0.0, -1.2, -0.6, 0.6, 0.0, 0.0),
)

# 仿真推导结果;K 取额定 9/3.5 N·m 除以 0.2 rad,D 取目标阻尼比并截断到协议上限.
RECOMMENDED_STIFFNESS = (45.0, 45.0, 45.0, 17.5, 17.5, 17.5)
RECOMMENDED_DAMPING = (3.32, 5.0, 5.0, 1.28, 0.78, 0.34)


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


def derive(model_path=None):
    """推导仿真建议增益并给出完整预算报告.

    刚度 = 额定力矩 / 参考偏角;阻尼 = min(协议上限, 2ζ·sqrt(K·M)),M 取 Home 姿态
    对角惯量.刚度本身由额定力矩决定,因此弹簧项恰好等于额定力矩.
    """
    parameters = load_motor_control_parameters()
    rated = np.asarray(parameters.arm.rated_torque, dtype=float)
    effort = np.asarray(parameters.arm.effort_limit, dtype=float)
    gravity, inertia = reference_dynamics(model_path)
    stiffness = rated / REFERENCE_DEFLECTION_RAD
    damping = np.minimum(
        PROTOCOL_KD_MAX,
        np.round(2.0 * DAMPING_RATIO * np.sqrt(stiffness * inertia), 2),
    )
    report = evaluate_gains(stiffness, damping, rated, effort, gravity, inertia)
    report["rated_torque_nm"] = rated.tolist()
    report["effort_limit_nm"] = effort.tolist()
    report["recommended_stiffness"] = stiffness.tolist()
    report["recommended_damping"] = damping.tolist()
    report["violations"] = budget_violations(report, rated, effort)
    report["matches_documented_profile"] = (
        bool(np.allclose(stiffness, RECOMMENDED_STIFFNESS, atol=1e-9))
        and bool(np.allclose(damping, RECOMMENDED_DAMPING, atol=1e-9))
    )
    return report


def main(argv=None):
    """打印预算报告;有违规或文档常量与实际推导不一致时返回 1."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", help="MJCF 场景路径,默认使用仿真包内置模型")
    parser.add_argument("--output", type=Path, help="可选的 JSON 报告输出路径")
    args = parser.parse_args(argv)
    report = derive(args.model)
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output is None:
        print(text)
    else:
        args.output.write_text(text + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    for item in report["violations"]:
        print(f"budget violation: {item}", file=sys.stderr)
    if not report["matches_documented_profile"]:
        print(
            "recommended gains drifted from RECOMMENDED_STIFFNESS/RECOMMENDED_DAMPING: "
            f'{report["recommended_stiffness"]} / {report["recommended_damping"]}',
            file=sys.stderr,
        )
        return 1
    return 1 if report["violations"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
