"""抓取候选的"先验几何闸门"（不依赖 IK 的快速否决）。

在系统中的位置：抓取候选 IK 过滤器在每个候选送入 IK/碰撞检查之前调用本模块。
先做便宜的数值检查（夹爪开度、离地高度、工作空间范围），可以把明显不可行的候选
提前剔除，避免为它们付出昂贵的 IK 与碰撞检查开销。

判定顺序（短路返回，第一个不通过的原因即最终 reason）：
    1. 夹爪开度下界 -> "jaw_width too small"
    2. 夹爪开度上界 -> "jaw_width too large"
    3. 抓取点 Z 下界（防撞台面）-> "grasp z too low"
    4. 工作空间包围盒与"抓取点到物体中心距离"（默认关闭）-> 由工作空间闸门给出原因

单位约定：所有长度为米，坐标系为候选位姿所在的工作坐标系（通常为 base_link）。
本模块是纯函数、无 ROS 依赖，不做任何运动下发。
"""

from __future__ import annotations

from dataclasses import dataclass

from .candidate_workspace_gate import CandidateWorkspaceGateConfig, candidate_workspace_gate


@dataclass(frozen=True)
class CandidateGateConfig:
    """闸门阈值配置。

    min_jaw_width_m: 夹爪最小开度，默认 0.006 m（6 mm）。
        小于此值说明夹爪几乎闭合，通常意味着候选张开了也不会夹住物体，
        或与物体/台面几何矛盾；调大会更严格。
    max_jaw_width_m: 夹爪最大开度，默认 0.085 m（85 mm，实测最大张开行程附近）。
        超过即物理上夹不住；调大会放进夹不住的候选。
    min_grasp_z_m: 抓取点最低高度，默认 0.0 m（即工作坐标系地面）。
        抬高该值相当于加了一道"防撞台面"的保守门限；<= 0 时不额外限制。
    workspace_gate_enabled: 是否启用工作空间包围盒检查，默认 False（关闭）。
    workspace_min_xyz / workspace_max_xyz: 包围盒最小/最大角点，单位米，仅在上项为 True 时生效。
        默认 (0.18, -0.35, 0.0) ~ (0.64, 0.35, 0.45)，
        注意这是 base_link 下"前方偏一侧"的区间，与 grasp_pose_policy.yaml 中的
        candidate_workspace_min_xyz/max_xyz 取值不同，实际生效值由节点参数传入。
    max_grasp_to_object_center_m: 抓取点到物体中心的最大允许距离，默认 0.15 m。
        防止把抓取点放到物体轮廓之外（例如深度分割边缘的离群点）；
        仅在工作空间闸门启用且该值 > 0 时生效。
    """

    min_jaw_width_m: float = 0.006
    max_jaw_width_m: float = 0.085
    min_grasp_z_m: float = 0.0
    workspace_gate_enabled: bool = False
    workspace_min_xyz: tuple[float, float, float] = (0.18, -0.35, 0.0)
    workspace_max_xyz: tuple[float, float, float] = (0.64, 0.35, 0.45)
    max_grasp_to_object_center_m: float = 0.15


@dataclass(frozen=True)
class CandidateGateResult:
    """闸门结果。

    accepted: 是否通过全部检查。
    reason: 未通过时的英文原因文本（会原样进入日志与统计），通过时为空串。
    """

    accepted: bool
    reason: str = ""


def evaluate_candidate_gate(
    *,
    jaw_width_m: float,
    grasp_position_xyz: tuple[float, float, float],
    object_center_xyz: tuple[float, float, float] | None,
    config: CandidateGateConfig = CandidateGateConfig(),
) -> CandidateGateResult:
    """对单个候选执行先验几何闸门检查。

    参数:
        jaw_width_m: 候选所需夹爪开度，单位米。
        grasp_position_xyz: 抓取点位置 (x, y, z)，单位米，位于工作坐标系。
        object_center_xyz: 物体中心位置，单位米；为 None 时跳过"抓取点离物体中心过远"检查。
        config: 阈值配置。
    返回:
        CandidateGateResult；任一检查失败立即返回 False 及其原因。
    """
    width = float(jaw_width_m)
    if width < float(config.min_jaw_width_m):
        return CandidateGateResult(False, f"jaw_width too small ({width:.3f}m)")
    if width > float(config.max_jaw_width_m):
        return CandidateGateResult(False, f"jaw_width too large ({width:.3f}m)")

    grasp_z = float(grasp_position_xyz[2])
    if grasp_z < float(config.min_grasp_z_m):
        return CandidateGateResult(False, f"grasp z too low ({grasp_z:.3f}m)")

    # 子闸门负责包围盒与"离物体中心距离"两项；关闭时直接返回 accepted=True
    workspace = candidate_workspace_gate(
        grasp_position_xyz=grasp_position_xyz,
        object_center_xyz=object_center_xyz,
        config=CandidateWorkspaceGateConfig(
            enabled=bool(config.workspace_gate_enabled),
            min_xyz=config.workspace_min_xyz,
            max_xyz=config.workspace_max_xyz,
            max_grasp_to_object_center_m=float(config.max_grasp_to_object_center_m),
        ),
    )
    if not workspace.accepted:
        return CandidateGateResult(False, workspace.reason)

    return CandidateGateResult(True)
