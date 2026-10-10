"""抓取候选的「工作空间 + 贴近物体中心」门控。

在候选进入昂贵的逆解/运动可行性检查之前，先用一个轴对齐的坐标盒把明显够不到
或明显偏离物体的候选剔除。拒绝原因（``reason``）是英文诊断文本，会写入日志与
服务响应，属于对外可见内容，勿改文案。

本模块是纯函数判定，不查询坐标变换、不访问硬件；坐标系由调用方保证（通常是
机械臂基座系，单位米）。所有开关默认关闭，便于在不改代码的前提下灰度启用。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CandidateWorkspaceGateConfig:
    """工作空间门控参数；``enabled`` 为 False 时门控整体放行。"""

    # 总开关。默认 False：关闭时本门控一律接受，保持历史行为不变。
    enabled: bool = False
    # 允许盒的最小角点 (x, y, z)，单位米，基座系；闭区间下界。
    min_xyz: tuple[float, float, float] = (0.18, -0.35, 0.0)
    # 允许盒的最大角点 (x, y, z)，单位米，基座系；闭区间上界。
    # 与 min_xyz 一起构成轴对齐包围盒，任一维越界即拒绝。
    max_xyz: tuple[float, float, float] = (0.64, 0.35, 0.45)
    # 抓取点与物体中心的最大允许距离，单位米。超过说明抓取点落在物体之外
    # （例如点云噪声或选错候选）；<= 0 表示跳过该距离检查。
    max_grasp_to_object_center_m: float = 0.15


@dataclass(frozen=True)
class CandidateWorkspaceGateResult:
    """门控结果；``reason`` 为英文诊断文本，空串表示接受。"""

    accepted: bool
    reason: str = ""


def candidate_workspace_gate(
    *,
    grasp_position_xyz: tuple[float, float, float],
    object_center_xyz: tuple[float, float, float] | None,
    config: CandidateWorkspaceGateConfig,
) -> CandidateWorkspaceGateResult:
    """判定抓取点是否落在允许的工作空间盒内、且足够靠近物体中心。

    参数：
        grasp_position_xyz: 抓取点位置 (x, y, z)，单位米，基座系。
        object_center_xyz: 物体中心位置，单位米；为 None 时跳过距离检查
            （例如上游没有给出物体中心）。
        config: 门控参数；``enabled`` 为 False 时立即接受。

    返回：
        CandidateWorkspaceGateResult。拒绝时 ``reason`` 带上原始坐标/距离，
        便于从日志定位是哪一维越界。
    """

    if not config.enabled:
        return CandidateWorkspaceGateResult(True)

    x, y, z = (float(grasp_position_xyz[0]), float(grasp_position_xyz[1]), float(grasp_position_xyz[2]))
    min_x, min_y, min_z = (float(config.min_xyz[0]), float(config.min_xyz[1]), float(config.min_xyz[2]))
    max_x, max_y, max_z = (float(config.max_xyz[0]), float(config.max_xyz[1]), float(config.max_xyz[2]))
    # 三维同时落在闭区间内才算通过；任一维越界直接给出拒绝理由。
    if not (min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z):
        return CandidateWorkspaceGateResult(
            False,
            f"grasp outside workspace ({x:.3f}, {y:.3f}, {z:.3f})",
        )

    # 距离检查只在拿到物体中心、且阈值 > 0 时生效，避免把"阈值 0"误解为要求重合。
    if object_center_xyz is not None and float(config.max_grasp_to_object_center_m) > 0.0:
        ox, oy, oz = (
            float(object_center_xyz[0]),
            float(object_center_xyz[1]),
            float(object_center_xyz[2]),
        )
        distance = ((x - ox) ** 2 + (y - oy) ** 2 + (z - oz) ** 2) ** 0.5
        if distance > float(config.max_grasp_to_object_center_m):
            return CandidateWorkspaceGateResult(
                False,
                f"grasp too far from object center ({distance:.3f}m)",
            )

    return CandidateWorkspaceGateResult(True)
