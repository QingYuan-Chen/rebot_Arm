"""抓取候选目标位姿策略模块（视觉包内部策略层）。

职责与位置：
    本模块把「一个抓取候选」（物体上的抓取点位置 + 候选姿态）展开成一组
    **机械臂末端目标位姿变体**，每个变体由 (预抓取位姿 pregrasp, 抓取位姿 grasp, 标签 label)
    组成，随后由候选 IK 过滤节点逐个做逆解与碰撞检查，取第一个可行的变体。

    当前正式路线只保留 ``preserve_candidate_pose``：使用 GraspNet 候选的完整
    6D 姿态，再按配置补充平行夹爪对称姿态。候选是否可达由 IK、碰撞和工作空间
    闸门判定；其他姿态策略已退出候选过滤主链路。

对外接口：
    仅提供纯计算函数与配置数据类，不涉及 ROS 话题/服务/动作，也不发布任何消息；
    不做任何运动校验，是否可行完全由调用方（IK / 碰撞 / 工作空间门控）判定。

关键约定：
    - 所有位置单位为米（m），姿态四元数一律是 (x, y, z, w) 顺序，角度单位为弧度（rad）；
    - 位姿均表达在机械臂基准坐标系（调用方传入前已做坐标变换）下；
    - 变体标签是稳定字符串（如 hybrid_geometry_yaw0_z0），用于日志、候选排序与现场排查，
      不得随意改动。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .pose_variant_policy import PoseVariantConfig, build_parallel_jaw_pose_variants, normalize_quaternion, quat_multiply
from .visual_grasp_pose_policy import (
    BaseAxisGraspPolicyConfig,
    OfficialGeometryGraspPolicyConfig,
    build_base_axis_grasp_targets,
    build_hybrid_geometry_grasp_targets,
    build_official_geometry_grasp_targets,
    build_preserve_candidate_grasp_targets,
)
from ..visual_grasp_sequence import PoseTarget


@dataclass(frozen=True)
class CandidateTargetVariant:
    """一个候选目标变体：预抓取位姿 + 抓取位姿 + 稳定标签。

    标签（label）会原样进入日志与候选筛选结果，是排查「最终选中了哪个策略分支」的唯一线索，
    因此必须保持稳定、可读、互相不重复。
    """

    pregrasp: PoseTarget
    grasp: PoseTarget
    label: str


@dataclass(frozen=True)
class CandidateTargetPolicyConfig:
    """候选目标生成策略配置（全部为不可变字段，构造后不再修改）。

    配置项与「候选 IK 过滤节点」的 ROS 参数一一对应；在仿真 bench 中先用
    candidate_joint6_symmetry_enabled=true、candidate_max_joint6_delta_rad=1.5708
    打开关节 6 对称变体，可显著提高候选可达率。
    """

    # 候选过滤正式且唯一的姿态策略。
    pose_policy: str = "preserve_candidate_pose"
    # 基准固定抓取姿态四元数 (x, y, z, w)；base_axis 分支使用它，
    # 也是姿态策略全不命中时的最终兜底。默认 (0, 0, 0, 1) 即与基准坐标系对齐。
    fixed_grasp_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    # 末端接近方向单位向量 (x, y, z)，表达在基准坐标系；预抓取点 = 抓取点沿该方向后退。
    # 默认沿用旧仓库 +X 工作区，因此实际配置为 (1, 0, 0)。
    base_approach_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    # 预抓取点相对抓取点沿接近轴后退的距离（m），默认 0.08。
    base_pregrasp_distance_m: float = 0.08
    # 末端坐标系下的 TCP 偏移 (x, y, z)（m）：真实夹爪 TCP 相对末端连杆原点的位移，
    # 输出目标位姿时会反向扣除，使夹爪指尖落在期望点上。现场实测为 (-0.04, 0.0, 0.0)。
    tcp_offset_xyz: tuple[float, float, float] = (-0.04, 0.0, 0.0)
    # 基准坐标系下的整体平移量 (x, y, z)（m），用于补偿候选点相对物体中心的偏差，
    # 所有变体统一叠加；默认 (0, 0, 0) 表示不补偿。
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 预抓取点 Z 的下限（m），0 表示不限制；>0 时会把预抓取点抬高到该平面以上。
    pregrasp_min_z_m: float = 0.0
    # 抓取位姿在基准坐标系 Z 方向的整体补偿量（m），默认 0（指爪宽度等偏差在标定中处理）。
    grasp_base_z_offset_m: float = 0.0
    # 绕基准 Z 轴的偏航角偏移枚举值（rad），用于在同一候选上尝试多个抓取方位角；
    # 空元组会被 _nonempty_offsets 归一化为 (0.0,)，即只保留原始姿态。
    orientation_yaw_offsets_rad: tuple[float, ...] = (0.0,)
    # 相对于抓取点的额外 Z 偏移枚举值（m），用于补偿深度/物体高度误差，向上为正；
    # 空元组同样归一化为 (0.0,)。
    candidate_grasp_z_offsets_m: tuple[float, ...] = (0.0,)
    # 平行夹爪对称变体相关配置（是否启用、对称旋转角，默认绕夹爪轴 180 度）。
    pose_variant_config: PoseVariantConfig = PoseVariantConfig()


def _yaw_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """由绕 Z 轴的偏航角构造四元数 (x, y, z, w) = (0, 0, sin(yaw/2), cos(yaw/2))。"""
    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _nonempty_offsets(values: tuple[float, ...]) -> tuple[float, ...]:
    """把空的偏移枚举归一化为 (0.0,)，保证至少生成一个「零偏移」变体。

    这是防空保护：若 ROS 参数被误设为空列表，返回空结果会让整条抓取链无候选可用，
    因此这里统一退化为不偏移。
    """
    return values if values else (0.0,)


def _base_axis_config(
    config: CandidateTargetPolicyConfig,
    *,
    fixed_orientation_xyzw: tuple[float, float, float, float],
    grasp_z_offset_m: float,
) -> BaseAxisGraspPolicyConfig:
    """把总体策略配置映射为「固定基准轴」子策略配置。

    只做字段搬运：接近轴、预抓取距离、TCP 偏移、基准偏移、预抓取下限与抓取 Z 补偿。
    """
    return BaseAxisGraspPolicyConfig(
        fixed_orientation_xyzw=fixed_orientation_xyzw,
        approach_axis_xyz=config.base_approach_axis_xyz,
        pregrasp_distance_m=float(config.base_pregrasp_distance_m),
        tcp_offset_xyz=config.tcp_offset_xyz,
        target_base_offset_xyz=config.target_base_offset_xyz,
        pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        grasp_z_offset_m=float(grasp_z_offset_m),
    )


def _official_config(
    config: CandidateTargetPolicyConfig,
    *,
    grasp_z_offset_m: float,
) -> OfficialGeometryGraspPolicyConfig:
    """把总体策略配置映射为「候选几何」子策略配置。

    与 _base_axis_config 的区别：不使用固定姿态与基准接近轴，
    接近轴由候选姿态自身的 X 轴给出（在 visual_grasp_pose_policy 内部计算）。
    """
    return OfficialGeometryGraspPolicyConfig(
        pregrasp_distance_m=float(config.base_pregrasp_distance_m),
        tcp_offset_xyz=config.tcp_offset_xyz,
        target_base_offset_xyz=config.target_base_offset_xyz,
        pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        grasp_z_offset_m=float(grasp_z_offset_m),
    )


def _symmetric_orientations(
    *,
    base_label: str,
    orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
    include_original: bool = True,
) -> list[tuple[str, tuple[float, float, float, float]]]:
    """返回 (标签, 姿态四元数) 列表：原姿态 + 可选平行夹爪对称姿态。

    include_original=False 用于「原姿态已经单独生成过变体」的场合，避免重复候选。
    """
    return build_parallel_jaw_pose_variants(
        base_label=base_label,
        orientation_xyzw=orientation_xyzw,
        config=config.pose_variant_config,
        include_original=include_original,
    )


def _build_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    orientation_xyzw: tuple[float, float, float, float] | None,
    grasp_z_offset_extra_m: float,
    mode: str,
    config: CandidateTargetPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    """按指定模式生成一对 (预抓取位姿, 抓取位姿)，均由同一个 TCP 偏移换算到末端连杆位姿。

    参数：
        grasp_position_xyz：抓取点位置（m，基准坐标系）。
        candidate_orientation_xyzw：候选原始姿态，仅在 orientation_xyzw 为 None 时作为兜底姿态。
        orientation_xyzw：本次变体要使用的姿态；None 表示复用候选姿态。
        grasp_z_offset_extra_m：本变体额外的抓取 Z 偏移（m），与配置里的 grasp_base_z_offset_m 相加。
        mode：目标生成模式，取 "hybrid_geometry"、"preserve_candidate_pose"、
            "official_geometry"，其余值（含 "base_axis"）走固定基准轴分支。
    返回：
        (pregrasp, grasp) 两个末端的位姿目标。
    """
    orientation = orientation_xyzw or candidate_orientation_xyzw
    # 抓取高度补偿 = 全局基准补偿 + 本变体的枚举偏移。
    grasp_z_offset_m = float(config.grasp_base_z_offset_m) + float(grasp_z_offset_extra_m)
    base_config = _base_axis_config(
        config,
        fixed_orientation_xyzw=orientation,
        grasp_z_offset_m=grasp_z_offset_m,
    )
    if mode == "hybrid_geometry":
        # 只保留候选姿态的 yaw，接近轴仍用基准轴，兼顾「朝向贴合物体」与「安装朝向确定」。
        return build_hybrid_geometry_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            candidate_orientation_xyzw=orientation,
            config=base_config,
        )
    if mode == "preserve_candidate_pose":
        # 原样保留候选姿态，接近轴取候选姿态 X 轴。
        return build_preserve_candidate_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            grasp_orientation_xyzw=orientation,
            config=_official_config(config, grasp_z_offset_m=grasp_z_offset_m),
        )
    if mode == "official_geometry":
        # 使用候选姿态与其自身 X 轴作为接近方向（上游官方几何约定）。
        return build_official_geometry_grasp_targets(
            grasp_position_xyz=grasp_position_xyz,
            grasp_orientation_xyzw=orientation,
            config=_official_config(config, grasp_z_offset_m=grasp_z_offset_m),
        )
    # 兜底：固定姿态 + 固定基准接近轴，完全不依赖候选姿态质量。
    return build_base_axis_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        config=base_config,
    )


def _append_preserve_candidate_variants(
    variants: list[CandidateTargetVariant],
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
) -> None:
    """追加「保留候选姿态」变体：原姿态 + 平行夹爪对称姿态，均不做 yaw/Z 枚举。

    该分支用于候选姿态本身已可信（例如上游已按物体主轴给出抓取朝向）的场景。
    """
    for label, target_orientation in _symmetric_orientations(
        base_label="preserve_candidate_pose",
        orientation_xyzw=candidate_orientation_xyzw,
        config=config,
    ):
        pregrasp, grasp = _build_targets(
            grasp_position_xyz=grasp_position_xyz,
            candidate_orientation_xyzw=candidate_orientation_xyzw,
            orientation_xyzw=target_orientation,
            grasp_z_offset_extra_m=0.0,
            mode="preserve_candidate_pose",
            config=config,
        )
        variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=label))


def _append_yaw_z_variants(
    variants: list[CandidateTargetVariant],
    *,
    grasp_position_xyz: tuple[float, float, float],
    base_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
    mode: str,
    label_prefix: str,
) -> None:
    """按 yaw 偏移 × Z 偏移的笛卡尔积追加变体，并为每个组合补一组夹爪对称变体。

    label 形如 ``{label_prefix}_yaw{i}_z{j}``，其中 i/j 是枚举下标而不是角度数值，
    因此调整偏移列表长度会改变可见标签，但不会改变代码逻辑。
    """
    yaw_offsets = _nonempty_offsets(config.orientation_yaw_offsets_rad)
    z_offsets = _nonempty_offsets(config.candidate_grasp_z_offsets_m)
    for yaw_index, yaw_offset in enumerate(yaw_offsets):
        # 左乘 yaw 四元数 = 在基准坐标系下额外绕 Z 轴旋转（先转 yaw 再套用候选姿态）。
        orientation = normalize_quaternion(quat_multiply(_yaw_quaternion(yaw_offset), base_orientation_xyzw))
        for z_index, z_offset in enumerate(z_offsets):
            label = f"{label_prefix}_yaw{yaw_index}_z{z_index}"
            pregrasp, grasp = _build_targets(
                grasp_position_xyz=grasp_position_xyz,
                candidate_orientation_xyzw=base_orientation_xyzw,
                orientation_xyzw=orientation,
                grasp_z_offset_extra_m=float(z_offset),
                mode=mode,
                config=config,
            )
            variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=label))
            # 同一 yaw/Z 组合下再补平行夹爪对称姿态；include_original=False 避免重复原姿态。
            for symmetry_label, target_orientation in _symmetric_orientations(
                base_label=label,
                orientation_xyzw=orientation,
                config=config,
                include_original=False,
            ):
                pregrasp, grasp = _build_targets(
                    grasp_position_xyz=grasp_position_xyz,
                    candidate_orientation_xyzw=base_orientation_xyzw,
                    orientation_xyzw=target_orientation,
                    grasp_z_offset_extra_m=float(z_offset),
                    mode=mode,
                    config=config,
                )
                variants.append(CandidateTargetVariant(pregrasp=pregrasp, grasp=grasp, label=symmetry_label))


def build_candidate_target_variants(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: CandidateTargetPolicyConfig,
) -> list[CandidateTargetVariant]:
    """候选目标生成的唯一入口：按策略把单个候选展开为按顺序尝试的位姿变体列表。

    参数：
        grasp_position_xyz：抓取点位置（m，基准坐标系），调用方需先完成 TF 变换。
        candidate_orientation_xyzw：候选姿态四元数 (x, y, z, w)，内部会归一化。
        config：策略配置，至少决定 pose_policy 分支。

    返回：
        变体列表，顺序即建议的尝试顺序；列表可能只含兜底预设，但不会为空。

    说明：
        这里只做几何展开，不做任何可达性判断；变体是否可用由候选 IK 过滤节点
        依次做逆解与碰撞检查后决定。
    """
    # 保留参数键以兼容旧 launch，但不再接受已经退出主链路的策略。
    pose_policy = str(config.pose_policy).strip()
    if pose_policy != "preserve_candidate_pose":
        raise ValueError(
            "candidate pose_policy only supports preserve_candidate_pose; "
            f"got {pose_policy!r}"
        )
    variants: list[CandidateTargetVariant] = []
    candidate_orientation = normalize_quaternion(candidate_orientation_xyzw)
    _append_preserve_candidate_variants(
        variants,
        grasp_position_xyz=grasp_position_xyz,
        candidate_orientation_xyzw=candidate_orientation,
        config=config,
    )
    return variants
