"""视觉抓取位姿策略：把「抓取候选位姿」换算成「预抓取点 + 抓取点」两个 TCP 目标。

在系统中的位置
--------------
本模块是纯计算层：输入是视觉侧给出的抓取点位置（以及可选的候选姿态四元数），
输出是 ``(pregrasp, grasp)`` 两个 ``PoseTarget``。它不做任何 ROS 通信、不读写
文件、也不下发运动指令；调用方（候选变体策略、视觉抓取执行器节点）拿到结果后
才去做规划与执行。因此这里没有任何安全门控，**改动本模块只会改变几何目标，
不会绕过碰撞检查或控制器保护**。

三个策略族（由调用方按 ``pose_policy`` 选择）
--------------------------------------------
1. 基座轴固定姿态族（本模块的 ``build_base_axis_grasp_targets``）：
   姿态完全由配置给死，与候选姿态无关；接近方向由 ``approach_axis_xyz`` 指定。
2. 混合几何族（``build_hybrid_geometry_grasp_targets``）：
   只保留候选姿态绕竖直轴的偏航角，俯仰/滚转让位给固定姿态，
   这样夹爪朝向仍贴合目标，但接近方向可控。
3. 官方几何族（``build_official_geometry_grasp_targets``）与
   ``build_preserve_candidate_grasp_targets``：
   完整保留候选的 6D 姿态，接近轴直接取该姿态旋转矩阵的第一列（即候选
   TCP 的局部 x 轴）。

坐标与单位约定
--------------
* 所有位置单位为米，四元数顺序为 ``(x, y, z, w)``；
* 输入 ``grasp_position_xyz`` 是**抓取点（指尖闭合点）**在目标坐标系下的位置；
* 返回值是 **TCP 目标点**，由内部再减去 ``tcp_offset_xyz`` 换算而来；
* 位置输出统一四舍五入到小数点后 6 位，用于抑制浮点噪声造成的位姿抖动。
"""

from __future__ import annotations

from dataclasses import dataclass

from .approach_policy import ApproachPolicyConfig, build_pregrasp_tcp, normalize_vector
from ..visual_grasp_sequence import PoseTarget
from .pose_variant_policy import (
    build_parallel_jaw_symmetric_orientation,
    quaternion_to_rotation_matrix,
)


@dataclass(frozen=True)
class BaseAxisGraspPolicyConfig:
    """基座轴固定姿态策略的配置。

    该族策略完全忽略候选姿态，姿态由 ``fixed_orientation_xyzw`` 一次性给死，
    因此“接近轴”与“末端姿态”解耦：接近方向由 ``approach_axis_xyz`` 单独描述。
    适用于目标被竖直夹取、或需要强制某个固定抓取姿态的离线/仿真流程。
    """

    # 固定的 TCP 姿态四元数 (x, y, z, w)，内部会归一化；零四元数会抛 ValueError。
    fixed_orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    # 接近轴方向（抓取点坐标系下的任意非零向量，内部归一化）；沿该轴的反方向退开得到预抓取点。
    # 典型取值 (1, 0, 0) 表示沿末端 x 轴进给；(0, 0, -1) 表示自上而下。
    approach_axis_xyz: tuple[float, float, float] = (1.0, 0.0, 0.0)
    # 预抓取点相对抓取点沿接近轴的退让距离，单位 m；调大→进给行程更长、更安全但更慢。
    pregrasp_distance_m: float = 0.08
    # TCP 相对法兰/末端参考点的平移偏移，单位 m，在 TCP 自身姿态下表达。
    # 用于把「指尖点」折算成机器人控制器真正接收的末端点位姿（本站默认 -0.04 m 量级）。
    tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 对外的整体位置补偿（目标坐标系），单位 m。用于修正标定残差或按固定量挪动抓取点；
    # 与 tcp_offset_xyz 不同，它不随姿态旋转，直接加在抓取点上。
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 预抓取点 z 的下限，单位 m。<= 0 表示不启用该保护；> 0 时把预抓取点抬高到至少该高度，
    # 防止退让后撞到地面或桌面以下。
    pregrasp_min_z_m: float = 0.0
    # 抓取点（闭合点）的竖直补偿，单位 m。正值上抬、负值下压。
    # 注意该偏移只作用于抓取点：预抓取点由 ``base`` 而非 ``grasp_tcp`` 推导，
    # 所以它不会把预抓取点一起抬高。
    grasp_z_offset_m: float = 0.0


@dataclass(frozen=True)
class OfficialGeometryGraspPolicyConfig:
    """官方几何策略的配置：只保留几何/偏移参数，不含固定姿态与固定接近轴。

    姿态与接近轴全部来自抓取候选本身（见 ``build_official_geometry_grasp_targets``），
    因此这里没有 ``fixed_orientation_xyzw`` / ``approach_axis_xyz`` 两个键。
    """

    # 预抓取点沿候选 TCP x 轴退开的距离，单位 m。
    pregrasp_distance_m: float = 0.08
    # TCP 相对法兰/末端参考点的平移偏移，单位 m，在候选姿态下表达。
    tcp_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 对外的整体位置补偿（目标坐标系），单位 m。
    target_base_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # 预抓取点 z 的下限，单位 m；<= 0 表示不启用。
    pregrasp_min_z_m: float = 0.0
    # 抓取点的竖直补偿，单位 m。本族预抓取点由 ``grasp_tcp`` 推导，
    # 因此该偏移会**同时**作用于抓取点与预抓取点。
    grasp_z_offset_m: float = 0.0


def _normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """归一化三维方向向量；零向量会抛 ValueError（沿用共享接近策略的实现）。"""
    return normalize_vector(vector)


def _normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """把四元数 (x, y, z, w) 归一化为单位四元数。

    范数 <= 1e-9 视为未配置的零四元数并抛 ``ValueError``，避免后续矩阵退化。
    """
    x, y, z, w = (float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3]))
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        raise ValueError("fixed_grasp_orientation_xyzw must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_to_rotation_matrix(quaternion: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    """单位四元数 → 3x3 旋转矩阵（按行给出，行主序）。

    采用标准公式 ``R = I + 2w[v]x + 2[v]x^2`` 展开后的分量形式，
    其中 ``v = (x, y, z)`` 为四元数向量部分。返回的是「局部 → 父坐标系」的旋转。
    """
    x, y, z, w = _normalize_quaternion(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _subtract_tcp_offset(
    tcp_position: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    tcp_offset_xyz: tuple[float, float, float],
) -> tuple[float, float, float]:
    """把指尖点换算成末端参考点位姿：``P_ref = P_tcp - R(orientation) · offset``。

    偏移必须在 TCP 自身姿态下旋转到目标坐标系后再相减，不能直接按坐标轴减，
    否则姿态一变偏移方向就错了。结果四舍五入到 1e-6 m，抑制浮点抖动。
    """
    rotation = _quat_to_rotation_matrix(orientation_xyzw)
    ox, oy, oz = (float(tcp_offset_xyz[0]), float(tcp_offset_xyz[1]), float(tcp_offset_xyz[2]))
    dx = rotation[0][0] * ox + rotation[0][1] * oy + rotation[0][2] * oz
    dy = rotation[1][0] * ox + rotation[1][1] * oy + rotation[1][2] * oz
    dz = rotation[2][0] * ox + rotation[2][1] * oy + rotation[2][2] * oz
    return (
        round(float(tcp_position[0]) - dx, 6),
        round(float(tcp_position[1]) - dy, 6),
        round(float(tcp_position[2]) - dz, 6),
    )


def _x_axis_from_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    """取旋转矩阵的第一列，即姿态的局部 x 轴在父坐标系下的方向。

    本站约定：夹爪进给方向就是 TCP 的 x 轴，所以候选姿态的 x 轴即为其接近轴。
    """
    rotation = _quat_to_rotation_matrix(quaternion)
    return (rotation[0][0], rotation[1][0], rotation[2][0])


def _yaw_from_quaternion(quaternion: tuple[float, float, float, float]) -> float:
    """提取绕竖直轴（z 轴）的偏航角，单位 rad，范围 (-pi, pi]。

    取的是旋转矩阵第一列在 xy 平面的极角 ``atan2(R[1][0], R[0][0])``。
    math 采用函数内延迟导入，避免模块导入期产生额外开销。
    """
    rotation = _quat_to_rotation_matrix(quaternion)
    return __import__("math").atan2(rotation[1][0], rotation[0][0])


def _quaternion_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """绕 z 轴纯偏航旋转 → 四元数 (0, 0, sin(yaw/2), cos(yaw/2))，单位 rad。

    x、y 分量恒为 0，因此该姿态竖直轴固定，俯仰/滚转全被丢弃。
    """
    math = __import__("math")
    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_base_axis_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    config: BaseAxisGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    """【基座轴固定姿态族】按固定姿态 + 指定接近轴生成 ``(预抓取, 抓取)`` 目标。

    处理顺序：
    1. 归一化接近轴与固定姿态；
    2. ``base = grasp_position_xyz + target_base_offset_xyz``（抓取点整体补偿）；
    3. ``grasp_tcp = base`` 叠加 ``grasp_z_offset_m``；
    4. 以 **base**（而非 ``grasp_tcp``）为基准沿接近轴反方向退开
       ``pregrasp_distance_m``，并做 ``pregrasp_min_z_m`` 下限保护；
    5. 两个点分别减去 TCP 偏移，得到控制器可用的末端点位姿。

    注意与官方几何族的差异：本族预抓取点基于 ``base``，所以 ``grasp_z_offset_m``
    只抬高抓取点、不会连带抬高预抓取点。

    参数：
        grasp_position_xyz：抓取点（指尖闭合点）位置，单位 m；
        config：见 ``BaseAxisGraspPolicyConfig``。

    返回：
        ``(pregrasp, grasp)``，两者的姿态同为归一化后的 ``fixed_orientation_xyzw``。
    """
    axis = _normalize_vector(config.approach_axis_xyz)
    orientation = _normalize_quaternion(config.fixed_orientation_xyzw)
    base = (
        float(grasp_position_xyz[0]) + float(config.target_base_offset_xyz[0]),
        float(grasp_position_xyz[1]) + float(config.target_base_offset_xyz[1]),
        float(grasp_position_xyz[2]) + float(config.target_base_offset_xyz[2]),
    )
    grasp_tcp = (
        base[0],
        base[1],
        base[2] + float(config.grasp_z_offset_m),
    )
    pregrasp_tcp = build_pregrasp_tcp(
        grasp_tcp_xyz=base,
        approach_axis_xyz=axis,
        config=ApproachPolicyConfig(
            pregrasp_distance_m=float(config.pregrasp_distance_m),
            pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        ),
    )
    return (
        PoseTarget(
            position=_subtract_tcp_offset(pregrasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
        PoseTarget(
            position=_subtract_tcp_offset(grasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
    )


def build_hybrid_geometry_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    candidate_orientation_xyzw: tuple[float, float, float, float],
    config: BaseAxisGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    """【混合几何族】只保留候选姿态的偏航角，其余姿态换成配置里的固定姿态。

    折中目的：候选的 6D 姿态常常带不可达的俯仰/滚转，直接照搬容易 IK 失败；
    这里仅保留绕竖直轴的偏航，使夹爪开合方向仍对齐目标，而接近方向沿用
    ``config.approach_axis_xyz`` 与固定姿态，可控性更好。

    实现上把偏航角重新合成一个纯 yaw 四元数，再复用固定姿态族逻辑，
    因此继承其全部偏移与安全下限语义（含预抓取点基于 ``base`` 的细节）。
    """
    yaw = _yaw_from_quaternion(candidate_orientation_xyzw)
    return build_base_axis_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        config=BaseAxisGraspPolicyConfig(
            fixed_orientation_xyzw=_quaternion_from_yaw(yaw),
            approach_axis_xyz=config.approach_axis_xyz,
            pregrasp_distance_m=config.pregrasp_distance_m,
            tcp_offset_xyz=config.tcp_offset_xyz,
            target_base_offset_xyz=config.target_base_offset_xyz,
            pregrasp_min_z_m=config.pregrasp_min_z_m,
            grasp_z_offset_m=config.grasp_z_offset_m,
        ),
    )


def build_official_geometry_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    grasp_orientation_xyzw: tuple[float, float, float, float],
    config: OfficialGeometryGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    """【官方几何族】完整保留候选 6D 姿态，接近轴取其旋转矩阵的 x 轴。

    与固定姿态族的区别：
    * 姿态不做任何降维，候选给什么姿态就用什么姿态；
    * 接近轴不再是配置项，而是由候选姿态推导（绕该轴退让得到预抓取点）；
    * 预抓取点以 ``grasp_tcp``（已叠加 ``grasp_z_offset_m``）为基准，
      因此竖直偏移会同时抬高抓取点与预抓取点。

    姿态零四元数会抛 ``ValueError``，接近轴为零向量也会抛 ``ValueError``。
    """
    orientation = _normalize_quaternion(grasp_orientation_xyzw)
    approach_axis = _normalize_vector(_x_axis_from_quaternion(orientation))
    base = (
        float(grasp_position_xyz[0]) + float(config.target_base_offset_xyz[0]),
        float(grasp_position_xyz[1]) + float(config.target_base_offset_xyz[1]),
        float(grasp_position_xyz[2]) + float(config.target_base_offset_xyz[2]),
    )
    grasp_tcp = (
        base[0],
        base[1],
        base[2] + float(config.grasp_z_offset_m),
    )
    pregrasp_tcp = build_pregrasp_tcp(
        grasp_tcp_xyz=grasp_tcp,
        approach_axis_xyz=approach_axis,
        config=ApproachPolicyConfig(
            pregrasp_distance_m=float(config.pregrasp_distance_m),
            pregrasp_min_z_m=float(config.pregrasp_min_z_m),
        ),
    )
    return (
        PoseTarget(
            position=_subtract_tcp_offset(pregrasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
        PoseTarget(
            position=_subtract_tcp_offset(grasp_tcp, orientation, config.tcp_offset_xyz),
            orientation=orientation,
        ),
    )


def build_preserve_candidate_grasp_targets(
    *,
    grasp_position_xyz: tuple[float, float, float],
    grasp_orientation_xyzw: tuple[float, float, float, float],
    config: OfficialGeometryGraspPolicyConfig,
) -> tuple[PoseTarget, PoseTarget]:
    """【保留候选位姿族】语义上等同于官方几何族，此处仅作为策略名入口保留。

    调用方以 ``pose_policy="preserve_candidate_pose"`` 选择该分支，
    直接转发到 ``build_official_geometry_grasp_targets``，
    保证两条策略名的数值行为完全一致（便于回归比对）。
    """
    return build_official_geometry_grasp_targets(
        grasp_position_xyz=grasp_position_xyz,
        grasp_orientation_xyzw=grasp_orientation_xyzw,
        config=config,
    )
