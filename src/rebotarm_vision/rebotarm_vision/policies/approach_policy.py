"""预抓取位姿的几何构造：沿逼近轴反方向退开一段距离。

抓取序列先到「预抓取点」，再沿逼近轴直进到抓取点；预抓取点由抓取点沿逼近轴
反向平移 ``pregrasp_distance_m`` 得到，再做最低高度钳制。本模块
只做向量运算，不做规划、不查碰撞；单位米，坐标系与输入一致。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachPolicyConfig:
    """预抓取几何参数。"""

    # 预抓取点相对抓取点沿逼近反方向的距离，单位米。调大留出更长的直进段，
    # 逼近更稳，但更容易在狭窄空间里先撞到别的物体。
    pregrasp_distance_m: float = 0.08
    # 预抓取点的绝对最低高度，单位米；> 0 时生效，防止预抓取点落到台面以下。
    pregrasp_min_z_m: float = 0.0


def normalize_vector(vector_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """把三维向量归一化为单位向量；零向量抛 ValueError。

    异常文本是对外可观测的接口（英文原样保留），提示 ``approach_axis_xyz``
    配置有误。
    """

    x, y, z = (float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2]))
    norm = (x * x + y * y + z * z) ** 0.5
    # 1e-9 是浮点零判定阈值：低于它归一化会得到 NaN/Inf，因此直接报错。
    if norm <= 1e-9:
        raise ValueError("approach_axis_xyz must be non-zero")
    return (x / norm, y / norm, z / norm)


def build_pregrasp_tcp(
    *,
    grasp_tcp_xyz: tuple[float, float, float],
    approach_axis_xyz: tuple[float, float, float],
    config: ApproachPolicyConfig = ApproachPolicyConfig(),
) -> tuple[float, float, float]:
    """由抓取点计算预抓取点（工具中心点位置）。

    公式：pregrasp = grasp - normalize(axis) × pregrasp_distance_m。随后仅在
    pregrasp_min_z_m > 0 时把 z 钳制到不低于该下限。

    返回三元组 (x, y, z)，单位米，与输入同坐标系。
    """

    axis = normalize_vector(approach_axis_xyz)
    pregrasp = (
        float(grasp_tcp_xyz[0]) - axis[0] * float(config.pregrasp_distance_m),
        float(grasp_tcp_xyz[1]) - axis[1] * float(config.pregrasp_distance_m),
        float(grasp_tcp_xyz[2]) - axis[2] * float(config.pregrasp_distance_m),
    )
    min_z = float(config.pregrasp_min_z_m)
    # 仅当显式配置了正的下限时才钳制：0 表示「不限制」而不是「必须在地面上」。
    if min_z > 0.0:
        pregrasp = (pregrasp[0], pregrasp[1], max(pregrasp[2], min_z))
    return pregrasp
