"""平行夹爪的对称抓取姿态变体（关节 6 对称位）。

背景：两指平行夹爪的两根手指结构对称，把工具姿态绕**夹爪接近轴**（本仓库约定接近轴为
工具坐标系的局部 x 轴）旋转 180° 后，两指的占位集合与原来完全相同——物理上是同一次
抓取，但逆运动学会解出关节 6 相差约 π 的另一个解。因此当候选姿态因关节 6 接近限位或
接近奇异被拒时，可以补一个等价姿态再试，提高抓取成功率而不改变实际夹取效果。

本模块只做四元数运算，不依赖 ROS；坐标与四元数约定和 ``transform_points`` 模块一致：
四元数为 ``xyzw`` 顺序的单位四元数，右手坐标系，角度单位为弧度。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def normalize_quaternion(quaternion_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """归一化四元数，返回单位四元数。

    与 ``transform_points`` 中的同名函数不同：模长 <= 1e-9 视为非法输入并抛
    ``ValueError``（错误消息为英文常量），因为这里的输入直接决定下发的抓取姿态，
    静默回退成单位旋转会掩盖上游的数据错误。
    """

    x, y, z, w = (float(v) for v in quaternion_xyzw)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        raise ValueError("quaternion must be non-zero")
    return (x / norm, y / norm, z / norm, w / norm)


def quat_multiply(
    left_xyzw: tuple[float, float, float, float],
    right_xyzw: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """四元数 Hamilton 乘积，表示旋转复合 ``R(left)·R(right)``。

    这里**不做归一化**（与 ``transform_points`` 的版本不同），保持纯乘法语义；
    调用方按需自行归一化。乘法不可交换：左乘=在父坐标系中叠加旋转，右乘=绕自身
    局部轴旋转。
    """

    lx, ly, lz, lw = left_xyzw
    rx, ry, rz, rw = right_xyzw
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_to_rotation_matrix(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    """单位四元数 → 3x3 旋转矩阵（行主序）。

    内部先归一化以抑制累积漂移。矩阵的第 1 列即局部 x 轴（夹爪接近轴）在父坐标系
    中的方向；对称变体正是保持该列不变、把第 2/3 列取反，这一点由单元测试锁定。
    """

    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def build_parallel_jaw_symmetric_orientation(
    orientation_xyzw: tuple[float, float, float, float],
    *,
    angle_rad: float = math.pi,
) -> tuple[float, float, float, float]:
    """构造绕工具**局部 x 轴**旋转 ``angle_rad`` 后的等价姿态四元数。

    实现要点：把轴角 (x 轴, angle) 写成四元数 ``(sin(θ/2), 0, 0, cos(θ/2))``（w 分量为
    ``cos(θ/2)``，x 分量为 ``sin(θ/2)``），再与原姿态**右乘**——右乘才表示绕自身局部轴
    旋转；若写成左乘则等于绕父坐标系 x 轴旋转，物理含义完全不同。

    默认角度 π（180°）：两指对称的平行夹爪在 180° 下占位不变，同时把关节 6 的解推到
    相差约 π 的另一支。返回值已归一化，可直接下发给运动侧。
    """

    half = float(angle_rad) * 0.5
    local_x_rotation = (math.sin(half), 0.0, 0.0, math.cos(half))
    return normalize_quaternion(quat_multiply(normalize_quaternion(orientation_xyzw), local_x_rotation))


@dataclass(frozen=True)
class PoseVariantConfig:
    """姿态变体生成开关。

    ``joint6_symmetry_enabled``：是否追加关节 6 对称变体；关闭时只返回原始姿态。
    ``joint6_symmetry_angle_rad``：对称旋转角，单位弧度，默认 π（180°）。只有 π 对
    对称平行夹爪才严格等价，改成其他角度前必须先确认夹爪几何，否则会生成物理上不同
    的抓取姿态。
    """

    joint6_symmetry_enabled: bool = True
    joint6_symmetry_angle_rad: float = math.pi


def build_parallel_jaw_pose_variants(
    *,
    base_label: str,
    orientation_xyzw: tuple[float, float, float, float],
    config: PoseVariantConfig = PoseVariantConfig(),
    include_original: bool = True,
) -> list[tuple[str, tuple[float, float, float, float]]]:
    """生成候选抓取姿态及其对称变体。

    参数：
    - ``base_label``：原始姿态的标签，由调用方给出（例如上层按姿态来源命名）；
    - ``orientation_xyzw``：候选姿态四元数（xyzw，单位长度），返回时会被归一化；
    - ``config``：见 ``PoseVariantConfig``；
    - ``include_original``：是否把原始姿态作为第一个变体返回。置 False 时只返回对称
      变体，调用方需自行保证仍有一个可行姿态。

    返回 ``[(label, orientation_xyzw), ...]``，顺序固定为「原始姿态在前、对称变体在
    后」；追加项的标签为 ``f"{base_label}_parallel_jaw_symmetric"``，该后缀是上层识别
    姿态来源的约定字符串，不可改动。
    """

    variants: list[tuple[str, tuple[float, float, float, float]]] = []
    if include_original:
        variants.append((base_label, normalize_quaternion(orientation_xyzw)))
    if config.joint6_symmetry_enabled:
        variants.append(
            (
                f"{base_label}_parallel_jaw_symmetric",
                build_parallel_jaw_symmetric_orientation(
                    orientation_xyzw,
                    angle_rad=float(config.joint6_symmetry_angle_rad),
                ),
            )
        )
    return variants
