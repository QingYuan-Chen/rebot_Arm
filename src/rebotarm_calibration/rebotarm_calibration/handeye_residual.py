"""手眼标定残差分析：用“多姿态 + 固定标定物”数据集判定一组手眼外参是否可信。

职责与位置
    本模块是纯离线数学工具：不依赖 ROS、不做文件读写、不接触硬件，由标定包的命令行入口
    与离线运行脚本调用。输入是一份采集数据集，输出是一份判定报告（纯字典），由调用方
    决定是否采信该外参。

数据模型（``schema_version`` 固定为 1，字段名即对外接口，不可随意更名）
    payload
        ``end_to_camera``  待评估的手眼外参：末端坐标系到相机坐标系的刚体变换，用
                           ``translation`` + ``rotation_xyzw`` 表示；
        ``samples``        每个采样姿态一项，含 ``base_to_end``（TF 实测的基座到末端）
                           与 ``camera_to_marker``（视觉 PnP 实测的相机到标定物）。
    变换统一写作 ``{"translation": [x, y, z], "rotation_xyzw": [x, y, z, w]}``：
    平移单位为米（m），四元数顺序为 (x, y, z, w)，与 ROS 消息字段顺序一致。

核心思路
    标定物在整个采集中保持不动。对每个姿态把候选外参复合进去，把标定物位姿推算到基座
    坐标系：``base_to_marker = base_to_end @ end_to_camera @ camera_to_marker``。若外参
    正确，各姿态推算出的 ``base_to_marker`` 应几乎重合，其离散程度就是位置残差（米）与
    姿态残差（度）；外参有偏时残差会随姿态变化放大。

    残差小并不等于外参正确：若各采样姿态几乎相同，推算结果会一起平移/旋转，误差方向
    不可观测。因此报告额外给出“姿态多样性”门（末端平移与旋转的最大两两跨度），多样性
    不足时直接判不通过——即使残差为 0。

判定门
    ``pose_diversity.pass`` 与四项残差门（位置 RMS/最大、姿态 RMS/最大）全部通过，
    ``acceptance.pass`` 才为真；任何一项不过都不得部署该外参。
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np


# 数据集与报告共用的 schema 版本：外部脚本按该常量解析 JSON，改动即破坏兼容
SCHEMA_VERSION = 1


def robot_pose_key(transform):
    """采集与求解共用已有八位小数精度的机器人姿态重复判定。"""
    return tuple(round(float(value), 8) for value in transform_matrix(transform).ravel())


def transform_matrix(transform: Mapping[str, object]) -> np.ndarray:
    """把 ``{"translation", "rotation_xyzw"}`` 表示的变换转成 4x4 齐次矩阵。

    平移单位为米；四元数按 (x, y, z, w) 读取，并先按模长归一化，因此传入未归一化的
    四元数不会污染旋转计算。模长 <= 1e-12 视为零四元数并抛 ValueError（无法定义旋转）。
    返回 float64 的 4x4 矩阵，最后一行为 [0, 0, 0, 1]。
    """
    translation = _vector(transform.get("translation"), 3, "translation")
    quaternion = _vector(transform.get("rotation_xyzw"), 4, "rotation_xyzw")
    norm = float(np.linalg.norm(quaternion))
    # 零四元数无法归一化，会静默产生 NaN 旋转矩阵，因此在这里直接拒绝
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("rotation_xyzw norm must be positive")
    x, y, z, w = quaternion / norm
    # 单位四元数 -> 旋转矩阵的标准展开式（已归一化，可直接代入 x，y，z，w）
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def matrix_transform(matrix: np.ndarray) -> dict[str, list[float]]:
    """4x4 齐次矩阵 -> ``{"translation", "rotation_xyzw"}`` 字典（``transform_matrix`` 的逆）。

    做三重合法性校验，避免把非刚体变换写进报告：必须是有限值的 4x4；最后一行必须是
    [0, 0, 0, 1]（容差 1e-9）；左上 3x3 必须正交且行列式为 +1（容差 1e-8）。返回的四元数
    已归一化并统一取 w >= 0 的等价表示，便于跨文件/跨次运行比较。
    """
    value = np.asarray(matrix, dtype=np.float64)
    # NaN/Inf 会污染后续残差统计，且写进 JSON 后下游无法解析
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError("matrix must be a finite 4x4 transform")
    # 齐次矩阵最后一行必须为 [0, 0, 0, 1]，否则不是刚体变换
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("matrix must have homogeneous bottom row [0, 0, 0, 1]")
    rotation = value[:3, :3]
    # 正交约束（R^T R = I）保证是纯旋转；det = +1 进一步排除镜像反射
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, abs_tol=1e-8
    ):
        raise ValueError("matrix rotation must be orthonormal with determinant +1")
    return {
        "translation": [float(item) for item in value[:3, 3]],
        "rotation_xyzw": [float(item) for item in _matrix_quaternion(value[:3, :3])],
    }


def analyze_handeye_residual(
    payload: Mapping[str, object],
    *,
    min_samples: int = 5,
    min_end_translation_span_m: float = 0.05,
    min_end_rotation_span_deg: float = 20.0,
    max_position_rms_m: float = 0.005,
    max_position_residual_m: float = 0.010,
    max_rotation_rms_deg: float = 1.5,
    max_rotation_residual_deg: float = 3.0,
) -> dict[str, object]:
    """评估一份手眼标定数据集，返回残差报告与判定结果。

    参数
        payload：数据集映射，字段见模块 docstring；``schema_version`` 不等于 1 时拒绝。
        min_samples：最少采样姿态数，默认 5；小于 2 直接拒绝（无法构成两两统计）。
        min_end_translation_span_m：姿态多样性门——末端位置的最大两两距离（米），
            默认 0.05，即要求末端确实平移过至少 5 cm。
        min_end_rotation_span_deg：姿态多样性门——末端姿态的最大两两夹角（度），
            默认 20.0；平移与旋转两个维度都必须达标。
        max_position_rms_m：标定物位置残差 RMS 上限（米），默认 0.005。
        max_position_residual_m：单个姿态的位置残差上限（米），默认 0.010；比 RMS 宽松，
            用于放过个别偏差点，但不放过系统性偏差。
        max_rotation_rms_deg：姿态残差 RMS 上限（度），默认 1.5。
        max_rotation_residual_deg：单个姿态的姿态残差上限（度），默认 3.0。
        所有门限必须是有限正数，否则抛 ValueError——配成 0 或 NaN 会让门形同虚设。

    返回
        纯字典报告：``sample_count``、``mean_base_to_marker``（标定物在基座系下的平均
        位姿）、``position_residual`` / ``rotation_residual``（各含 rms / max / median）、
        ``pose_diversity``、``acceptance``，以及带逐样本残差的 ``samples``。
        调用方必须按 fail-closed 处理：只有 ``acceptance.pass`` 为真才可部署该外参。
    """
    # 少于 2 个样本无法计算“两两”跨度与均值残差，直接拒绝
    if int(min_samples) < 2:
        raise ValueError("min_samples must be at least 2")
    # 统一校验所有门限：有限且为正，避免 0/NaN 让判定恒真
    positive_limits = {
        "min_end_translation_span_m": min_end_translation_span_m,
        "min_end_rotation_span_deg": min_end_rotation_span_deg,
        "max_position_rms_m": max_position_rms_m,
        "max_position_residual_m": max_position_residual_m,
        "max_rotation_rms_deg": max_rotation_rms_deg,
        "max_rotation_residual_deg": max_rotation_residual_deg,
    }
    if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in positive_limits.values()):
        raise ValueError("all residual and diversity limits must be finite and positive")
    if int(payload.get("schema_version", -1)) != SCHEMA_VERSION:
        raise ValueError("unsupported schema_version")
    samples = payload.get("samples")
    if not isinstance(samples, list) or len(samples) < min_samples:
        raise ValueError(f"at least {min_samples} samples are required")
    # 待评估的手眼外参：末端坐标系 -> 相机坐标系
    handeye = transform_matrix(_mapping(payload.get("end_to_camera"), "end_to_camera"))

    base_to_end = []
    base_to_marker = []
    normalized_samples = []
    for index, sample in enumerate(samples):
        source = _mapping(sample, f"samples[{index}]")
        base_end = transform_matrix(_mapping(source.get("base_to_end"), "base_to_end"))
        camera_marker = transform_matrix(
            _mapping(source.get("camera_to_marker"), "camera_to_marker")
        )
        # 把标定物位姿推算到基座坐标系：T_base_marker = T_base_end · T_end_cam · T_cam_marker
        marker = base_end @ handeye @ camera_marker
        base_to_end.append(base_end)
        base_to_marker.append(marker)
        normalized_samples.append(
            {
                # label 缺失时回退为序号命名，保证报告里每项都可追溯
                "label": str(source.get("label", f"sample_{index + 1}")),
                "base_to_marker": matrix_transform(marker),
            }
        )

    positions = np.stack([value[:3, 3] for value in base_to_marker])
    rotations = np.stack([value[:3, :3] for value in base_to_marker])
    mean_position = np.mean(positions, axis=0)
    mean_rotation = _mean_rotation(rotations)
    # 位置残差：各姿态推算位置到平均位置的欧氏距离（米）
    position_residuals = np.linalg.norm(positions - mean_position, axis=1)
    # 姿态残差：各姿态推算姿态相对平均姿态的测地夹角（度）
    rotation_residuals = np.array(
        [_rotation_angle_deg(mean_rotation.T @ rotation) for rotation in rotations]
    )
    # 多样性只取末端实测位姿的跨度，不掺入待评估外参，避免“用被检对象证明自己”
    end_translation_span = _max_pairwise_distance(
        np.stack([value[:3, 3] for value in base_to_end])
    )
    end_rotation_span = max(
        _rotation_angle_deg(left[:3, :3].T @ right[:3, :3])
        for index, left in enumerate(base_to_end)
        for right in base_to_end[index + 1 :]
    )
    observability = rotation_observability(base_to_end)
    diversity_pass = (
        observability["pass"] and
        end_translation_span >= float(min_end_translation_span_m)
        and end_rotation_span >= float(min_end_rotation_span_deg)
    )
    position_rms = float(math.sqrt(float(np.mean(position_residuals**2))))
    position_max = float(np.max(position_residuals))
    rotation_rms = float(math.sqrt(float(np.mean(rotation_residuals**2))))
    rotation_max = float(np.max(rotation_residuals))
    # 判定：多样性门与四项残差门必须同时通过，缺一即判不合格（fail closed）
    acceptance_pass = (
        diversity_pass
        and position_rms <= float(max_position_rms_m)
        and position_max <= float(max_position_residual_m)
        and rotation_rms <= float(max_rotation_rms_deg)
        and rotation_max <= float(max_rotation_residual_deg)
    )
    # 把逐样本残差写回报告，便于定位是哪一次采样拖累了整体结果
    for item, position_error, rotation_error in zip(
        normalized_samples, position_residuals, rotation_residuals
    ):
        item["position_residual_m"] = float(position_error)
        item["rotation_residual_deg"] = float(rotation_error)

    return {
        "observability": observability,
        "sample_count": len(samples),
        "mean_base_to_marker": matrix_transform(
            _matrix_from_rotation_translation(mean_rotation, mean_position)
        ),
        "position_residual": {
            "rms_m": position_rms,
            "max_m": position_max,
            "median_m": float(np.median(position_residuals)),
        },
        "rotation_residual": {
            "rms_deg": rotation_rms,
            "max_deg": rotation_max,
            "median_deg": float(np.median(rotation_residuals)),
        },
        "pose_diversity": {
            "end_translation_span_m": float(end_translation_span),
            "end_rotation_span_deg": float(end_rotation_span),
            "required_translation_span_m": float(min_end_translation_span_m),
            "required_rotation_span_deg": float(min_end_rotation_span_deg),
            "pass": bool(diversity_pass),
        },
        "acceptance": {
            "max_position_rms_m": float(max_position_rms_m),
            "max_position_residual_m": float(max_position_residual_m),
            "max_rotation_rms_deg": float(max_rotation_rms_deg),
            "max_rotation_residual_deg": float(max_rotation_residual_deg),
            "pass": bool(acceptance_pass),
        },
        "samples": normalized_samples,
    }


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """断言 ``value`` 是映射类型，否则带着字段名抛 ValueError（便于定位坏数据）。"""
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _vector(value: object, size: int, label: str) -> np.ndarray:
    """把长度必须为 ``size`` 的有限数值序列转成 float64 向量，否则抛 ValueError。

    字符串与字节串本身也是序列，会被逐字符转 float 从而产生误导性错误，因此先排除。
    """
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{label} must contain {size} finite values")
    result = np.asarray([float(item) for item in value], dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {size} finite values")
    return result


def _matrix_from_rotation_translation(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """由 3x3 旋转块与 3 维平移向量组装 4x4 齐次矩阵（平移单位米）。"""
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _mean_rotation(rotations: np.ndarray) -> np.ndarray:
    """求一组旋转矩阵的平均：正交 Procrustes 问题，用 SVD 投影回 SO(3)。

    直接对矩阵元素取平均会得到非正交矩阵。做法是先取算术平均 M，再对 M 做 SVD
    ``M = U S V^T``，用 ``U V^T`` 作为最接近的旋转。若 det(U V^T) < 0（说明平均结果
    落在反射分支），翻转 U 的最后一列后重算，保证返回真旋转而不是镜像。
    """
    left, _, right = np.linalg.svd(np.mean(rotations, axis=0))
    result = left @ right
    if np.linalg.det(result) < 0.0:
        left[:, -1] *= -1.0
        result = left @ right
    return result


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    """由旋转矩阵的迹求其转角（度）：cos θ = (tr(R) - 1) / 2。

    迹的浮点误差可能让余弦略超出 [-1, 1] 使 acos 报 NaN，故先做区间截断。
    """
    cosine = min(1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _max_pairwise_distance(points: np.ndarray) -> float:
    """一组三维点的最大两两欧氏距离（米），用作位置多样性指标。

    调用方保证至少 2 个点，否则生成器为空、max 会抛 ValueError。
    """
    return max(
        float(np.linalg.norm(left - right))
        for index, left in enumerate(points)
        for right in points[index + 1 :]
    )


def _matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 单位四元数，返回顺序 (x, y, z, w)。

    用迹分支：迹大于 0 时数值最稳；否则改用对角元最大的那个分量作分母，避免接近 180°
    时开方除以近零量导致精度崩溃。最后归一化，并统一取 w >= 0 的等价表示（q 与 -q 表示
    同一旋转），使输出可比较、可复现。
    """
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[2, 1] - rotation[1, 2]) / scale]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [(rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [(rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale, (rotation[1, 0] - rotation[0, 1]) / scale]
            )
    quaternion /= np.linalg.norm(quaternion)
    # 统一符号：固定 w >= 0，消除 q 与 -q 的表示二义性
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def rotation_observability(transforms, *, maximum_condition_number=100.0):
    """核对相对旋转是否对三个平移方向都提供足够约束。

    堆叠 R_i.T R_j - I 后，公共旋转轴会形成零空间；秩和条件数检查用于采样激励门，
    绝对精度另由残差及物理测量评估。
    """
    if not math.isfinite(maximum_condition_number) or maximum_condition_number <= 1:
        raise ValueError("maximum_condition_number must be finite and greater than one")
    blocks = [a[:3, :3].T @ b[:3, :3] - np.eye(3)
              for i, a in enumerate(transforms) for b in transforms[i + 1:]]
    singular = np.linalg.svd(np.vstack(blocks), compute_uv=False) if blocks else np.zeros(3)
    rank = int(np.count_nonzero(singular > max(1e-10, singular[0] * 1e-8)))
    condition = float(singular[0] / singular[-1]) if rank == 3 else None
    return {"rank": rank, "singular_values": singular.tolist(),
            "condition_number": condition, "maximum_condition_number": maximum_condition_number,
            "pass": rank == 3 and condition <= maximum_condition_number}
