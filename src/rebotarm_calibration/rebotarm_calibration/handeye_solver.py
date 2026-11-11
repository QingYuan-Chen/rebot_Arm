"""眼在手上(eye-in-hand)手眼标定的求解与评估核心.

求解的目标是 ``base->end · end->camera · camera->marker = base->marker`` 中的
``end->camera``:末端在基座系中的位姿 ``base_to_end`` 由机器人正运动学给出,标记在
相机系中的位姿 ``camera_to_marker`` 由视觉给出;标记在标定期间固定不动,因此
``base_to_marker`` 是常量,多组姿态即可联立解出相机安装位姿.

模块只做纯计算(一次 OpenCV 求解 + 残差评估),不涉及 ROS 通信,TF 查询与文件读写:
输入是若干组 4x4 齐次变换,输出是 4x4 的 ``end->camera`` 以及位置(m)/姿态(度)
残差报告.是否采纳某个解由上层工具按残差与姿态多样性门限判定,本模块不做取舍.

坐标系与约定:所有变换都是 4x4 齐次矩阵,旋转部分正交且行列式为 +1(右手系,无镜像
无缩放);四元数按 ``xyzw`` 顺序,与 :mod:`handeye_residual` 的序列化格式一致.
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import cv2
import numpy as np

from .handeye_residual import matrix_transform, transform_matrix, rotation_observability


# 求解方法名 → OpenCV 枚举的映射.键名(TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS)是对外
# 接口:上层工具与配置按它们选择方法,也是评估报告里的方法标识,不要翻译或改名.
HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_eye_in_hand(
    samples: Sequence[Mapping[str, object]],
    *,
    method: str = "TSAI",
) -> np.ndarray:
    """由多组位姿样本求解 ``end->camera``(眼在手上).

    ``samples`` 中每一项都是映射,需含 ``base_to_end``(末端在基座系中的位姿)与
    ``camera_to_marker``(标记在相机系中的位姿),值可以是 4x4 齐次矩阵,也可以是
    :func:`handeye_residual.transform_matrix` 支持的 ``translation``/``rotation_xyzw``
    字典.``method`` 取 :data:`HAND_EYE_METHODS` 的键,大小写不敏感.

    至少需要 3 组样本:样本少于 3 组时旋转轴约束不足以定解,直接拒绝(OpenCV 的下限
    是 2 组,这里收紧以保证解可被评估).返回值是 4x4 的 ``end->camera``;若结果含
    NaN/Inf 则抛 ``ValueError``,避免把坏解写进标定文件.
    """
    if len(samples) < 3:
        raise ValueError("at least three samples are required for hand-eye solve")
    method_name = str(method).upper()
    if method_name not in HAND_EYE_METHODS:
        raise ValueError(f"unsupported hand-eye method: {method}")

    base_to_end = []
    camera_to_marker = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise ValueError(f"samples[{index}] must be a mapping")
        base_to_end.append(
            _sample_transform(sample.get("base_to_end"), "base_to_end")
        )
        camera_to_marker.append(
            _sample_transform(sample.get("camera_to_marker"), "camera_to_marker")
        )

    if not rotation_observability(base_to_end)["pass"]:
        raise ValueError("hand-eye motion is unobservable or ill-conditioned; use multiple rotation axes")
    rotation, translation = cv2.calibrateHandEye(
        [value[:3, :3] for value in base_to_end],
        [value[:3, 3] for value in base_to_end],
        [value[:3, :3] for value in camera_to_marker],
        [value[:3, 3] for value in camera_to_marker],
        method=HAND_EYE_METHODS[method_name],
    )
    # OpenCV 返回旋转矩阵与平移向量,这里重新组装成统一的 4x4 齐次变换
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{method_name} hand-eye solve returned non-finite values")
    _validate_transform(result, "solution")
    return result


def evaluate_eye_in_hand(
    samples: Sequence[Mapping[str, object]],
    end_to_camera: np.ndarray,
    *,
    reference_base_to_marker: np.ndarray | None = None,
) -> dict[str, object]:
    """用给定的 ``end_to_camera`` 评估多样本一致性(正向重算 + 残差统计).

    对每个样本计算 ``base->marker = base->end · end->camera · camera->marker``;标记
    固定不动,所以这些结果应当一致,离散程度就是标定质量的度量.

    参考值来源有两种:未提供 ``reference_base_to_marker`` 时用样本自身的平均变换,
    ``reference_source`` 记为 ``evaluation_mean``(属于自评估,结果偏乐观);提供时
    使用给定值,记为 ``provided``-典型用法是用训练集算出的标记位姿去评估留出集,
    避免"自己评自己".

    位置残差单位为 m,姿态残差为两旋转矩阵的测地线夹角(度);报告同时给出
    RMS/最大/中位数与逐样本明细(``samples`` 中的 ``label`` 沿用输入的标签).
    """
    if not samples:
        raise ValueError("at least one evaluation sample is required")
    handeye = np.asarray(end_to_camera, dtype=np.float64)
    _validate_transform(handeye, "end_to_camera")

    marker_values = []
    labels = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, Mapping):
            raise ValueError(f"samples[{index}] must be a mapping")
        base_end = _sample_transform(sample.get("base_to_end"), "base_to_end")
        camera_marker = _sample_transform(sample.get("camera_to_marker"), "camera_to_marker")
        marker_values.append(base_end @ handeye @ camera_marker)
        labels.append(str(sample.get("label", f"sample_{index + 1}")))

    if reference_base_to_marker is None:
        reference = average_transforms(marker_values)
        reference_source = "evaluation_mean"
    else:
        reference = np.asarray(reference_base_to_marker, dtype=np.float64)
        _validate_transform(reference, "reference_base_to_marker")
        reference_source = "provided"

    position_errors = np.array(
        [np.linalg.norm(value[:3, 3] - reference[:3, 3]) for value in marker_values],
        dtype=np.float64,
    )
    # 姿态残差:R_ref^T @ R_i 的等效转角,即两个姿态之间的测地线距离(度)
    rotation_errors = np.array(
        [rotation_angle_deg(reference[:3, :3].T @ value[:3, :3]) for value in marker_values],
        dtype=np.float64,
    )
    details = []
    for label, value, position_error, rotation_error in zip(
        labels, marker_values, position_errors, rotation_errors
    ):
        details.append(
            {
                "label": label,
                "base_to_marker": matrix_transform(value),
                "position_residual_m": float(position_error),
                "rotation_residual_deg": float(rotation_error),
            }
        )
    return {
        "sample_count": len(samples),
        "reference_source": reference_source,
        "reference_base_to_marker": matrix_transform(reference),
        "position_residual": _residual_summary(position_errors, "m"),
        "rotation_residual": _residual_summary(rotation_errors, "deg"),
        "samples": details,
    }


def average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """多组 4x4 变换求平均:平移取算术平均,旋转取投影回 SO(3) 后的平均.

    旋转矩阵元素不能直接平均(结果不再正交),这里用 SVD 把元素均值投影回旋转群:
    ``R = U @ V^T``.若 ``det(R) < 0``,说明投影落到了含镜像的 O(3) 上,把 U 的最后
    一列取反后重算,强制得到行列式 +1 的右手系旋转.传入的每个变换都会先做刚体性校验.
    """
    if not transforms:
        raise ValueError("at least one transform is required")
    values = [np.asarray(value, dtype=np.float64) for value in transforms]
    for index, value in enumerate(values):
        _validate_transform(value, f"transforms[{index}]")
    left, _, right = np.linalg.svd(
        np.mean(np.stack([value[:3, :3] for value in values]), axis=0)
    )
    rotation = left @ right
    if np.linalg.det(rotation) < 0.0:
        # 元素均值投影到了含镜像的正交矩阵上:翻转 U 的最后一列,回到 SO(3)
        left[:, -1] *= -1.0
        rotation = left @ right
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = np.mean(
        np.stack([value[:3, 3] for value in values]), axis=0
    )
    return result


def rotation_angle_deg(rotation: np.ndarray) -> float:
    """旋转矩阵对应的测地线转角(单位:度),即绕等效轴转过的角度.

    由迹公式 ``cos(theta) = (trace(R) - 1) / 2`` 求得;``min/max`` 夹紧到 [-1, 1] 是
    浮点保护,避免 trace 略微越界时 ``acos`` 返回 NaN.衡量两个姿态差异时传入
    ``R_ref^T @ R``,也可用于判断样本姿态是否足够分散(跨度不足则手眼解不可信).
    """
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    cosine = min(1.0, max(-1.0, (float(np.trace(value)) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def _mapping(value: object, label: str) -> Mapping[str, object]:
    """确认样本字段是映射(字典)类型,失败时抛 ``ValueError`` 并带上字段名."""
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _validate_transform(value: np.ndarray, label: str) -> None:
    """校验 4x4 齐次变换的合法性,不合法即抛 ``ValueError``.

    要求:形状为 4x4 且元素有限;最后一行为 ``[0, 0, 0, 1]``(齐次性,容差 1e-9);
    旋转块正交(``R^T R = I``,容差 1e-7,同时排除缩放);行列式为 +1(排除镜像,
    容差 1e-7).在残差统计之前拦住"看起来像变换但不是刚体变换"的输入,避免得到
    无物理意义的报告.
    """
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise ValueError(f"{label} must be a finite 4x4 transform")
    if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{label} must be homogeneous")
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7):
        raise ValueError(f"{label} rotation must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-7):
        raise ValueError(f"{label} rotation determinant must be +1")


def _residual_summary(values: np.ndarray, unit: str) -> dict[str, float]:
    """把残差数组汇总成 ``rms_*`` / ``max_*`` / ``median_*`` 三个键.

    ``unit`` 既参与拼键名(``rms_m`` 为位置残差,``rms_deg`` 为姿态残差),也是这些
    报告键的对外契约,不要翻译;上层工具按这些键读取验收所需的统计量.
    """
    return {
        f"rms_{unit}": float(math.sqrt(float(np.mean(values**2)))),
        f"max_{unit}": float(np.max(values)),
        f"median_{unit}": float(np.median(values)),
    }


def _sample_transform(value, label):
    if isinstance(value, Mapping):
        return transform_matrix(value)
    result = np.asarray(value, dtype=np.float64)
    _validate_transform(result, label)
    return result
