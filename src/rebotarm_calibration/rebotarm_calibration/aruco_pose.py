"""ArUco 标记检测与相机→标记位姿估计(相机内参已知).

流程:BGR 图像 → ArUco 角点检测(亚像素细化)→ 用已知内参与畸变系数,以正方形标记的
4 个角点做 IPPE 平面位姿求解 → 输出 ``camera_to_marker`` 4x4 齐次变换,角点像素坐标,
像素面积与重投影 RMSE.

本模块只做单帧纯计算:不订阅话题,不查询 TF,不写文件.上层节点负责取图(连同
CameraInfo 里的内参 ``k`` 与畸变 ``d``)并把结果用于 TCP 参考点测量或手眼标定样本.
位姿解算以标记边长为唯一尺度来源,因此 ``marker_length_m`` 必须与实际打印尺寸一致,
否则平移量会按比例整体失真.

坐标系约定:``camera_to_marker`` 把标记坐标系中的点变换到相机坐标系,标记原点位于
标记中心,z 轴垂直标记平面向外,单位为米.
"""

from __future__ import annotations

import math
from typing import Sequence

import cv2
import numpy as np

from .handeye_residual import matrix_transform


def detector_parameters():
    """构造 ArUco 检测参数:角点使用亚像素细化.

    兼容两代 OpenCV Python 绑定:新版提供 ``DetectorParameters`` 对象,旧版只有工厂函数
    ``DetectorParameters_create()``.开启亚像素细化(``CORNER_REFINE_SUBPIX``)可显著
    降低角点像素误差,而角点精度直接决定位姿精度,对深度方向尤其明显.
    """
    parameters = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector")
        else cv2.aruco.DetectorParameters_create()
    )
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return parameters


def detect_aruco_pose(
    color_bgr: np.ndarray,
    *,
    camera_matrix: np.ndarray,
    distortion: Sequence[float],
    marker_length_m: float,
    dictionary_name: str = "DICT_4X4_50",
    marker_id: int = 0,
) -> dict[str, object]:
    """检测指定 id 的 ArUco 标记并解算 ``camera_to_marker``.

    参数:
    - ``color_bgr``:BGR 三通道图像(即 ``bgr8`` 编码解出的格式),必须是 3 通道;
    - ``camera_matrix``:3x3 相机内参矩阵(来自 CameraInfo 的 ``k``),单位像素;
    - ``distortion``:畸变系数序列(来自 CameraInfo 的 ``d``,通常 5 个),内部整理为
      列向量后交给 OpenCV;
    - ``marker_length_m``:标记边长(黑色边框外沿的边长),单位 m,必须为正且有限;
    - ``dictionary_name``:字典名,必须与打印出来的标记一致(默认 ``DICT_4X4_50``),
      名称不存在于 OpenCV 命名空间时抛 ``ValueError``;
    - ``marker_id``:目标标记 id;未检测到该 id 时抛 ``ValueError``.多标记场景下只取
      指定 id,避免误用画面里的其他标记.

    返回字典(键名是对外接口):
    - ``camera_to_marker``:4x4 齐次变换,标记原点在标记中心,z 轴垂直标记平面向外;
    - ``corners_px``/``center_px``/``area_px2``:4 个角点,中心与像素面积,供上层判断
      可见性,距离与稳定性(面积过小意味着角点量化误差占比过大);
    - ``reprojection_rmse_px``:用解出的位姿把标记角点重投影回图像,与实测角点比较的
      RMS 误差(像素),是位姿可信度最直接的指标.
    """
    if color_bgr is None or color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
        raise ValueError("color_bgr must be a BGR image")
    length = float(marker_length_m)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("marker_length_m must be finite and positive")
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"unsupported ArUco dictionary: {dictionary_name}")
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("camera_matrix must be finite 3x3")
    coefficients = np.asarray(list(distortion), dtype=np.float64).reshape(-1, 1)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
        raise ValueError("invalid camera intrinsics")
    if coefficients.size not in (0, 4, 5, 8, 12, 14) or not np.all(np.isfinite(coefficients)):
        raise ValueError("unsupported or non-finite distortion coefficients")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    parameters = detector_parameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(color_bgr)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            color_bgr,
            dictionary,
            parameters=parameters,
        )
    if ids is None:
        raise ValueError("target ArUco marker not detected")
    # 检测器返回的 ids 约定为 Nx1 数组,这里取每行首元素展平,再按 id 定位对应角点
    # (不同 OpenCV 大版本的 ids 形状可能不同,此处依赖 Nx1 约定)
    flat_ids = [int(value[0]) for value in ids]
    if int(marker_id) not in flat_ids:
        raise ValueError(f"ArUco marker id {marker_id} not detected")
    image_points = np.asarray(corners[flat_ids.index(int(marker_id))][0], dtype=np.float64)
    half = length * 0.5
    # 标记坐标系下的 4 个角点,顺序与检测器输出一致:左上,右上,右下,左下(单位 m);
    # 原点在标记中心,z 轴垂直标记平面向外,因此平面内 z 分量为 0
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    # IPPE_SQUARE 专用于"已知边长的平面正方形"位姿求解,平面位姿可能存在歧义;
    # 上层还必须检查质量与时间一致性;相机内参与畸变系数直接沿用标定值
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        matrix,
        coefficients,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        raise ValueError("ArUco IPPE pose solve failed")
    # Rodrigues:旋转向量(轴角,模长为转角弧度)→ 旋转矩阵
    rotation, _ = cv2.Rodrigues(rvec)
    camera_to_marker = np.eye(4, dtype=np.float64)
    camera_to_marker[:3, :3] = rotation
    camera_to_marker[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(camera_to_marker)) or np.any((rotation @ object_points.T + np.asarray(tvec).reshape(3, 1))[2] <= 0):
        raise ValueError("marker pose must be finite and in front of camera")
    # 重投影校验:把解出的位姿重新投影成像素,与实测角点比较,得到位姿质量的直接指标
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, coefficients)
    projected = projected.reshape(-1, 2)
    reprojection_rmse = math.sqrt(float(np.mean(np.sum((projected - image_points) ** 2, axis=1))))
    return {
        "marker_id": int(marker_id),
        "corner_refinement": "CORNER_REFINE_SUBPIX",
        "corners_px": image_points.tolist(),
        "center_px": np.mean(image_points, axis=0).tolist(),
        "area_px2": abs(float(cv2.contourArea(image_points.astype(np.float32)))),
        "reprojection_rmse_px": reprojection_rmse,
        "camera_to_marker": matrix_transform(camera_to_marker),
    }
