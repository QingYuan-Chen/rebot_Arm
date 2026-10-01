"""ArUco 单标记检测与「相机点 → 基座坐标」变换的视觉辅助函数。

用途：为标定流程提供基准点测量——在相机彩色图中检测指定 id 的 ArUco 标记，用已知相机
内参与标记实际边长解出标记中心在**相机光学坐标系**下的三维坐标（单位 m），再经手眼/
机器人 TF 变换到基座坐标系，得到机器人可以对齐的参考点。上层（标定包的自动参考点
提供者）只负责取图与查 TF，数学部分都在这里。

坐标系与单位约定：

- 图像为 OpenCV 习惯的 BGR 三通道；
- 相机光学坐标系：x 向右、y 向下、z 沿光轴指向前方，单位 m；
- ``marker_length_m`` 是标记黑色边框外沿的边长，是唯一的尺度来源：写错会让平移量按比例
  整体失真；
- 不订阅话题、不查 TF、不写文件，全部是纯计算，便于在无硬件环境下测试。
"""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np

from .tcp_calibration import quaternion_to_rotation_matrix


def resolve_aruco_dictionary_id(name: str | int, *, cv2_module=cv2) -> int:
    """把字典名解析为 OpenCV 的字典编号。

    接受的写法：整数编号、纯数字字符串（"5"）、完整名 ``DICT_5X5_100``、以及省略前缀的
    短名 ``5X5_100``（会自动补 ``DICT_``）。查找前统一转大写并去空白，因此大小写不敏感。
    找不到对应字典时抛 ``ValueError``（消息为英文常量）。

    ``cv2_module`` 允许注入替身模块，使单元测试不需要真的安装 ArUco 模块。
    """

    if isinstance(name, int):
        return int(name)

    normalized = str(name).strip().upper()
    if normalized.isdigit():
        return int(normalized)
    candidates = [normalized]
    if not normalized.startswith("DICT_"):
        candidates.append(f"DICT_{normalized}")

    for candidate in candidates:
        if hasattr(cv2_module.aruco, candidate):
            return int(getattr(cv2_module.aruco, candidate))
    raise ValueError(f"unsupported ArUco dictionary: {name}")


def build_camera_matrix(
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """由针孔模型参数组装 3x3 相机内参矩阵（float64）。

    ``fx`` / ``fy``：x、y 方向焦距，单位像素（通常由标定或 CameraInfo 的 ``k`` 给出）。
    ``cx`` / ``cy``：主点（光轴与像面交点）的像素坐标，理想情况下接近图像中心。
    矩阵按 OpenCV 约定填成 ``[[fx,0,cx],[0,fy,cy],[0,0,1]]``，即假设无像素斜切、无畸变。
    """

    return np.array(
        [
            [float(fx), 0.0, float(cx)],
            [0.0, float(fy), float(cy)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def detect_aruco_center_in_camera(
    color_bgr: np.ndarray,
    *,
    camera_matrix: np.ndarray,
    marker_length_m: float,
    dictionary_name: str | int = "DICT_4X4_50",
    marker_id: int = 0,
    dist_coeffs: Sequence[float] | None = None,
) -> tuple[float, float, float]:
    """检测指定 id 的 ArUco 标记，返回其中心在相机坐标系下的三维坐标（米）。

    参数：
    - ``color_bgr``：BGR 图像；为空（``None``）直接抛 ``ValueError``；
    - ``camera_matrix``：``build_camera_matrix`` 给出的 3x3 内参，单位像素；
    - ``marker_length_m``：标记边长（黑框外沿），单位 m，必须 > 0，是唯一的尺度基准；
    - ``dictionary_name``：字典名或编号，必须与打印的标记一致；
    - ``marker_id``：目标标记 id；图中检测到其他标记但缺这一个时同样报错，避免拿错点；
    - ``dist_coeffs``：畸变系数序列（CameraInfo 的 ``d``），``None`` 表示按无畸变处理，
      此时内部用 5 个 0 作为系数。

    返回 ``(x, y, z)``：标记中心在相机光学坐标系中的位置，单位 m，x 向右、y 向下、
    z 向前。检测用 ``cv2.aruco.ArucoDetector``（新版 API，参数取 OpenCV 默认值，未额外
    开启角点亚像素细化）；位姿由单标记正方形位姿解算给出，其平移向量即标记中心。

    异常：标记未检测到、目标 id 不在结果中、或输入非法（无图像、边长非正）时抛
    ``ValueError``；调用方应按「本帧测量失败」处理。
    """

    if color_bgr is None:
        raise ValueError("missing color image")
    if marker_length_m <= 0.0:
        raise ValueError("marker_length_m must be positive")

    dictionary_id = resolve_aruco_dictionary_id(dictionary_name)
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(color_bgr)
    if ids is None or len(ids) == 0:
        raise ValueError("target ArUco marker not detected")

    # ids 是 (N,1) 二维数组，展平后按 id 定位目标标记在 corners 中的下标。
    flat_ids = [int(value[0]) for value in ids]
    if int(marker_id) not in flat_ids:
        raise ValueError(f"ArUco marker id {marker_id} not detected")
    index = flat_ids.index(int(marker_id))

    distortion = np.zeros((5, 1), dtype=np.float64)
    if dist_coeffs is not None:
        distortion = np.asarray(list(dist_coeffs), dtype=np.float64).reshape(-1, 1)

    # 只把命中的那一个标记的角点交给解算器；tvecs 形状为 (1,1,3)，取 [0][0] 得到平移向量。
    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[index]],
        float(marker_length_m),
        camera_matrix,
        distortion,
    )
    tvec = np.asarray(tvecs[0][0], dtype=np.float64)
    return (float(tvec[0]), float(tvec[1]), float(tvec[2]))


def transform_camera_point_to_base(
    transform_stamped,
    *,
    camera_point_xyz: Sequence[float],
) -> tuple[float, float, float]:
    """把相机坐标系下的点变换到基座坐标系。

    ``transform_stamped`` 方向应为「基座 ← 相机」。函数只做几何变换，
    TF 方向和时间新鲜度由调用方负责。
    """
    transform = transform_stamped.transform
    rotation = transform.rotation
    matrix = quaternion_to_rotation_matrix(
        (rotation.x, rotation.y, rotation.z, rotation.w)
    )
    translation = np.array(
        (transform.translation.x, transform.translation.y, transform.translation.z),
        dtype=np.float64,
    )
    result = translation + matrix @ np.asarray(camera_point_xyz, dtype=np.float64)
    return tuple(round(float(value), 6) for value in result)
