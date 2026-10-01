"""相机原始帧 → 图像/相机信息消息的转换层。

上游相机驱动给出的是 BGR 彩色图（H×W×3 数组）、毫米深度图（H×W 的 16 位数组）
和一个相机参数字典；本模块把它们装进图像消息与相机信息消息，供检测、点云与
抓取节点消费。转换本身不做任何图像处理，也不做单位换算。

单位与编码约定：
    - 彩色：``bgr8``，每像素 3 字节，行字节数 = 宽 × 3；
    - 深度：``mono16``，每像素 2 字节，数值单位为毫米（1 mm = 0.001 m）；
    - 相机信息：焦距与主点单位为像素，畸变模型与内参矩阵按针孔模型给出。
"""

from __future__ import annotations

from typing import Mapping

import numpy as np
from sensor_msgs.msg import CameraInfo, Image


def _intrinsic_value(camera_info: Mapping[str, object], name: str, index: int) -> float:
    """读取单个内参，按 ``name`` → ``k[index]`` → ``0.0`` 回退。

    相机配置来自 YAML，``k`` 可能缺失或长度不足；把这项容错集中在纯函数中，
    让消息组装逻辑只关注 ROS 字段映射，并避免四个内参重复实现同一回退规则。
    """

    if name in camera_info and camera_info[name] is not None:
        return float(camera_info[name])
    intrinsic_matrix = camera_info.get("k", ())
    try:
        return float(intrinsic_matrix[index])  # type: ignore[index]
    except (IndexError, KeyError, TypeError, ValueError):
        return 0.0


def color_to_msg(image_bgr, stamp, frame_id: str) -> Image:
    """把 BGR 彩色图打包成 ``bgr8`` 图像消息。

    stamp 与 frame_id 原样写入 header（时间基准与坐标系由调用方统一决定）。
    ``ascontiguousarray`` 保证底层内存连续，否则 ``tobytes`` 出来的字节流顺序
    可能与行序不一致。step 按 3 通道 × 宽计算，即假设输入是紧凑排列。
    """

    image_bgr = np.ascontiguousarray(image_bgr)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(image_bgr.shape[0])
    msg.width = int(image_bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(image_bgr.shape[1] * image_bgr.shape[2])
    msg.data = image_bgr.tobytes()
    return msg


def depth_to_msg(depth_mm, stamp, frame_id: str) -> Image:
    """把 16 位毫米深度图打包成 ``mono16`` 图像消息。

    深度值单位毫米，0 表示无效/无回波（由消费方自行判断，本函数不做填充或
    截断）。step 固定按每像素 2 字节计算，与 16 位深度图的假设一致。
    """

    depth_mm = np.ascontiguousarray(depth_mm)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(depth_mm.shape[0])
    msg.width = int(depth_mm.shape[1])
    msg.encoding = "mono16"
    msg.is_bigendian = 0
    msg.step = int(depth_mm.shape[1] * 2)
    msg.data = depth_mm.tobytes()
    return msg


def depth_image_to_array(msg: Image) -> np.ndarray:
    """将 ``mono16``/``16UC1`` 深度消息解码为毫米 ``uint16`` 数组。

    数据长度必须与图像尺寸一致；无效编码或尺寸会抛出 ``ValueError``，
    调用节点负责拒绝该帧。返回值与原消息共享只读数据缓冲区。
    """

    if msg.encoding not in ("mono16", "16UC1"):
        raise ValueError(f"unsupported depth encoding: {msg.encoding}")
    if msg.height <= 0 or msg.width <= 0:
        raise ValueError(f"invalid depth dimensions: {msg.width}x{msg.height}")
    depth = np.frombuffer(msg.data, dtype=np.uint16)
    expected = msg.height * msg.width
    if depth.size != expected:
        raise ValueError(f"depth payload has {depth.size} pixels, expected {expected}")
    return depth.reshape((msg.height, msg.width))


def camera_info_to_msg(camera_info: dict, stamp, frame_id: str) -> CameraInfo:
    """由相机参数字典构造相机信息消息。

    字典字段与回退规则（键名区分大小写）：
        - ``fx``/``fy``/``cx``/``cy``：优先取同名键；缺失时分别回退到内参数组
          键 ``k`` 的第 0、4、2、5 项（列主序的 3×3 内参矩阵）；再缺失则取 0.0。
        - ``height``/``width``：图像像素尺寸，缺失记 0（消费方需自行容错）。
        - ``distortion_model``：畸变模型名，默认 ``plumb_bob``。
        - ``d``（兼容大写 ``D``）：畸变系数，默认空列表。
    输出中 R 固定为单位阵、P 由内参扩展成 3×4（平移列为 0），即假设图像已去
    畸变且无立体外参——这是单目 RGB-D 相机的常见约定。
    """

    fx = _intrinsic_value(camera_info, "fx", 0)
    fy = _intrinsic_value(camera_info, "fy", 4)
    cx = _intrinsic_value(camera_info, "cx", 2)
    cy = _intrinsic_value(camera_info, "cy", 5)
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(camera_info.get("height", 0))
    msg.width = int(camera_info.get("width", 0))
    msg.distortion_model = str(camera_info.get("distortion_model", "plumb_bob"))
    msg.d = [float(value) for value in camera_info.get("d", camera_info.get("D", []))]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg
