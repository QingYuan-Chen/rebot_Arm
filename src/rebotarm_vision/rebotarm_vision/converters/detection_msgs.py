"""检测结果转换器：把推理后端的原始结果对象转换成统一的检测消息。

职责与位置
----------
YOLO 类后端（在线视觉节点与离线回放节点）推理后拿到的是第三方结果对象（张量形式的
框、类别、置信度，可能带旋转框与实例掩膜），本模块把它们规范化成消息数组再对外发布：
轴对齐检测框（``x_min``/``y_min``/``x_max``/``y_max``、``center_u``/``center_v``）、
可选旋转框（``has_obb`` 与 ``obb_*``）、可选掩膜多边形（``has_mask`` 与
``mask_polygon_xy``）。

坐标与单位约定
--------------
所有像素坐标都在彩色图像坐标系下：原点在图像左上角，u 向右（列）、v 向下（行），
尺寸与彩色图一致。消息头填入调用方给定的时间戳与彩色坐标系 frame_id；下游要拿三维
抓取点时，会结合相机内参与深度图按像素反投影，因此这里的像素坐标必须与深度图严格
对齐（同帧、同分辨率）。

依赖说明
--------
本模块只依赖数值库与消息类型，不做任何过滤决策；类别筛选由 ``allowed_classes``
参数在转换时顺带完成（比较前会去空白并转小写），是否发布空帧由调用方决定。
"""

from __future__ import annotations

from typing import Any

import numpy as np
from rebotarm_msgs.msg import Detection2D, Detection2DArray


class MaskPolygonError(ValueError):
    """模型已提供实例掩膜，但其数据无法作为该目标的像素采样区域。"""


def _tensor_to_numpy(value: Any):
    """把推理后端的张量/数组/标量统一转成 numpy 数组。

    顺序很重要：先 ``detach`` 断开计算图，再 ``cpu`` 搬回主机内存，最后 ``numpy``；
    缺少这些方法的对象（普通列表、标量）退回 ``np.asarray``。``None`` 原样返回，
    由调用方按"该字段缺失"处理。
    """
    if value is None:
        return None
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def _safe_attr_row(container: Any, attr: str, index: int):
    """取出 ``container.attr[index]`` 并转成 numpy；缺失或越界时返回 None。

    旋转框属于可选输出，不同后端或不同帧不一定都有，因此这里把异常吞掉返回
    None，让调用方降级为"无该信息"，而不是让整帧检测失败。
    """
    values = getattr(container, attr, None)
    if values is None:
        return None
    try:
        return _tensor_to_numpy(values[index])
    except Exception:
        return None


def _obb_metadata(result: Any, index: int):
    """提取第 ``index`` 个目标的旋转框元数据，取不到时返回 None。

    返回值字典字段：``cx``/``cy`` 为中心像素坐标；``w``/``h`` 为旋转后宽高（像素）；
    ``theta`` 为绕图像法向的旋转角（单位 rad，直接取模型 xywhr 的第 5 个分量）；
    ``points`` 为四角点展开的 (x0,y0,...,x3,y3) 列表（像素），缺失时为空列表。

    角点形状在不同后端下有 (1,4,2)、(4,2)、(8,) 等差异，这里统一归一到 4x2；只要
    角点数量对不上就退回空列表，让下游按"仅有中心与尺寸"使用。
    """
    obb = getattr(result, "obb", None)
    if obb is None:
        return None
    xywhr = _safe_attr_row(obb, "xywhr", index)
    corners = _safe_attr_row(obb, "xyxyxyxy", index)
    if xywhr is None:
        return None
    xywhr = np.asarray(xywhr).reshape(-1)
    # xywhr 必须凑齐 (cx, cy, w, h, theta) 五个分量，否则视为无效旋转框。
    if xywhr.size < 5:
        return None
    points = []
    if corners is not None:
        corners = np.asarray(corners, dtype=np.float32)
        if corners.ndim == 3 and corners.shape[0] == 1:
            corners = corners[0]
        if corners.ndim == 1 and corners.size == 8:
            corners = corners.reshape(4, 2)
        if corners.shape == (4, 2):
            points = [float(v) for v in corners.reshape(-1)]
    return {
        "cx": float(xywhr[0]),
        "cy": float(xywhr[1]),
        "w": float(xywhr[2]),
        "h": float(xywhr[3]),
        "theta": float(xywhr[4]),
        "points": points,
    }


def _mask_polygon(result: Any, index: int) -> list[float]:
    """提取第 ``index`` 个目标的实例掩膜多边形，返回展开的 (x0,y0,x1,y1,...) 像素列表。

    模型未提供掩膜时返回空列表，保留合法的框检测。已提供的掩膜须为至少三个有限顶点
    的 (N,2) 数组；损坏时拒绝整帧转换，避免下游扩大到整个检测框采样深度。
    """
    masks = getattr(result, "masks", None)
    if masks is None:
        return []
    polygons = getattr(masks, "xy", None)
    if polygons is None:
        raise MaskPolygonError(f"mask[{index}]: xy is missing")
    try:
        polygon = polygons[index]
    except (IndexError, KeyError, TypeError) as exc:
        raise MaskPolygonError(f"mask[{index}]: polygon is missing") from exc
    if polygon is None:
        raise MaskPolygonError(f"mask[{index}]: polygon is None")
    # 张量搬运故障保留原异常；普通数组的数值/形状问题才属于可拒绝的模型数据。
    if any(hasattr(polygon, attr) for attr in ("detach", "cpu", "numpy")):
        polygon = _tensor_to_numpy(polygon)
    try:
        polygon = np.asarray(polygon, dtype=np.float32)
    except (TypeError, ValueError, OverflowError) as exc:
        raise MaskPolygonError(f"mask[{index}]: polygon must contain numeric coordinates") from exc
    if polygon.ndim != 2 or polygon.shape[1] != 2:
        raise MaskPolygonError(f"mask[{index}]: expected shape (N, 2), got {polygon.shape}")
    if polygon.shape[0] < 3:
        raise MaskPolygonError(f"mask[{index}]: at least 3 vertices are required")
    if not np.isfinite(polygon).all():
        raise MaskPolygonError(f"mask[{index}]: coordinates must be finite")
    return [float(value) for value in polygon.reshape(-1)]


def result_to_detection_array_msg(
    results,
    stamp,
    frame_id: str,
    *,
    allowed_classes: set[str] | None = None,
) -> Detection2DArray:
    """把一帧的推理结果转换成检测消息数组。

    参数：
    - ``results``：推理后端返回的结果序列，每项需带 ``names``（类别下标到类别名的
      映射）、``boxes``（轴对齐检测框集合），可选带 ``obb``、``masks``。
    - ``stamp``：产生该帧彩色图的时间戳，写入消息头与每个检测项，供下游做时间对齐
      与新鲜度判定。
    - ``frame_id``：彩色图像坐标系名，同时写入消息头与每个检测项。
    - ``allowed_classes``：类别白名单（调用方已去空白并转小写）；为 None 表示不过滤，
      否则只保留 ``class_name`` 去空白并转小写后命中的检测。

    返回：一条检测数组消息。没有任何检测、或全部被白名单过滤时为空的 ``detections``
    列表（不是 None），调用方据此发布空帧以表明"本帧已处理但无目标"。
    任一参与转换的掩膜损坏时抛出 ``MaskPolygonError``，已积累的部分检测随本次转换丢弃。

    像素坐标：``center_u``/``center_v`` 由框角点取整后求中点再四舍五入；``x_min``
    等直接取截断后的整数，因此保留检测框原始的像素边界约定。置信度原样透传，不做
    二次阈值判断。检测框缺失（``boxes`` 为 None）的结果会被整条跳过。
    """
    msg = Detection2DArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    for result in results:
        names = getattr(result, "names", {})
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            continue
        for index in range(len(boxes)):
            box = boxes[index]
            det = Detection2D()
            det.header.stamp = stamp
            det.header.frame_id = frame_id

            # 后端把单个目标的框/类别/置信度都放在长度为 1 的批量维度里，取 [0] 后展平。
            xyxy = np.asarray(_tensor_to_numpy(box.xyxy[0])).reshape(-1)
            cls_id = int(np.asarray(_tensor_to_numpy(box.cls[0])).reshape(-1)[0])
            conf = float(np.asarray(_tensor_to_numpy(box.conf[0])).reshape(-1)[0])

            # 截断为整数像素；取前 4 个分量即 (x_min, y_min, x_max, y_max)。
            x1, y1, x2, y2 = [int(v) for v in xyxy[:4]]
            det.class_name = names.get(cls_id, str(cls_id))
            # 白名单在转换阶段就过滤掉，避免不关心的类别进入下游抓取链路。
            if allowed_classes is not None and det.class_name.strip().lower() not in allowed_classes:
                continue
            det.confidence = conf
            det.center_u = int(round((x1 + x2) / 2.0))
            det.center_v = int(round((y1 + y2) / 2.0))
            det.x_min = x1
            det.y_min = y1
            det.x_max = x2
            det.y_max = y2

            obb_meta = _obb_metadata(result, index)
            if obb_meta is not None:
                det.has_obb = True
                det.obb_cx = obb_meta["cx"]
                det.obb_cy = obb_meta["cy"]
                det.obb_w = obb_meta["w"]
                det.obb_h = obb_meta["h"]
                det.obb_theta = obb_meta["theta"]
                det.obb_points_xy = obb_meta["points"]
            else:
                det.has_obb = False
                det.obb_points_xy = []

            # 掩膜至少要 3 个点（6 个浮点数）才算有效，与下游的填充多边形要求一致。
            mask_polygon = _mask_polygon(result, index)
            det.has_mask = len(mask_polygon) >= 6
            det.mask_polygon_xy = mask_polygon

            msg.detections.append(det)

    return msg
