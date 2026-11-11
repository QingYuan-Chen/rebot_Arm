"""检测结果可视化：在 BGR 彩色图上叠加框、掩膜轮廓与标签。

用途是调试预览与标注图像发布（例如给操作界面看的一路带框图）。函数始终在输入
图像的副本上绘制，不修改调用方的原图；也不做任何坐标变换——传入的检测框必须
已经是该图像的像素坐标。

绘制约定：
    - 普通检测框：绿色矩形 (BGR 0, 255, 0)，线宽 2；
    - 旋转框（OBB）：4 点闭合折线，浅蓝 (255, 200, 0)，抗锯齿；
    - 实例掩膜：多边形轮廓，深蓝 (255, 120, 0)，抗锯齿；
    - 标签：``类别名 置信度``（置信度保留两位小数），黄色文字。
"""

from __future__ import annotations

import cv2
import numpy as np


def draw_detections(image_bgr, detection_msg):
    """把一条检测结果消息画到 BGR 图副本上并返回。

    参数：
        image_bgr: H×W×3 的 BGR 图像（通常是相机原图）。
        detection_msg: 检测结果消息，逐项读取 ``detections``；每个检测需有
            x_min/y_min/x_max/y_max（像素）与 class_name/confidence，
            可选 has_obb + obb_points_xy（8 个数 = 4 点 × (x, y)）、
            has_mask + mask_polygon_xy。

    返回：
        叠加绘制后的新图像；输入图像保持不变。
    """

    annotated = image_bgr.copy()
    for det in detection_msg.detections:
        cv2.rectangle(
            annotated,
            (det.x_min, det.y_min),
            (det.x_max, det.y_max),
            (0, 255, 0),
            2,
        )
        # 旋转框：仅在标记存在且点数正好是 4 个 (x, y) 时才画，避免脏数据 reshape 出错。
        if det.has_obb and len(det.obb_points_xy) == 8:
            points = np.asarray(det.obb_points_xy, dtype=np.float32).reshape(4, 2)
            cv2.polylines(
                annotated,
                [np.round(points).astype(np.int32)],
                True,
                (255, 200, 0),
                2,
                cv2.LINE_AA,
            )
        # 掩膜轮廓：用 getattr 兼容没有 has_mask 字段的旧版消息；至少 3 个点才成多边形。
        elif getattr(det, "has_mask", False) and len(det.mask_polygon_xy) >= 6:
            points = np.asarray(det.mask_polygon_xy, dtype=np.float32).reshape(-1, 2)
            cv2.polylines(
                annotated,
                [np.round(points).astype(np.int32)],
                True,
                (255, 120, 0),
                2,
                cv2.LINE_AA,
            )
        label = f"{det.class_name} {det.confidence:.2f}"
        # 文字基线至少留 16 像素，防止贴近上边缘时标签被裁掉。
        cv2.putText(
            annotated,
            label,
            (det.x_min, max(det.y_min - 8, 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return annotated
