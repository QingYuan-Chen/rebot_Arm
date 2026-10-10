"""Gemini 2 相机的本地调试预览脚本（不是 ROS 节点）。

直接用深度相机 SDK 打开设备，把彩色图与深度伪彩图显示在两个 OpenCV 窗口里，并在画面上
叠加 min/max/mean 等像素统计，用于快速判断「相机是否出图、彩色是否过曝/全黑、深度是否有
有效值」。生产抓取链路不依赖这个脚本，它只服务于现场排障与安装调参。

需要图形界面：OpenCV highgui 走 Qt 后端，显示相关的环境变量（例如
``QT_QPA_PLATFORM``、``QT_QPA_FONTDIR``）由上层启动文件设置。安装后可用包内注册的命令
``rebotarm_debug_camera_preview`` 启动，也可以直接以 ``python3 -m
rebotarm_vision.diagnostics.debug_camera_preview`` 执行；按 q/Q 或 ESC 退出。
"""

from __future__ import annotations

import cv2
import numpy as np

from ..camera.gemini2_driver import Gemini2Config, Gemini2Driver


def _depth_to_u8(depth_mm: np.ndarray | None) -> np.ndarray | None:
    """把深度图（单位 mm）映射成 JET 伪彩 BGR 图，仅用于人眼观察。

    对比度拉伸用有效像素（> 0）的 5% 与 95% 分位数作为近端与远端：取分位数而不是
    min/max，是为了不让个别离群点（远端噪点、反光点）把整幅图压成一片暗色。
    无深度（值为 0）的像素在输出中强制置 0，对应 JET 色表的深蓝端，便于一眼区分
    「没测到」与「很远的有效值」。

    ``depth_mm`` 为 ``None`` 时返回 ``None``，表示这一帧没有深度；全部像素都无效时
    返回全黑图像（形状仍是三通道）。
    """

    if depth_mm is None:
        return None
    valid = depth_mm[depth_mm > 0]
    if valid.size == 0:
        return np.zeros((depth_mm.shape[0], depth_mm.shape[1], 3), dtype=np.uint8)
    near = float(np.percentile(valid, 5))
    far = float(np.percentile(valid, 95))
    # 视野内深度几乎一致时 near == far，加 1 mm 避免后面除零。
    if far <= near:
        far = near + 1.0
    clipped = np.clip(depth_mm.astype(np.float32), near, far)
    normalized = ((clipped - near) / (far - near) * 255.0).astype(np.uint8)
    normalized[depth_mm == 0] = 0
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def main() -> None:
    """打开相机、循环预览，直到按键退出；退出时保证释放设备与窗口。"""

    # 彩色固定 640x480@30；深度宽高传 0 表示不指定，由驱动选相机默认深度 profile。
    # enable_align=False：预览不需要深度对齐到彩色，省掉对齐带来的开销与视野裁剪；
    # frame_timeout_ms=1000 是单帧等待上限，超时返回空帧后循环继续，不会卡死。
    driver = Gemini2Driver(
        Gemini2Config(
            color_width=640,
            color_height=480,
            color_fps=30,
            enable_depth=True,
            depth_width=0,
            depth_height=0,
            depth_fps=30,
            frame_timeout_ms=1000,
            enable_align=False,
        )
    )

    driver.open()
    # 丢掉前 15 帧：自动曝光/白平衡收敛前的画面偏暗，统计值没有参考意义。
    driver.warmup(15)

    cv2.namedWindow("Gemini2 Color", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Gemini2 Depth", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # allow_partial=True：彩色或深度只到一路时也返回，便于定位是哪一路缺数据。
            color_bgr, depth_mm = driver.get_frame(allow_partial=True)

            if color_bgr is not None:
                overlay = color_bgr.copy()
                stats = (
                    f"color min={int(color_bgr.min())} "
                    f"max={int(color_bgr.max())} mean={float(color_bgr.mean()):.1f}"
                )
                # putText 参数依次为：图像、文本、文本左下角像素坐标、字体、字号缩放、
                # BGR 颜色、线宽、抗锯齿方式。
                cv2.putText(
                    overlay,
                    stats,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Gemini2 Color", overlay)

            depth_vis = _depth_to_u8(depth_mm)
            if depth_vis is not None:
                # 统计仍按原始毫米值计算，因此这里打印的是真实深度范围而非伪彩灰度。
                stats = (
                    f"depth min={int(depth_mm.min())} "
                    f"max={int(depth_mm.max())} mean={float(depth_mm.mean()):.1f}"
                )
                cv2.putText(
                    depth_vis,
                    stats,
                    (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("Gemini2 Depth", depth_vis)

            # waitKey 只有在高亮窗口时才能收到按键；返回值高位可能带修饰键，取低 8 位比较。
            # 27 是 ESC 的 ASCII 码。
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
    finally:
        driver.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
