"""Gemini 2 深度相机的设备访问层（相机 SDK 封装）。

本模块位于视觉包的最底层：向上只暴露「打开 / 预热 / 取帧 / 关闭」这一组
与具体相机型号无关的能力（打开、预热、取帧和关闭），
不接触 ROS 话题、服务或运动规划，因此可在 ROS 之外的调试脚本里单独使用。

职责边界：

- 负责与相机 SDK 交互，按配置挑选彩色/深度流的档位、开启取流、取帧并做格式转换；
- 负责把 SDK 的图像数据整理成 OpenCV/NumPy 约定：彩色为 ``uint8`` 的 BGR 三通道，
  深度为 ``uint16`` 的**毫米**单通道（0 表示无效像素，沿用相机 SDK 的无效值约定）；
- 负责把标定信息（内参、畸变、彩色↔深度外参、深度比例尺）与帧元数据整理成
  纯 Python 字典供上层转换成消息；
- **不负责**任何安全门控。取帧失败只返回 ``None`` 并记录一句英文调试短语，
  是否继续运行、是否闭锁失败由上层视觉节点的空帧计数与失效闭锁逻辑决定。

关键约定与注意点：

- 相机 SDK（pyorbbecsdk）采用**延迟导入**：只在 ``open`` / ``get_frame`` 内部导入。
  这样在没有连接相机、甚至没有安装该 SDK 的机器上，仍然可以导入本模块并复用
  其中的纯计算辅助方法（例如把流档位转换成字典、计算相机内参载荷）。
- 时间戳单位：SDK 返回微秒，对外接口 ``get_frame_timestamp_ns`` 返回纳秒。
  硬件时间戳（``timestamp_us``）与主机时钟无关；系统时间戳
  （``system_timestamp_us``）与主机墙钟同源，优先用于给真实相机帧打 ROS 时间戳
  （仿真时间下由上层改为使用收帧时刻）。
- 调试信息字段 ``last_debug_message`` 刻意保持英文短语（如 ``frame_ok``、
  ``exception:RuntimeError:...``），未经翻译，便于与运行日志和现场排障对照。
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class Gemini2Config:
    """Gemini 2 驱动的取流配置。

    这些字段与视觉节点的 ``camera.*`` ROS 参数一一对应，由上层在构造时填入；
    本类只承载数值，不做校验，因此非法组合（例如指定了设备不支持的分辨率）
    会在 ``open`` 阶段以致命错误暴露，而不会在此处被悄悄纠正。

    字段含义（长度单位为像素，时间单位为毫秒）：

    - color_width / color_height：彩色流分辨率。任一值 <= 0 表示「不指定」，
      由 SDK 返回默认档位；显式指定而设备不支持时会直接报错（fail-closed）。
    - color_fps：彩色流帧率（fps），需是设备该分辨率下支持的档位之一。
    - enable_depth：是否开启深度流。为 False 时只输出彩色，深度相关载荷全部为空。
    - depth_width / depth_height：深度流分辨率；<= 0 表示交给 SDK 选默认档位。
      开启对齐时应选用与彩色匹配、且经现场验证可用的组合。
    - depth_fps：深度流帧率（fps）。
    - frame_timeout_ms：单次 ``wait_for_frames`` 的等待上限，超时按「本帧无数据」处理，
      避免整个视觉循环被相机阻塞。
    - enable_align：是否请求深度到彩色的对齐（D2C）。开启后深度像素与彩色像素一一对应，
      深度流的内参会改为彩色内参，可直接用同一套像素坐标取三维点。
    - enable_frame_sync：是否请求 SDK 按设备硬件时间戳配对彩色与深度帧。
    """

    color_width: int
    color_height: int
    color_fps: int
    enable_depth: bool
    depth_width: int
    depth_height: int
    depth_fps: int
    frame_timeout_ms: int
    enable_align: bool
    enable_frame_sync: bool = True
    color_format: str = "MJPG"


class Gemini2Driver:
    """Gemini 2 相机驱动：一次 ``open``/``close`` 对应一次设备取流会话。

    典型生命周期：构造 → ``open`` → ``warmup`` → 反复 ``get_frame`` → ``close``。
    本类不是线程安全的：内部缓存了最近一帧、最新元数据与调试信息，应始终由
    同一个执行线程（通常是节点定时器回调）串行调用。

    内部缓存的作用：

    - ``_last_color_bgr`` / ``_last_depth_mm``：最近一次成功解出的图像。
      默认取帧模式下用于「部分失败」补齐，避免因偶发单流丢帧让整帧作废；
      调用方若要求严格同帧，应使用 ``allow_partial=True`` 并自行丢弃不完整帧。
    - ``_camera_info_by_stream``：按流名缓存内参，键为 ``color`` / ``depth`` /
      ``depth_native``（原生未对齐深度）。
    - ``_calibration_info`` / ``_device_info`` / ``_stream_profiles`` /
      ``_frame_metadata``：分别在 ``open``、每次取帧时刷新。

    所有 ``get_*`` 接口都返回深拷贝，调用方可安全修改返回值而不污染驱动状态。
    """

    def __init__(self, config: Gemini2Config) -> None:
        self._config = config
        self._pipeline = None
        self._format_convert_filter = None
        self._last_color_bgr: Optional[np.ndarray] = None
        self._last_depth_mm: Optional[np.ndarray] = None
        self._camera_info_by_stream: dict[str, dict] = {}
        self._device_info: dict = {}
        self._calibration_info: dict = {}
        self._stream_profiles: dict[str, dict] = {}
        self._frame_metadata: dict[str, dict] = {}
        self._depth_scale_mm: Optional[float] = None
        # 最近一次取帧结果/失败原因的英文短语；未开流前为 driver_not_opened。
        self._last_debug_message = "driver_not_opened"

    @staticmethod
    def _ensure_contiguous(array: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """把数组整理成 C 连续内存布局。

        相机 SDK 的缓冲往往是切片或视图，转成消息、送进 OpenCV 或做跨库传参时
        需要连续内存；``None`` 原样返回，方便与「本帧无数据」的语义串起来。
        """
        if array is None:
            return None
        return np.ascontiguousarray(array)

    @staticmethod
    def _select_video_profile(
        profiles,
        *,
        stream_name: str,
        width: int,
        height: int,
        formats: tuple,
        fps: int,
    ):
        """在设备支持的档位里挑选一个视频流档位，找不到就报错。

        参数：

        - ``profiles``：SDK 的流档位列表对象；
        - ``stream_name``：仅用于拼装错误信息（``color`` / ``depth``）；
        - ``width`` / ``height``：期望分辨率；任一 <= 0 视为「不指定」，
          直接返回 SDK 默认档位（这是唯一允许走默认档的情形）；
        - ``formats``：可接受的像素格式，**按优先级从高到低**逐个尝试；
        - ``fps``：期望帧率。

        失败语义：显式指定了分辨率却没有任何格式能匹配时抛 ``RuntimeError``，
        而不是退回默认档位。静默降级会让上层用错误的分辨率去解释内参与像素坐标，
        因此在配置阶段就 fail-closed。异常信息里带上 ``stream_name`` 和各格式的
        失败原因，便于现场快速判断是分辨率、格式还是帧率不被支持。
        """
        if width <= 0 or height <= 0:
            return profiles.get_default_video_stream_profile()

        errors = []
        for fmt in formats:
            try:
                return profiles.get_video_stream_profile(width, height, fmt, fps)
            except Exception as exc:
                errors.append(f"{fmt}: {type(exc).__name__}: {exc}")
        requested = f"{width}x{height}@{fps} formats={list(map(str, formats))}"
        raise RuntimeError(
            f"requested {stream_name} profile unavailable: {requested}; "
            + "; ".join(errors)
        )

    @staticmethod
    def _camera_info_from_profile(profile) -> dict:
        """从流档位中取出相机内参与畸变系数，整理成消息友好的字典。

        单位约定：``fx`` / ``fy`` / ``cx`` / ``cy`` 为像素；``width`` / ``height``
        为该校准适用的图像尺寸。畸变模型固定声明为 ``rational_polynomial``，
        因此 ``d`` 的顺序必须是 OpenCV 的 8 参数有理模型顺序：
        ``[k1, k2, p1, p2, k3, k4, k5, k6]``——径向 3 项、切向 2 项、分子高阶 3 项。
        这个顺序直接决定去畸变与三维重建结果，不能按 SDK 属性的书写顺序调整。
        """
        intrinsic = profile.get_intrinsic()
        distortion = profile.get_distortion()
        return {
            "width": int(intrinsic.width),
            "height": int(intrinsic.height),
            "fx": float(intrinsic.fx),
            "fy": float(intrinsic.fy),
            "cx": float(intrinsic.cx),
            "cy": float(intrinsic.cy),
            "distortion_model": "rational_polynomial",
            "d": [
                float(distortion.k1),
                float(distortion.k2),
                float(distortion.p1),
                float(distortion.p2),
                float(distortion.k3),
                float(distortion.k4),
                float(distortion.k5),
                float(distortion.k6),
            ],
        }

    @staticmethod
    def _profile_payload(profile) -> dict:
        """把单个视频流档位压成 ``{width, height, fps, format}`` 字典。

        供日志/诊断展示「设备到底支持哪些档位」，字段取值原样来自 SDK（格式为字符串）。
        """
        return {
            "width": int(profile.get_width()),
            "height": int(profile.get_height()),
            "fps": int(profile.get_fps()),
            "format": str(profile.get_format()),
        }

    @classmethod
    def _profile_list_payload(cls, profiles) -> list[dict]:
        """列出全部**视频流**档位；非视频流（如 IMU 等）档位直接跳过。"""
        result = []
        for index in range(profiles.get_count()):
            profile = profiles.get_stream_profile_by_index(index)
            if not profile.is_video_stream_profile():
                continue
            result.append(cls._profile_payload(profile.as_video_stream_profile()))
        return result

    @staticmethod
    def _extrinsic_payload(extrinsic) -> dict:
        """把两个流之间的外参整理成 ``{rotation, translation_mm}``。

        ``rotation`` 是行主序 3x3 旋转矩阵，``translation_mm`` 是长度为 3 的平移向量，
        **单位为毫米**（SDK 原始单位，未换算成米）。两者共同定义「源坐标系下的点
        变换到目标坐标系」的刚体变换，由调用方按名约定解读方向。
        """
        rotation = np.asarray(extrinsic.rot, dtype=float).reshape(3, 3)
        translation = np.asarray(extrinsic.transform, dtype=float).reshape(3)
        return {
            "rotation": rotation.tolist(),
            "translation_mm": translation.tolist(),
        }

    @staticmethod
    def _optional_call(obj, method_name: str, default=None):
        """调用可能不存在的方法，取不到值时返回 ``default``。

        不同 SDK 版本、不同固件/设备暴露的查询接口并不一致（设备信息、深度比例尺等）。
        这里统一做成「尽力而为」：属性缺失或调用抛异常都不影响取流主流程，
        只让对应字段退化为默认值。
        """
        method = getattr(obj, method_name, None)
        if not callable(method):
            return default
        try:
            return method()
        except Exception:
            return default

    def _color_formats(self, ob_format):
        requested = str(self._config.color_format).strip().upper()
        formats = {"MJPG": ob_format.MJPG, "RGB": ob_format.RGB}
        if requested not in formats:
            raise ValueError(
                f"unsupported camera.color_format: {requested}; expected MJPG or RGB"
            )
        return (formats[requested],)

    def open(self) -> None:
        """打开设备并启动取流，同时固化本次会话的标定与设备信息。

        流程与要点：

        1. 延迟导入相机 SDK，缺失时异常上抛，由上层节点决定报错方式；
        2. 彩色流优先尝试 MJPG（同等分辨率下带宽更省），失败再退到 RGB；
        3. 深度流仅在 ``enable_depth`` 为真时配置，格式限定为 Y16
           （16 位原始深度，单位由帧上的 depth scale 决定）；
        4. ``enable_align`` 为真时请求**硬件对齐**（HW_MODE，Depth-to-Color），
           由设备侧完成深度到彩色坐标系的映射，比软件对齐省 CPU。对齐后深度帧的
           像素坐标与彩色一致，故其内参改用彩色内参，另把原生深度内参单独留档在
           ``depth_native`` 键下，供需要原生几何的场景使用；
        5. 设备信息逐项用 ``_optional_call`` 采集，接口不存在时留空串/0，
           不因个别字段缺失而中断打开流程。

        副作用：启动取流后设备开始出图；若此处抛异常，调用方必须保证不留下半开状态
        （本类的 ``close`` 允许在 ``open`` 失败后调用，会静默忽略停止时的异常）。
        """
        from pyorbbecsdk import Config, FormatConvertFilter, OBAlignMode, OBFormat, OBSensorType, Pipeline

        self._pipeline = Pipeline()
        self._format_convert_filter = FormatConvertFilter()
        config = Config()

        color_profiles = self._pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = self._select_video_profile(
            color_profiles,
            stream_name="color",
            width=self._config.color_width,
            height=self._config.color_height,
            formats=self._color_formats(OBFormat),
            fps=self._config.color_fps,
        )
        config.enable_stream(color_profile)
        self._stream_profiles["color"] = {
            "selected": self._profile_payload(color_profile),
            "supported": self._profile_list_payload(color_profiles),
        }

        depth_profile = None
        if self._config.enable_depth:
            depth_profiles = self._pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            depth_profile = self._select_video_profile(
                depth_profiles,
                stream_name="depth",
                width=self._config.depth_width,
                height=self._config.depth_height,
                formats=(OBFormat.Y16,),
                fps=self._config.depth_fps,
            )
            config.enable_stream(depth_profile)
            self._stream_profiles["depth"] = {
                "selected": self._profile_payload(depth_profile),
                "supported": self._profile_list_payload(depth_profiles),
            }

        # 请求硬件对齐：设备侧把深度图重投影到彩色相机坐标系。
        if self._config.enable_align:
            config.set_align_mode(OBAlignMode.HW_MODE)
        # 老版本 SDK 没有该接口，故先用 getattr 探测再按需调用。
        set_depth_scale_require = getattr(config, "set_depth_scale_require", None)
        if self._config.enable_depth and callable(set_depth_scale_require):
            # 要求深度帧必须给出 depth scale（原始值→毫米的换算系数），
            # 否则取到的 16 位整数无法换算成物理距离。
            set_depth_scale_require(True)

        # Gemini 2 支持 SDK 侧硬件时间戳配对；失败时保持显式错误，避免误以为已同步。
        if self._config.enable_frame_sync and self._config.enable_depth:
            enable_frame_sync = getattr(self._pipeline, "enable_frame_sync", None)
            if not callable(enable_frame_sync):
                raise RuntimeError("camera SDK does not provide enable_frame_sync")
            enable_frame_sync()

        self._pipeline.start(config)
        device = self._pipeline.get_device()
        device_info = device.get_device_info()
        self._device_info = {
            "name": self._optional_call(device_info, "get_name", ""),
            "serial_number": self._optional_call(device_info, "get_serial_number", ""),
            "firmware_version": self._optional_call(device_info, "get_firmware_version", ""),
            "hardware_version": self._optional_call(device_info, "get_hardware_version", ""),
            "connection_type": self._optional_call(device_info, "get_connection_type", ""),
            "uid": self._optional_call(device_info, "get_uid", ""),
            # vid/pid 为 USB 厂商号/产品号，取不到时退化为 0。
            "vid": self._optional_call(device_info, "get_vid", 0),
            "pid": self._optional_call(device_info, "get_pid", 0),
            "supported_min_sdk_version": self._optional_call(
                device_info, "get_supported_min_sdk_version", ""
            ),
        }
        self._camera_info_by_stream["color"] = self._camera_info_from_profile(color_profile)
        if depth_profile is not None:
            native_depth_info = self._camera_info_from_profile(depth_profile)
            # depth_native 始终保存原生深度内参，便于诊断对齐是否真的生效。
            self._camera_info_by_stream["depth_native"] = native_depth_info
            # 对外暴露的 depth 内参：对齐后像素坐标已属于彩色相机，必须用彩色内参；
            # 未对齐时使用原生深度内参。
            self._camera_info_by_stream["depth"] = deepcopy(
                self._camera_info_by_stream["color"]
                if self._config.enable_align
                else native_depth_info
            )
            self._calibration_info = {
                "color_to_depth": self._extrinsic_payload(
                    color_profile.get_extrinsic_to(depth_profile)
                ),
                "depth_to_color": self._extrinsic_payload(
                    depth_profile.get_extrinsic_to(color_profile)
                ),
                # 显式告知上层：depth 流的内参含义取决于是否开启对齐。
                "aligned_depth_uses_color_coordinates": bool(self._config.enable_align),
            }

    def warmup(self, frames: int) -> bool:
        """丢弃前若干帧，等待自动曝光/自动白平衡收敛。

        参数 ``frames`` 为最多尝试读取的帧数（负值按 0 处理）。返回 True 表示
        彩色与深度都至少成功取到过一次；未开启深度时深度条件直接视为已满足。
        过程使用 ``allow_partial=True``，单帧只出彩色或只出深度也算部分成功，
        以便在只有单个流可用时也能尽快结束预热；未达标只返回 False 而不抛异常，
        由调用方决定是否带警告继续运行。
        """
        got_color = False
        got_depth = not self._config.enable_depth
        for _ in range(max(frames, 0)):
            color_bgr, depth_mm = self.get_frame(allow_partial=True)
            got_color = got_color or color_bgr is not None
            got_depth = got_depth or depth_mm is not None
            if got_color and got_depth:
                return True
        return got_color and got_depth

    def get_frame(self, allow_partial: bool = False) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """取一帧，返回 ``(彩色 BGR, 深度毫米)``，任一路不可用时该位置为 ``None``。

        彩色输出为 ``uint8``、形状 ``(height, width, 3)``、BGR 通道序（OpenCV 约定）：
        MJPG 帧经格式转换滤镜解成 BGR888；RGB 帧用 ``cvtColor`` 换序；其余格式按
        原始字节直接重排。

        深度输出为 ``uint16``、形状 ``(height, width)``，**单位毫米**，0 表示无效像素。
        换算公式为 ``round(原始值 * depth_scale)``：``depth_scale`` 取自帧对象
        （``get_depth_scale``，毫米/单位），取不到时按 1.0 处理；结果再截断到
        ``uint16`` 上限，防止超量程值回绕成近距噪声。

        ``allow_partial`` 控制**同帧性**与**缓存回退**：

        - True：直接返回本次读到的结果，允许只有一路，缺失的一路为 ``None``，
          不做缓存补齐（离线调试/预热用，避免拿到时间上不同步的旧帧）；
        - False（默认）：某一路本次失败时用最近一次成功的缓存补齐，尽力保证调用方
          拿到成对的彩色与深度；若两路都从未成功，则返回 ``(None, None)``。

        时间戳与帧元数据按流记录在 ``_frame_metadata`` 中（含 depth_scale）。
        任何异常（含超时后 SDK 抛错）都被捕获：写一条英文调试短语并返回两个 ``None``，
        取流失败不向上抛，避免打断上层视觉循环。
        """
        if self._pipeline is None:
            self._last_debug_message = "pipeline_none"
            return None, None

        try:
            from pyorbbecsdk import OBFormat

            frames = self._pipeline.wait_for_frames(self._config.frame_timeout_ms)
            if frames is None:
                self._last_debug_message = "wait_for_frames_none"
                return None, None

            color_bgr = None
            color_frame = frames.get_color_frame()
            if color_frame is not None:
                width = color_frame.get_width()
                height = color_frame.get_height()
                fmt = color_frame.get_format()
                if fmt == OBFormat.MJPG and self._format_convert_filter is not None:
                    from pyorbbecsdk import OBConvertFormat

                    self._format_convert_filter.set_format_convert_format(
                        OBConvertFormat.MJPG_TO_BGR888
                    )
                    converted = self._format_convert_filter.process(color_frame)
                    if converted is not None:
                        converted_data = np.asanyarray(converted.get_data(), dtype=np.uint8)
                        # 期望转换输出恰为 height*width*3 字节（BGR888）；np.resize 在
                        # 元素数不匹配时会循环重复数据，因此该前提不成立时图像会失真。
                        color_bgr = np.resize(converted_data, (height, width, 3))
                    else:
                        self._last_debug_message = "mjpg_convert_failed"
                else:
                    raw = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
                    if fmt == OBFormat.RGB:
                        # 相机输出 RGB，OpenCV 流程统一用 BGR，这里做一次通道换序。
                        rgb = np.ascontiguousarray(np.resize(raw, (height, width, 3)))
                        color_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    else:
                        # 其余格式（如设备已直接给出 BGR）按 3 通道原样重排。
                        color_bgr = np.resize(raw, (height, width, 3))
                color_bgr = self._ensure_contiguous(color_bgr)
                self._last_color_bgr = color_bgr
                self._frame_metadata["color"] = self._frame_metadata_payload(color_frame)
            else:
                self._last_debug_message = "color_frame_none"

            depth_mm = None
            depth_frame = frames.get_depth_frame()
            if depth_frame is not None:
                width = depth_frame.get_width()
                height = depth_frame.get_height()
                # 深度缓冲是小端 uint16 的连续字节流，按行优先重排成二维数组。
                depth_bytes = bytes(depth_frame.get_data())
                depth_raw = np.frombuffer(depth_bytes, dtype=np.uint16).reshape(height, width)
                # depth_scale 单位为「毫米/原始单位」；缺失时按 1.0 处理（即原始值已为毫米）。
                depth_scale_mm = float(self._optional_call(depth_frame, "get_depth_scale", 1.0))
                self._depth_scale_mm = depth_scale_mm
                # 先升到 float32 做乘法与四舍五入，再截断回 uint16：直接整数相乘会溢出回绕。
                depth_mm = np.rint(depth_raw.astype(np.float32) * depth_scale_mm)
                depth_mm = np.clip(depth_mm, 0, np.iinfo(np.uint16).max).astype(np.uint16)
                depth_mm = self._ensure_contiguous(depth_mm)
                self._last_depth_mm = depth_mm
                self._frame_metadata["depth"] = self._frame_metadata_payload(
                    depth_frame,
                    depth_scale_mm=depth_scale_mm,
                )
            else:
                self._last_debug_message = "depth_frame_none"

            if allow_partial:
                # 严格模式：不补齐，如实反映本次读到的内容。
                if color_bgr is not None and depth_mm is not None:
                    self._last_debug_message = (
                        f"partial_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
                        f"depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                    )
                elif color_bgr is not None:
                    self._last_debug_message = (
                        f"partial_color_only color={color_bgr.shape[1]}x{color_bgr.shape[0]}"
                    )
                elif depth_mm is not None:
                    self._last_debug_message = (
                        f"partial_depth_only depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                    )
                return color_bgr, depth_mm

            # 部分失败时用最近一次成功帧补齐，尽量给出成对数据。
            if color_bgr is None:
                color_bgr = self._last_color_bgr
            if depth_mm is None:
                depth_mm = self._last_depth_mm

            if color_bgr is not None and depth_mm is not None:
                self._last_debug_message = (
                    f"frame_ok color={color_bgr.shape[1]}x{color_bgr.shape[0]} "
                    f"depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                )
            elif color_bgr is not None:
                self._last_debug_message = (
                    f"frame_color_only color={color_bgr.shape[1]}x{color_bgr.shape[0]}"
                )
            elif depth_mm is not None:
                self._last_debug_message = (
                    f"frame_depth_only depth={depth_mm.shape[1]}x{depth_mm.shape[0]}"
                )
            else:
                self._last_debug_message = "frame_none_after_fallback"

            return color_bgr, depth_mm
        except Exception as exc:
            # 只记录异常类型与消息，不打断上层循环；同时丢弃本帧数据。
            self._last_debug_message = f"exception:{type(exc).__name__}:{exc}"
            return None, None

    def close(self) -> None:
        """停止取流并释放句柄；可重复调用。

        停止过程中的 SDK 异常被静默忽略（设备已拔出、尚未真正启动等），
        保证关闭路径永远不抛异常、不会阻塞上层的退出流程。
        """
        if self._pipeline is None:
            return
        try:
            self._pipeline.stop()
        except Exception:
            pass
        self._pipeline = None
        self._format_convert_filter = None

    @classmethod
    def _frame_metadata_payload(cls, frame, *, depth_scale_mm=None) -> dict:
        """整理单帧元数据。

        字段含义：``width`` / ``height`` 为像素；``format`` 为 SDK 的格式名；
        ``timestamp_us`` 是设备时间戳（微秒，与主机时钟无关）；``system_timestamp_us``
        是与主机墙钟同源的系统时间戳（微秒，用于给真实相机帧打 ROS 时间戳）；
        ``frame_index`` 为设备侧帧序号（用于判断是否拿到重复帧）。
        ``depth_scale_mm`` 仅在深度帧上附带，单位毫米/原始单位——
        传感器量程切换后该系数会变化，所以必须逐帧记录而不能只在打开时取一次。
        """
        payload = {
            "width": int(cls._optional_call(frame, "get_width", 0)),
            "height": int(cls._optional_call(frame, "get_height", 0)),
            "format": str(cls._optional_call(frame, "get_format", "")),
            "timestamp_us": int(cls._optional_call(frame, "get_timestamp_us", 0)),
            "system_timestamp_us": int(
                cls._optional_call(frame, "get_system_timestamp_us", 0)
            ),
            "frame_index": int(cls._optional_call(frame, "get_index", 0)),
        }
        if depth_scale_mm is not None:
            payload["depth_scale_mm"] = float(depth_scale_mm)
        return payload

    def get_camera_info(self, stream: str = "depth") -> Optional[dict]:
        """按流名取相机内参（深拷贝），未知流名返回 ``None``。

        可用键：``color``、``depth``（对齐开启时即彩色坐标下的内参）、
        ``depth_native``（原生深度内参）。返回字典含 ``width`` / ``height`` /
        ``fx`` / ``fy`` / ``cx`` / ``cy`` / ``distortion_model`` / ``d``。
        """
        payload = self._camera_info_by_stream.get(stream)
        return deepcopy(payload) if payload is not None else None

    def get_device_info(self) -> dict:
        """取设备信息（型号、序列号、固件/硬件版本、连接方式、USB vid/pid 等）深拷贝。"""
        return deepcopy(self._device_info)

    def get_calibration_info(self) -> dict:
        """取标定信息：彩色↔深度外参、对齐标志，以及当前深度比例尺。

        ``depth_scale_mm`` 为最近一帧深度帧上的换算系数（毫米/原始单位），
        未取到深度帧时为 ``None``；外参平移向量单位是毫米。
        """
        payload = deepcopy(self._calibration_info)
        payload["depth_scale_mm"] = self._depth_scale_mm
        return payload

    def get_stream_profiles(self) -> dict:
        """取各流实际选中的档位与设备支持的全部视频档位（供诊断打印）。"""
        return deepcopy(self._stream_profiles)

    def get_frame_metadata(self) -> dict:
        """取各流最近一次成功取帧的元数据（按流名索引，见 ``_frame_metadata_payload``）。"""
        return deepcopy(self._frame_metadata)

    def get_frame_timestamp_ns(self, stream: str) -> Optional[int]:
        """把某流的系统时间戳从微秒换算成纳秒；未知或为 0 时返回 ``None``。

        使用系统时间戳而非设备时间戳，是因为它落在主机墙钟域，可直接作为 ROS 时间戳
        并与其它节点比较；仿真时间下由上层改为使用收帧时刻，故此处只负责换算。
        """
        timestamp_us = int(
            self._frame_metadata.get(stream, {}).get("system_timestamp_us", 0) or 0
        )
        return timestamp_us * 1000 if timestamp_us > 0 else None

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message
