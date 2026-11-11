"""D405 USB 采集：输出 BGR、毫米 uint16 深度、SDK 标定和帧元数据。

对外提供 open/warmup/get_frame/close 与只读诊断查询，供视觉节点和独立预览使用。
SDK 延迟导入；同一线程串行调用。只返回本次 frameset，超时或缺流均不复用旧帧。
配置、标定及 SDK 异常直接上抛，超时由视觉节点的既有空帧策略处理。
文件依次包含配置、会话管理、几何转换、采集及诊断接口，不接触 ROS 或运动控制。
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class D405Config:
    """流尺寸、帧率必须命中设备档位；深度单位始终取 SDK 的实际比例尺。"""

    color_width: int = 640
    color_height: int = 480
    color_fps: int = 30
    enable_depth: bool = True
    depth_width: int = 640
    depth_height: int = 480
    depth_fps: int = 30
    frame_timeout_ms: int = 1000
    enable_align: bool = True
    color_format: str = "BGR8"


class D405Driver:
    """拥有一次 pipeline 会话；关闭幂等，启动后的初始化失败会立即释放设备。"""

    def __init__(self, config: D405Config) -> None:
        self._config = config
        self._pipeline = None
        self._align = None
        self._camera_info_by_stream: dict[str, dict] = {}
        self._device_info: dict = {}
        self._calibration_info: dict = {}
        self._stream_profiles: dict = {}
        self._frame_metadata: dict = {}
        self._depth_scale_mm: float | None = None
        self._last_debug_message = "driver_not_opened"

    def open(self) -> None:
        """只选择 D405；显式档位不可用时由 SDK 报错，不降级到其他分辨率。"""
        import pyrealsense2 as rs

        if self._pipeline is not None:
            raise RuntimeError("D405 pipeline already opened")
        cfg = self._config
        if cfg.color_format not in {"BGR8", "RGB8"}:
            raise ValueError("camera.color_format must be BGR8 or RGB8")
        if any(value <= 0 for value in (
            cfg.color_width, cfg.color_height, cfg.color_fps, cfg.frame_timeout_ms,
        )):
            raise ValueError("D405 color dimensions, fps and timeout must be positive")
        if cfg.enable_depth and (
            cfg.depth_width <= 0 or cfg.depth_height <= 0 or cfg.depth_fps != cfg.color_fps
        ):
            raise ValueError("D405 RGB-D requires positive depth dimensions and equal fps")

        pipeline = rs.pipeline()
        config = rs.config()
        config.disable_all_streams()
        config.enable_stream(
            rs.stream.color, cfg.color_width, cfg.color_height,
            rs.format.bgr8 if cfg.color_format == "BGR8" else rs.format.rgb8,
            cfg.color_fps,
        )
        if cfg.enable_depth:
            config.enable_stream(
                rs.stream.depth, cfg.depth_width, cfg.depth_height, rs.format.z16, cfg.depth_fps,
            )
        device = config.resolve(rs.pipeline_wrapper(pipeline)).get_device()
        name = device.get_info(rs.camera_info.name)
        if name.split()[-1] != "D405":
            raise RuntimeError(f"camera.type=d405 requires D405; selected device: {name}")
        # global_time 与主机墙钟同源；仅有硬件时钟时上层使用接收时间。
        for sensor in device.query_sensors():
            if sensor.supports(rs.option.global_time_enabled):
                sensor.set_option(rs.option.global_time_enabled, 1.0)

        profile = pipeline.start(config)
        self._pipeline = pipeline
        try:
            self._device_info = {
                "name": name,
                "serial_number": device.get_info(rs.camera_info.serial_number),
                "firmware_version": device.get_info(rs.camera_info.firmware_version),
                "connection_type": device.get_info(rs.camera_info.usb_type_descriptor),
            }
            color = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self._camera_info_by_stream["color"] = self._camera_info_from_profile(color)
            self._stream_profiles = {"color": {"selected": self._profile_payload(color)}}
            self._calibration_info = {}
            if cfg.enable_depth:
                depth = profile.get_stream(rs.stream.depth).as_video_stream_profile()
                self._depth_scale_mm = self._scale_mm(device.first_depth_sensor().get_depth_scale())
                native = self._camera_info_from_profile(depth)
                self._camera_info_by_stream["depth_native"] = native
                self._camera_info_by_stream["depth"] = deepcopy(
                    self._camera_info_by_stream["color"] if cfg.enable_align else native
                )
                self._stream_profiles["depth"] = {"selected": self._profile_payload(depth)}
                self._calibration_info = {
                    "color_to_depth": self._extrinsic_payload(color.get_extrinsics_to(depth)),
                    "depth_to_color": self._extrinsic_payload(depth.get_extrinsics_to(color)),
                    "aligned_depth_uses_color_coordinates": cfg.enable_align,
                }
            self._align = rs.align(rs.stream.color) if cfg.enable_depth and cfg.enable_align else None
            self._last_debug_message = "pipeline_opened"
        except Exception:
            self.close()
            raise

    @staticmethod
    def _scale_mm(meters_per_unit: float) -> float:
        scale = float(meters_per_unit) * 1000.0
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(f"invalid D405 depth scale: {meters_per_unit}")
        return scale

    @staticmethod
    def _camera_info_from_profile(profile) -> dict:
        import pyrealsense2 as rs

        intrinsic = profile.get_intrinsics()
        # 与 RealSense ROS 的 CameraInfo 映射一致，保留 SDK 原模型供诊断。
        if intrinsic.model == rs.distortion.kannala_brandt4:
            model, coefficients = "equidistant", intrinsic.coeffs[:4]
        elif intrinsic.model in (
            rs.distortion.none, rs.distortion.brown_conrady,
            rs.distortion.modified_brown_conrady, rs.distortion.inverse_brown_conrady,
        ):
            model, coefficients = "plumb_bob", intrinsic.coeffs
        else:
            raise ValueError(f"unsupported D405 distortion model: {intrinsic.model}")
        return {
            "width": intrinsic.width, "height": intrinsic.height,
            "fx": intrinsic.fx, "fy": intrinsic.fy, "cx": intrinsic.ppx, "cy": intrinsic.ppy,
            "distortion_model": model, "d": list(coefficients),
            "sdk_distortion_model": str(intrinsic.model),
        }

    @staticmethod
    def _profile_payload(profile) -> dict:
        return {
            "width": profile.width(), "height": profile.height(),
            "fps": profile.fps(), "format": str(profile.format()),
        }

    @staticmethod
    def _extrinsic_payload(extrinsic) -> dict:
        # SDK 的旋转按列存储、平移为米；诊断接口沿用行矩阵与毫米。
        return {
            "rotation": np.asarray(extrinsic.rotation).reshape(3, 3, order="F").tolist(),
            "translation_mm": (np.asarray(extrinsic.translation) * 1000.0).tolist(),
        }

    @staticmethod
    def _metadata(frame) -> dict:
        import pyrealsense2 as rs

        timestamp = float(frame.get_timestamp())
        domain = frame.get_frame_timestamp_domain()
        return {
            "timestamp_ms": timestamp,
            "timestamp_domain": str(domain),
            "system_timestamp_ns": (
                round(timestamp * 1_000_000)
                if domain in (rs.timestamp_domain.system_time, rs.timestamp_domain.global_time)
                else None
            ),
            "frame_index": frame.get_frame_number(),
            "width": frame.get_width(), "height": frame.get_height(),
            "format": str(frame.get_profile().format()),
        }

    def get_frame(self, allow_partial: bool = False) -> tuple[np.ndarray | None, np.ndarray | None]:
        """读取同步 frameset；部分模式返回可用流，完整模式拒绝残帧，均无历史补帧。"""
        if self._pipeline is None:
            raise RuntimeError("D405 pipeline is not opened")
        self._frame_metadata = {}
        available, frames = self._pipeline.try_wait_for_frames(self._config.frame_timeout_ms)
        if not available:
            self._last_debug_message = "frame_timeout"
            return None, None
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if self._align is not None:
            # 对齐依赖两路；缺彩色时禁止把原生深度标成彩色坐标下的深度。
            if color and depth:
                frames = self._align.process(frames)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
            else:
                depth = None

        color_bgr = depth_mm = None
        if color:
            color_bgr = np.asanyarray(color.get_data())
            if self._config.color_format == "RGB8":
                color_bgr = color_bgr[..., ::-1]
            color_bgr = np.array(color_bgr, copy=True, order="C")
            self._frame_metadata["color"] = self._metadata(color)
        if depth and self._config.enable_depth:
            self._depth_scale_mm = self._scale_mm(depth.get_units())
            raw = np.asanyarray(depth.get_data())
            depth_mm = np.clip(
                np.rint(raw.astype(np.float64) * self._depth_scale_mm), 0, 65535,
            ).astype(np.uint16)
            self._frame_metadata["depth"] = {
                **self._metadata(depth), "depth_scale_mm": self._depth_scale_mm,
            }
        complete = color_bgr is not None and (not self._config.enable_depth or depth_mm is not None)
        self._last_debug_message = "frame_ok" if complete else "partial_frame"
        if not complete and not allow_partial:
            return None, None
        return color_bgr, depth_mm

    def warmup(self, frames: int) -> bool:
        """丢弃指定数量的启动帧，至少取得一次完整 RGB-D 才报告就绪。"""
        ready = False
        for _ in range(max(0, frames)):
            color, depth = self.get_frame()
            ready |= color is not None and (not self._config.enable_depth or depth is not None)
        return ready

    def close(self) -> None:
        """释放 pipeline；停止失败直接上抛，保留句柄供调用方再次清理。"""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._align = None
        self._camera_info_by_stream.clear()
        self._frame_metadata.clear()
        self._device_info.clear()
        self._calibration_info.clear()
        self._stream_profiles.clear()
        self._depth_scale_mm = None
        self._last_debug_message = "pipeline_closed"

    def get_camera_info(self, stream: str = "depth") -> dict | None:
        return deepcopy(self._camera_info_by_stream.get(stream))

    def get_device_info(self) -> dict:
        return deepcopy(self._device_info)

    def get_calibration_info(self) -> dict:
        return {**deepcopy(self._calibration_info), "depth_scale_mm": self._depth_scale_mm}

    def get_stream_profiles(self) -> dict:
        return deepcopy(self._stream_profiles)

    def get_frame_metadata(self) -> dict:
        return deepcopy(self._frame_metadata)

    def get_frame_timestamp_ns(self, stream: str) -> int | None:
        return self._frame_metadata.get(stream, {}).get("system_timestamp_ns")

    @property
    def last_debug_message(self) -> str:
        return self._last_debug_message
