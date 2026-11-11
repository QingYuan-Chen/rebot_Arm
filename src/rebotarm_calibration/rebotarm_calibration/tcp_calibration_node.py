"""TCP 标定节点：交互采集多组末端位姿，解算工具中心点（TCP）相对末端坐标系的偏移。

职责与位置
    本模块是标定包的可执行入口 ``rebotarm_tcp_calibration``（节点名同名），以交互方式运行：
    操作者按提示把机械臂摆到目标位姿，按回车触发一次采样；位姿记录由本节点完成，解算交给
    纯数学模块（见 tcp_calibration）。节点只读 TF 与相机，不发布运动指令、不使能硬件，现场
    动作由操作者通过既有受保护流程执行。

三种参考模式（参数 ``reference_mode``）
    ``pivot``  经典 pivot（多点）法：不需要已知参考点，只要求每次采样都把同一个物理尖点
               固定在空间同一处；由解算模块联合求解 TCP 偏移与基座系下的固定 pivot 位置。
    ``manual`` 参考点已知：操作者给出基座系下的参考位置（``tcp_reference_position``，米），
               每次采样按 ``R^T (p_ref - p_end)`` 反算该姿态下的 TCP 局部偏移再做统计。
    ``aruco``  参考点由视觉测得：先对 ArUco 标定板做 PnP，把板中心在基座系下的位置"冻结"
               为参考点，之后按 manual 的方式采样。

对外接口
    订阅（仅 ``aruco`` 模式）：``aruco.image_topic`` 彩色图与 ``aruco.camera_info_topic``
    相机内参，均使用传感器数据 QoS；
    TF：``base_frame -> end_link_frame``，``aruco`` 模式还需要
    ``base_frame -> aruco.camera_frame``。
    输出：stdout 的 JSON 分析结果，以及可选的 ``output_path`` 文件；``preflight_only``
    为真时只做参考点冻结与预检，不进入采样循环。
    结果 JSON 的 ``schema_version`` 固定为 1，其字段名与解算结果一样属于对外接口。

安全约束
    1. 采样以人工确认为节拍，本节点不规划、不执行任何运动，也不假定硬件已使能；
    2. 参考点门（有效帧数不足、逐轴标准差超限）失败一律抛异常终止，绝不带着不稳定的
       参考点继续采样；
    3. 解算结果仅作候选打印，不自动写入任何配置——部署必须由人工确认全部门通过后另行完成。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .aruco_pose import detect_aruco_pose
from .tcp_calibration import (
    analyze_tcp_pivot_samples,
    analyze_tcp_samples,
    estimate_sample_offset,
    format_tcp_offset_yaml,
    quaternion_to_rotation_matrix,
    _rotation_distance_deg,
)


def _tuple3(values, name: str) -> tuple[float, float, float]:
    """把长度必须为 3 的序列转成 float 三元组；长度不符时抛 ValueError。

    ``name`` 只用于异常消息，便于定位是哪个参数传错（例如 ``tcp_reference_position``）。
    """
    items = list(values)
    if len(items) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = tuple(float(v) for v in items)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must be finite")
    return result


def _transform_point(transform_stamped, point) -> tuple[float, float, float]:
    """用一条 TF 把 ``point`` 从其源坐标系变换到目标坐标系。

    变换语义为 ``p_target = t + R · p_source``：``transform_stamped`` 是 target<-source 的
    变换（例如 base<-camera），``point`` 为源坐标系下的三维点（米）。四元数按 (x, y, z, w)
    取出，旋转矩阵按标准刚体约定构造，因此本函数不做缩放或镜像。
    """
    transform = transform_stamped.transform
    rotation = transform.rotation
    matrix = quaternion_to_rotation_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
    translation = np.array(
        (transform.translation.x, transform.translation.y, transform.translation.z),
        dtype=np.float64,
    )
    result = translation + matrix @ np.asarray(point, dtype=np.float64)
    return tuple(float(value) for value in result)


def estimate_offset_from_transform(
    transform_stamped,
    *,
    tcp_reference_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    """从一条 base->end_link 的 TF 直接反算 TCP 局部偏移（``estimate_sample_offset`` 的薄封装）。

    ``tcp_reference_position`` 为基座系下的参考点（米）。末端坐标系下的偏移等于
    ``R^T (p_ref - p_end)``，其中 R 为末端姿态；这样得到的偏移与末端姿态无关，可在多个
    姿态间做统计。测试与离线脚本直接调用本函数，不依赖节点对象。
    """
    transform = transform_stamped.transform
    translation = transform.translation
    rotation = transform.rotation
    return estimate_sample_offset(
        end_link_position=(translation.x, translation.y, translation.z),
        end_link_orientation_xyzw=(rotation.x, rotation.y, rotation.z, rotation.w),
        tcp_reference_position=tcp_reference_position,
    )


class AutoArucoReferenceProvider:
    """兼容用的 ArUco 参考点提供器：抓一帧图、识别标定板、换算到基座坐标系。

    与节点内的 ``aruco`` 模式不同，它不冻结多帧、也不做帧数与标准差门（那部分逻辑在
    :meth:`TcpCalibrationNode.prepare_reference` 中），因此只保留给旧调用方与聚焦测试
    使用，主流程不引用它。依赖以构造参数注入（驱动、TF 缓冲、检测器），便于测试替身。
    """

    def __init__(
        self,
        *,
        driver,
        tf_buffer,
        base_frame: str,
        camera_frame: str,
        lookup_timeout_sec: float,
        detector,
    ) -> None:
        self.driver = driver
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.lookup_timeout_sec = lookup_timeout_sec
        self.detector = detector

    def reference_position(self) -> tuple[float, float, float]:
        """返回当前帧标定板中心在基座坐标系下的位置（米）。

        取图失败时抛 RuntimeError；图中找不到目标标定板时由检测器抛异常，本方法不吞异常。
        TF 查询使用构造时给定的超时，取缓冲区里最新的可用变换。
        """
        color_bgr, _ = self.driver.get_frame()
        # 抓图失败必须立刻失败：拿上一帧旧图当参考点会静默引入偏差
        if color_bgr is None:
            raise RuntimeError("failed to capture color image for ArUco reference")
        camera_point = self.detector(color_bgr)
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )
        return _transform_point(tf_msg, camera_point)


class TcpCalibrationNode(Node):
    """TCP 标定交互节点：声明参数、准备参考点、逐次采样并汇总结果。

    生命周期由 :func:`main` 驱动（构造 -> 准备参考点 -> 循环采样 -> 汇总），自身不注册
    定时器；只有 ``aruco`` 模式会在准备阶段用 ``rclpy.spin_once`` 主动抽帧。节点不发布
    任何消息、不调用任何服务，图像回调仅缓存最新一帧。
    """

    def __init__(self) -> None:
        """读取并校验参数后建立 TF 缓冲与（可选的）相机订阅。

        参数在构造时一次性读取并做下限钳制：``sample_count`` 至少 1、
        ``lookup_timeout_sec`` 至少 0.05 s、``aruco.reference_frames`` 至少 1、
        ``aruco.maximum_frames`` 不得小于 ``reference_frames``、
        ``aruco.maximum_reference_std_m`` 不得为负。``reference_mode`` 只接受
        ``pivot`` / ``manual`` / ``aruco``，其余取值直接抛 ValueError（fail closed）。
        """
        super().__init__("rebotarm_tcp_calibration")
        # ---- 模式与坐标系参数 ----
        # 参考点来源：pivot / manual / aruco，见模块 docstring；默认 pivot 不需要外部参考点
        self.declare_parameter("reference_mode", "pivot")
        # 基座坐标系名：参考点与解算结果都表达在该坐标系下
        self.declare_parameter("base_frame", "base_link")
        # 末端坐标系名：TCP 偏移就是相对该坐标系的局部平移（米）
        self.declare_parameter("end_link_frame", "end_link")
        # 仅 manual/aruco 使用：基座系下的参考点 (x, y, z)，单位米
        self.declare_parameter("tcp_reference_position", [0.0, 0.0, 0.0])
        # 采样次数：pivot 法建议 >= 5 且各次姿态差异明显，否则解算病态
        self.declare_parameter("sample_count", 5)
        # 仅冻结参考点后立即退出，用于现场预检（不采样、不落盘样本）
        self.declare_parameter("preflight_only", False)
        # TF 查询超时（秒）；链路缺失时决定多久放弃，下限钳制到 0.05 s
        self.declare_parameter("lookup_timeout_sec", 0.5)
        # 结果 JSON 落盘路径；留空表示只在 stdout 打印
        self.declare_parameter("output_path", "")
        # ---- ArUco 参考模式参数 ----
        # 彩色图与相机内参话题（仅 aruco 模式订阅），使用传感器数据 QoS
        self.declare_parameter("aruco.image_topic", "/camera/color/image_raw")
        self.declare_parameter("aruco.camera_info_topic", "/camera/color/camera_info")
        # 相机光学坐标系名：必须存在 base_frame -> 该坐标系 的 TF，且与图像同源
        self.declare_parameter("aruco.camera_frame", "camera_depth_frame")
        # ArUco 字典名：必须与标定纸实际打印的字典一致，否则检测不到
        self.declare_parameter("aruco.dictionary", "DICT_4X4_50")
        # 目标标定板 id：同一张纸上其它 id 会被忽略
        self.declare_parameter("aruco.marker_id", 0)
        # 标定板黑块边长（米），用于 PnP 求解尺度；量错会按比例放大测距误差
        self.declare_parameter("aruco.marker_length_m", 0.10)
        # 冻结参考点所需的最少有效帧数
        self.declare_parameter("aruco.reference_frames", 30)
        # 抽帧尝试上限，避免相机没有新帧时无限等待
        self.declare_parameter("aruco.maximum_frames", 45)
        # 冻结参考点的逐轴标准差上限（米），默认 0.002（2 mm）；
        # 超限说明标定板或机械臂在采集期间晃动，必须重新冻结
        self.declare_parameter("aruco.maximum_reference_std_m", 0.002)

        for name, default in {
            "capture_timeout_sec": 15.0, "maximum_age_sec": 0.5,
            "stability_window_sec": 0.4, "stability_translation_m": 0.001,
            "stability_rotation_deg": 0.5, "aruco.maximum_reprojection_rmse_px": 1.0,
            "aruco.minimum_area_px2": 400.0, "aruco.maximum_distance_m": 2.0,
            "aruco.maximum_info_skew_sec": 0.1,
        }.items():
            self.declare_parameter(name, default)
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, name.replace(".", "_"), value)

        self.reference_mode = str(self.get_parameter("reference_mode").value).strip().lower()
        # 大小写与首尾空白都会被规范化；非法值立即失败，不进入后续流程
        if self.reference_mode not in {"pivot", "manual", "aruco"}:
            raise ValueError("reference_mode must be 'pivot', 'manual', or 'aruco'")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.end_link_frame = str(self.get_parameter("end_link_frame").value)
        self.camera_frame = str(self.get_parameter("aruco.camera_frame").value)
        self.tcp_reference_position = _tuple3(
            self.get_parameter("tcp_reference_position").value,
            "tcp_reference_position",
        )
        # 至少保留 1 次采样：配成 0 会让采样循环被静默跳过，最后报"样本不足"
        self.sample_count = max(1, int(self.get_parameter("sample_count").value))
        self.preflight_only = bool(self.get_parameter("preflight_only").value)
        self.lookup_timeout_sec = max(0.05, float(self.get_parameter("lookup_timeout_sec").value))
        self.output_path = str(self.get_parameter("output_path").value).strip()
        self.reference_frames = max(1, int(self.get_parameter("aruco.reference_frames").value))
        # maximum_frames 必须 >= reference_frames，否则参考点永远凑不够帧数
        self.maximum_frames = max(
            self.reference_frames,
            int(self.get_parameter("aruco.maximum_frames").value),
        )
        self.maximum_reference_std_m = max(
            0.0, float(self.get_parameter("aruco.maximum_reference_std_m").value)
        )

        for name in ("sample_count", "aruco.reference_frames", "aruco.maximum_frames"):
            value = self.get_parameter(name).value
            if int(value) != value or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("lookup_timeout_sec", "aruco.maximum_reference_std_m", "aruco.marker_length_m"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.sample_count < 5 and not self.preflight_only:
            raise ValueError("sample_count must be at least five")
        if int(self.get_parameter("aruco.maximum_frames").value) < self.reference_frames:
            raise ValueError("maximum_frames must cover reference_frames")
        if self.capture_timeout_sec <= self.stability_window_sec:
            raise ValueError("capture_timeout_sec must exceed stability_window_sec")
        self.tf_buffer = Buffer()
        # Callbacks are serviced explicitly during bounded capture windows.
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        # 只保留最新一帧：抽帧节拍由 prepare_reference 控制，积压旧帧没有意义
        self.latest_image: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.samples: list[dict[str, object]] = []
        self.reference_capture: dict[str, object] | None = None
        # 只有 aruco 模式需要相机；其他模式不订阅，避免无谓占用图像带宽
        if self.reference_mode == "aruco":
            self.create_subscription(
                Image,
                str(self.get_parameter("aruco.image_topic").value),
                self._image_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("aruco.camera_info_topic").value),
                self._camera_info_callback,
                qos_profile_sensor_data,
            )

    def _image_callback(self, message: Image) -> None:
        # 回调只做缓存：抽帧与门控统一放在 prepare_reference 里，便于同步推理
        self.latest_image = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        # 内参与图像分别缓存，采样时各取"当前最新"
        self.latest_camera_info = message

    def _lookup(self, target: str, source: str, stamp=None):
        """查询 target <- source 的最新 TF，超时由 ``lookup_timeout_sec`` 控制。

        未指定 stamp 时查询最新变换；指定时查询图像拍摄时刻。查询不阻塞，
        外层采集循环负责处理回调、重试和总超时。
        """
        return self.tf_buffer.lookup_transform(
            target,
            source,
            rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
        )

    def _check_stamp(self, stamp):
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age < -0.05 or age > self.maximum_age_sec:
            raise ValueError(f"stale or invalid timestamp: age={age:.3f}s")
        return stamp_ns

    def prepare_reference(self) -> tuple[float, float, float]:
        """按当前模式准备并返回基座坐标系下的参考点（米），同时记录冻结过程数据。

        ``pivot`` 模式没有先验参考点，直接返回参数值（真正的 pivot 位置由解算器联合求出）；
        ``manual`` 模式直接采用参数给出的参考点；``aruco`` 模式主动抽帧并冻结参考点，任何
        门不过都抛异常终止。``aruco`` 模式的冻结规则：

        1. 每帧必须是新的时间戳（同一帧不重复计入），有效帧数达到 ``aruco.reference_frames``
           才算通过；
        2. 抽帧尝试次数超过 ``aruco.maximum_frames`` 仍未凑够帧数即判失败；
        3. 各帧位置的逐轴标准差最大值不得超过 ``aruco.maximum_reference_std_m``，否则判定
           标定板/机械臂在晃动，拒绝继续；
        4. 参考点取各帧位置均值；同时记录标定板法向在基座系下的离散程度供人工复核，
           法向指标只写入结果、不参与自动门控。
        """
        if self.reference_mode == "pivot":
            self.reference_capture = {
                # 记录"参考点未知"这一事实，随结果 JSON 一起落盘备查
                "mode": "classic_tcp_pivot_unknown_reference",
                # instruction 是给现场操作者的硬性要求：每次采样维持同一物理尖点
                "instruction": "keep one physical pivot point fixed for every sample",
            }
            return self.tcp_reference_position
        if self.reference_mode == "manual":
            self.reference_capture = {
                "mode": "manual",
                "reference_position": list(self.tcp_reference_position),
            }
            return self.tcp_reference_position

        accepted: list[tuple[float, float, float]] = []
        normals: list[np.ndarray] = []
        attempts = 0
        last_stamp = None
        deadline = time.monotonic() + self.capture_timeout_sec
        rejected = []
        frame_quality = []
        dictionary = str(self.get_parameter("aruco.dictionary").value)
        marker_id = int(self.get_parameter("aruco.marker_id").value)
        marker_length = float(self.get_parameter("aruco.marker_length_m").value)
        # 抽帧节拍：spin_once 每次最多等 0.2 s 处理一轮回调（图像+内参），
        # 两个条件分别限制尝试总数与已接受帧数，保证不无限等待也不超采
        while attempts < self.maximum_frames and len(accepted) < self.reference_frames:
            if time.monotonic() >= deadline:
                raise RuntimeError(f"ArUco capture timeout: accepted={len(accepted)}, attempts={attempts}")
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown during reference capture")
            rclpy.spin_once(self, timeout_sec=0.05)
            image = self.latest_image
            info = self.latest_camera_info
            if image is None or info is None:
                continue
            stamp = (int(image.header.stamp.sec), int(image.header.stamp.nanosec))
            # 用 (sec, nanosec) 去重：同一帧被反复使用会让"帧数达标"变成假象
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            attempts += 1
            try:
                image_ns = self._check_stamp(image.header.stamp)
                info_ns = self._check_stamp(info.header.stamp)
                if abs(image_ns - info_ns) / 1e9 > self.aruco_maximum_info_skew_sec:
                    raise ValueError("Image/CameraInfo timestamp mismatch")
                if image.header.frame_id != self.camera_frame or info.header.frame_id != self.camera_frame:
                    raise ValueError("Image/CameraInfo must match configured optical camera_frame")
                if (image.width, image.height) != (info.width, info.height):
                    raise ValueError("Image/CameraInfo resolution mismatch")
                if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
                    raise ValueError("unsupported CameraInfo distortion model")
                color = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                # K 为 3x3 行主序内参矩阵（像素单位）；畸变系数单独取自 info.d
                matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
                marker = detect_aruco_pose(
                    color,
                    camera_matrix=matrix,
                    distortion=list(info.d),
                    marker_length_m=marker_length,
                    dictionary_name=dictionary,
                    marker_id=marker_id,
                )
                # PnP 给出标定板中心在相机坐标系下的位置（米）
                camera_point = marker["camera_to_marker"]["translation"]
                if marker["reprojection_rmse_px"] > self.aruco_maximum_reprojection_rmse_px:
                    raise ValueError("ArUco reprojection error too large")
                if marker["area_px2"] < self.aruco_minimum_area_px2:
                    raise ValueError("ArUco marker too small")
                if not 0 < np.linalg.norm(camera_point) <= self.aruco_maximum_distance_m:
                    raise ValueError("ArUco distance out of range")
                base_camera = self._lookup(self.base_frame, self.camera_frame, image.header.stamp)
                # 换算到基座系，作为参考点的一个候选
                accepted.append(_transform_point(base_camera, camera_point))
                base_rotation = base_camera.transform.rotation
                base_to_camera_rotation = quaternion_to_rotation_matrix(
                    (base_rotation.x, base_rotation.y, base_rotation.z, base_rotation.w)
                )
                camera_to_marker_rotation = quaternion_to_rotation_matrix(
                    marker["camera_to_marker"]["rotation_xyzw"]
                )
                # 旋转矩阵第 3 列即标定板法向（板 z 轴）在基座系下的单位向量，
                # 用它检查采集期间板子是否被移动或翻动
                normals.append((base_to_camera_rotation @ camera_to_marker_rotation)[:, 2])
                frame_quality.append({"image_stamp_ns": image_ns, "area_px2": marker["area_px2"],
                                      "reprojection_rmse_px": marker["reprojection_rmse_px"]})
            except Exception as exc:
                # 单帧失败不终止：检测不到目标、PnP 失败、TF 缺失都只记日志后跳过
                rejected.append({"stamp": stamp, "reason": str(exc)})
                self.get_logger().warn(f"ArUco reference frame rejected: {exc}")
        # fail closed：帧数不够时绝不拿部分数据当参考点
        if len(accepted) < self.reference_frames:
            raise RuntimeError(
                f"ArUco reference gate failed: accepted={len(accepted)}/{attempts}, "
                f"required={self.reference_frames}"
            )
        values = np.asarray(accepted, dtype=np.float64)
        reference = np.mean(values, axis=0)
        # 逐轴标准差衡量冻结期内参考点的一致性（米）
        axis_std = np.std(values, axis=0)
        # 稳定性门：任一轴标准差超限都说明参考点不可信
        if float(np.max(axis_std)) > self.maximum_reference_std_m:
            raise RuntimeError(
                f"ArUco reference unstable: max axis std={float(np.max(axis_std)):.6f} m"
            )
        self.tcp_reference_position = tuple(float(value) for value in reference)
        # 平均法向后归一化：单位向量直接取平均只是小角度近似，这里仅作诊断输出
        mean_normal = np.mean(np.stack(normals), axis=0)
        if np.linalg.norm(mean_normal) < 1e-6:
            raise RuntimeError("ArUco plane normals are inconsistent")
        mean_normal /= np.linalg.norm(mean_normal)
        # 逐帧法向与平均法向的夹角（度）；clip 到 [-1, 1] 防浮点越界让 arccos 报错
        normal_errors_deg = [
            float(
                np.degrees(
                    np.arccos(np.clip(np.dot(normal / np.linalg.norm(normal), mean_normal), -1.0, 1.0))
                )
            )
            for normal in normals
        ]
        self.reference_capture = {
            # 标记参考点是在机器人对准动作之前冻结的，便于事后排查相关误差
            "mode": "aruco_frozen_before_alignment",
            "reference_position": reference.tolist(),
            "axis_std_m": axis_std.tolist(),
            "plane_normal_base": mean_normal.tolist(),
            "plane_normal_rms_deg": float(np.sqrt(np.mean(np.square(normal_errors_deg)))),
            "plane_normal_max_deg": float(np.max(normal_errors_deg)),
            "frame_quality": frame_quality,
            "rejected_frames": rejected,
            "accepted": len(accepted),
            "attempts": attempts,
            "camera_frame": self.camera_frame,
            "marker_length_m": marker_length,
            "marker_id": marker_id,
            "dictionary": dictionary,
        }
        return self.tcp_reference_position

    def capture_sample(self) -> dict[str, object]:
        """采集一个样本：记录当前 base->end_link 位姿并追加到样本列表。

        返回的字典同时是结果 JSON 中 ``samples`` 的一项（字段名为对外接口）：
        ``monotonic_ns`` 是采集时刻的单调时钟，仅用于事后核对采样顺序与间隔，避免依赖可能
        跳变的系统时间；``end_link_position`` 为基座系位置（米）；
        ``end_link_orientation_xyzw`` 为基座系姿态四元数 (x, y, z, w)。非 ``pivot`` 模式
        下额外记录当次使用的参考点与该姿态反算出的 TCP 偏移，便于逐点复核。
        """
        deadline = time.monotonic() + self.capture_timeout_sec
        window = []
        last_error = "no fresh TF"
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError("ROS shutdown during sample capture")
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                tf_msg = self._lookup(self.base_frame, self.end_link_frame)
                stamp_ns = self._check_stamp(tf_msg.header.stamp)
                if window and stamp_ns <= window[-1][0]:
                    continue
                t = tf_msg.transform
                position = np.array([t.translation.x, t.translation.y, t.translation.z])
                rotation = quaternion_to_rotation_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
                if not np.all(np.isfinite(position)):
                    raise ValueError("non-finite TF position")
                if window and any(np.linalg.norm(position - p) > self.stability_translation_m or
                                  _rotation_distance_deg(rotation, r) > self.stability_rotation_deg
                                  for _, p, r in window):
                    window = []
                window.append((stamp_ns, position, rotation))
                if len(window) >= 3 and (stamp_ns - window[0][0]) / 1e9 >= self.stability_window_sec:
                    break
            except Exception as exc:
                last_error = str(exc)
                window = []
        else:
            raise RuntimeError(f"TF capture timeout: {last_error}; require fresh stable samples")
        transform = tf_msg.transform
        sample: dict[str, object] = {
            "tf_stamp_ns": stamp_ns,
            "monotonic_ns": time.monotonic_ns(),
            "end_link_position": [
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            ],
            "end_link_orientation_xyzw": [
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            ],
        }
        # pivot 模式的参考点未知，偏移由解算器联合求解，这里不预计算
        if self.reference_mode != "pivot":
            sample["tcp_reference_position"] = list(self.tcp_reference_position)
            sample["offset_xyz"] = list(
                estimate_sample_offset(
                    end_link_position=sample["end_link_position"],
                    end_link_orientation_xyzw=sample["end_link_orientation_xyzw"],
                    tcp_reference_position=self.tcp_reference_position,
                )
            )
        self.samples.append(sample)
        return sample

    def result(self) -> dict[str, object]:
        """汇总本次会话：参考点冻结记录、全部原始样本与解算结果。

        ``pivot`` 模式用多点 pivot 解算器（参考点未知），其余模式用已知参考点解算器；两者的
        数学假设不同，不能混用。返回字典的 ``schema_version`` 固定为 1，供落盘与外部脚本
        按版本解析。
        """
        analysis = (
            analyze_tcp_pivot_samples(self.samples)
            if self.reference_mode == "pivot"
            else analyze_tcp_samples(self.samples)
        )
        return {
            "schema_version": 1,
            "base_frame": self.base_frame,
            "end_link_frame": self.end_link_frame,
            "reference_mode": self.reference_mode,
            "reference_capture": self.reference_capture,
            "samples": self.samples,
            "analysis": analysis,
        }


def main(args=None) -> None:
    """命令行入口：准备参考点 -> 逐样本人工确认采集 -> 打印并可选落盘分析结果。

    ``preflight_only`` 为真时在冻结参考点后立即返回，用于现场预检（不产生样本）。采样循环
    用 ``input()`` 等待操作者回车，因此 Ctrl-C 与输入结束（EOF）都属正常退出路径；无论正常
    结束还是被中断，都会销毁节点并关闭 rclpy。
    """
    rclpy.init(args=args)
    node = None
    try:
        node = TcpCalibrationNode()
        reference = node.prepare_reference()
        if node.reference_mode == "pivot":
            # pivot 模式没有先验参考点，必须提醒操作者固定住同一个物理尖点
            node.get_logger().info(
                "Classic TCP pivot mode ready. Keep one sharp physical pivot point fixed "
                "for every sample; the pivot base coordinate will be solved jointly."
            )
        else:
            # 参考点已冻结：打印它在基座系下的具体数值与坐标系名，便于现场核对
            node.get_logger().info(
                "TCP reference frozen before alignment at "
                f"({reference[0]:+.6f}, {reference[1]:+.6f}, {reference[2]:+.6f}) "
                f"in {node.base_frame}. Keep the fixture fixed."
            )
        if node.preflight_only:
            # 预检模式：只输出参考点冻结过程，不采样、不写样本
            result = {
                "schema_version": 1,
                "preflight_only": True,
                "reference_capture": node.reference_capture,
            }
            print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
            if node.output_path:
                path = Path(node.output_path).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            return
        # 每个样本都要求人工对位后回车触发，确保操作者确认机械臂已稳定再记录
        for index in range(node.sample_count):
            input(
                f"Sample {index + 1}/{node.sample_count}: align the physical grasp TCP "
                "to the frozen reference, keep the actuator in controlled hold, then press Enter..."
            )
            sample = node.capture_sample()
            # 非 pivot 模式逐点打印反算偏移，便于现场立即发现异常点
            if "offset_xyz" in sample:
                offset = sample["offset_xyz"]
                node.get_logger().info(
                    f"sample {len(node.samples)} offset="
                    f"({offset[0]:+.6f}, {offset[1]:+.6f}, {offset[2]:+.6f})"
                )
            else:
                node.get_logger().info(f"pivot sample {len(node.samples)} captured")

        result = node.result()
        analysis = result["analysis"]
        print("")
        print("TCP calibration analysis:")
        print(json.dumps(analysis, ensure_ascii=False, indent=2, allow_nan=False))
        if analysis["passed"]:
            print(format_tcp_offset_yaml(analysis["tcp_offset_xyz"]))
        else:
            print("REJECTED: " + ", ".join(k for k, v in analysis["gates"].items() if not v))
        # 只打印候选结果：是否部署由人工在核对全部门后另行决定
        print("Candidate is not deployed automatically; deploy only when every gate passes.")
        if node.output_path:
            path = Path(node.output_path).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(f"raw result written to {path}")
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        # 人工中断与输入结束是交互式标定的正常退出路径，不视为失败
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
