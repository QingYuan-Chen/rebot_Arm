"""GraspNet 抓取候选生成节点（进程内推理基线）。

职责与位置
----------
本节点是"2D 检测 + RGB-D"到"6D 抓取候选"之间的桥梁：接收彩色图、深度图、相机内参与
上游检测结果，在**本进程内**调用 GraspNet 基线推理得到抓取位姿，再以候选数组消息发布给
下游的候选过滤/运动校验环节。

对外接口（话题均为参数可配，括号内为默认值）
------------------------------------------
- 订阅 sensor_msgs/Image 彩色图（`input_color_topic`，`/camera/color/image_raw`）
- 订阅 sensor_msgs/Image 深度图（`input_depth_topic`，`/camera/depth/image_raw`）
- 订阅 sensor_msgs/CameraInfo 内参（`input_camera_info_topic`，`/camera/depth/camera_info`）
- 订阅 rebotarm_msgs/Detection2DArray 检测结果（`input_detections_topic`，`/grasp/detections`）
- 发布 rebotarm_msgs/GraspCandidateArray 抓取候选（`output_candidates_topic`，
  `/grasp/graspnet_candidates`）

数据流与"手动时间同步"
----------------------
本节点不使用消息过滤器，而是自己用定长缓存（`color_frame_cache` / `depth_frame_cache`，
各保留最近 8 帧）做时间戳配对：
1. 只有携带候选的检测消息会进入 `pending_inprocess_detections`；
2. 彩色、深度、内参以及检测回调都会调用 `_try_process_inprocess_detection` 尝试推进流程；
3. 处理时按"时间戳最接近检测时间戳"的原则从缓存中取彩色帧与深度帧，并检查三者两两之间的
   时间偏差不超过 `max_input_skew_ms`（默认 100 ms，见该参数的注释）。

安全与失败语义
--------------
- **失败即闭合（fail closed）**：后端未配置、推理抛异常、或后端报告不可用时，一律只发布
  `best_index = -1` 的空候选数组，绝不给出可能被误执行的位姿。
- 本节点只产生候选，不做规划、不校验可达性、不接触硬件；是否执行由下游运动校验与执行门控决定。

单帧只取一个目标
----------------
每帧只挑置信度最高的那一个检测送推理（`max(..., key=confidence)`），因此候选数组
`class_name` 等字段描述的都是这同一个目标。
"""

from __future__ import annotations

from collections import deque
import json
import os

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray, GraspCandidateArray

from ..graspnet_baseline_adapter import (
    InProcessGraspNetBackend,
    closest_timestamped_frame,
    payload_to_candidate_array,
)
from ..converters.image_msgs import depth_image_to_array
from ..policies.camera_info_policy import calibrated_intrinsics


def color_image_to_array(msg: Image) -> np.ndarray:
    """把彩色图像消息转成 BGR 排列、内存连续的 uint8 数组（形状 HxWx3）。

    OpenCV 与 GraspNet 侧统一按 BGR 解读颜色，因此 `rgb8` 输入在这里翻转通道顺序，
    使推理入口只有一种颜色约定。仅接受 `bgr8` 与 `rgb8` 两种编码；
    其他编码（如 yuv、rgb16）抛 `ValueError`，由调用方记警告并丢弃该帧。
    """
    if msg.encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"unsupported color encoding: {msg.encoding}")
    channels = 3
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
    if msg.encoding == "rgb8":
        # ::-1 反转为 BGR，与 OpenCV/GraspNet 的颜色约定保持一致。
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


class GraspNetBaselineNode(Node):
    """把 RGB-D 与检测结果转成 GraspNet 抓取候选的 ROS 2 节点。

    生命周期：构造时读取参数、创建发布/订阅并尝试加载进程内推理后端；`rclpy.spin` 期间全部
    逻辑在**单线程执行器**上以回调方式串行运行，因此各回调之间无需加锁。
    后端在构造后不再重建：模型路径配置错误只在启动日志里报错，并以"发布空候选"体现，
    不会让节点崩溃。

    `source_mode` 固定为 `in_process`，仅用于日志中标识候选来源，并与后端适配器返回载荷里的
    来源字段保持一致。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_graspnet_baseline_node")
        # 输入/输出话题与坐标系。output_frame_id 是深度光学坐标系，也是候选位姿的默认参考系。
        self.declare_parameter("input_color_topic", "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("input_detections_topic", "/grasp/detections")
        self.declare_parameter("output_candidates_topic", "/grasp/graspnet_candidates")
        self.declare_parameter("output_frame_id", "camera_depth_frame")
        # 彩色/深度/检测三者允许的最大时间偏差，单位毫秒；超出则本帧不推理，等新帧到达后重试。
        self.declare_parameter("max_input_skew_ms", 100)
        # 深度图"1 个单位代表多少米"：16UC1 深度图 1 单位 = 1 mm，故默认 0.001；若上游已给米制浮点深度则应配 1.0。
        self.declare_parameter("depth_scale_m_per_unit", 0.001)
        # 以下四项为 GraspNet 后端配置；默认从同名环境变量取值，空字符串表示未配置（后端不可用）。
        self.declare_parameter("model_root", os.environ.get("GRASPNET_MODEL_ROOT", ""))
        self.declare_parameter(
            "checkpoint_path", os.environ.get("GRASPNET_CHECKPOINT_PATH", "")
        )
        # 推理设备，形如 "cuda:0" / "cpu"；本节点只做透传，由后端解释。
        self.declare_parameter("device", "cuda:0")
        # 后端包装模块名与可选的模块文件路径；模块内需提供 GraspNetBaselineInference 类。
        self.declare_parameter("backend_module", "graspnet_baseline_inference")
        self.declare_parameter(
            "backend_module_path", os.environ.get("GRASPNET_BACKEND_MODULE_PATH", "")
        )
        self.declare_parameter("max_grasps", 20)
        # 夹爪最大开口，单位 m；用于在后端过滤夹不住的候选（本站夹爪可信行程约 0.006 ~ 0.085 m）。
        self.declare_parameter("max_jaw_width_m", 0.085)
        # 送入网络的点云采样上限：越大越准，但显存占用与单帧耗时线性上升。
        self.declare_parameter("max_points", 20000)
        # 有效深度区间，单位 m；区间外像素视为无效（过近多为噪声，过远超出工作空间）。
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 1.20)
        # 内参回退值（cx/cy 对应 1280x720 图像中心）；实际推理使用 CameraInfo 消息里的实时内参。
        self.declare_parameter("fx", 692.562744140625)
        self.declare_parameter("fy", 692.2272338867188)
        self.declare_parameter("cx", 641.2417602539062)
        self.declare_parameter("cy", 361.8166198730469)

        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.source_mode = "in_process"
        # 最近一帧彩色/深度及各自时间戳（纳秒）；时间戳为 0 表示尚无有效数据。
        self.latest_color_bgr: np.ndarray | None = None
        self.latest_depth_mm: np.ndarray | None = None
        self.latest_color_timestamp_ns = 0
        self.latest_depth_timestamp_ns = 0
        self.latest_depth_frame_id = self.output_frame_id
        # 定长缓存，用于"按时间戳就近配对"；8 帧约覆盖 30 FPS 下 0.27 s 的历史。
        self.color_frame_cache = deque(maxlen=8)
        self.depth_frame_cache = deque(maxlen=8)
        self.latest_camera_info: dict[str, float] | None = None
        self.latest_camera_info_timestamp_ns = 0
        self.latest_camera_info_frame_id = ""
        # 已处理检测的时间戳水位线：小于等于它的检测视为重复/乱序，直接丢弃。
        self.last_inprocess_detection_timestamp_ns = 0
        # 等待彩色/深度帧到齐的检测消息；同一时刻只保留最新一条，旧目标会被新目标覆盖。
        self.pending_inprocess_detections: Detection2DArray | None = None
        self._warned_backend = False
        self.backend = self._create_inprocess_backend()

        self.candidates_pub = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter("output_candidates_topic").value),
            10,
        )
        # 相机类数据使用 sensor 数据 QoS（best effort、深度较小），与相机驱动端保持一致。
        self.create_subscription(
            Image,
            str(self.get_parameter("input_color_topic").value),
            self._on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("input_depth_topic").value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("input_camera_info_topic").value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        # 检测结果用 KEEP_LAST/BEST_EFFORT/深度 1：只需最新一帧，避免积压的旧检测拖慢实时性。
        detection_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Detection2DArray,
            str(self.get_parameter("input_detections_topic").value),
            self._on_detections,
            detection_qos,
        )
        self.get_logger().info(
            "GraspNet baseline candidate node ready: "
            f"output={str(self.get_parameter('output_candidates_topic').value)}, "
            f"source_mode={self.source_mode}, "
            f"backend_available={self.backend.available if self.backend is not None else 'n/a'}"
        )

    def _create_inprocess_backend(self) -> InProcessGraspNetBackend:
        """构造进程内 GraspNet 后端；构造失败不抛异常，只把后端置为不可用。

        `max_points` 至少取 1，避免误配 0 或负数导致后端采样阶段出现空点云或除零。
        """
        backend = InProcessGraspNetBackend(
            model_root=str(self.get_parameter("model_root").value),
            checkpoint_path=str(self.get_parameter("checkpoint_path").value),
            device=str(self.get_parameter("device").value),
            module_name=str(self.get_parameter("backend_module").value),
            module_path=str(self.get_parameter("backend_module_path").value),
            num_point=max(1, int(self.get_parameter("max_points").value)),
        )
        if not backend.available:
            # 模型配置缺失只降级为"发布空候选"：节点与话题链路保持存活，便于现场排障。
            self.get_logger().error(
                "GraspNet in-process backend unavailable; candidates will fail closed: "
                f"{backend.backend_error}"
            )
        return backend

    def _on_color(self, msg: Image) -> None:
        try:
            self.latest_color_bgr = color_image_to_array(msg)
            self.latest_color_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
            self.color_frame_cache.append(
                (self.latest_color_timestamp_ns, self.latest_color_bgr)
            )
            if self.source_mode == "in_process":
                self._try_process_inprocess_detection()
        except ValueError as exc:
            # 编码不受支持：记警告并保留上一帧数据，不更新缓存。
            self.get_logger().warn(str(exc))

    def _on_depth(self, msg: Image) -> None:
        try:
            self.latest_depth_mm = depth_image_to_array(msg)
            self.latest_depth_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
            # frame_id 为空时退回参数中的默认坐标系，避免下游变换拿到空字符串。
            self.latest_depth_frame_id = str(msg.header.frame_id or self.output_frame_id)
            self.depth_frame_cache.append(
                (
                    self.latest_depth_timestamp_ns,
                    self.latest_depth_mm,
                    self.latest_depth_frame_id,
                )
            )
            if self.source_mode == "in_process":
                self._try_process_inprocess_detection()
        except ValueError as exc:
            self.get_logger().warn(str(exc))

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # CameraInfo 常与深度帧同时或稍晚到达，这里也触发一次处理尝试，避免检测消息被无限期挂起。
        info = {
            "width": int(msg.width),
            "height": int(msg.height),
            "fx": float(msg.k[0]),
            "fy": float(msg.k[4]),
            "cx": float(msg.k[2]),
            "cy": float(msg.k[5]),
        }
        if not calibrated_intrinsics(info, width=msg.width, height=msg.height):
            self.latest_camera_info = None
            self.get_logger().warn("invalid CameraInfo; GraspNet candidates withheld")
            return
        self.latest_camera_info = info
        self.latest_camera_info_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
        self.latest_camera_info_frame_id = msg.header.frame_id
        self._try_process_inprocess_detection()

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._on_inprocess_detections(msg)

    def _on_inprocess_detections(self, msg: Detection2DArray) -> None:
        # 只缓存不立即处理：等 _try_process 在数据齐备时统一推进，保证"每帧最多一次推理"。
        self.pending_inprocess_detections = msg
        self._try_process_inprocess_detection()

    def _try_process_inprocess_detection(self) -> None:
        """尝试用当前缓存完成一次推理；条件不满足时保留 pending，等后续回调再次触发。

        只有在真正完成一次推理（或确认无候选）后才推进
        `last_inprocess_detection_timestamp_ns` 水位线，因此等待图像期间同一帧不会被重复处理。
        """
        msg = self.pending_inprocess_detections
        if msg is None:
            return
        detection_timestamp_ns = self._stamp_to_ns(msg.header.stamp)
        if detection_timestamp_ns <= self.last_inprocess_detection_timestamp_ns:
            # 乱序或重复的检测（时间戳未前进）：丢弃，避免用旧结果覆盖新结果。
            self.pending_inprocess_detections = None
            return
        if not msg.detections:
            self.pending_inprocess_detections = None
            self.last_inprocess_detection_timestamp_ns = detection_timestamp_ns
            # 无检测也要发空候选，让下游立刻清空上一帧候选，而不是继续沿用陈旧位姿。
            self._publish_empty(
                timestamp_ns=detection_timestamp_ns,
                frame_id=self.latest_depth_frame_id,
            )
            return
        if self.backend is None or self.latest_camera_info is None:
            # 后端或内参尚未就绪：保留 pending，等对应回调再次触发。
            return
        color_frame = closest_timestamped_frame(
            self.color_frame_cache, detection_timestamp_ns
        )
        depth_frame = closest_timestamped_frame(
            self.depth_frame_cache, detection_timestamp_ns
        )
        if color_frame is None or depth_frame is None:
            # 缓存里还没有可用帧（例如刚启动）：保留 pending 等首帧到达。
            return
        color_timestamp_ns, color_bgr = color_frame
        depth_timestamp_ns, depth_mm, depth_frame_id = depth_frame
        if not calibrated_intrinsics(
            self.latest_camera_info, width=depth_mm.shape[1], height=depth_mm.shape[0]
        ) or self.latest_camera_info_frame_id != depth_frame_id:
            return
        # 毫秒 → 纳秒；三路时间戳两两比较，任一对超过阈值就放弃本帧。
        max_skew_ns = int(self.get_parameter("max_input_skew_ms").value) * 1_000_000
        color_skew_ns = abs(color_timestamp_ns - detection_timestamp_ns)
        depth_skew_ns = abs(depth_timestamp_ns - detection_timestamp_ns)
        rgbd_skew_ns = abs(color_timestamp_ns - depth_timestamp_ns)
        info_skew_ns = abs(self.latest_camera_info_timestamp_ns - depth_timestamp_ns)
        if max(color_skew_ns, depth_skew_ns, rgbd_skew_ns, info_skew_ns) > max_skew_ns:
            # 配对的图像回调可能还没被调度执行。
            # 因此只保留这条最新的检测继续等待，等彩色/深度帧到达后再重试。
            return
        self.pending_inprocess_detections = None
        self.last_inprocess_detection_timestamp_ns = detection_timestamp_ns
        # 单帧只处理置信度最高的一个目标，其余检测丢弃（多目标抓取由上游分帧驱动）。
        detection = max(msg.detections, key=lambda item: float(item.confidence))
        if not self.backend.available:
            self._publish_empty(timestamp_ns=depth_timestamp_ns, frame_id=depth_frame_id)
            return
        # 检测报文用普通 dict 传给后端，键名与后端约定的字段一一对应。
        detection_payload = {
            "x_min": int(detection.x_min),
            "y_min": int(detection.y_min),
            "x_max": int(detection.x_max),
            "y_max": int(detection.y_max),
            "confidence": float(detection.confidence),
            "class_name": str(detection.class_name),
        }
        # 掩膜多边形至少需要 3 个顶点（6 个浮点数）才有效；否则后端退回使用矩形框裁剪点云。
        if bool(getattr(detection, "has_mask", False)) and len(
            getattr(detection, "mask_polygon_xy", [])
        ) >= 6:
            detection_payload["mask_polygon_xy"] = [
                float(value) for value in detection.mask_polygon_xy
            ]
        try:
            # 深度在此换算为米制 float32：depth_mm 的名字来自 16UC1 的毫米语义，
            # depth_scale_m_per_unit 默认 0.001；后端按米制浮点深度处理（其内部 depth_scale_m = 1.0）。
            payload = self.backend.infer(
                timestamp_ns=depth_timestamp_ns,
                frame_id=depth_frame_id,
                color_bgr=color_bgr,
                depth_m=depth_mm.astype(np.float32)
                * float(self.get_parameter("depth_scale_m_per_unit").value),
                camera_info=self.latest_camera_info,
                detection=detection_payload,
                max_grasps=int(self.get_parameter("max_grasps").value),
                max_jaw_width_m=float(self.get_parameter("max_jaw_width_m").value),
                min_depth_m=float(self.get_parameter("min_depth_m").value),
                max_depth_m=float(self.get_parameter("max_depth_m").value),
            )
        except Exception as exc:
            # 推理异常（显存不足、模型报错等）绝不向上抛：发布空候选并等下一帧。
            self.get_logger().warn(
                f"GraspNet in-process inference failed: {type(exc).__name__}: {exc}"
            )
            self._publish_empty(timestamp_ns=depth_timestamp_ns, frame_id=depth_frame_id)
            return
        self._log_stage_counts(
            timestamp_ns=depth_timestamp_ns,
            frame_id=depth_frame_id,
            class_name=str(detection.class_name),
            candidates=payload.get("candidates", []),
        )
        # 载荷里可能带后端自定义字段，这里统一转成本站点定义的候选数组消息再发布。
        candidates = payload_to_candidate_array(
            payload,
            fallback_frame_id=depth_frame_id,
            max_candidates=int(self.get_parameter("max_grasps").value),
        )
        self.candidates_pub.publish(candidates)

    def _log_stage_counts(
        self,
        *,
        timestamp_ns: int,
        frame_id: str,
        class_name: str,
        candidates,
    ) -> None:
        """把后端各阶段计数以单行 JSON 写入日志，用于分辨候选是在哪一步被过滤掉的。

        记录里同时带上本节点侧配置（`max_grasps` / `max_jaw_width_m`）与最终发布的最高分
        （`published_top_score`，无候选时为 null），便于与后端内部计数对照定位。
        """
        if self.backend is None:
            return
        stage_counts = self.backend.last_stage_counts
        if not stage_counts:
            # 后端未提供阶段计数（例如旧版后端）时静默跳过，不制造噪声日志。
            return
        top_score = None
        if candidates:
            top_score = float(candidates[0].get("score", 0.0))
        record = {
            "event": "graspnet_stage_counts",
            "transport": "in_process",
            "timestamp_ns": int(timestamp_ns),
            "frame_id": str(frame_id),
            "class_name": str(class_name),
            "max_grasps": int(self.get_parameter("max_grasps").value),
            "max_jaw_width_m": float(self.get_parameter("max_jaw_width_m").value),
            **stage_counts,
            "published_top_score": top_score,
        }
        # separators 去掉空格，保证一条记录占一行，便于 grep 与日志解析。
        self.get_logger().info(json.dumps(record, separators=(",", ":")))

    def _publish_empty(
        self,
        *,
        timestamp_ns: int = 0,
        frame_id: str | None = None,
    ) -> None:
        """发布空候选数组，作为"本帧没有可执行抓取"的显式信号。

        `best_index = -1` 是下游判断"无候选"的约定；`timestamp_ns` 为 0 时保持 header 默认值
        （即未设置时间戳），表示这次空结果不对应任何确定的时间基准。
        """
        msg = GraspCandidateArray()
        msg.header.frame_id = str(frame_id or self.output_frame_id)
        if timestamp_ns > 0:
            # 整数纳秒拆成 ROS 时间戳的秒 + 纳秒两个字段。
            msg.header.stamp.sec = timestamp_ns // 1_000_000_000
            msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        msg.best_index = -1
        self.candidates_pub.publish(msg)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        # 统一换算成整数纳秒再比较，避免浮点秒带来的精度损失。
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None) -> None:
    """节点入口：注册中断处理，保证 Ctrl-C 时仍能清理节点与通信上下文。"""
    rclpy.init(args=args)
    node = GraspNetBaselineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # 二次 Ctrl-C 时仍要走完清理流程，避免留下半销毁的上下文。
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
