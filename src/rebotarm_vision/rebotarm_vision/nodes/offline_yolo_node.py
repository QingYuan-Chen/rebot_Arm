"""在 MuJoCo 虚拟 RGB 图像流上跑 YOLO，完全不接触真实硬件。

定位与数据流
------------
本节点是「纯软件」的 YOLO 适配器：订阅 RGB 图像 → 调用检测器推理 →
转换成统一检测消息 → 分流发布。它属于离线/仿真感知链路，不做任何抓取决策，
也不下发运动指令，因此可以随时启停而不影响真机安全。

话题一览
--------
* 订阅 ``offline_yolo.input_topic``（默认 ``/camera/color/image_raw``，8 位彩色图）；
* 发布 ``offline_yolo.raw_detection_topic``（默认 ``/grasp/offline_yolo/raw_detections``）：
  模型原始输出，**不做类别过滤**，仅用于诊断/调参，下游不应直接拿去抓取；
* 发布 ``offline_yolo.detection_topic``（默认 ``/grasp/offline_yolo/detections``）：
  只保留 ``target_classes`` 白名单内的检测，是下游真正消费的接口；
* 可选发布 ``offline_yolo.annotated_topic`` 带框图像，用于人工目视确认。

安全约束：类别白名单 + fail-closed
----------------------------------
两路检测输出都使用传感器数据 QoS，保证与图像时间戳同步。类别门是**显式白名单**
（allow-list）：白名单为空表示「拒绝一切检测」而不是「放行一切」。
任一帧在解码、推理或转换阶段抛异常时，本节点立刻在两路话题上各发布一条空数组
（保留原时间戳与 frame_id），使下游看到「本帧无目标」而不是沿用上一帧的陈旧结果，
这就是 fail-closed 行为。

由于 Ultralytics 只在真正需要推理时才导入，纯图像/过滤逻辑的单元测试无需安装它。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rebotarm_msgs.msg import Detection2DArray

from ..converters.detection_msgs import result_to_detection_array_msg
from ..converters.image_msgs import color_to_msg
from ..offline_yolo import decode_image_to_bgr, filter_detection_array, parse_target_classes


class OfflineYoloNode(Node):
    """带显式目标类别门控的纯软件 YOLO 适配器。

    模型原始输出单独发布一路用于诊断；只有落在 ``target_classes`` 白名单里的
    检测才会进入过滤后的输出。标准 MuJoCo 场景使用 ``bottle`` 目标，
    它存在于官方 COCO 标签集中（即无需开放词表提示词即可检出）。

    生命周期：构造时声明并校验参数、加载检测器、建立发布/订阅；
    之后完全由图像回调驱动，单线程执行器下无并发问题。
    """

    def __init__(self) -> None:
        super().__init__("rebotarm_offline_yolo_node")
        self._declare_parameters()
        self._load_parameters()

        # 延迟导入：让纯图像/过滤逻辑的测试不必安装 Ultralytics。
        from .detector.yolo_detector import YoloDetector

        self._detector = YoloDetector(
            model_path=self.model_path,
            device=self.device,
            conf_threshold=self.conf_threshold,
            iou_threshold=self.iou_threshold,
            use_world=self.use_world,
            custom_classes=self.target_classes,
        )
        self._raw_pub = self.create_publisher(
            Detection2DArray, self.raw_detection_topic, qos_profile_sensor_data
        )
        self._filtered_pub = self.create_publisher(
            Detection2DArray, self.detection_topic, qos_profile_sensor_data
        )
        self._annotated_pub = None
        if self.publish_annotated:
            self._annotated_pub = self.create_publisher(
                Image, self.annotated_topic, qos_profile_sensor_data
            )
        self._subscription = self.create_subscription(
            Image, self.input_topic, self._on_image, qos_profile_sensor_data
        )
        # fail-closed 闩锁：出错期间只记录一次错误日志，恢复后再记录一次，避免刷屏。
        self._failure_latched = False
        self._frame_count = 0
        self.get_logger().info(
            "offline YOLO initialized: "
            f"input={self.input_topic}, model={self.model_path}, device={self.device}, "
            f"target_classes={list(self.target_classes) or '<none; reject all>'}"
        )

    def _declare_parameters(self) -> None:
        # 输入彩色图像话题，需与虚拟相机发布的话题一致。
        self.declare_parameter("offline_yolo.input_topic", "/camera/color/image_raw")
        # 未过滤的模型原始输出话题，仅用于诊断。
        self.declare_parameter(
            "offline_yolo.raw_detection_topic", "/grasp/offline_yolo/raw_detections"
        )
        # 过滤后的检测话题，下游抓取链路的正式输入。
        self.declare_parameter(
            "offline_yolo.detection_topic", "/grasp/offline_yolo/detections"
        )
        # 带检测框的可视化图像话题；仅当 publish_annotated 为真时才创建发布者。
        self.declare_parameter(
            "offline_yolo.annotated_topic", "/camera/color/offline_yolo_annotated"
        )
        # YOLO 权重文件路径，必填；空值或文件不存在都会在参数校验阶段直接抛错。
        self.declare_parameter("offline_yolo.model_path", "")
        # 推理设备：cpu 或 GPU 序号字符串（如 "0"）；语义由 Ultralytics 解释。
        self.declare_parameter("offline_yolo.device", "cpu")
        # 置信度阈值，取值 [0, 1]；调低→召回更高但误检更多（默认 0.05 偏召回）。
        self.declare_parameter("offline_yolo.conf_threshold", 0.05)
        # NMS IoU 阈值，取值 [0, 1]；调小→重叠框抑制更激进（默认 0.45）。
        self.declare_parameter("offline_yolo.iou_threshold", 0.45)
        # 是否使用开放词表（World/YOLOE）检查点：需要文本提示词来指定类别。
        # 注意检测器内部还会根据权重文件名中的 world/yoloe 字样自动视为启用。
        self.declare_parameter("offline_yolo.use_world", False)
        # 类别白名单。保持为字符串类型：launch 参数通常以「看起来像 YAML」的字符串传入；
        # parse_target_classes 同时兼容字符串和原生字符串数组两种写法（直接跑节点时用后者）。
        self.declare_parameter("offline_yolo.target_classes", "['cube']")
        # 是否发布带框图像；关掉可省去绘制开销与一路图像带宽。
        self.declare_parameter("offline_yolo.publish_annotated", True)

    def _load_parameters(self) -> None:
        self.input_topic = str(self.get_parameter("offline_yolo.input_topic").value)
        self.raw_detection_topic = str(
            self.get_parameter("offline_yolo.raw_detection_topic").value
        )
        self.detection_topic = str(
            self.get_parameter("offline_yolo.detection_topic").value
        )
        self.annotated_topic = str(
            self.get_parameter("offline_yolo.annotated_topic").value
        )
        self.model_path = str(self.get_parameter("offline_yolo.model_path").value).strip()
        self.device = str(self.get_parameter("offline_yolo.device").value).strip()
        self.conf_threshold = float(
            self.get_parameter("offline_yolo.conf_threshold").value
        )
        self.iou_threshold = float(
            self.get_parameter("offline_yolo.iou_threshold").value
        )
        self.use_world = bool(self.get_parameter("offline_yolo.use_world").value)
        self.target_classes = parse_target_classes(
            self.get_parameter("offline_yolo.target_classes").value
        )
        self.publish_annotated = bool(
            self.get_parameter("offline_yolo.publish_annotated").value
        )
        # 参数校验：一律在启动期失败，避免节点带病运行到第一帧才报错。
        if not self.input_topic.strip() or not self.detection_topic.strip():
            raise ValueError("offline YOLO input and detection topics must be non-empty")
        if not self.model_path:
            raise RuntimeError("offline_yolo.model_path must not be empty")
        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"offline YOLO model not found: {self.model_path}")
        if not self.device:
            raise ValueError("offline_yolo.device must not be empty")
        # 阈值必须是概率，超出 [0, 1] 的配置直接拒绝。
        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError("offline_yolo.conf_threshold must be in [0, 1]")
        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError("offline_yolo.iou_threshold must be in [0, 1]")

    def _on_image(self, image_msg: Image) -> None:
        # 先把时间戳与坐标系取出来：出错时也要用它们发空消息，保证下游能按帧对齐。
        stamp = image_msg.header.stamp
        frame_id = str(getattr(image_msg.header, "frame_id", ""))
        try:
            image_bgr = decode_image_to_bgr(image_msg)
            results = self._detector.infer(image_bgr)
            raw = result_to_detection_array_msg(results, stamp, frame_id)
            filtered = filter_detection_array(raw, self.target_classes)
        except Exception as exc:
            # fail-closed：本帧两路都发空数组，绝不让下游沿用上一帧的检测结果。
            self._publish_empty(stamp, frame_id)
            if not self._failure_latched:
                self._failure_latched = True
                self.get_logger().error(
                    f"offline YOLO fail-closed: {type(exc).__name__}: {exc}"
                )
            return

        if self._failure_latched:
            self._failure_latched = False
            self.get_logger().info("offline YOLO stream recovered; fail-closed latch cleared")
        self._raw_pub.publish(raw)
        self._filtered_pub.publish(filtered)
        self._frame_count += 1
        # 每 20 帧汇总一次计数，避免逐帧日志淹没终端。
        if self._frame_count % 20 == 0:
            self.get_logger().info(
                f"offline YOLO frame={self._frame_count} raw={len(raw.detections)} "
                f"target={len(filtered.detections)}"
            )
        if self._annotated_pub is not None:
            try:
                from .utils.visualization import draw_detections

                annotated = draw_detections(image_bgr, raw)
                self._annotated_pub.publish(color_to_msg(annotated, stamp, frame_id))
            except Exception as exc:
                # 仅可视化失败：检测结果已经正常发布，不改变本帧语义、也不补发空消息，
                # 只借用同一个闩锁保证该警告不会逐帧刷屏。
                if not self._failure_latched:
                    self.get_logger().warning(
                        f"offline YOLO annotation disabled for this frame: {type(exc).__name__}: {exc}"
                    )

    def _publish_empty(self, stamp, frame_id: str) -> None:
        """在两路检测话题上各发一条空数组，保留原时间戳与坐标系。

        空数组的语义是「本帧确认没有目标」；下游据此判空而不是回退到上一次结果。
        """
        raw = Detection2DArray()
        raw.header.stamp = stamp
        raw.header.frame_id = frame_id
        filtered = Detection2DArray()
        filtered.header.stamp = stamp
        filtered.header.frame_id = frame_id
        self._raw_pub.publish(raw)
        self._filtered_pub.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
