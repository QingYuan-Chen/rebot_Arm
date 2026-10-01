"""reBotArm 视觉前端节点：相机取帧、图像/内参发布与本地目标检测。

职责与位置
----------
本节点是运行时视觉链路的最前端，直接访问 RGB-D 相机：按固定频率取一帧彩色图与一帧
深度图，把两路图像与各自的相机内参发布到 ROS 话题，并在启用时用本地权重的 YOLO 模型
做 2D 目标检测。下游的抓取候选生成、运动校验与执行门控只消费本节点发布的话题，不直接
访问相机（调试预览、手眼标定等工具另有各自的相机入口）。

对外接口（话题名固定，属于跨包契约，不得随意更名）
--------------------------------------------------
- 发布 ``sensor_msgs/Image`` 彩色图，编码 ``bgr8``：``/camera/color/image_raw``
- 发布 ``sensor_msgs/Image`` 深度图，编码 ``mono16``（单位毫米，uint16）：``/camera/depth/image_raw``
- 发布 ``sensor_msgs/CameraInfo`` 彩色内参：``/camera/color/camera_info``
- 发布 ``sensor_msgs/CameraInfo`` 深度内参：``/camera/depth/camera_info``
- 发布 ``rebotarm_msgs/Detection2DArray`` 检测结果：``/grasp/detections``
- 可选发布 ``sensor_msgs/Image`` 标注图（矩形框/旋转框/掩码/类别标签）：``/camera/color/annotated``

关键流程（单线程定时器循环，无额外工作线程）
--------------------------------------------
每次 ``_on_timer`` 依次执行：取帧 -> 判断本帧是否"完整" -> 生成时间戳 -> 发布图像与内参
-> （可选）YOLO 推理并发布检测结果 -> （可选）刷新本地预览窗口。

检测结果话题**每个周期都会发布**：没有完整帧或未启用检测时发布空数组，这样下游的
"消息新鲜度"判断不会因为订阅端长时间收不到消息而误判数据源已死。

安全约束
--------
- **失败即闭合（fail closed）**：连续空帧/残帧达到 ``camera.max_empty_frames`` 后只锁存
  一条错误日志并继续发布空检测，绝不用上一帧的陈旧图像伪造检测结果；锁存位仅用于诊断，
  不参与机械臂控制。
- 本节点不发布运动指令、不访问电机 SDK、不调用规划或执行接口；视觉候选必须经下游的规划、
  碰撞检查与执行门控后才可能下发。
- 参数在启动时一次性校验（相机类型、QoS 可靠性取值、检测模型路径等），非法配置直接抛
  异常终止启动，宁可起不来也不要带病运行。
"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from rebotarm_msgs.msg import Detection2DArray

from ..camera.gemini2_driver import Gemini2Config, Gemini2Driver
from ..policies.camera_info_policy import calibrated_intrinsics
from ..converters.detection_msgs import result_to_detection_array_msg
from ..converters.image_msgs import camera_info_to_msg, color_to_msg, depth_to_msg
from ..detector.yolo_detector import YoloDetector
from ..policies.timestamp_policy import select_frame_timestamp_ns
from ..utils.visualization import draw_detections


class RebotArmVisionNode(Node):
    """视觉输入与检测节点。

    生命周期：``__init__`` 里依次声明/加载参数、创建发布者、创建相机驱动与检测器、
    打开并预热相机、按需创建预览窗口，最后启动定时器；``destroy_node`` 关闭相机并销毁
    预览窗口。全部工作都在 rclpy 单线程执行器的定时器回调中完成，因此驱动调用、编码与
    发布的先后顺序天然串行，不需要额外加锁。

    内部状态（都不参与运动控制，只影响日志与可视化）：
    - ``empty_frame_count``：连续异常帧计数，完整帧到达时清零；
    - ``_vision_failure_latched``：视觉输入失败的诊断锁存位，恢复后由回调清除；
    - ``_detection_log_countdown``：检测结果日志的降频计数器。
    """

    def __init__(self) -> None:
        """按依赖顺序完成初始化。

        顺序不可随意调整：发布者需要已加载的参数（如 ``ros.publish_annotated``）才能决定
        是否创建；相机必须在定时器启动前打开并预热，否则第一次回调会拿到未就绪的采集管线。
        """
        super().__init__("rebotarm_vision_node")

        self._declare_parameters()
        self._load_parameters()
        self._preview_window_created = False
        image_qos = self._image_qos_profile()

        # 四路图像/内参话题共用同一 QoS，保证同源的彩色图、深度图与内参在传输行为上一致。
        self.color_pub = self.create_publisher(
            Image,
            "/camera/color/image_raw",
            image_qos,
        )
        self.depth_pub = self.create_publisher(
            Image,
            "/camera/depth/image_raw",
            image_qos,
        )
        self.color_camera_info_pub = self.create_publisher(
            CameraInfo,
            "/camera/color/camera_info",
            image_qos,
        )
        self.depth_camera_info_pub = self.create_publisher(
            CameraInfo,
            "/camera/depth/camera_info",
            image_qos,
        )
        # 检测结果是下游规划的关键输入，使用 rclpy 默认 QoS（KEEP_LAST、深度 10、可靠传输），
        # 与图像话题的 best_effort 策略区分开：图像可以丢帧，检测结果不应因 QoS 被静默丢弃。
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            "/grasp/detections",
            10,
        )
        # 未启用标注图时保持 None，回调里据此跳过额外的绘制与编码，避免无谓开销。
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                "/camera/color/annotated",
                image_qos,
            )

        self.camera = self._create_camera_driver()

        # 检测器按需构造：关闭检测时既不加载模型也不占显存，本节点可退化为纯相机桥接。
        self.detector = None
        if self.enable_detection:
            self.detector = YoloDetector(
                model_path=self.model_path,
                device=self.device,
                conf_threshold=self.conf_threshold,
                iou_threshold=self.iou_threshold,
                use_world=self.use_world,
                custom_classes=self.custom_classes,
            )
            # 开放词表（World/YOLOE）权重需要文本提示词；这里只报告提示词是否设置成功，
            # 设置失败时检测器自行回退到闭集模式，不阻止节点启动。
            if self.use_world:
                if self.detector.open_vocab_enabled:
                    self.get_logger().info("YOLO open-vocabulary prompts enabled")
                elif self.detector.class_prompt_error:
                    self.get_logger().warn(
                        "YOLO open-vocabulary prompts unavailable, "
                        f"falling back to closed-set mode: {self.detector.class_prompt_error}"
                    )

        # open() 会按请求的分辨率/帧率匹配设备 profile，匹配不到直接抛异常；
        # warmup 只是丢弃前若干帧等自动曝光/白平衡稳定，不完整也不致命，后续取帧仍会继续重试。
        self.camera.open()
        ready = self.camera.warmup(self.warmup_frames)
        if not ready:
            self.get_logger().warn("camera warmup incomplete, continuing with frame retries")

        # 下面三个计数器/锁存位只服务于日志与状态展示，不影响帧的发布与检测结果内容。
        self.empty_frame_count = 0
        self._vision_failure_latched = False
        self._detection_log_countdown = 0
        # 预览窗口是无头服务器上最容易失败的一环：没有 DISPLAY 或 OpenCV 建窗失败都只降级为
        # "不预览"，不影响任何话题发布。
        if self.show_preview:
            display_env = os.environ.get("DISPLAY", "").strip()
            if not display_env:
                self.show_preview = False
                self.get_logger().warn(
                    "preview disabled because DISPLAY is not set; "
                    "launch from the Ubuntu desktop terminal to see the local window"
                )
            else:
                try:
                    cv2.namedWindow(self.preview_window_name, cv2.WINDOW_AUTOSIZE)
                    self._preview_window_created = True
                    self.get_logger().info(
                        f"preview window enabled on DISPLAY={display_env}"
                    )
                except Exception as exc:
                    self.show_preview = False
                    self.get_logger().warn(
                        f"preview disabled because OpenCV window creation failed: {exc}"
                    )
        # 频率下限钳到 1 Hz：防止配置为 0 或负数时除零或产生忙等定时器。
        period = 1.0 / max(self.loop_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info("rebotarm_vision_node initialized")

    def _create_camera_driver(self):
        """按 ``camera.type`` 构造相机驱动。

        目前只支持 ``gemini2``（Orbbec Gemini 2，USB 直连）。未知类型直接抛异常而**不**
        回退到其它后端：避免配置写错时静默切换到非预期的数据源。
        """
        if self.camera_type == "gemini2":
            return Gemini2Driver(
                Gemini2Config(
                    color_width=self.color_width,
                    color_height=self.color_height,
                    color_fps=self.color_fps,
                    enable_depth=self.enable_depth,
                    depth_width=self.depth_width,
                    depth_height=self.depth_height,
                    depth_fps=self.depth_fps,
                    frame_timeout_ms=self.frame_timeout_ms,
                    enable_align=self.enable_align,
                    enable_frame_sync=self.enable_frame_sync,
                    color_format=self.color_format,
                )
            )
        raise RuntimeError(f"unsupported camera.type: {self.camera_type}")

    def _declare_parameters(self) -> None:
        """声明全部 ROS 参数及默认值（默认值 = 无 YAML/launch 覆盖时的实际行为）。

        ``camera.*`` 为采集参数，``yolo.*`` 为检测参数，``ros.*`` 为发布/可视化参数。
        默认值故意偏保守：不启用检测、不弹预览窗口，仅做相机桥接。
        """
        # 相机后端类型；当前仅 "gemini2" 合法，其它取值会在创建驱动时抛错。
        self.declare_parameter("camera.type", "gemini2")
        # 彩色流分辨率（像素）。必须命中设备支持的 profile，否则驱动抛 profile unavailable。
        self.declare_parameter("camera.color_width", 640)
        self.declare_parameter("camera.color_height", 480)
        # 彩色流帧率（Hz），需与所选 profile 完全一致。
        self.declare_parameter("camera.color_fps", 30)
        # 彩色流像素格式。注意：该参数当前未被 _load_parameters 或驱动读取，驱动固定按
        # MJPG -> RGB 的顺序尝试设备 profile；保留声明只为兼容 YAML/launch 的覆盖习惯。
        self.declare_parameter("camera.color_format", "MJPG")
        # 是否启用深度流。关闭时本节点只发布彩色图，深度话题不再有数据。
        self.declare_parameter("camera.enable_depth", True)
        # 深度流分辨率（像素）。宽或高任一为 0 时使用设备默认 profile（同驱动约定）。
        self.declare_parameter("camera.depth_width", 0)
        self.declare_parameter("camera.depth_height", 0)
        # 深度流帧率（Hz）。
        self.declare_parameter("camera.depth_fps", 30)
        # 是否启用深度到彩色的硬件对齐（D2C）。开启后深度图与彩色图逐像素对应，下游可以
        # 直接用彩色像素坐标取深度；此时深度内参改用彩色内参。
        self.declare_parameter("camera.enable_align", False)
        self.declare_parameter("camera.enable_frame_sync", True)
        # 打开相机后用于预热的取帧次数上限，等待自动曝光/白平衡稳定；数值越大启动越慢。
        self.declare_parameter("camera.warmup_frames", 15)
        # 单次等待一帧的超时（毫秒）。超时按空帧处理；过小会频繁判空，过大会拖慢循环。
        self.declare_parameter("camera.frame_timeout_ms", 1000)
        # 连续空帧/残帧的告警阈值：达到后锁存一次 fail-closed 错误并继续发布空检测。
        self.declare_parameter("camera.max_empty_frames", 30)
        # 检测模型权重路径。启用检测时不能为空且文件必须存在，否则启动失败。
        self.declare_parameter("yolo.model_path", "")
        # 推理设备："cpu"，或 GPU 序号字符串（如 "0"），透传给推理框架。
        self.declare_parameter("yolo.device", "cpu")
        # 检测置信度阈值，取值 0~1；调高减少误检，调低提高召回。
        self.declare_parameter("yolo.conf_threshold", 0.5)
        # 非极大值抑制的 IoU 阈值，取值 0~1；调低会更激进地抑制重叠框。
        self.declare_parameter("yolo.iou_threshold", 0.45)
        # 是否按开放词表（World/YOLOE）权重使用，用 custom_classes 作为文本提示词。
        self.declare_parameter("yolo.use_world", True)
        # 允许输出的类别名列表：既作为开放词表的文本提示词，也作为闭集模式下的输出白名单
        # （比较时去空白并转小写）。
        self.declare_parameter("yolo.custom_classes", ["cup", "bottle", "banana"])
        # 彩色图像消息 header.frame_id，必须与 TF 树中的相机坐标系名一致，否则下游变换失败。
        self.declare_parameter("ros.frame_id_color", "camera_color_frame")
        # 深度图像消息 header.frame_id；开启对齐后名字不变，但坐标系含义为彩色光学系。
        self.declare_parameter("ros.frame_id_depth", "camera_depth_frame")
        # 是否额外发布画好矩形框/掩码/类别标签的标注图到 /camera/color/annotated。
        self.declare_parameter("ros.publish_annotated", True)
        # 是否弹出本地 OpenCV 预览窗口（需要 DISPLAY；无显示环境会自动降级为不预览）。
        self.declare_parameter("ros.show_preview", False)
        # 预览窗口标题，仅本地显示使用。
        self.declare_parameter("ros.preview_window_name", "RebotArm Vision Preview")
        # 取帧/发布循环频率（Hz）；内部按 max(该值, 1.0) 计算定时器周期，防止除零或忙等。
        self.declare_parameter("ros.loop_rate_hz", 10.0)
        # 图像类话题的 QoS 可靠性取值：best_effort 或 reliable。640x480 大帧在本机 Fast DDS
        # 上用 best_effort 容易分片丢失，故 Ubuntu 原生配置覆盖为 reliable。
        self.declare_parameter("ros.image_reliability", "best_effort")
        # 是否启用 YOLO 检测；关闭时只发布图像与内参，检测话题固定发布空数组。
        self.declare_parameter("ros.enable_detection", False)

    def _load_parameters(self) -> None:
        """把参数读入实例属性并做启动期校验。

        所有取值都显式做类型转换（YAML 覆盖后类型可能变化）；校验失败一律抛异常终止启动，
        不在运行期做"猜测性"回退。``use_sim_time`` 是 ROS 运行时的内置参数，这里读取它是
        为了决定帧时间戳使用相机硬件时间还是节点时钟（见 ``_frame_stamp``）。
        """
        self.camera_type = str(self.get_parameter("camera.type").value)
        self.color_width = int(self.get_parameter("camera.color_width").value)
        self.color_height = int(self.get_parameter("camera.color_height").value)
        self.color_fps = int(self.get_parameter("camera.color_fps").value)
        self.color_format = str(self.get_parameter("camera.color_format").value).strip().upper()
        if self.color_format not in {"MJPG", "RGB"}:
            raise RuntimeError("camera.color_format must be 'MJPG' or 'RGB'")
        self.enable_depth = bool(self.get_parameter("camera.enable_depth").value)
        self.depth_width = int(self.get_parameter("camera.depth_width").value)
        self.depth_height = int(self.get_parameter("camera.depth_height").value)
        self.depth_fps = int(self.get_parameter("camera.depth_fps").value)
        self.enable_align = bool(self.get_parameter("camera.enable_align").value)
        self.enable_frame_sync = bool(self.get_parameter("camera.enable_frame_sync").value)
        self.warmup_frames = int(self.get_parameter("camera.warmup_frames").value)
        self.frame_timeout_ms = int(self.get_parameter("camera.frame_timeout_ms").value)
        self.max_empty_frames = int(self.get_parameter("camera.max_empty_frames").value)
        self.model_path = str(self.get_parameter("yolo.model_path").value)
        self.device = str(self.get_parameter("yolo.device").value)
        self.conf_threshold = float(self.get_parameter("yolo.conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("yolo.iou_threshold").value)
        self.use_world = bool(self.get_parameter("yolo.use_world").value)
        self.custom_classes = list(self.get_parameter("yolo.custom_classes").value)
        self.frame_id_color = str(self.get_parameter("ros.frame_id_color").value)
        self.frame_id_depth = str(self.get_parameter("ros.frame_id_depth").value)
        self.publish_annotated = bool(self.get_parameter("ros.publish_annotated").value)
        self.show_preview = bool(self.get_parameter("ros.show_preview").value)
        self.preview_window_name = str(self.get_parameter("ros.preview_window_name").value)
        self.loop_rate_hz = float(self.get_parameter("ros.loop_rate_hz").value)
        self.image_reliability = str(
            self.get_parameter("ros.image_reliability").value
        ).strip().lower()
        self.enable_detection = bool(self.get_parameter("ros.enable_detection").value)
        self.ros_use_sim_time = bool(self.get_parameter("use_sim_time").value)

        # 只接受两个已知取值，写错时不静默退回默认策略，避免出现"以为可靠其实不可靠"。
        if self.image_reliability not in {"best_effort", "reliable"}:
            raise RuntimeError(
                "ros.image_reliability must be 'best_effort' or 'reliable'"
            )

        # 启用检测就必须给出可用权重：路径为空或文件不存在都直接失败（fail closed），
        # 不允许"先起来再等模型"。
        if self.enable_detection:
            if not self.model_path:
                raise RuntimeError("yolo.model_path must not be empty")
            if not Path(self.model_path).exists():
                raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

    def _image_qos_profile(self) -> QoSProfile:
        """构造图像/内参话题共用的 QoS。

        固定 KEEP_LAST、深度 5、VOLATILE：图像流只关心最新一帧，历史帧没有重放价值；
        可靠性与 ``ros.image_reliability`` 联动，在低延迟与不丢大帧之间取舍。这里不使用
        ROS 的传感器数据 QoS 预设，因为本机实测大尺寸 bgr8 帧在该预设下会被丢弃。
        """
        reliability = (
            ReliabilityPolicy.RELIABLE
            if self.image_reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _on_timer(self) -> None:
        """定时器回调：完成"取帧 -> 发布 -> 检测 -> 预览"的一个完整周期。

        缺帧、残帧等失败情形都走提前 return 的分支，不会继续用不可靠数据往下算；无论走哪条
        分支，检测话题都会收到一条消息（正常结果或空数组），使下游能够区分"没有目标"与
        "视觉链路已死"。
        """
        # 正式视觉链路只接受本次采集的帧，不用驱动的历史缓存补齐缺失的流。
        color_bgr, depth_mm = self.camera.get_frame(allow_partial=True)
        # 彩色与深度同时为空 = 本轮完全没有可用输入：计一次空帧、发空检测并记录失败。
        # 日志每 10 次一条，避免相机掉线时刷屏。
        if color_bgr is None and depth_mm is None:
            self.empty_frame_count += 1
            self._publish_empty_detection()
            self._record_frame_failure("empty")
            if self.empty_frame_count % 10 == 0:
                self.get_logger().warn(
                    f"empty frames encountered: {self.empty_frame_count}, "
                    f"driver_state={self.camera.last_debug_message}"
                )
            return

        depth_expected = self.enable_depth
        # "完整帧"的定义：必须有彩色图；启用深度时还必须有深度图。启用深度却缺深度时宁可
        # 不发布检测结果，也不用陈旧深度融合出可能被下游执行的抓取位姿。
        frame_complete = color_bgr is not None and (not depth_expected or depth_mm is not None)
        if not frame_complete:
            self.empty_frame_count += 1
            self._record_frame_failure("partial")
            if self.empty_frame_count % 10 == 0:
                self.get_logger().warn(
                    f"partial frames encountered: {self.empty_frame_count}, "
                    f"driver_state={self.camera.last_debug_message}"
                )
        else:
            # 完整帧到达即认为数据流恢复：清零连续计数并解除诊断锁存。
            self.empty_frame_count = 0
            if self._vision_failure_latched:
                self.get_logger().info("vision frame stream recovered; fail-closed latch cleared")
            self._vision_failure_latched = False
        # 接收时刻作为兜底时间戳：相机未提供硬件时间戳或使用仿真时钟时采用。
        receipt_stamp = self.get_clock().now().to_msg()
        color_stamp = self._frame_stamp("color", receipt_stamp)
        depth_stamp = self._frame_stamp("depth", receipt_stamp)
        # 预览默认直接复用彩色图；若本轮生成了标注图，稍后会被替换掉。
        preview_image = color_bgr

        color_info = self._camera_info_payload("color", color_bgr) if color_bgr is not None else None
        depth_info = self._camera_info_payload("depth", depth_mm) if depth_mm is not None else None
        if color_bgr is not None:
            self.color_pub.publish(color_to_msg(color_bgr, color_stamp, self.frame_id_color))
            if color_info is not None:
                self.color_camera_info_pub.publish(
                    camera_info_to_msg(color_info, color_stamp, self.frame_id_color)
                )
        if depth_mm is not None:
            self.depth_pub.publish(depth_to_msg(depth_mm, depth_stamp, self.frame_id_depth))
            if depth_info is not None:
                self.depth_camera_info_pub.publish(
                    camera_info_to_msg(depth_info, depth_stamp, self.frame_id_depth)
                )

        # 残帧：图像部分已按各自可用性发布，但本轮不推理，只补一条空检测保证话题节奏。
        if not frame_complete:
            self._publish_empty_detection(color_stamp)
            return

        # 仅二维显示可以继续；抓取候选所需的 CameraInfo 缺失时发布空检测。
        if self.enable_depth and (color_info is None or depth_info is None):
            self._publish_empty_detection(color_stamp)
            if not getattr(self, "_missing_camera_info_logged", False):
                self.get_logger().warn(
                    "calibrated CameraInfo unavailable; detections withheld "
                    f"(color={color_info is not None}, depth={depth_info is not None})"
                )
                self._missing_camera_info_logged = True
            return
        self._missing_camera_info_logged = False

        if self.detector is not None and color_bgr is not None:
            results = self.detector.infer(color_bgr)
            detection_msg = result_to_detection_array_msg(
                results,
                color_stamp,
                self.frame_id_color,
                # 类别白名单按"去空白 + 转小写"归一化，与转换器内部的比较方式一致；
                # 空字符串项被过滤掉；注意空集合表示过滤掉全部检测，而非不限制类别。
                allowed_classes={
                    str(class_name).strip().lower()
                    for class_name in self.custom_classes
                    if str(class_name).strip()
                },
            )
            self.detection_pub.publish(detection_msg)
            # 检测日志降频：每 20 个检测周期打印一条（10 Hz 循环下约 2 秒一条），
            # 只抽样打印前 5 个类别名。
            self._detection_log_countdown += 1
            if self._detection_log_countdown >= 20:
                self._detection_log_countdown = 0
                sample_names = [det.class_name for det in detection_msg.detections[:5]]
                self.get_logger().info(
                    f"detection_count={len(detection_msg.detections)} sample={sample_names}"
                )

            # 标注只画一次：需要发布标注图时把同一张结果复用给本地预览，避免重复绘制。
            if self.annotated_pub is not None:
                annotated = draw_detections(color_bgr, detection_msg)
                self.annotated_pub.publish(
                    color_to_msg(annotated, color_stamp, self.frame_id_color)
                )
                preview_image = annotated
            elif self.show_preview:
                preview_image = draw_detections(color_bgr, detection_msg)

        if self.show_preview and self._preview_window_created and preview_image is not None:
            cv2.imshow(self.preview_window_name, preview_image)
            # waitKey(1) 兼作 GUI 事件泵；取低 8 位得到键码，27 对应 ESC。
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                self.show_preview = False
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
                self.get_logger().info("preview window closed by user")
            # 窗口被标题栏关闭时可见性属性会小于 1，此时同步清理状态，避免后续 imshow 抛异常。
            elif cv2.getWindowProperty(self.preview_window_name, cv2.WND_PROP_VISIBLE) < 1:
                self.show_preview = False
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
                self.get_logger().info("preview window closed")

    def _record_frame_failure(self, kind: str) -> None:
        """在连续异常帧数达到阈值时锁存一次 fail-closed 错误。

        ``kind`` 取 ``"empty"``（彩色与深度都为空）或 ``"partial"``（只到一半），仅用于
        日志区分。阈值下限钳到 1，保证配置成 0 或负数时仍能触发一次告警；锁存位让错误日志
        只打印一次，数据流恢复后由 ``_on_timer`` 清除。
        """
        threshold = max(int(self.max_empty_frames), 1)
        if self.empty_frame_count < threshold or self._vision_failure_latched:
            return
        self._vision_failure_latched = True
        self.get_logger().error(
            "vision input fail-closed: "
            f"{self.empty_frame_count} consecutive {kind} frames reached "
            f"camera.max_empty_frames={threshold}; empty detections will continue"
        )

    def _publish_empty_detection(self, stamp=None) -> None:
        """发布一条不含任何检测的检测数组消息。

        空消息同样带合法的时间戳与 ``frame_id``，下游的"消息新鲜度"判断才有意义：
        收到空数组 = 本轮没有目标，收不到消息 = 视觉链路异常。``stamp`` 为 None 时取当前
        节点时钟，供没有可用帧时间戳的早退路径使用。
        """
        detection_msg = Detection2DArray()
        detection_msg.header.stamp = stamp or self.get_clock().now().to_msg()
        detection_msg.header.frame_id = self.frame_id_color
        self.detection_pub.publish(detection_msg)

    def _frame_stamp(self, stream: str, fallback_stamp):
        """为指定流选择帧时间戳。

        ``stream`` 为 ``"color"`` 或 ``"depth"``；``fallback_stamp`` 是本轮取帧的接收时刻。
        策略由 ``select_frame_timestamp_ns`` 决定：使用仿真时钟时硬件时间戳与仿真时钟不可比，
        必须改用接收时刻；否则优先采用相机硬件时间戳（更接近真实曝光时刻），拿不到才回退。
        返回 ROS 时间消息；走回退分支时直接返回传入的 ``fallback_stamp``，避免重复构造。
        """
        camera_timestamp_ns = None
        getter = getattr(self.camera, "get_frame_timestamp_ns", None)
        if callable(getter):
            camera_timestamp_ns = getter(stream)
        # 把 ROS 时间消息摊平成纳秒整数，便于与相机硬件时间戳比较。
        receipt_timestamp_ns = (
            int(fallback_stamp.sec) * 1_000_000_000 + int(fallback_stamp.nanosec)
        )
        timestamp_ns = select_frame_timestamp_ns(
            camera_timestamp_ns,
            receipt_timestamp_ns,
            use_sim_time=self.ros_use_sim_time,
        )
        if timestamp_ns == receipt_timestamp_ns:
            return fallback_stamp
        return Time(nanoseconds=timestamp_ns).to_msg()

    def _camera_info_payload(self, stream: str, image):
        """仅返回与当前图像尺寸一致的驱动内参；缺失或无效时返回 ``None``。"""
        getter = getattr(self.camera, "get_camera_info", None)
        if callable(getter):
            try:
                camera_info = getter(stream)
            except TypeError:
                camera_info = getter()
            height, width = image.shape[:2]
            if isinstance(camera_info, dict) and calibrated_intrinsics(
                camera_info, width=width, height=height
            ):
                return camera_info
        return None

    def destroy_node(self):
        """释放资源：先关相机与预览窗口，再交给父类销毁节点。

        ``finally`` 保证相机关闭即使抛异常也会执行父类清理；重复调用安全
        （``_preview_window_created`` 在销毁后置 False）。
        """
        try:
            self.camera.close()
            if self._preview_window_created:
                cv2.destroyWindow(self.preview_window_name)
                self._preview_window_created = False
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """进程入口：初始化 ROS、创建节点、自旋，退出时保证节点与上下文被清理。"""
    rclpy.init(args=args)
    node = RebotArmVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 无论是正常退出（Ctrl+C）还是启动/自旋异常，都要释放相机；仅在上下文仍有效时关闭，
        # 避免重复 shutdown 报错。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
