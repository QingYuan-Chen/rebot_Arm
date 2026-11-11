"""目标检测器封装：对 ultralytics 的 YOLO 模型做一层薄包装。

职责边界：本类只负责「加载模型 + 单帧推理 + 汇报开放词表状态」，不做节点通信、
不做深度融合或坐标变换、不做抓取判定。节点侧拿到 ``infer`` 的原始结果后，
再交给转换器组装成检测消息。

两种工作模式：
    - 闭集模式：普通 COCO 权重，类别集合由权重自身决定；
    - 开放词表模式：World/YOLOE 权重，需要文本提示（自定义类别名）。提示设置
      设置失败时在初始化阶段报告错误。
"""

from __future__ import annotations

from typing import Sequence

from ultralytics import YOLO


class YoloDetector:
    """YOLO 推理封装：构造时加载权重，之后可反复调用 ``infer``。"""

    def __init__(
        self,
        model_path: str,
        device: str,
        conf_threshold: float,
        iou_threshold: float,
        use_world: bool,
        custom_classes: Sequence[str],
    ) -> None:
        """加载权重并尝试启用开放词表。

        参数：
            model_path: 权重文件路径（本地 ``.pt``）。文件名含 world/yoloe 时
                会被视为开放词表的显式开关，见下方注释。
            device: 推理设备字符串，原样透传给推理后端，如 "cpu"、"cuda:0"。
            conf_threshold: 置信度阈值 [0, 1]，低于它的检测框被丢弃；调高更保守。
            iou_threshold: 非极大值抑制的 IoU 阈值 [0, 1]，越小越激进地抑制重叠框。
            use_world: 启动参数显式要求开放词表模式。
            custom_classes: 开放词表的文本提示类别名序列；为空表示不启用。
        """

        self._model = YOLO(model_path)
        self._device = device
        self._conf_threshold = conf_threshold
        self._iou_threshold = iou_threshold
        self._open_vocab_enabled = False
        # World/YOLOE 权重依赖文本提示才能工作。这里同时把「权重身份」当作显式
        # 开关，这样即使启动文件的参数替换把 use_world 悄悄置假，开放词表配置也
        # 不会被静默降级；而普通 COCO 权重的行为保持完全不变。
        world_checkpoint = "world" in model_path.lower() or "yoloe" in model_path.lower()
        if use_world and not custom_classes:
            raise ValueError("open-vocabulary mode requires custom classes")
        if (use_world or world_checkpoint) and custom_classes:
            self._model.set_classes(list(custom_classes))
            self._open_vocab_enabled = True

    def infer(self, image_bgr):
        """对单帧 BGR 图像推理，返回后端的原始结果列表（框/掩膜/置信度等）。

        每帧都显式关闭 verbose，避免推理库自身的逐帧日志淹没节点日志。
        """

        return self._model.predict(
            image_bgr,
            verbose=False,
            device=self._device,
            conf=self._conf_threshold,
            iou=self._iou_threshold,
        )

    # 开放词表是否真正生效（set_classes 调用成功）；供节点打印启动诊断。
    @property
    def open_vocab_enabled(self) -> bool:
        return self._open_vocab_enabled
