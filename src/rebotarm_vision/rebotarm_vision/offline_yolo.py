"""纯软件 RGB YOLO 验证路径使用的纯函数集合.

输入来自外部发布或回放的 ``Image`` 消息,本模块把消息解码成图像数组,并按目标类别
白名单过滤检测结果.图像来源由发布者决定;解码与过滤独立于 ROS 节点生命周期,
可直接用合成消息检查类别门和解码失败行为.

安全约定:任何解码异常一律向上抛(由节点捕获后按 fail-closed 处理,发布空检测);
目标类别白名单为空表示**拒绝所有检测**,而不是放行全部.
"""

from __future__ import annotations

import ast
import json
from typing import Iterable

import numpy as np

# 支持的图像编码 → (通道数, 通道布局).布局名决定解码时如何重排通道;
# 只支持 8 位彩色编码,单色/YUV/浮点等编码会直接抛 ValueError.
_ENCODING_CHANNELS = {
    "rgb8": (3, "rgb"),
    "bgr8": (3, "bgr"),
    "rgba8": (4, "rgba"),
    "bgra8": (4, "bgra"),
}


def parse_target_classes(value) -> tuple[str, ...]:
    """把 ROS 参数解析成规范化(小写,去多余空白)的类别名元组.

    启动文件常把列表写成"看起来像 YAML 的字符串",因此这里依次尝试 JSON 与 Python
    字面量解析,两者都失败时退回按逗号切分.``None`` 或空字符串返回空元组-注意空元组
    的语义是**拒绝所有检测**(fail-closed),不是"不过滤".

    非序列,非字符串的输入(例如字典,数字)抛 ``ValueError``.
    """

    if value is None:
        return ()
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return ()
        parsed = None
        # 先按 JSON 解析(启动参数里常见 ['cube'] 这种写法),再退到 Python 字面量;
        # 只有解析出列表/元组/集合才采纳,否则继续尝试下一种解析方式.
        for parser in (json.loads, ast.literal_eval):
            try:
                parsed = parser(text)
            except (ValueError, SyntaxError, TypeError, json.JSONDecodeError):
                continue
            if isinstance(parsed, (list, tuple, set)):
                value = parsed
                break
        else:
            value = text.split(",")
    if isinstance(value, (str, bytes)):
        value = [value]
    try:
        names = tuple(_normalize_class_name(item) for item in value)
    except TypeError as exc:
        raise ValueError("target_classes must be a sequence or comma-separated string") from exc
    return tuple(name for name in names if name)


def _normalize_class_name(value) -> str:
    """把类别名规范成可比较的形式:去首尾空白,内部连续空白折叠为一个空格,转小写.

    因此 ``" Bottle "``,``"bottle"``,``"BOTTLE"`` 会被视为同一个类别,与模型输出的
    大小写差异无关.
    """

    return " ".join(str(value).strip().casefold().split())


def decode_image_to_bgr(image_msg) -> np.ndarray:
    """把 8 位彩色图像消息解码为连续内存的 BGR 数组.

    图像消息的 ``step``(每行字节数)可以大于 ``width * channels``(行末有填充字节),
    所以先按 ``(height, step)`` 还原再切掉填充,不能直接按 ``width`` reshape.数据通过
    memoryview 读取以避免多余拷贝,最后用 ``np.ascontiguousarray`` 保证结果连续,供
    OpenCV/推理后端直接使用.

    返回 ``(height, width, 3)`` 的 uint8 BGR 数组.以下情况抛 ``ValueError``:编码不在
    支持列表,宽高非正,``step`` 小于单行实际字节数,数据长度不足.
    """

    encoding = str(getattr(image_msg, "encoding", "")).strip().casefold()
    try:
        channels, layout = _ENCODING_CHANNELS[encoding]
    except KeyError as exc:
        raise ValueError(
            f"offline YOLO supports rgb8/bgr8/rgba8/bgra8, got {encoding!r}"
        ) from exc

    height = int(getattr(image_msg, "height", 0))
    width = int(getattr(image_msg, "width", 0))
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    row_bytes = width * channels
    # step 缺失或为 0 时按无填充处理;小于单行字节数则数据必然错位.
    step = int(getattr(image_msg, "step", 0) or row_bytes)
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than row width {row_bytes}")
    payload = memoryview(getattr(image_msg, "data", b""))
    required = step * height
    if len(payload) < required:
        raise ValueError(f"image payload has {len(payload)} bytes, needs at least {required}")

    rows = np.frombuffer(payload[:required], dtype=np.uint8).reshape(height, step)
    pixels = rows[:, :row_bytes].reshape(height, width, channels)
    # 统一转成 BGR(OpenCV 默认通道序):宽高各用一步切片完成通道反转.
    if layout == "rgb":
        bgr = pixels[..., ::-1]
    elif layout == "bgr":
        bgr = pixels
    elif layout == "rgba":
        bgr = pixels[..., 2::-1]
    else:  # bgra
        bgr = pixels[..., :3]
    return np.ascontiguousarray(bgr)


def filter_detection_array(
    detection_msg,
    target_classes: Iterable[str],
):
    """只保留 ``class_name`` 在白名单内的检测(显式 allow-list).

    白名单外的类别一律丢弃;白名单为空则输出空检测-这是刻意的 fail-closed 行为,
    防止配置漏填导致任意目标被抓取.返回与入参同类型的消息对象,``header`` 原样复制以
    保留时间戳与坐标系;对缺少 ``detections`` 字段的消息类型做了兼容处理.
    """

    allowed = {_normalize_class_name(name) for name in target_classes if str(name).strip()}
    filtered = type(detection_msg)()
    filtered.header = detection_msg.header
    if not hasattr(filtered, "detections"):
        filtered.detections = []
    for detection in detection_msg.detections:
        if _normalize_class_name(getattr(detection, "class_name", "")) in allowed:
            filtered.detections.append(detection)
    return filtered
