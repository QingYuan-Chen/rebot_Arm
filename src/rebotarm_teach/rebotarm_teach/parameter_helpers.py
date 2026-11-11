"""节点参数/QoS 小工具:集中放置示教节点创建订阅时使用的 QoS 覆盖项."""

from __future__ import annotations


def sensor_qos_kwargs() -> dict[str, object]:
    """返回关节反馈等传感器类订阅的 QoS 参数(调用方目前只读取 ``depth``).

    - ``depth=10``:KEEP_LAST 队列长度 10 帧,容忍回调短暂滞后而不丢帧堆积.
    - ``reliability="best_effort"``:与调用方实际采用的可靠性等级一致(允许丢帧,
      不重传历史帧),这里只是把该约定写在一处;调用方构造 QoSProfile 时仍显式指定
      BEST_EFFORT,因此改动本字典的该键不会直接改变运行行为.
    """
    return {
        "depth": 10,
        "reliability": "best_effort",
    }
