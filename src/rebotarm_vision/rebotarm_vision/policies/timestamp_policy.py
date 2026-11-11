"""视觉帧时间戳选择策略（纯函数，不接触时钟与相机）。

职责：为一帧彩色/深度图选定"节点时钟域"下的时间戳。相机硬件时间戳更接近真实曝光
时刻，但它与仿真时钟不可比；因此在仿真时间组合里必须改用收帧时刻，否则下游的
"消息新鲜度"判断会把所有帧都误判为过期（或永远新鲜）。
"""

from __future__ import annotations


def select_frame_timestamp_ns(
    camera_timestamp_ns: int | None,
    receipt_timestamp_ns: int,
    *,
    use_sim_time: bool,
) -> int:
    """在节点时钟域内为一帧选择时间戳。

    参数（时间戳单位统一为纳秒，整数）：
    - ``camera_timestamp_ns``：相机驱动给的硬件时间戳（通常来自设备系统时钟）；
      ``None`` 表示该驱动/该流没有提供时间戳。
    - ``receipt_timestamp_ns``：本节点收到这帧时的时钟读数，取自当前生效的 ROS 时钟。
    - ``use_sim_time``：节点是否使用仿真时钟。为真时硬件时间戳与仿真时钟不同源、
      不可比较，必须丢弃。

    返回：选定的时间戳（ns）。

    决策顺序：
    1. ``use_sim_time`` 为真 → 一律返回 ``receipt_timestamp_ns``，让时间戳落在仿真
       时钟域内，新鲜度检查才有意义；
    2. 否则优先返回正值相机硬件时间戳（更接近真实曝光时刻）；
    3. 相机时间戳缺失（``None``）或非正（0/负值通常是设备未填充的哨兵值）
       → 回退到 ``receipt_timestamp_ns``。

    说明：硬件系统时间戳只在墙钟（非仿真）ROS 时间下可用，其余情况退回收帧时刻是
    有意为之，不是精度退化；调用方可以据此在"返回收帧时刻"时复用已有时间消息。
    """

    if use_sim_time:
        return int(receipt_timestamp_ns)
    if camera_timestamp_ns is not None and int(camera_timestamp_ns) > 0:
        return int(camera_timestamp_ns)
    return int(receipt_timestamp_ns)
