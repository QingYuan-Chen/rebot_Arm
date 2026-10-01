"""消息时间戳新鲜度判定工具。

用途：拒绝「时间戳未设置、过旧或来自未来」的感知/规划消息，避免用陈旧数据
驱动真实机械臂。所有函数都是纯函数，当前时刻由调用方以纳秒传入，因此同时
适用于系统时钟与仿真时钟。
"""

from __future__ import annotations


def stamp_to_ns(stamp) -> int | None:
    """把 ROS 风格时间戳（sec + nanosec）换算成纳秒。

    stamp 为 None，或 sec 与 nanosec 同时为 0（约定表示「未设置」）时返回 None；
    注意 sec=0、nanosec=0 的真实时刻无法与「未设置」区分，一律按未设置处理。
    1_000_000_000 是秒 → 纳秒的换算系数。
    """

    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec == 0 and nanosec == 0:
        return None
    return sec * 1_000_000_000 + nanosec


def message_age_sec(stamp, *, now_ns: int) -> float | None:
    """返回消息年龄（秒）= 当前时刻 - 时间戳；时间戳未设置时返回 None。

    返回值可能为负（时间戳来自未来），调用方需自行决定容忍范围。
    """

    stamp_ns = stamp_to_ns(stamp)
    if stamp_ns is None:
        return None
    return (int(now_ns) - stamp_ns) / 1_000_000_000.0


def is_message_fresh(
    stamp,
    *,
    now_ns: int,
    max_age_sec: float,
    max_future_sec: float = 0.25,
) -> bool:
    """判断消息年龄是否落在允许区间 [-max_future_sec, max(max_age_sec, 0)]。

    参数：
        stamp: 消息时间戳。
        now_ns: 当前时刻，单位纳秒（必须与时间戳在同一时钟域）。
        max_age_sec: 允许的最大年龄，单位秒；负值按 0 处理，即不放宽过去侧。
        max_future_sec: 允许的「未来」容差，单位秒，默认 0.25，用于吸收时钟
            抖动或轻微回跳；取绝对值，负数也按正数处理。

    返回：
        True 表示新鲜可用；时间戳未设置（无法判定）时返回 False，视为不可用。
    """

    age_sec = message_age_sec(stamp, now_ns=now_ns)
    if age_sec is None:
        return False
    return -abs(float(max_future_sec)) <= age_sec <= max(float(max_age_sec), 0.0)
