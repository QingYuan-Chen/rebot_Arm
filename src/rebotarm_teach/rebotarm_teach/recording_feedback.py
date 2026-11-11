"""从"新的且时间对齐"的反馈中构造示教样本。

示教数据的唯一来源是硬件反馈：关节状态提供位置/速度/力矩，电机状态提供健康码，
两者必须属于同一反馈批次，否则本模块拒绝该帧。返回 ``None`` 表示"这帧不算数但不算错误"
（重复帧或录制开始前的旧帧）；抛出 ``ValueError`` 表示反馈本身不可信，调用方应当等待下一帧。
"""

import math
from collections.abc import Sequence

from .teach_recording import TeachSample


def stamp_nanoseconds(stamp) -> int:
    """把 ROS 时间戳（``sec`` + ``nanosec``）折算成整数纳秒，便于做同批次相等比较。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ordered_feedback_vector(
    values: Sequence[float],
    *,
    source_names: Sequence[str],
    source_indices: Sequence[int],
) -> tuple[float, ...]:
    """按目标关节顺序重排反馈向量，并拒绝无效数值。

    该函数只处理已经完成关节名匹配的数值向量，因此不依赖 ROS 消息，
    可单独测试。``source_indices`` 通常由 ``feedback_teach_sample`` 根据
    ``joint_names`` 计算得到。
    """
    if len(values) != len(source_names):
        raise ValueError("joint feedback vector length mismatch")
    result = tuple(float(values[index]) for index in source_indices)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("joint feedback contains non-finite values")
    return result


def feedback_teach_sample(
    msg, *, joint_names, motor_status, motor_stamps, arm_state,
    now_ns: int, started_ns: int, last_stamp_ns: int | None,
    timeout_sec: float, require_motor_status: bool,
) -> TeachSample | None:
    """把一帧关节状态反馈转换成 ``TeachSample``，不合格时返回 ``None`` 或抛异常。

    参数：

    - ``msg``：关节状态消息，需含 ``header.stamp`` / ``name`` / ``position``，
      可选 ``velocity`` / ``effort``。
    - ``joint_names``：期望的关节及其顺序；输出向量按该顺序重排（与消息内的顺序无关）。
    - ``motor_status`` / ``motor_stamps``：``{关节名: 状态码}`` 与 ``{关节名: 时间戳(ns)}``，
      仅在 ``require_motor_status`` 为真时参与校验。
    - ``now_ns`` / ``started_ns``：当前时刻与录制开始时刻（ns，同一时钟源）。
    - ``last_stamp_ns``：上一已写入样本的时间戳（ns），用于去重；首帧传 ``None``。
    - ``timeout_sec``：允许的反馈时延上限（s）；``require_motor_status`` 为真时还要求
      每个关节都有同一批次的电机状态码，且码值只能是 0（未使能）或 1（已使能）。

    返回样本的 ``stamp`` 是相对录制起点的秒数（``(stamp - started_ns) / 1e9``）。

    ``None``：时间戳早于录制开始，或不晚于上一已写入样本（重复/乱序帧）。
    ``ValueError``：反馈过期或来自未来、关节名重复或缺失、向量长度不符、
    数值非有限、电机状态缺失或不同批次、状态码未知。
    """
    stamp = stamp_nanoseconds(msg.header.stamp)
    # 录制开始前与已写过的帧直接丢弃：示教采样必须严格单调递增。
    if stamp < started_ns or (last_stamp_ns is not None and stamp <= last_stamp_ns):
        return None
    # 时延窗口：迟到超过 timeout_sec，或时间戳跑到 now_ns 之后（时钟异常）都判为不可信。
    if not 0 <= now_ns - stamp <= int(timeout_sec * 1_000_000_000):
        raise ValueError("joint feedback timestamp is stale or in the future")
    names = tuple(msg.name)
    # 名字重复会导致 index() 取错列；缺关节则说明这帧反馈不完整（例如只发布了部分关节）。
    if len(names) != len(set(names)) or not set(joint_names).issubset(names):
        raise ValueError("joint feedback is incomplete or duplicated")
    if len(msg.position) != len(names):
        raise ValueError("joint feedback position length mismatch")
    # 记录每列在消息中的下标，下面按期望关节顺序重排所有向量。
    indices = [names.index(name) for name in joint_names]

    if require_motor_status:
        # 时间戳必须与关节状态完全一致，才能保证"位置"与"健康状态"属于同一帧反馈。
        if any(motor_stamps.get(name) != stamp for name in joint_names):
            raise ValueError("waiting for motor status from the same feedback batch")
        # 0=未使能，1=已使能；其余值（例如 255）表示未知或故障。
        if any(motor_status.get(name) not in (0, 1) for name in joint_names):
            raise ValueError("motor feedback is unknown or unhealthy")
    return TeachSample(
        stamp=(stamp - started_ns) / 1_000_000_000,
        joint_names=tuple(joint_names),
        positions=ordered_feedback_vector(
            msg.position, source_names=names, source_indices=indices
        ),
        velocities=(
            ordered_feedback_vector(msg.velocity, source_names=names, source_indices=indices)
            if msg.velocity
            else ()
        ),
        efforts=(
            ordered_feedback_vector(msg.effort, source_names=names, source_indices=indices)
            if msg.effort
            else ()
        ),
        # 只保留本次记录关节范围内的状态，避免把夹爪等无关电机带进样本。
        motor_status={name: motor_status[name] for name in joint_names if name in motor_status},
        arm_state=arm_state,
    )
