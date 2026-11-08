"""Trigger 服务调用小工具：同步等待结果，失败不抛异常只返回 ``(False, 原因)``。

供示教录制/回放客户端复用：这些调用发生在 UI 请求路径上，宁可快速失败并回传
原因字符串，也不能因为服务缺失或超时把异常抛给上层。
"""

from __future__ import annotations

import time

from std_srvs.srv import Trigger


def call_trigger_service(client, *, timeout_sec: float) -> tuple[bool, str]:
    """调用 ``Trigger`` 服务，返回 ``(是否成功, 消息)``。

    等待分为两段：服务发现最多等 ``min(timeout_sec, 0.5)`` 秒；随后按 20 ms 间隔
    轮询 future，最多再等 ``timeout_sec`` 秒（两段相加是实际最坏耗时）。
    服务不可用或超时分别返回 ``"service unavailable"`` / ``"service timeout"``，
    调用异常返回 False 并把异常文本原样作为消息回传。
    """
    try:
        # 服务发现与调用共用同一个超时预算：发现阶段最多等 0.5 s，避免长时间卡住 UI。
        if not client.wait_for_service(timeout_sec=min(float(timeout_sec), 0.5)):
            return False, "service unavailable"
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        if not future.done():
            return False, "service timeout"
        response = future.result()
        return bool(response.success), str(response.message)
    except Exception as exc:
        return False, str(exc)
