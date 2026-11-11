"""遥操作节点调用控制器 Trigger 服务的同步辅助。

控制器侧把「停止轨迹」「使能/失能」等动作暴露为 ``std_srvs`` 的 Trigger 服务，
本模块提供在 ROS 回调/定时器线程里同步等待其结果的统一封装。
"""

from __future__ import annotations

import time

from std_srvs.srv import Trigger


def call_trigger_service(client, *, timeout_sec: float) -> tuple[bool, str]:
    """同步调用一次 Trigger 服务，返回 ``(success, message)``。

    参数 ``client``：已由调用方创建好的服务客户端（本函数不创建、不销毁它）；
    ``timeout_sec``：总超时时间（秒）。语义分两段：
    - 等待服务端可用最多 ``min(timeout_sec, 0.5)`` 秒——服务端没起来时快速失败，
      避免操作者按下停止后长时间无响应；
    - 发出请求后再等到 ``timeout_sec`` 截止，期间每 20 ms 轮询一次 future，
      保证调用方（通常是单线程 executor 的一个回调）不会被永久阻塞。
    任何异常都被转换成 ``(False, 异常文本)`` 返回，绝不向调用方抛出。
    """
    try:
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
