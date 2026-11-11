"""整臂命令的服务客户端门面：把面板的"使能/失能/回原点"命令落到控制器服务上。

面板节点为每条整臂命令各准备一个 Trigger 服务客户端，本模块负责按统一顺序调用它们：
    1. 先用命令白名单校验入参，未知命令直接拒绝，不触发任何服务调用；
    2. 对会改变控制权的命令（回原点、失能）先请求停止正在跟踪的轨迹；
    3. 再调用命令对应的服务，并按命令类型选择等待超时。

返回值统一为字典（accepted、state、command、message、trajectory_stop_requested），
面板的状态存储与 HTTP 响应直接使用这个形状。

线程模型
    call_trigger_service 是同步阻塞调用，内部用 future.done() 轮询加 20 ms 小睡；
    当前调用方是 HTTP 请求线程，不能把它放进会长时间占用 ROS 执行器的回调里。
"""

from __future__ import annotations

import time
from typing import Any

from std_srvs.srv import Trigger

from .arm_command_api import (
    arm_command_timeout_sec,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
)


class ArmControlClient:
    """整臂人工命令的服务客户端门面（门面 = 对上层隐藏多次服务调用的顺序与超时细节）。"""

    def __init__(
        self,
        *,
        enable_client: Any,
        disable_client: Any,
        safe_home_client: Any,
        trajectory_stop_client: Any,
    ) -> None:
        """保存四个 Trigger 服务客户端。

        enable_client→使能服务，disable_client→失能服务，safe_home_client→回安全原点服务，
        trajectory_stop_client→轨迹停止服务。字典的键就是 arm_command_api 中定义的三条合法
        命令名，execute 会用规范化后的命令名直接索引，因此键名不能改。
        这里只保存引用，不等待服务就绪——服务可用性在真正调用时判断。
        """
        self._clients = {
            "safe_home": safe_home_client,
            "enable": enable_client,
            "disable": disable_client,
        }
        self._trajectory_stop_client = trajectory_stop_client

    def execute(self, command: object) -> dict:
        """执行一条整臂命令，返回统一结构的结果字典。

        流程与安全语义
            - 未知命令返回 accepted=False、state="rejected"，且不调用任何服务；
            - 需要先停轨迹的命令（回原点、失能）会先调用轨迹停止服务，该调用固定只等 2 s，
              失败也不阻断主命令，但会把停止结果并入 message；
            - 主命令按 arm_command_timeout_sec 给出的超时等待，成功与否取自服务响应的
              success 字段，state 相应为 "done" 或 "failed"。
        返回键：accepted、state、command、message、trajectory_stop_requested。
        本方法不校验机械臂当前是否允许该命令（如回放锁定），那是调用方的职责。
        """
        normalized = normalize_arm_command(command)
        if normalized is None:
            return {
                "accepted": False,
                "state": "rejected",
                "command": str(command or ""),
                "message": "unknown arm command",
            }
        stop_requested = False
        stop_message = ""
        if should_stop_trajectory_before_arm_command(normalized):
            # 固定 2 s：停轨迹只是"让路"，失败不阻止主命令，但结果会附加到 message 里。
            stop_requested, stop_message = self.call_trigger_service(
                self._trajectory_stop_client,
                timeout_sec=2.0,
            )
        ok, message = self.call_trigger_service(
            self._clients[normalized],
            timeout_sec=arm_command_timeout_sec(normalized),
        )
        if stop_message:
            # 把停轨迹的结果拼进消息，便于前端区分"主命令失败"与"停轨迹失败"。
            message = f"{message}; trajectory_stop: {stop_message}" if message else f"trajectory_stop: {stop_message}"
        return {
            "accepted": ok,
            "state": "done" if ok else "failed",
            "command": normalized,
            "message": message or ("done" if ok else "failed"),
            "trajectory_stop_requested": stop_requested,
        }

    @staticmethod
    def call_trigger_service(client: Any, *, timeout_sec: float) -> tuple[bool, str]:
        """同步调用一个 Trigger 服务，返回 (success, message)。

        timeout_sec 是整体时间预算（秒）：先用最多 0.5 s 等待服务出现，服务不可用则立刻
        返回 ("service unavailable")；随后轮询 future，超时返回 ("service timeout")。
        等待服务只占预算的一小部分（min 到 0.5 s），剩余时间留给真正的调用；即便传入 0
        或负值也至少等 100 ms（max 到 0.1 s）。
        轮询间隔 20 ms 是响应速度与 CPU 占用的折中。任何异常都被吞掉并转成失败消息，
        调用方不会因为服务端异常而崩溃。
        """
        try:
            if not client.wait_for_service(timeout_sec=min(timeout_sec, 0.5)):
                return False, "service unavailable"
            # 这些整臂服务都是空请求的 Trigger，响应只有 success 与 message 两个字段。
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + max(float(timeout_sec), 0.1)
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    return bool(response.success), str(response.message)
                time.sleep(0.02)
            return False, "service timeout"
        except Exception as exc:
            return False, str(exc)
