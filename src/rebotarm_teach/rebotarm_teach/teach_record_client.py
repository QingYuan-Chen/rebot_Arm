"""示教录制服务客户端：把"设置路径 / 开始 / 停止"三个动作编排成一次调用。

本模块是示教包提供给上层操作界面（状态面板）的适配层，本身不实现录制逻辑，
也不访问硬件，只按固定顺序调用由构造函数注入的服务客户端：

    开始：设置记录路径（可选） -> 启动重力补偿 -> 启动录制
    停止：停止录制 -> 停止重力补偿

所有客户端都要求是 ROS 服务客户端，返回值为可直接放进状态 payload 的字典；
字典里的 ``state`` / ``accepted`` 等键名是对外接口，不要改动。
"""

from __future__ import annotations

import time
from typing import Any, Callable

from .service_call_helpers import call_trigger_service


class TeachRecordClient:
    """示教录制服务编排器，供上层操作界面调用。

    构造时注入全部服务客户端与"设置路径"请求工厂；实例无状态，
    每次调用都重新与服务交互，因此不保存上一次的录制路径或结果。
    """

    def __init__(
        self,
        *,
        set_path_client: Any,
        start_client: Any,
        stop_client: Any,
        gravity_start_client: Any,
        gravity_stop_client: Any,
        record_path_request_factory: Callable[[], Any],
    ) -> None:
        self._set_path_client = set_path_client
        self._start_client = start_client
        self._stop_client = stop_client
        self._gravity_start_client = gravity_start_client
        self._gravity_stop_client = gravity_stop_client
        # 请求工厂用于延迟构造设置路径请求对象（避免本模块直接依赖消息类型）。
        self._record_path_request_factory = record_path_request_factory

    def start(self, payload: dict | None = None) -> dict:
        """按顺序启动重力补偿与录制，返回状态 payload。

        ``payload["record_path"]`` 非空时先设置记录路径；该步失败会立即返回
        ``state="blocked"``，不会继续启动录制。重力补偿已处于运行状态时其服务会返回
        含 "already" 的消息，这属于正常情况，不阻塞录制启动。
        """
        payload = payload or {}
        requested_path = str(payload.get("record_path", "")).strip()
        normalized_path = ""
        if requested_path:
            set_path_ok, set_path_message, normalized_path = self._set_record_path(requested_path)
            if not set_path_ok:
                return {
                    "accepted": False,
                    "state": "blocked",
                    "message": f"record path: {set_path_message}",
                    "record_path": normalized_path or requested_path,
                }
        # 先开重力补偿，操作者才能在示教时自由拖动机械臂；即使在录制节点侧已自动开启，
        # 这里重复调用也只会得到 already 类消息，不影响后续流程。
        gravity_ok, gravity_message = call_trigger_service(
            self._gravity_start_client,
            timeout_sec=2.0,
        )
        gravity_already_active = "already" in gravity_message.lower()
        if not gravity_ok and not gravity_already_active:
            # 不得在重力补偿未建立时启动录制：否则录制节点会保持 active，
            # 但因等待 GRAVITY_COMP 持续发布 waiting，界面会显示“停止后又恢复录制”。
            return {
                "accepted": False,
                "state": "blocked",
                "message": f"gravity: {gravity_message or gravity_ok}; record: not started",
                "gravity_started": gravity_ok,
                "record_started": False,
                "record_path": normalized_path or requested_path,
            }
        record_ok, record_message = call_trigger_service(
            self._start_client,
            timeout_sec=2.0,
        )
        # 放行条件：录制服务必须成功；重力补偿成功或返回 already（已在运行）均可。
        accepted = record_ok
        return {
            "accepted": accepted,
            "state": "starting" if accepted else "blocked",
            "message": f"gravity: {gravity_message or gravity_ok}; record: {record_message or record_ok}",
            "gravity_started": gravity_ok,
            "record_started": record_ok,
            "record_path": normalized_path or requested_path,
        }

    def stop(self) -> dict:
        """先停录制再停重力补偿（顺序反了会让机械臂在无补偿状态下突然下坠）。

        放行结论只取决于录制服务：录制停止失败即 ``state="failed"``，
        重力补偿停止失败只体现在返回字段里，不影响 ``accepted``。
        """
        record_ok, record_message = call_trigger_service(
            self._stop_client,
            timeout_sec=2.0,
        )
        gravity_ok, gravity_message = call_trigger_service(
            self._gravity_stop_client,
            timeout_sec=2.0,
        )
        return {
            "accepted": record_ok,
            "state": "stopped" if record_ok else "failed",
            "message": f"record: {record_message or record_ok}; gravity: {gravity_message or gravity_ok}",
            "record_stopped": record_ok,
            "gravity_stopped": gravity_ok,
        }

    def _set_record_path(self, requested_path: str) -> tuple[bool, str, str]:
        """设置记录文件路径，返回 ``(是否成功, 消息, 服务端规范化后的路径)``。

        任何失败（服务不可用、超时、异常）都返回 False，不向上抛异常；
        超时上限硬编码为 2.0 s，轮询间隔 20 ms。
        """
        try:
            # 先等 0.5 s 服务发现；找不到就当作不可用，交给上层报 blocked。
            if not self._set_path_client.wait_for_service(timeout_sec=0.5):
                return False, "record path service unavailable", ""
            request = self._record_path_request_factory()
            request.record_path = requested_path
            future = self._set_path_client.call_async(request)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not future.done():
                time.sleep(0.02)
            if not future.done():
                return False, "record path service timeout", ""
            response = future.result()
            return (
                bool(getattr(response, "success", False)),
                str(getattr(response, "message", "")),
                str(getattr(response, "normalized_path", "")),
            )
        except Exception as exc:
            return False, str(exc), ""
