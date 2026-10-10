"""视觉抓取执行器的 ROS 服务输入输出。

流程编排和安全决策由节点及运行时负责；本模块仅负责
客户端可用性检查、异步结果等待和请求分发。
"""

from __future__ import annotations

import time
import threading
from collections.abc import Callable

import rclpy
from std_srvs.srv import Trigger


class VisualGraspIoGateway:
    def __init__(self, *, logger, clients: dict[str, object], cancelled: Callable[[], bool], legacy_wait=None, on_timeout=None, monotonic=time.monotonic, sleep=time.sleep, context_ok=rclpy.ok):
        self.logger = logger
        self.clients = clients
        self.cancelled = cancelled
        self.legacy_wait = legacy_wait
        self.on_timeout = on_timeout
        self.monotonic = monotonic
        self.sleep = sleep
        self.context_ok = context_ok
        self._stop_lock = threading.RLock()
        self._pending_calls = []
        self._stop_calls = []
        self._stop_by_name = {}
        self._confirmation = None
        self._stop_pending = False
        self._stop_started = 0.0
        self._stop_failure = ""

    def wait(self, future, timeout_sec: float) -> bool:
        if self.legacy_wait is not None:
            return bool(self.legacy_wait(future, timeout_sec))
        deadline = self.monotonic() + max(0.0, float(timeout_sec))
        while self.context_ok() and not future.done() and self.monotonic() < deadline:
            if self.cancelled():
                return False
            self.sleep(0.02)
        return self.context_ok() and not self.cancelled() and future.done()

    @staticmethod
    def _response_received(future):
        if not future.done():
            return False
        try:
            return future.result() is not None
        except Exception:
            return False

    def call(self, name: str, request, timeout_sec: float):
        client = self.clients[name]
        if client is None:
            return None, f"{name} service unavailable"
        deadline = self.monotonic() + max(0.0, timeout_sec)
        if hasattr(client, "wait_for_service"):
            while not client.wait_for_service(timeout_sec=min(.05, max(0.0, deadline - self.monotonic()))):
                if self.cancelled():
                    return None, f"{name} service call stopped"
                if not self.context_ok() or self.monotonic() >= deadline:
                    return None, f"{name} service unavailable"
        if self.cancelled() and self.legacy_wait is None:
            return None, f"{name} service call stopped"
        with self._stop_lock:
            if self._stop_pending:
                return None, "motion stop not confirmed"
            future = client.call_async(request)
            self._pending_calls = [f for f in self._pending_calls if not self._response_received(f)]
            self._pending_calls.append(future)
        if not self.wait(future, timeout_sec):
            # 继续跟踪远端请求；取消本地 Future 不会取消 ROS 服务，
            # 反而会让延迟发生的执行失去跟踪。
            if self.cancelled():
                return None, f"{name} service call stopped"
            if self.on_timeout is not None:
                self.on_timeout(name)
            return None, f"{name} service call timed out"
        result = future.result()
        if result is None:
            return None, f"{name} service returned no result"
        return result, ""

    @property
    def stop_pending(self):
        with self._stop_lock:
            return self._stop_pending

    def stop(self, name: str) -> None:
        with self._stop_lock:
            if self._stop_failure:
                self._stop_calls = [f for f in self._stop_calls if not f.done()]
            self._stop_pending = True
            self._stop_started = self.monotonic()
            self._stop_failure = ""
            # 每次停止（包括失败后的清理）都会使此前的停止证明失效。
            if self._confirmation is not None and not self._confirmation.done():
                self._stop_calls.append(self._confirmation)
            self._confirmation = None
            try:
                client = self.clients[name]
                prior = self._stop_by_name.get(name)
                if prior is not None and not prior.done():
                    return
                if client.wait_for_service(timeout_sec=0.2):
                    future = client.call_async(Trigger.Request())
                    self._stop_by_name[name] = future
                    self._stop_calls.append(future)
            except Exception as exc:
                if self.logger is not None:
                    self.logger.warn(f"failed to request {name}: {exc}")

    def poll_stop_confirmation(self, timeout_sec=15.0):
        """仅在视觉工作线程退出后调用，不阻塞执行器。

        等待所有旧远端请求结束，再向运动层获取新的停止完成证明。
        这样可避免 ExecutePose 请求晚于此前的停止请求到达运动层所造成的竞态。
        若仍有请求未返回，则继续禁止接收新的执行任务。
        """
        with self._stop_lock:
            if not self._stop_pending:
                return None, ""
            if self._stop_failure:
                return False, self._stop_failure
            if self.monotonic()-self._stop_started > timeout_sec:
                self._stop_failure = "stop confirmation timeout; check downstream services and feedback, then retry stop"
                return False, self._stop_failure
            try:
                pending = self._pending_calls + self._stop_calls
                if any(not f.done() for f in pending):
                    return None, "waiting for old service requests to finish"
                if any(f.cancelled() or f.exception() is not None or f.result() is None for f in pending):
                    self._stop_failure = "old service result unknown; stop remains unconfirmed"
                    return False, self._stop_failure
                for future in self._stop_calls:
                    result = future.result()
                    if result is None or not result.success:
                        self._stop_failure = getattr(result, 'message', 'stop returned no response')
                        return False, self._stop_failure
                if self._confirmation is None:
                    client = self.clients['motion_stop']
                    if not client.wait_for_service(timeout_sec=0.0):
                        return None, "motion stop service unavailable"
                    self._confirmation = client.call_async(Trigger.Request())
                    return None, "confirming motion stop"
                if not self._confirmation.done():
                    return None, "confirming motion stop"
                result = self._confirmation.result()
                if result is None or not result.success:
                    self._stop_failure = getattr(result, 'message', 'motion stop returned no result')
                    return False, self._stop_failure
                self._stop_pending = False
                self._pending_calls.clear()
                self._stop_calls.clear()
                self._confirmation = None
                return True, str(result.message)
            except Exception as exc:
                self._stop_failure = f"stop confirmation failed: {exc}"
                return False, self._stop_failure


def io_gateway_for(node) -> VisualGraspIoGateway:
    from .visual_grasp_state import _state_for as state_for

    gateway = getattr(node, "_io_gateway", None)
    if gateway is None:
        clients = {
            "execute_pose": getattr(node, "_execute_pose_client", None),
            "publish_preview": getattr(node, "_publish_preview_client", None),
            "motion_stop": getattr(node, "_motion_stop_client", None),
            "trajectory_stop": getattr(node, "_trajectory_stop_client", None),
            "safe_home": getattr(node, "_safe_home_client", None),
            "gripper": getattr(node, "_gripper_client", None),
            "gripper_grasp": getattr(node, "_gripper_grasp_client", None),
        }
        gateway = VisualGraspIoGateway(
            logger=node.get_logger() if hasattr(node, "get_logger") else None,
            clients=clients,
            cancelled=lambda: not state_for(node).is_running(),
            legacy_wait=getattr(node, "_wait_for_future", None),
            on_timeout=lambda name: state_for(node).mark_aborting(f"{name} timed out"),
        )
        node._io_gateway = gateway
    return gateway
