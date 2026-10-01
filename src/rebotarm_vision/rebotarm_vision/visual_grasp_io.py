"""ROS service I/O for the visual grasp executor.

The workflow and safety decisions stay in the node/runtime; this module owns
client availability, future waiting, and request dispatch only.
"""

from __future__ import annotations

import time
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

    def wait(self, future, timeout_sec: float) -> bool:
        if not hasattr(future, "done") and self.legacy_wait is not None:
            return bool(self.legacy_wait(future, timeout_sec))
        deadline = self.monotonic() + max(0.0, float(timeout_sec))
        while self.context_ok() and not future.done() and self.monotonic() < deadline:
            if self.cancelled():
                return False
            self.sleep(0.02)
        return self.context_ok() and not self.cancelled() and future.done()

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
        future = client.call_async(request)
        if not self.wait(future, timeout_sec):
            if hasattr(future, "cancel"):
                future.cancel()
            if self.cancelled():
                return None, f"{name} service call stopped"
            if self.on_timeout is not None:
                self.on_timeout(name)
            return None, f"{name} service call timed out"
        result = future.result()
        if result is None:
            return None, f"{name} service returned no result"
        return result, ""

    def stop(self, name: str) -> None:
        try:
            client = self.clients[name]
            if client.wait_for_service(timeout_sec=0.2):
                client.call_async(Trigger.Request())
        except Exception as exc:
            # A best-effort stop boundary must attempt the other stop channel too.
            if self.logger is not None:
                self.logger.warn(f"failed to request {name}: {exc}")


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
