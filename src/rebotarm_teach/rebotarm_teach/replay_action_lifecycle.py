"""FollowJointTrajectory callback lifecycle for teach replay."""

from __future__ import annotations

import time
from concurrent.futures import Future
from typing import Any

from trajectory_msgs.msg import JointTrajectory


class ReplayActionLifecycleMixin:
    def _on_teach_replay_goal_response(
        self, future: Future[Any], info_payload: dict, points: int, trajectory: JointTrajectory
    ) -> None:
        """目标响应回调：登记活动回放并发布 ``replaying`` 状态，随后挂结果回调。

        只有目标被接受才登记句柄、活动轨迹与单调起始时刻，并复位运行时监控器；
        被拒绝或异常只发状态、不登记（避免 stop/监控操作一个不存在的目标）。
        """
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status("replay", {"state": "rejected", "message": "teach replay goal rejected"})
            return
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = goal_handle
            self._active_teach_replay_trajectory = trajectory
            # 用单调时钟记录起点，供跟踪监控计算"已回放多久"，不受系统时间调整影响。
            self._active_teach_replay_started_at = time.monotonic()
            self._replay_runtime_monitor.reset()
        self._publish_status(
            "replay",
            {
                "state": "replaying",
                "message": "teach replay goal accepted",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                # 回放进行中就把本次监控阈值一起下发，界面可据此显示判据；
                # 判定与停止只发生在 check_tracking 中。
                "runtime_monitor": {
                    "enabled": bool(self.get_parameter("replay_monitor_enabled").value),
                    "max_tracking_error_rad": float(self.get_parameter("max_tracking_error_rad").value),
                    "max_live_velocity_rad_s": float(self.get_parameter("max_live_velocity_rad_s").value),
                },
                "dry_run": False,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_teach_replay_result(fut, info_payload, points))

    def _on_teach_replay_cancel_response(self, future: Future[Any]) -> None:
        """取消响应回调：区分"取消已受理"与"取消前目标已结束"两种情况。"""
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._publish_status("replay", {"state": state, "message": message})
        if not goals_canceling:
            # 没有目标被取消说明回放已自然结束：结果回调可能不会再来，这里兜底清理状态。
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._replay_runtime_monitor.reset()

    def _on_teach_replay_result(
        self, future: Future[Any], info_payload: dict, points: int
    ) -> None:
        """回放结果回调：把动作终态翻译成界面状态，并清理活动回放登记。

        状态码按动作规范解释：``status == 4``（SUCCEEDED）且 ``error_code == 0`` 记 ``done``；
        ``status == 5``（CANCELED）时，如果运行时监控器已请求过停止则记 ``safety_stop``
        （这是我们主动刹停，不是操作员取消），否则记 ``canceled``；其余记 ``failed``。
        """
        previous_replay = self._snapshot().teleop.get("replay", {})
        with self._teach_replay_lock:
            monitor_stop_requested = self._replay_runtime_monitor.stop_requested
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._publish_status("replay", {"state": "failed", "message": str(exc)})
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
                self._active_teach_replay_trajectory = None
                self._active_teach_replay_started_at = None
                self._replay_runtime_monitor.reset()
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "safety_stop" if monitor_stop_requested else "canceled"
        else:
            state = "failed"
        message = f"teach replay result status={status}, error_code={error_code}: {error_string}"
        # 保留进行中发布的监控明细（原因/最差关节/实测误差），便于事后定位安全停止原因。
        runtime_monitor = previous_replay.get("runtime_monitor") if isinstance(previous_replay, dict) else None
        if status == 5 and monitor_stop_requested:
            # 安全停止时用监控器留下的说明覆盖通用结果文本，明确"是监控刹停导致的取消"。
            previous_message = str(previous_replay.get("message", "")) if isinstance(previous_replay, dict) else ""
            message = (
                f"action canceled after runtime monitor stop: {previous_message}"
                if previous_message
                else "action canceled after runtime monitor stop"
            )
        self._publish_status(
            "replay",
            {
                "state": state,
                "message": message,
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "runtime_monitor": runtime_monitor,
                "dry_run": False,
            },
        )
        # 无论成功、失败还是取消，活动回放登记必须清空，否则会导致后续 stop/监控操作
        # 指向已结束的目标。
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = None
            self._active_teach_replay_trajectory = None
            self._active_teach_replay_started_at = None
            self._replay_runtime_monitor.reset()
