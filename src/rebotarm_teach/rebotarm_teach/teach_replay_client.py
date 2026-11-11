"""示教回放的停止/取消门面：组合目标取消与控制器轨迹停止保持请求。

停止语义有两条路径：

- 有活动回放目标：先请求取消该目标，再用很短超时（0.2 s）尽力请求控制器轨迹停止；
  取消 future 交给调用方挂回调，本模块不等待其完成。
- 没有活动回放目标：不取消任何目标，但仍请求控制器 ``trajectory_stop``（0.8 s 超时），
  用于清除可能的残留运动；成功时同样视为"已请求停止"。

返回值中的 ``cancel_future`` 是内部字段，不能直接放进状态 payload；
``state`` / ``message`` 等键名是对外接口。
"""

from __future__ import annotations

from typing import Any

from .service_call_helpers import call_trigger_service
from .teach_recording import validate_teach_replay_stop_request


class TeachReplayClient:
    """回放停止动作门面：无状态，可被反复调用。"""

    def stop(self, goal_handle: Any | None, *, trajectory_stop_client: Any) -> dict:
        """请求停止回放，返回即时结论（不等待取消真正完成）。

        ``goal_handle`` 为当前活动回放目标句柄，``None`` 表示没有活动目标。
        无活动目标时补发控制器轨迹停止保持请求：服务成功则返回 accepted，
        失败则原样返回门控结论（``state="idle"``）。有活动目标时若取消调用抛异常，
        返回 ``state="failed"``，此时调用方不应认为回放已停止。
        """
        decision = validate_teach_replay_stop_request(goal_handle is not None)
        if not decision.accepted:
            # 没有活动目标也要清残留运动：这是"停止"语义的兜底，不代表门控被放行。
            stop_requested, _message = call_trigger_service(
                trajectory_stop_client,
                timeout_sec=0.8,
            )
            if stop_requested:
                return {
                    "accepted": True,
                    "state": "stop_requested",
                    "message": "controller trajectory_stop requested",
                    "cancel_future": None,
                }
            return {
                "accepted": False,
                "state": decision.state,
                "message": decision.message,
                "cancel_future": None,
            }
        try:
            # 取消是异步的：本方法只发起请求，最终结果由调用方在 future 回调里处理。
            future = goal_handle.cancel_goal_async()
            call_trigger_service(trajectory_stop_client, timeout_sec=0.2)
        except Exception as exc:
            return {
                "accepted": False,
                "state": "failed",
                "message": f"failed to request teach replay cancel: {exc}",
                "cancel_future": None,
            }
        return {
            "accepted": True,
            "state": decision.state,
            "message": decision.message,
            "cancel_future": future,
        }
