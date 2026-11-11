"""Web 命令安全闸门：决定浏览器发来的命令是否允许真正下发到硬件。

这是面板唯一的执行模式判定点。前端请求先被包装成 WebCommandRequest，再由路由方法裁决：
只有 execution_mode 取值恰为 "execute" 时才受理；其它取值（默认 "dry_run"）一律返回
accepted=False、state="dry_run" 的"空跑"结果，不产生任何硬件影响。这样默认启动的面板
只能查看状态，必须由操作者在启动命令里显式打开执行模式才会真的动。

本模块是纯逻辑，不含 ROS 调用与 IO，便于单测；被拒绝/阻断的响应统一由
blocked_legacy_response 生成，保证前端拿到的字段齐全、可直接渲染。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class WebCommandRequest:
    """一条来自浏览器的命令请求（不可变）。

    字段说明
        intent：命令意图名，由面板节点按具体接口填写（如 move_home、stop_robot、
            keyboard_step、teach_replay_execute 等），只用于回显与日志，此处不做校验。
        execution_mode：执行模式，来自面板参数；只有 "execute" 才会放行。
        payload：前端原始请求体（含 confirm 确认串与各项数值参数），原样携带给业务层。
        request_id：前端生成的请求标识，用于把响应与请求对上，可为空串。
    """

    intent: str
    execution_mode: str
    payload: dict[str, Any]
    request_id: str = ""

    @classmethod
    def from_payload(cls, payload: dict[str, Any], *, execution_mode: str) -> "WebCommandRequest":
        """从 HTTP 请求体构建请求对象；缺失字段一律退化为安全的空值。

        execution_mode 会去空白并转小写，因此 "Execute"、" execute " 都能正确识别；
        取不到有效值时退化为 "dry_run"，即最保守的不执行模式。
        intent 与 request_id 缺省为空串；payload 保存的是浅拷贝，
        调用方随后修改原字典不会影响本对象。
        """
        data = dict(payload)
        return cls(
            intent=str(data.get("intent", "") or ""),
            execution_mode=str(execution_mode or "dry_run").strip().lower(),
            payload=data,
            request_id=str(data.get("request_id", "") or ""),
        )


class WebCommandGateway:
    """执行闸门与统一响应构造器；本身无状态，可被多个 HTTP 请求线程共用。"""

    def route(self, request: WebCommandRequest) -> dict[str, Any]:
        """裁决一条请求：非 "execute" 模式返回空跑结果，否则返回"受理"结果。

        本方法只判断执行模式，不校验 payload 内容；state="accepted" 只代表闸门放行，
        真正的业务校验在后续的适配层完成。两种分支返回的字段名保持一致，
        前端只需读 accepted/state/message 即可统一处理。
        """
        if request.execution_mode != "execute":
            return {
                "accepted": False,
                "state": "dry_run",
                "intent": request.intent,
                "request_id": request.request_id,
                "message": "web command dry-run; launch with execution_mode:=execute to run hardware commands",
            }
        return {
            "accepted": True,
            "state": "accepted",
            "intent": request.intent,
            "request_id": request.request_id,
        }

    def blocked_legacy_response(
        self,
        *,
        intent: str,
        message: str,
        execution_mode: str,
        request_id: str = "",
        blocked_legacy_execution: bool = False,
    ) -> dict[str, Any]:
        """构造"被阻断"的统一响应（accepted=False，state="blocked"）。

        用于命令因功能开关未打开、面板处于只读模式、回放进行中锁定等原因被拒绝时的回显。
        blocked_legacy_execution 标记这次拒绝来自旧协议执行路径，供前端做差异化提示；
        execution_mode 为空时回显为 "dry_run"，与真实生效的默认模式一致，
        避免前端显示的当前模式与实际行为不符。
        """
        return {
            "accepted": False,
            "state": "blocked",
            "intent": intent,
            "execution_mode": str(execution_mode or "dry_run"),
            "request_id": request_id,
            "blocked_legacy_execution": bool(blocked_legacy_execution),
            "message": message,
        }
