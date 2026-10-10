"""面板 POST 路由表与分发器。

把"请求路径 → 面板节点上的处理方法"集中成一张表：HTTP 层只需先判断路径是否允许，
再调用分发函数，新增接口不必改 HTTP 代码，也让"对外暴露了哪些写操作入口"一目了然。

路由描述里的 handler_name 是面板节点上的方法名（分发时用 getattr 解析），三种调用形态：
    只声明 payload=True：调用 handler(请求体字典)；
    只声明 argument="..."：调用 handler(固定字符串参数)，三条整臂命令共用一个处理方法；
    两者都不声明：调用 handler()，即无参数命令。

安全语义
    只有登记在本表中的路径才会被 HTTP 层接受，其余路径一律 404；分发器还会校验处理
    结果必须是字典，避免把任意对象塞进 JSON 响应，让编程错误尽早暴露。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


class StatusPanelApiError(ValueError):
    """面板 API 层的路径/返回类型错误；继承 ValueError 便于与请求解析错误统一捕获。"""


@dataclass(frozen=True)
class PostRoute:
    """一条 POST 路由的描述。

    handler_name：面板节点上的方法名（字符串形式，延迟到分发时才解析）。
    payload：是否需要读取并传入 JSON 请求体；无请求体的命令为 False。
    argument：需要传给处理方法的固定参数（如整臂命令名）；None 表示不传。
    """

    handler_name: str
    payload: bool = False
    argument: str | None = None


# 全部允许 POST 的路径及其处理方法；路径字符串是前端约定的对外接口，不可改写。
POST_ROUTES: dict[str, PostRoute] = {
    "/api/calibration/gravity": PostRoute("_handle_calibration_gravity", payload=True),
    "/api/calibration/command": PostRoute("_handle_calibration_command", payload=True),
    # 预览一次点动执行：只做参数校验与规划检查，不直接下发硬件目标
    "/api/execute_preview": PostRoute("_handle_execute_preview", payload=True),
    # 停止正在执行的 web 点动目标
    "/api/stop_execute": PostRoute("_handle_stop_execute"),
    # 设置夹爪开度；是否真实下发由 use_hardware 决定，为假时只更新仿真反馈
    "/api/set_gripper": PostRoute("_handle_set_gripper", payload=True),
    # 进入键盘点动模式（一次性开关），之后用 keyboard_key 逐次点动
    "/api/keyboard_enable": PostRoute("_handle_keyboard_enable", payload=True),
    # 退出键盘点动模式
    "/api/keyboard_disable": PostRoute("_handle_keyboard_disable"),
    # 单次键盘点动（方向键 → 关节增量）
    "/api/keyboard_key": PostRoute("_handle_keyboard_key", payload=True),
    # 开始示教记录（同时启动重力补偿）
    "/api/teach_record_start": PostRoute("_handle_teach_record_start", payload=True),
    # 停止示教记录（同时停止重力补偿）
    "/api/teach_record_stop": PostRoute("_handle_teach_record_stop"),
    # 回放前干跑：只做质量评估、起始对齐与碰撞预检查，不驱动硬件
    "/api/teach_dry_run": PostRoute("_handle_teach_dry_run", payload=True),
    # 执行回放：需先有一次成功且未过期的干跑结果，并受执行模式闸门约束
    "/api/teach_replay_execute": PostRoute("_handle_teach_replay_execute", payload=True),
    # 停止回放（取消动作目标并请求控制器停轨迹）
    "/api/teach_replay_stop": PostRoute("_handle_teach_replay_stop"),
    # 以下三条整臂命令共用一个处理方法，靠 argument 区分命令名
    "/api/arm_safe_home": PostRoute("_handle_arm_service_command", argument="safe_home"),
    "/api/arm_enable": PostRoute("_handle_arm_service_command", argument="enable"),
    "/api/arm_disable": PostRoute("_handle_arm_service_command", argument="disable"),
}


def post_paths() -> frozenset[str]:
    """返回全部允许 POST 的路径集合（供自检与文档生成使用）。"""
    return frozenset(POST_ROUTES)


def is_allowed_post_path(path: str) -> bool:
    """判断路径是否在 POST 白名单内；HTTP 层据此决定直接 404 还是继续分发。"""
    return path in POST_ROUTES


def _dispatch_post_request(node: object, path: str, payload_reader: Callable[[], dict]) -> dict:
    """把请求分发到面板节点上对应的处理方法，并返回其结果字典。

    payload_reader 是惰性读取函数：只有路由声明需要请求体时才会被调用，
    因此无请求体的命令不会去读请求流（避免阻塞或误读后续数据）。
    异常：路径未登记，或处理方法返回值不是字典，都会抛出 StatusPanelApiError。
    """
    route = POST_ROUTES.get(path)
    if route is None:
        raise StatusPanelApiError(f"unknown POST path: {path}")
    handler = getattr(node, route.handler_name)
    # 三种调用形态：带请求体 / 带固定参数 / 无参，由路由描述决定。
    if route.payload:
        result = handler(payload_reader())
    elif route.argument is not None:
        result = handler(route.argument)
    else:
        result = handler()
    if not isinstance(result, dict):
        raise StatusPanelApiError(f"{route.handler_name} must return a dict")
    return result


def dispatch_post_request(node, path, payload_reader):
    from contextlib import nullcontext
    # Calibration has its own lock and never commands motion. Do not let its
    # potentially long solve hold the operator lock or delay a stop request.
    if path == '/api/calibration/command':
        return _dispatch_post_request(node, path, payload_reader)
    lock = getattr(node, '_operator_request_lock', None)
    with lock if lock is not None else nullcontext():
        if getattr(node, '_calibration_gravity_owned', False) and path not in {
            '/api/calibration/command', '/api/calibration/gravity', '/api/stop_execute',
            '/api/teach_record_stop', '/api/teach_dry_run', '/api/teach_replay_stop',
            '/api/keyboard_disable', '/api/arm_disable',
        }:
            return {'accepted': False, 'state': 'blocked',
                    'message': '标定拖动占用控制，请先明确退出重力补偿'}
        return _dispatch_post_request(node, path, payload_reader)
