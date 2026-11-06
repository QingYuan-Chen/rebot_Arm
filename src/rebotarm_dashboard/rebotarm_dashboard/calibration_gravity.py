"""Operator-only gravity-mode gate. Never enables or disables an arm."""
import math


def gravity_rejection(command, confirmed, *, hardware, web_enabled, execution_mode,
                      arm, status_age, busy):
    if command not in ('start', 'stop') or confirmed is not True:
        return '明确确认进入或退出重力补偿后再操作'
    if not hardware or not web_enabled or execution_mode != 'execute':
        return '当前工作台未允许真实硬件操作'
    if not math.isfinite(status_age) or not 0 <= status_age <= .5:
        return '控制器反馈过期，请检查连接'
    errors = arm.get('error_codes', [])
    # 仅重力故障保持允许显式交接回位置保持；设备与反馈错误继续阻断切换。
    hold_exit = command == 'stop' and all(code.startswith('GRAVITY_HOLD:') for code in errors)
    if not arm.get('enabled') or (errors and not hold_exit):
        return '需要已显式使能且无错误的控制器'
    if command == 'start' and (arm.get('state_machine') != 'IDLE' or busy):
        return '请先结束其他运动、键盘控制或示教任务'
    if command == 'stop' and arm.get('state_machine') != 'GRAVITY_COMP':
        return '当前反馈未确认重力补偿模式'
    return None


class GravityRequestTracker:
    """Retain timed-out futures: timeout is not proof that a mode change stopped."""
    def __init__(self):
        self.future = None
        self.command = None
        self.completed_at = None
        self.response = None

    def refresh(self, now):
        if self.future is not None and self.future.done():
            try:
                self.response = self.future.result()
            except Exception:
                self.response = None
            self.future = None
            self.completed_at = now
        return self.future is not None

    def call(self, client, request, command, *, timeout=3.):
        import threading
        import time
        if self.refresh(time.monotonic()):
            return False, '模式切换请求尚未结束；请等待反馈，不要重复发送'
        if not client.service_is_ready():
            return False, '重力补偿服务不可用'
        self.command = command
        self.completed_at = None
        self.response = None
        self.future = client.call_async(request)
        event = threading.Event()
        self.future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            return False, '模式切换请求超时，结果未知；请求仍被跟踪'
        self.refresh(time.monotonic())
        if self.response is None:
            return False, '模式切换响应异常，请核对控制器状态'
        return bool(self.response.success), str(self.response.message)

    def idle_confirmed(self, *, now, received_at, arm):
        if self.refresh(now) or received_at is None:
            return False
        return (0 <= now - received_at <= .5 and
                (self.completed_at is None or received_at > self.completed_at) and
                arm.get('enabled') is True and not arm.get('error_codes') and
                arm.get('state_machine') == 'IDLE')
