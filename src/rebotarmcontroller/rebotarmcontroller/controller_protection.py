"""设备故障的保护失能与任务清理协调。

对外接口：protect 首次故障先锁存、停止输出、
清理任务再尝试所有控制器失能；清理失败也不能跳过首个失能发送。
约束：只由唯一设备线程调用；CommandGuard 拥有去重锁存。保持 DISABLING
直到显式失能通过新鲜反馈确认；反馈恢复不自动清锁存，不周期重复失能。
"""
from __future__ import annotations
from .component_ports import ArmProtectionPort, GravityProtectionPort, GripperProtectionPort
from .command_guard import CommandGuard
from .controller_parameters import (
    _LOG
)


class ControllerProtection:
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def __init__(self, runtime, lifecycle, settings):
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._command_guard = CommandGuard()

    def bind(self, *, arm: ArmProtectionPort, gravity: GravityProtectionPort, gripper: GripperProtectionPort):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.gravity = gravity
        self.gripper = gripper
        self._bound = True

    def confirm_disabled(self):
        """显式失能以新帧确认后解除故障锁存；反馈恢复本身不得调用。"""
        self._command_guard.confirm_disabled()

    @property
    def decision(self):
        """不可变的当前保护决策；供诊断及跨语言样本读取，不提供复位写入口。"""
        return self._command_guard.protection

    def protect(self, reason: str, *, category: str = 'FEEDBACK') -> None:
        """每个故障会话只请求一次保护失能；确认失能前保留未知状态。

        先锁存，避免空闲采集或重入错误路径每周期重复写总线。即使发送失败也不
        自动重试；操作员可显式调用 disable 重试，只有新鲜失能反馈才解除锁存。
        """
        if not self._command_guard.begin_protection(reason, category):
            return
        message = str(reason)
        self.arm.clear_passthrough()
        self.gripper.clear_passthrough()
        # 先停止输出。任务收尾出错也必须继续尝试失能，不能因锁存而漏发首个保护命令。
        preparation_errors = []
        def attempt(label, operation):
            try:
                operation()
            except Exception as exc:
                preparation_errors.append(f'{label}: {exc}')
        attempt('stop output', self._backend.request_loop_stop)
        attempt('invalidate requests', lambda: self._backend.requests.invalidate('device', 'arm', 'gripper'))
        attempt('gripper stop', lambda: self.gripper.prepare_protection(message))
        attempt('finish arm task', lambda: self.arm.abort_task(message))
        attempt('stop execution', lambda: self._backend.set_execution_running(False))
        attempt('cancel trajectory', self._backend.cancel_trajectory)
        self.gravity.clear_execution()
        self._lifecycle.execution = "IDLE"
        self._lifecycle.set_state("DISABLING")
        code = f'{category}_PROTECTIVE_DISABLE'
        if code not in self._lifecycle.errors:
            self._lifecycle.errors.append(code)
        if preparation_errors:
            self._lifecycle.errors.append(f'{category}_PROTECTIVE_PREPARE_FAILED: ' + '; '.join(preparation_errors))

        controllers: list[object] = []
        disable_errors: list[str] = []
        try:
            for controller in self._backend.controllers():
                if all(controller is not existing for existing in controllers):
                    controllers.append(controller)
        except Exception as exc:
            disable_errors.append(f'controller lookup: {exc}')
        gripper_controller, _ = self.gripper.feedback_binding()
        if gripper_controller is not None and all(
            gripper_controller is not controller for controller in controllers
        ):
            controllers.append(gripper_controller)
        for controller in controllers:
            disable_all = getattr(controller, "disable_all", None)
            if not callable(disable_all):
                disable_errors.append(f'{type(controller).__name__}: disable_all unavailable')
                continue
            try:
                disable_all()
            except Exception as exc:
                disable_errors.append(f"{type(controller).__name__}: {exc}")
        if disable_errors:
            self._lifecycle.errors.append(
                f"{category}_PROTECTIVE_DISABLE_FAILED: " + "; ".join(disable_errors)
            )
        _LOG.error(
            "hardware writer stopped and controller protective disable requested: %s",
            message,
        )
