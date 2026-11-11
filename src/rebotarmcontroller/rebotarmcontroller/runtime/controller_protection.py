"""设备故障的保护失能与任务清理协调.

职责与位置
    设备故障后的清理与保护失能协调; 业务状态仍由各执行组件拥有.

对外接口
    protect 先锁存再停止输出, 清理任务和尝试失能; confirm_disabled 在验证通过后清锁存.

安全与并发约束
    只能由设备线程执行; 某项清理失败也要继续尝试其他保护动作.
    发送失能不等于状态已确认, 失败保留 DISABLING, 后续靠显式请求重试.

文件结构(按定义顺序)
    ControllerProtection: 持有保护决策锁存并协调各组件清理与失能; 不接管目标或反馈缓存.
"""

from __future__ import annotations
from ..contracts.component_ports import ArmProtectionPort, GravityProtectionPort, GripperProtectionPort
from .command_guard import CommandGuard
from ..controller_parameters import (
    _LOG
)


class ControllerProtection:
    """持有保护决策锁存并协调各组件清理与失能; 不接管目标或反馈缓存."""

    def __init__(self, runtime, lifecycle, settings):
        """保存协调依赖并创建本组件的保护锁存; 不创建任务或反馈镜像."""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False
        self._command_guard = CommandGuard()

    def bind(self, *, arm: ArmProtectionPort, gravity: GravityProtectionPort, gripper: GripperProtectionPort):
        """装配阶段一次注入明确组件接口; 运行后不替换依赖."""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.gravity = gravity
        self.gripper = gripper
        self._bound = True

    def confirm_disabled(self):
        """显式失能以新帧确认后解除故障锁存; 反馈恢复本身不得调用."""
        self.gravity.clear_hold_fault()
        self._command_guard.confirm_disabled()

    @property
    def decision(self):
        """不可变的当前保护决策; 供诊断及跨语言样本读取, 不提供复位写入口."""
        return self._command_guard.protection

    def protect(self, reason: str, *, category: str = 'FEEDBACK') -> None:
        """每个故障会话只请求一次保护失能; 确认失能前保留未知状态.

        先锁存, 避免空闲采集或重入错误路径每周期重复写总线. 即使发送失败也不
        自动重试; 操作员可显式调用 disable 重试, 只有新鲜失能反馈才解除锁存.
        """
        if not self._command_guard.begin_protection(reason, category):
            return
        message = str(reason)
        self.arm.clear_passthrough()
        self.gripper.clear_passthrough()
        # 先停止输出. 任务收尾出错也必须继续尝试失能, 不能因锁存而漏发首个保护命令.
        preparation_errors = []
        def attempt(label, operation):
            """逐项执行清理或失能动作, 收集异常后继续余下保护步骤."""
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
            # 六轴与夹爪可引用同一总线对象,按身份去重使一次保护只向该总线请求一次失能.
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
            try:
                controller.disable_all()
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
