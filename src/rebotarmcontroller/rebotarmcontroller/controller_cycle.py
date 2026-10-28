"""唯一设备周期的采集、反馈检查与执行分发。

对外接口：control_tick 依次刷新、检查、发送及推进夹爪；
feedback_tick 在失能和握手等待期间继续采集，检查旧任务代次。
约束：不持有目标或反馈镜像；会话重置、租约到期均委托各状态所有者。
命令发送失败交保护组件，普通程序异常不得伪装成反馈错误。
"""
from __future__ import annotations
from .component_ports import ArmCyclePort, CycleFeedbackPort, GripperCyclePort, ProtectionPort
from .sdk_driver import CommandTransportError
import time


class ControllerCycle:
    """拥有本职责的状态和事务；ROS 门面仅提交请求或读取结果。"""

    def __init__(self, runtime, lifecycle, settings):
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False

    def bind(self, *, arm: ArmCyclePort, feedback: CycleFeedbackPort, gripper: GripperCyclePort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口；运行后不替换依赖。"""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.feedback = feedback
        self.gripper = gripper
        self.protection = protection
        self._bound = True

    def clear_passthrough(self, *, arm_only=False):
        self.arm.clear_passthrough()
        if not arm_only:
            self.gripper.clear_passthrough()

    def reset_session(self):
        """会话重置委托状态所有者，旧缓存和代次不得跨连接复用。"""
        self.feedback.reset_session()
        self.gripper.reset_session()

    def _expire_passthrough(self):
        now = time.monotonic()
        self.arm.expire_passthrough(now)
        self.gripper.expire_passthrough(now)

    def feedback_tick(self):
        """设备事务等待和失能期间只采集；发布线程不再轮询总线。"""
        if self._lifecycle.connected and not self.feedback.refresh_active():
            self.feedback.refresh_feedback_if_due()
        if self._lifecycle.enabled and not self._backend._waiting:
            failure = self.feedback.arm_failure()
            if failure is not None:
                self.protection.protect(failure)
                return
            self._expire_passthrough()
        self.arm.check_task_generation()


    def control_tick(self, arm, dt: float, arm_callback) -> None:
        """机械臂命令、反馈批次与夹爪命令的唯一所有者。

        每周期顺序：刷新反馈 -> 检查反馈健康 -> 执行模式回调（位置跟踪或重力
        补偿）-> 推进夹爪状态机。若反馈刷新或回调因反馈类错误失败，则执行保护性
        失能并结束本次 tick；命令发送错误立即保护，其余异常交本地运行时登记和保护，
        以免把编程错误当成"反馈故障"而静默失能。
        """
        try:
            self.feedback.refresh_feedback_if_due()
        except Exception:
            feedback_failure = self.feedback.arm_failure()
            if feedback_failure is None:
                raise
            self.protection.protect(feedback_failure)
            return
        feedback_failure = self.feedback.arm_failure()
        if feedback_failure is not None:
            self.protection.protect(feedback_failure)
            return
        try:
            self._expire_passthrough()
            if not self.arm.passthrough_active():
                arm_callback(arm, dt)
        except CommandTransportError as exc:
            self.protection.protect(str(exc), category="COMMAND")
            return
        except Exception:
            feedback_failure = self.feedback.arm_failure()
            if feedback_failure is None:
                raise
            self.protection.protect(feedback_failure)
            return
        self.arm.update_task(self.feedback.joint_snapshot())
        self.gripper.tick()
        error = self.gripper.transport_error()
        if error is not None:
            self.protection.protect(error, category="COMMAND")
