"""唯一设备周期的采集, 反馈检查与执行分发.

职责与位置
    唯一设备周期的协调器; 把采集, 健康检查, 发送及任务更新串联起来.

对外接口
    feedback_tick 处理空闲/握手采集; control_tick 接收六轴发送回调并推进夹爪.

安全与并发约束
    不建第二份反馈或目标状态. 先验证反馈再发送; 传输故障进入保护, 普通计算异常保留原分类.

文件结构(按定义顺序)
    ControllerCycle: 协调采集, 健康检查与执行顺序, 不复制六轴, 夹爪或反馈所有者的状态.
"""

from __future__ import annotations
from ..contracts.component_ports import ArmCyclePort, CycleFeedbackPort, GripperCyclePort, ProtectionPort
from ..hardware.sdk_driver import CommandTransportError
import time


class ControllerCycle:
    """协调采集, 健康检查与执行顺序, 不复制六轴, 夹爪或反馈所有者的状态."""

    def __init__(self, runtime, lifecycle, settings):
        """保存运行时, 生命周期和参数引用; 依赖端口由装配阶段一次注入."""
        self._backend = runtime
        self._lifecycle = lifecycle
        self.settings = settings
        self._bound = False

    def bind(self, *, arm: ArmCyclePort, feedback: CycleFeedbackPort, gripper: GripperCyclePort, protection: ProtectionPort):
        """装配阶段一次注入明确组件接口; 运行后不替换依赖."""
        if self._bound:
            raise RuntimeError("component already bound")
        self.arm = arm
        self.feedback = feedback
        self.gripper = gripper
        self.protection = protection
        self._bound = True

    def clear_passthrough(self, *, arm_only=False):
        """清除六轴租约, 并按 arm_only 决定是否清除夹爪租约; 不发设备命令."""
        self.arm.clear_passthrough()
        if not arm_only:
            self.gripper.clear_passthrough()

    def reset_session(self):
        """会话重置委托状态所有者, 旧缓存和代次不得跨连接复用."""
        self.feedback.reset_session()
        self.arm.reset_session()
        self.gripper.reset_session()

    def _expire_passthrough(self):
        """以同一单调时刻检查两类租约, 由各所有者完成保持或中性释放."""
        now = time.monotonic()
        self.arm.expire_passthrough(now)
        self.gripper.expire_passthrough(now)

    def feedback_tick(self):
        """设备事务等待和失能期间只采集; 发布线程不再轮询总线."""
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
        """机械臂命令, 反馈批次与夹爪命令的唯一所有者.

        每周期顺序: 刷新反馈 -> 检查反馈健康 -> 执行模式回调(位置跟踪或重力
        补偿) -> 推进夹爪状态机. 若反馈刷新或回调因反馈类错误失败, 则执行保护性
        失能并结束本次 tick; 命令发送错误立即保护, 其余异常交本地运行时登记和保护,
        以免把编程错误当成"反馈故障"而静默失能.
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
            # 夹爪几何绑定在本周期输出之前复核；失效时先交给六轴停止保持事务接管。
            self.arm.check_prepared_context()
            # 组合所有者先核验两侧任务；实际六轴发送和夹爪推进仍由各自执行器完成。
            coordinated = self.arm.coordinated_tick()
            if not coordinated and not self.arm.passthrough_active():
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
        if not coordinated:
            self.gripper.tick()
        error = self.gripper.transport_error()
        if error is not None:
            self.protection.protect(error, category="COMMAND")
