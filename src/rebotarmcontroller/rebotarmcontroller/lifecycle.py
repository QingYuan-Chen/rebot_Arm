"""控制器生命周期与关闭时的安全协调。

职责：保存生命周期与执行状态，推进连接、显式使能及回滚、失能确认、
模式切换、置零准入和关闭验证。对外事务通过 ``LifecyclePorts`` 注入的
设备与组件接口运行，不接受 manager 对象，也不依赖 ROS 或 SDK 全局对象。

约束：事务须由唯一设备线程调用；生命周期锁保护状态交接。使能失败或
失能验证失败必须保留“可能仍使能”的状态；关闭只能释放已用新帧确认失能
的设备。``shutdown_policy`` 处理节点退出，健康任务失败保持使能，等待
操作员显式失能。夹爪零位验证、反馈采集和控制计算仍由各自接口负责。
"""
from dataclasses import dataclass
import threading
from typing import Callable, Protocol

from .sdk_driver import CommandTransportError


class LifecycleDevice(Protocol):
    """事务使用的设备能力；真实和离线实现遵守同一接口。"""

    mode: str
    control_loop_active: bool

    def connect(self): ...
    def disconnect(self): ...
    def release_disabled(self): ...
    def set_target(self, positions): ...
    def enable(self): ...
    def disable(self): ...
    def mode_pos_vel(self): ...
    def mode_mit(self): ...
    def mode_vel(self): ...
    def set_zero(self): ...
    def set_zero_single(self, name): ...


@dataclass(frozen=True)
class LifecyclePorts:
    """生命周期与其他所有者之间的窄接口；回调不保存第二份业务状态。"""

    device: LifecycleDevice
    joint_feedback: Callable
    gripper_status: Callable
    gripper_feedback_status: Callable
    gripper_motor: Callable
    gripper_coordinate_failure: Callable
    prepare_gripper_enable: Callable
    clear_gripper_transport_error: Callable
    reset_session: Callable
    initialize_gripper: Callable
    stop_gripper: Callable
    stop_control: Callable
    stop_gravity: Callable
    clear_passthrough: Callable
    hold: Callable
    start_position: Callable
    zero_gripper: Callable
    confirm_disabled: Callable
    protect: Callable

LIFECYCLE_STATES = frozenset({
    'DISCONNECTED', 'CONNECTED_DISABLED', 'ENABLING', 'ENABLED_HOLD',
    'TRAJECTORY_RUNNING', 'DISABLING',
})
EXECUTION_STATES = frozenset({'IDLE', 'TRAJ_RUNNING', 'LOWLEVEL_STREAMING', 'GRAVITY_COMP'})


class ControllerLifecycle:
    """保存一个设备会话的生命周期和公开执行状态。"""

    def __init__(self, ports: LifecyclePorts | None = None):
        self.ports = ports
        self.lock = threading.RLock()
        self.connected = False
        self.enabled = False
        self.errors = []
        self.state = 'DISCONNECTED'
        self.execution = 'IDLE'

    @property
    def ready_for_motion(self):
        """只有明确处于使能保持/轨迹状态才能接纳运动。"""
        return bool(self.connected and self.enabled
                    and self.state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING"))

    def set_state(self, state):
        if state not in LIFECYCLE_STATES:
            raise ValueError(f'unsupported lifecycle state: {state}')
        self.state = state

    def set_execution(self, state, *, enabled):
        if state not in EXECUTION_STATES:
            raise ValueError(f'unsupported state machine value: {state}')
        self.execution = state
        if enabled and self.state != 'DISABLING':
            if state == 'TRAJ_RUNNING':
                self.state = 'TRAJECTORY_RUNNING'
            elif state == 'IDLE':
                self.state = 'ENABLED_HOLD'

    def set_execution_state(self, state):
        """执行器通过本入口同步状态，不直接覆盖生命周期状态字段。"""
        self.set_execution(state, enabled=self.enabled)

    def require_connected(self):
        if not self.connected:
            raise RuntimeError("hardware is not connected")

    def require_enabled(self):
        """所有运动入口共享的使能准入，状态未知时必须先验证失能。"""
        self.require_connected()
        if self.state == "DISABLING":
            raise RuntimeError("hardware state is unconfirmed; verify disable before enabling again")
        if not self.enabled:
            raise RuntimeError("hardware is disabled; call explicit /rebotarm/enable first")

    def _idle(self):
        self.set_execution("IDLE", enabled=self.enabled)

    def _verify_disabled(self):
        self.ports.joint_feedback(expected_status=0)
        self.ports.gripper_status(0)

    def disable_all(self):
        """分别尝试六轴与夹爪失能；一侧失败不跳过另一侧。"""
        errors = []
        try:
            self.ports.device.disable()
        except Exception as exc:
            errors.append(f"arm disable: {exc}")
        motor = self.ports.gripper_motor()
        if motor is not None:
            try:
                motor.disable()
            except Exception as exc:
                errors.append(f"gripper disable: {exc}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def connect(self):
        """建立新会话并确认默认失能，不启动运动输出。"""
        with self.lock:
            if self.connected:
                return
            try:
                self.ports.clear_passthrough()
                self.ports.device.connect()
                self.ports.reset_session()
                self.connected = True
                self.ports.initialize_gripper()
                _, _, _, statuses = self.ports.joint_feedback()
                # 连接后的原始状态可为使能；先确认实际状态，再按既有连接策略失能。
                gripper_status = self.ports.gripper_feedback_status()
                if any(status != 0 for status in statuses) or gripper_status not in (None, 0):
                    self.disable_all()
                    self._verify_disabled()
                self.enabled = False
                self.ports.confirm_disabled()
                self.set_state("CONNECTED_DISABLED")
            except Exception:
                self.disconnect_failed_connect()
                raise

    def disconnect_failed_connect(self):
        """连接失败后逐步尽力清理；保留原始连接错误。"""
        for operation in (self.ports.stop_gripper, self.ports.stop_control,
                          self.disable_all, self.ports.device.disconnect):
            try:
                operation()
            except Exception:
                pass
        self.connected = self.enabled = False
        self.set_state("DISCONNECTED")

    def enable(self):
        """先以新鲜失能反馈建立保持目标，再使能；失败必须验证回滚结果。"""
        with self.lock:
            self.require_connected()
            if self.enabled:
                if self.state == "DISABLING":
                    raise RuntimeError("verify disable before enabling again")
                self.ports.hold()
                self.set_state("ENABLED_HOLD")
                return
            self.set_state("ENABLING")
            try:
                positions, _, _, _ = self.ports.joint_feedback(expected_status=0)
                self.ports.gripper_status(0)
                failure = self.ports.gripper_coordinate_failure()
                if failure is not None:
                    raise RuntimeError(failure)
                self.ports.device.set_target(positions)
                if self.ports.device.mode_pos_vel() is False:
                    raise RuntimeError("failed to enter position-velocity control mode")
                self.ports.prepare_gripper_enable()
                self.ports.device.enable()
                motor = self.ports.gripper_motor()
                if motor is not None:
                    motor.enable()
                self.ports.joint_feedback(expected_status=1)
                self.ports.gripper_status(1)
                self.enabled = True
                self.ports.clear_gripper_transport_error()
                self._idle()
                self.set_state("ENABLED_HOLD")
                self.ports.start_position(target=positions)
            except Exception as exc:
                rollback_error = self.rollback_enable()
                if rollback_error is not None:
                    raise RuntimeError(
                        f"enable failed: {exc}; rollback verification failed: {rollback_error}"
                    ) from exc
                raise RuntimeError(f"enable failed and was rolled back: {exc}") from exc

    def rollback_enable(self):
        """回滚各步骤独立尝试；未确认失能时禁止谎报安全状态。"""
        errors = []
        for label, operation in (("stop gripper loop", self.ports.stop_gripper),
                                 ("stop arm loop", self.ports.stop_control),
                                 ("disable command", self.disable_all)):
            try:
                operation()
            except Exception as exc:
                errors.append(f"{label}: {exc}")
        if not errors:
            try:
                self._verify_disabled()
            except Exception as exc:
                errors.append(f"disabled feedback: {exc}")
        self.execution = "IDLE"
        if errors:
            self.enabled = True
            self.errors.append("ENABLE_ROLLBACK_FAILED")
            self.set_state("DISABLING")
            return "; ".join(errors)
        self.enabled = False
        self.ports.confirm_disabled()
        self.set_state("CONNECTED_DISABLED" if self.connected else "DISCONNECTED")
        return None

    def disable(self):
        """暂停输出、发送失能并用新反馈确认；失败保留状态未知。"""
        with self.lock:
            self.require_connected()
            self.set_state("DISABLING")
            was_enabled = self.enabled
            self.enabled = False
            self.ports.clear_passthrough()
            try:
                self.ports.stop_gravity()
                self.ports.stop_gripper()
                self.ports.stop_control()
                self.disable_all()
                self._verify_disabled()
            except Exception:
                self.enabled = was_enabled
                self.errors.append("DISABLE_VERIFICATION_FAILED")
                raise
            self.enabled = False
            self.execution = "IDLE"
            self.ports.confirm_disabled()
            self.set_state("CONNECTED_DISABLED")

    def set_mode(self, mode):
        """模式事务保持原有交接顺序；设备传输失败进入保护路径。"""
        with self.lock:
            self.require_connected()
            if self.state == "DISABLING":
                raise RuntimeError("verify disable before changing mode")
            mode = mode.strip().lower()
            if mode not in ("mit", "pos_vel", "vel"):
                raise ValueError(f"unsupported mode: {mode}")
            self.ports.clear_passthrough(arm_only=True)
            self.ports.stop_gravity()
            if mode == self.ports.device.mode:
                if mode == "pos_vel" and self.enabled:
                    if self.ports.device.control_loop_active:
                        self.ports.hold()
                    else:
                        self.ports.start_position()
                self._idle()
                return True
            try:
                self.ports.stop_control()
                operation = {"mit": self.ports.device.mode_mit,
                             "pos_vel": self.ports.device.mode_pos_vel,
                             "vel": self.ports.device.mode_vel}[mode]
                ok = operation()
                if mode == "pos_vel" and self.enabled:
                    self.ports.start_position()
            except CommandTransportError as exc:
                self.ports.protect(str(exc), category="MODE")
                raise
            self._idle()
            return bool(ok)

    def set_zero(self, joint_name=""):
        """仅失能时允许置零；夹爪的新帧连续验证交由专属接口执行。"""
        with self.lock:
            self.require_connected()
            if self.enabled:
                raise RuntimeError("set_zero requires CONNECTED_DISABLED state")
            self.ports.stop_control()
            name = str(joint_name).strip()
            if name == "gripper":
                return self.ports.zero_gripper()
            if name:
                ok = self.ports.device.set_zero_single(name)
            else:
                self.ports.device.set_zero()
                ok = True
            self.enabled = False
            self._idle()
            return bool(ok)

    def shutdown(self):
        """失能验证失败保留连接；使能时拒绝释放，不自动失能。"""
        with self.lock:
            if not self.connected:
                return
            if self.enabled:
                raise RuntimeError("shutdown requires explicit disable first")
            self.ports.stop_gripper()
            self.ports.stop_gravity()
            self.ports.stop_control()
            try:
                self._verify_disabled()
            except Exception as exc:
                self.enabled = True
                self.set_state("DISABLING")
                self.errors.append(f"SHUTDOWN_STATE_UNCONFIRMED: {exc}")
                raise RuntimeError(f"shutdown disabled-state verification: {exc}") from exc
            try:
                self.ports.device.release_disabled()
            except Exception as exc:
                self.errors.append(f"SHUTDOWN_RELEASE_FAILED: {exc}")
                raise RuntimeError(f"shutdown release: {exc}") from exc
            self.connected = False
            self.set_state("DISCONNECTED")


def shutdown_policy(hardware, *, safe_home, logger):
    """仅在已确认失能时释放硬件；使能时保留控制以等待人工恢复。

    可选停靠成功也不等同于验证本轮安全基线；操作员需确认现场并显式失能。
    失败后尝试停止并保持，若设备或反馈故障则由硬件保护路径处置。
    返回 False 时调用方必须保留节点、执行器和设备连接。
    """
    if hardware is None:
        return True
    if hardware.connected and hardware.enabled:
        if safe_home is not None:
            try:
                safe_home()
            except Exception as exc:
                logger.error(f'shutdown safe_home failed: {exc}')
        try:
            hardware.stop_gravity_compensation()
            hardware.ensure_pos_vel_control()
            hardware.stop_active_motion()
        except Exception as exc:
            logger.error(f'shutdown hold failed; keeping controller active: {exc}')
        if hardware.enabled:
            logger.warn('shutdown deferred: arm remains enabled; confirm safety and explicitly disable')
            return False
    hardware.shutdown()
    return True
