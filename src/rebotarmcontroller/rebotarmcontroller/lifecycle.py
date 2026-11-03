"""控制器生命周期与关闭时的安全协调.

职责与位置
    连接, 使能回滚, 失能验证, 模式交接和关闭事务的状态所有者.

对外接口
    LifecycleDevice 描述设备能力, LifecyclePorts 注入其他所有者能力;
    ControllerLifecycle 接纳显式事务, shutdown_policy 处理节点退出.

安全与并发约束
    连接会检查并在需要时尝试失能, 不能当作严格只读操作. 失败不能伪报安全状态.
    健康任务失败保持使能; 关闭仅释放已通过新帧确认失能的设备, 不凭退出意图自动失能.

文件结构(按定义顺序)
    LifecycleDevice: 事务使用的设备能力; 真实和离线实现遵守同一接口.
    LifecyclePorts: 生命周期与其他所有者之间的窄接口; 回调不保存第二份业务状态.
    ControllerLifecycle: 保存一个设备会话的生命周期和公开执行状态.
    shutdown_policy: 仅在已确认失能时释放硬件; 使能时保留控制以等待人工恢复.
"""

from __future__ import annotations
from dataclasses import dataclass
import threading
from typing import Callable, Protocol

from .sdk_driver import CommandTransportError
from .device_requests import RequestRuntime, device_request
from .ros_ports import ArmCommandPort, GravityCommandPort, LifecycleCommandPort


class LifecycleDevice(Protocol):
    """事务使用的设备能力; 真实和离线实现遵守同一接口."""

    mode: str
    control_loop_active: bool

    def connect(self):
        """声明设备会话连接能力, 不隐含使能."""
        ...

    def disconnect(self):
        """声明驱动级失能和释放能力, 不能代替生命周期确认."""
        ...

    def release_disabled(self):
        """声明已确认失能后的释放能力; 真实驱动释放仍可能追加失能帧."""
        ...

    def set_target(self, positions):
        """声明写入六轴数值目标的能力, 位置单位 rad."""
        ...

    def enable(self):
        """声明驱动级使能事务, 完整失败回滚由生命周期协调."""
        ...

    def disable(self):
        """声明驱动级失能事务, 七路新帧确认由生命周期协调."""
        ...

    def mode_pos_vel(self):
        """声明六轴位置速度模式切换能力."""
        ...

    def mode_mit(self):
        """声明六轴 MIT 模式切换能力."""
        ...

    def mode_vel(self):
        """声明六轴速度模式切换能力."""
        ...

    def set_zero(self):
        """声明六轴置零能力, 前置失能条件由生命周期检查."""
        ...

    def set_zero_single(self, name):
        """声明指定六轴关节的置零能力, 不负责夹爪标定."""
        ...


@dataclass(frozen=True)
class LifecyclePorts:
    """生命周期与其他所有者之间的窄接口; 回调不保存第二份业务状态."""

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
EXECUTION_STATES = frozenset({'IDLE', 'TRAJ_PREPARING', 'TRAJ_RUNNING', 'LOWLEVEL_STREAMING', 'GRAVITY_COMP'})


class ControllerLifecycle:
    """保存一个设备会话的生命周期和公开执行状态."""

    def __init__(self, runtime: RequestRuntime, ports: LifecyclePorts | None = None):
        """保存唯一请求运行时与端口, 初始化失联, 未使能和空闲状态; 不连接硬件."""
        self._backend = runtime
        self.ports = ports
        self.lock = threading.RLock()
        self.connected = False
        self.enabled = False
        self.errors = []
        self.state = 'DISCONNECTED'
        self.execution = 'IDLE'

    @property
    def ready_for_motion(self):
        """只有明确处于使能保持/轨迹状态才能接纳运动."""
        return bool(self.connected and self.enabled
                    and self.state in ("ENABLED_HOLD", "TRAJECTORY_RUNNING"))

    def set_state(self, state):
        """校验生命周期枚举后写入; 调用方负责设备线程串行化和事务锁."""
        if state not in LIFECYCLE_STATES:
            raise ValueError(f'unsupported lifecycle state: {state}')
        self.state = state

    def set_execution(self, state, *, enabled):
        """校验执行枚举并同步派生生命周期状态, DISABLING 期间不伪报重新就绪."""
        if state not in EXECUTION_STATES:
            raise ValueError(f'unsupported state machine value: {state}')
        self.execution = state
        if enabled and self.state != 'DISABLING':
            if state == 'TRAJ_RUNNING':
                self.state = 'TRAJECTORY_RUNNING'
            elif state == 'IDLE':
                self.state = 'ENABLED_HOLD'

    def set_execution_state(self, state):
        """执行器通过本入口同步状态, 不直接覆盖生命周期状态字段."""
        self.set_execution(state, enabled=self.enabled)

    def require_connected(self):
        """未建立已确认设备会话时拒绝后续事务."""
        if not self.connected:
            raise RuntimeError("hardware is not connected")

    def require_enabled(self):
        """所有运动入口共享的使能准入, 状态未知时必须先验证失能."""
        self.require_connected()
        if self.state == "DISABLING":
            raise RuntimeError("hardware state is unconfirmed; verify disable before enabling again")
        if not self.enabled:
            raise RuntimeError("hardware is disabled; call explicit /rebotarm/enable first")

    def _idle(self):
        """按当前使能状态归并执行空闲, 保留失能确认尚未完成的生命周期."""
        self.set_execution("IDLE", enabled=self.enabled)

    def _verify_disabled(self):
        """通过注入端口确认六轴及夹爪失能; 新帧刷新由对应反馈端口保证."""
        self.ports.joint_feedback(expected_status=0)
        self.ports.gripper_status(0)

    def disable_all(self):
        """分别尝试六轴与夹爪失能; 一侧失败不跳过另一侧."""
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

    @device_request
    def connect(self):
        """建立会话并确认默认失能; 必要时会发送失能命令, 不是只读探测.

        请求在唯一设备线程执行, 生命周期锁覆盖连接, 反馈绑定和状态确认.
        调用前须完成现场安全检查; 失败仅尽力清理, 不能以软件已断开证明电机已失能.
        """
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
                # 连接后的原始状态可为使能; 先确认实际状态, 再按既有连接策略失能.
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
        """连接失败后逐步尽力清理; 保留原始连接错误."""
        for operation in (self.ports.stop_gripper, self.ports.stop_control,
                          self.disable_all, self.ports.device.disconnect):
            try:
                operation()
            except Exception:
                pass
        self.connected = self.enabled = False
        self.set_state("DISCONNECTED")

    @device_request
    def enable(self):
        """显式使能并开始位置保持; 首次使能失败时验证回滚结果.

        首次调用以新鲜失能反馈建立目标, 切换模式后使能六轴和夹爪, 再用新帧确认.
        已使能时再次调用会重新请求当前位置保持; 它不只是状态查询.
        回滚未确认失能时保留 DISABLING, 阻断运动, 不能报告安全关闭.
        """
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
                # 用使能前验证的失能位置建立保持目标, 防止沿用上一会话目标在使能后突然追赶.
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
        """回滚各步骤独立尝试; 未确认失能时禁止谎报安全状态."""
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
        # 回滚不明时用保守使能标志及 DISABLING 阻断恢复, 不能因命令已尝试就声称可释放.
        if errors:
            self.enabled = True
            self.errors.append("ENABLE_ROLLBACK_FAILED")
            self.set_state("DISABLING")
            return "; ".join(errors)
        self.enabled = False
        self.ports.confirm_disabled()
        self.set_state("CONNECTED_DISABLED" if self.connected else "DISCONNECTED")
        return None

    @device_request
    def disable(self):
        """停止输出, 发送失能并用七路新反馈确认; 不在此检查现场安全基线.

        正常卸力前的现场支撑和基线验证由调用方负责. 验证失败保留 DISABLING 和错误,
        恢复原使能标志, 不以命令已发送或服务已返回代替失能证据.
        """
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

    @device_request
    def set_mode(self, mode):
        """模式事务保持原有交接顺序; 设备传输失败进入保护路径."""
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

    @device_request
    def set_zero(self, joint_name=""):
        """仅失能时允许置零; 夹爪的新帧连续验证交由专属接口执行."""
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

    @device_request
    def shutdown(self):
        """仅在未使能且七路新帧确认失能后释放设备会话.

        使能时拒绝关闭; 确认失败保留连接并标记 DISABLING, 供显式恢复.
        底层释放仍可能追加失能帧, 因此不能用于未知状态下的只读探测收尾.
        """
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


def shutdown_policy(lifecycle: LifecycleCommandPort, *, arm: ArmCommandPort,
                    gravity: GravityCommandPort, safe_home, logger):
    """仅在已确认失能时释放硬件; 使能时保留控制以等待人工恢复.

    可选停靠成功也不等同于验证本轮安全基线; 操作员需确认现场并显式失能.
    失败后尝试停止并保持, 若设备或反馈故障则由硬件保护路径处置.
    返回 False 时调用方必须保留节点, 执行器和设备连接.
    """
    if lifecycle.connected and lifecycle.enabled:
        if safe_home is not None:
            try:
                safe_home()
            except Exception as exc:
                logger.error(f'shutdown safe_home failed: {exc}')
        try:
            gravity.stop_gravity_compensation()
            arm.ensure_pos_vel_control()
            arm.stop_active_motion()
        except Exception as exc:
            logger.error(f'shutdown hold failed; keeping controller active: {exc}')
        if lifecycle.enabled:
            logger.warn('shutdown deferred: arm remains enabled; confirm safety and explicitly disable')
            return False
    lifecycle.shutdown()
    return True
