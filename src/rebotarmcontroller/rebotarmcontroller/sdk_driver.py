"""基于 MotorBridge 句柄接口的达妙设备驱动。

``SdkMotorDriver`` 对外提供连接、设备句柄查询、反馈分组、使能/失能、
模式切换、位置与 MIT 批量发送、置零和资源释放。``MotorPort`` 与
``BusPort`` 把单次句柄调用限制在共享总线锁内；反馈状态在读出时复制为
``MotorFeedback``，批量发送失败通过 ``CommandTransportError.receipt``
保留已发送、失败及未尝试的电机。

构造时不打开设备；连接后一个 Controller 管理六轴与夹爪七个句柄。
驱动不负责 ROS 准入、控制线程生命周期或完整安全策略；调用方须先完成
现场检查、新鲜反馈和显式使能等上层条件。驱动使能/失能确认覆盖六轴，
夹爪状态由管理器另行验证；置零的上层前置检查也由管理器负责。

命令操作与总线事务分别串行化，稳定等待期间允许反馈访问总线；模式或
批量发送的部分失败不会自动重发。运行时持有时，端口断言所有设备访问来自唯一写入线程。
"""
from __future__ import annotations
import threading
import time
import numpy as np
from .controller_contracts import CommandReceipt, MotorFeedback
from .gravity_control import vector, POSITION_LIMITS

class CommandTransportError(RuntimeError):
    """携带实际发送回执的设备命令错误。"""
    def __init__(self, receipt):
        self.receipt = receipt
        super().__init__(f"{receipt.failed}: {receipt.detail}; sent={receipt.sent}; not_attempted={receipt.not_attempted}")

class MotorPort:
    """一个受管理的电机句柄；释放后所有操作均失败。"""
    def __init__(self, name, handle, bus_lock, owner=None):
        self.name, self._handle, self._lock = name, handle, bus_lock
        self._owner = owner
        self._closed = False

    def _call(self, method, *args):
        if self._owner is not None:
            self._owner()
        with self._lock:
            if self._closed:
                raise RuntimeError(f'{self.name}: motor handle is closed')
            return getattr(self._handle, method)(*args)

    def _send(self, method, *args):
        try:
            return self._call(method, *args)
        except Exception as exc:
            raise CommandTransportError(CommandReceipt((), self.name, (), str(exc))) from exc

    def get_state_with_sequence(self):
        state, sequence = self._call('get_state_with_sequence')
        return (None if state is None else MotorFeedback.copy_from(state)), int(sequence)
    def request_feedback(self): return self._call('request_feedback')
    def send_pos_vel(self, pos, vlim): return self._send('send_pos_vel', pos, vlim)
    def send_mit(self, pos, vel, kp, kd, tau): return self._send('send_mit', pos, vel, kp, kd, tau)
    def send_vel(self, vel): return self._send('send_vel', vel)
    def enable(self): return self._call('enable')
    def disable(self): return self._call('disable')
    def ensure_mode(self, mode, timeout): return self._call('ensure_mode', mode, timeout)
    def write_register_f32(self, address, value):
        try:
            return self._call('write_register_f32', address, value)
        except Exception as exc:
            raise RuntimeError(f'{self.name} register {address} write failed: {exc}') from exc
    def set_zero_position(self): return self._call('set_zero_position')
    def close(self):
        if self._owner is not None:
            self._owner()
        with self._lock:
            if not self._closed:
                self._handle.close()
                self._closed = True

class BusPort:
    """共享控制器句柄的受锁访问入口；关闭后拒绝操作。"""
    def __init__(self, handle, lock, owner=None):
        self._handle, self.bus_lock = handle, lock
        self._owner = owner
        self._closed = False

    def _call(self, method):
        if self._owner is not None:
            self._owner()
        with self.bus_lock:
            if self._closed: raise RuntimeError('controller handle is closed')
            return getattr(self._handle, method)()

    def poll_feedback_once(self): return self._call('poll_feedback_once')
    def enable_all(self): return self._call('enable_all')
    def disable_all(self): return self._call('disable_all')
    def shutdown(self): return self._call('shutdown')
    def close_bus(self): return self._call('close_bus')
    def close(self):
        if self._owner is not None:
            self._owner()
        with self.bus_lock:
            if not self._closed:
                self._handle.close()
                self._closed = True

class SdkMotorDriver:
    """持有唯一总线及电机句柄；构造可注入工厂和等待函数用于离线验证。"""
    def __init__(self, config, *, controller_factory=None, sleep=time.sleep):
        self.config = config
        self._factory = controller_factory
        self._sleep = sleep
        # 始终先取得命令锁再取得总线锁。反馈只使用总线锁，
        # 因此较长的稳定等待期间仍能采集反馈。
        self._command_lock = threading.RLock()
        self.bus_lock = threading.RLock()
        self._bus = None
        self._motors = {}
        self.mode = 'unknown'
        self.owner = None
        self.last_receipt = None

    @staticmethod
    def _open_controller(channel, baud):
        from motorbridge import Controller
        if channel.startswith(('/dev/tty', '/dev/pts/')):
            return Controller.from_dm_serial(channel, baud)
        return Controller(channel)

    @property
    def connected(self): return self._bus is not None

    def connect(self):
        """打开总线并注册七个电机；注册中途失败时释放已取得的资源。"""
        if self.owner is not None:
            self.owner()
        with self._command_lock, self.bus_lock:
            if self.connected: return
            factory = self._factory or self._open_controller
            self._bus = BusPort(factory(self.config.channel, self.config.timing.serial_baud), self.bus_lock, self.owner)
            try:
                for cfg in (*self.config.joints, self.config.gripper):
                    handle = self._bus._handle.add_damiao_motor(cfg.motor_id, cfg.feedback_id, cfg.model)
                    self._motors[cfg.name] = MotorPort(cfg.name, handle, self.bus_lock, self.owner)
            except BaseException:
                self.close()
                raise
            self.mode = 'unknown'

    def motor(self, name):
        if not self.connected: raise RuntimeError('hardware is not connected')
        return self._motors[name]

    def controllers(self): return () if self._bus is None else (self._bus,)
    def feedback_groups(self):
        """返回共享控制器上的六轴反馈组；夹爪由管理器单独加入采集组。"""
        if not self.connected: raise RuntimeError('hardware is not connected')
        return [(self._bus, [(j.name, self.motor(j.name)) for j in self.config.joints])]

    def _confirm_status(self, names, expected, retries=None, interval=None):
        # 状态码必须来自基线之后的新反馈，不能用旧缓存确认使能或失能。
        timing = self.config.timing
        retries = timing.status_retries if retries is None else retries
        interval = timing.status_interval if interval is None else interval
        baseline = {name: self.motor(name).get_state_with_sequence()[1] for name in names}
        pending = set(names)
        for _ in range(retries):
            with self.bus_lock:
                for name in pending: self.motor(name).request_feedback()
                self._bus.poll_feedback_once()
                for name in tuple(pending):
                    state, seq = self.motor(name).get_state_with_sequence()
                    # 按无符号 64 位序号差判断前进，允许计数器回绕。
                    advanced = 0 < ((seq-baseline[name]) & ((1 << 64)-1)) < (1 << 63)
                    if state is not None and advanced and state.status_code == expected:
                        pending.remove(name)
            if not pending: return
            self._sleep(interval)
        raise RuntimeError(f'fresh status_code={expected} not confirmed: {sorted(pending)}')

    def _set_enabled(self, enabled):
        # 驱动失能不负责停止或等待控制线程。
        with self._command_lock:
            if not self.connected: raise RuntimeError('hardware is not connected')
            # 端口只在实际写入时持有总线锁；确认期间保留命令锁，
            # 并在各次总线事务之间允许反馈采集。
            (self._bus.enable_all if enabled else self._bus.disable_all)()
            self._sleep(self.config.timing.command_motor_delay)
            self._confirm_status(tuple(j.name for j in self.config.joints), int(enabled))

    def enable(self): self._set_enabled(True)
    def disable(self): self._set_enabled(False)

    def set_mode(self, mode):
        """逐轴设置模式，任一轴失败时停止并保留部分操作回执。"""
        from motorbridge import Mode
        modes = dict(mit=Mode.MIT, pos_vel=Mode.POS_VEL, vel=Mode.VEL)
        if mode not in modes: raise ValueError(f'unsupported mode: {mode}')
        with self._command_lock:
            # 未完成整组六轴切换前，不能对外声称已进入目标模式。
            self.mode = 'unknown'
            sent = []
            for index,cfg in enumerate(self.config.joints):
                try:
                    motor = self.motor(cfg.name)
                    if mode == 'pos_vel':
                        with self.bus_lock:
                            for address, field in ((25,'vel_kp'), (26,'vel_ki'), (27,'pos_kp'), (28,'pos_ki')):
                                motor.write_register_f32(address, getattr(cfg,field))
                        self._sleep(self.config.timing.register_settle)
                    motor.ensure_mode(modes[mode], self.config.timing.mode_timeout_ms)
                    self._sleep(self.config.timing.mode_motor_settle)
                except Exception as exc:
                    names = tuple(j.name for j in self.config.joints)
                    self.last_receipt = CommandReceipt(tuple(sent),cfg.name,names[index+1:],f'{mode} mode: {exc}')
                    raise CommandTransportError(self.last_receipt) from exc
                sent.append(cfg.name)
            self._sleep(self.config.timing.mode_settle)
            self.mode = mode
            return True

    def send_positions(self, command):
        """验证六轴位置和速度上限后发送；成功仅表示设备写入成功。"""
        q = vector(command.positions, 6, 'position command')
        vlim = vector(command.velocity_limits, 6, 'velocity limits')
        bounds = np.asarray(POSITION_LIMITS)
        if (q < bounds[:,0]).any() or (q > bounds[:,1]).any():
            raise ValueError('position command outside hardware limits')
        if (vlim <= 0).any() or (vlim > [j.vlim for j in self.config.joints]).any():
            raise ValueError('velocity limit exceeds reviewed configuration')
        return self._send_joint_batch('send_pos_vel',q,vlim)

    def send_mit_batch(self, pos, vel, kp, kd, tau):
        values = tuple(vector(value,6,label) for value,label in
                       zip((pos,vel,kp,kd,tau),('MIT position','MIT velocity','MIT kp','MIT kd','MIT torque')))
        return self._send_joint_batch('send_mit',*values)

    def _send_joint_batch(self, method, *values):
        sent = []
        names = tuple(j.name for j in self.config.joints)
        with self._command_lock, self.bus_lock:
            for index, name in enumerate(names):
                try:
                    getattr(self.motor(name),method)(*(float(value[index]) for value in values))
                except Exception as exc:
                    # 已发送的电机可能已响应；记录回执并立即终止，不重发整批。
                    self.last_receipt = CommandReceipt(tuple(sent),name,names[index+1:],str(exc))
                    raise CommandTransportError(self.last_receipt) from exc
                sent.append(name)
        self.last_receipt = CommandReceipt(tuple(sent))
        return self.last_receipt

    def set_zero(self, names=None):
        """失能并逐轴确认新状态后写零；调用前仍须经过管理器安全门。"""
        names = tuple(j.name for j in self.config.joints) if names is None else tuple(names)
        if any(name not in {j.name for j in self.config.joints} for name in names):
            raise KeyError(f'unknown zero target: {names}')
        with self._command_lock, self.bus_lock:
            self.disable()
            self._sleep(self.config.timing.zero_disable_settle)
            for name in names:
                self._confirm_status((name,),0,self.config.timing.zero_retries,self.config.timing.zero_poll_interval)
                self.motor(name).set_zero_position()
                self._sleep(self.config.timing.zero_motor_settle)
        return True

    def close(self):
        """尝试释放所有句柄和总线，并汇总清理错误。"""
        errors = []
        with self._command_lock, self.bus_lock:
            if self._bus is None: return
            # 先释放所有电机句柄再释放控制器；逐项尝试全部清理步骤。
            for name,motor in tuple(self._motors.items()):
                try: motor.close()
                except Exception as exc: errors.append(f'{name} free: {exc}')
            self._motors.clear()
            for method in ('shutdown','close_bus','close'):
                try:
                    getattr(self._bus,method)()
                    if method == 'shutdown': self._sleep(self.config.timing.shutdown_settle)
                except Exception as exc: errors.append(f'{method}: {exc}')
            self._bus = None
            self.mode = 'unknown'
        if errors: raise RuntimeError('; '.join(errors))

    def disconnect(self):
        """执行驱动级失能和释放；不负责停止控制线程或验证安全基线。"""
        with self._command_lock:
            errors = []
            if not self.connected: return
            try: self.disable()
            except Exception as exc: errors.append(f'disable: {exc}')
            self._sleep(self.config.timing.disconnect_settle)
            try: self.close()
            except Exception as exc: errors.append(str(exc))
            if errors: raise RuntimeError('; '.join(errors))
