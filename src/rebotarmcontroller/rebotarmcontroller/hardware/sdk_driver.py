"""基于 MotorBridge 句柄接口的达妙设备驱动.

职责与位置
    MotorBridge 的受管理设备访问层; 只实现项目正在使用的通信能力.

对外接口
    MotorPort / BusPort 包装句柄, SdkMotorDriver 提供共享七路总线和六轴批量发送;
    CommandTransportError 通过 receipt 保留部分发送事实.

安全与并发约束
    命令锁先于总线锁; 受运行时持有时全部设备调用校验唯一写入线程.
    close 会调用底层 shutdown, 可能对登记电机额外发送失能帧, 不能当作只读释放.
    驱动不检查现场支撑或安全基线; 上层生命周期必须在释放前确认失能.

文件结构(按定义顺序)
    CommandTransportError: 携带实际发送回执的设备命令错误.
    MotorPort: 一个受管理的电机句柄; 释放后所有操作均失败.
    BusPort: 共享控制器句柄的受锁访问入口; 关闭后拒绝操作.
    SdkMotorDriver: 持有唯一总线及电机句柄; 构造可注入工厂和等待函数用于离线验证.
"""

from __future__ import annotations
import threading
import time
import logging
import numpy as np
from ..contracts.controller_contracts import CommandReceipt, MotorFeedback
from ..gravity.gravity_control import vector, POSITION_LIMITS

_AUDIT = logging.getLogger('rebotarm.audit')


class CommandTransportError(RuntimeError):
    """携带实际发送回执的设备命令错误."""

    def __init__(self, receipt):
        """保存已发送, 失败和未尝试电机的回执, 异常文本不能掩盖部分写入."""
        self.receipt = receipt
        super().__init__(f"{receipt.failed}: {receipt.detail}; sent={receipt.sent}; not_attempted={receipt.not_attempted}")


class MotorPort:
    """一个受管理的电机句柄; 释放后所有操作均失败."""

    def __init__(self, name, handle, bus_lock, owner=None):
        """包装一个电机句柄, 共享总线锁和可选唯一线程断言; 不创建连接."""
        self.name, self._handle, self._lock = name, handle, bus_lock
        self._owner = owner
        self._closed = False
        self._audit_sequence = None

    def _call(self, method, *args):
        """先校验所有权, 再在总线锁内调用现有句柄并记录审计; 释放后的端口拒绝操作."""
        if self._owner is not None:
            self._owner()
        with self._lock:
            if self._closed:
                raise RuntimeError(f'{self.name}: motor handle is closed')
            try:
                result = getattr(self._handle, method)(*args)
            except Exception as exc:
                _AUDIT.info('motor_call_error', extra={'audit': dict(name=self.name,
                    method=method, args=args, error=str(exc))})
                raise
            if _AUDIT.isEnabledFor(logging.INFO):
                if method == 'get_state_with_sequence':
                    state, sequence = result
                    if state is not None and sequence != self._audit_sequence:
                        _AUDIT.info('motor_state', extra={'audit': dict(name=self.name,
                            sequence=int(sequence), position=float(state.pos), velocity=float(state.vel),
                            torque=float(state.torq), status=int(state.status_code))})
                        self._audit_sequence = sequence
                else:
                    _AUDIT.info('motor_call', extra={'audit': dict(name=self.name, method=method, args=args)})
            return result

    def _send(self, method, *args):
        """包装单电机发送错误为带实际失败回执的 CommandTransportError, 不自动重发."""
        try:
            return self._call(method, *args)
        except Exception as exc:
            raise CommandTransportError(CommandReceipt((), self.name, (), str(exc))) from exc

    def get_state_with_sequence(self):
        """复制 SDK 缓存数值并返回原接收序号; 读取本身不能使旧样本变新."""
        state, sequence = self._call('get_state_with_sequence')
        return (None if state is None else MotorFeedback.copy_from(state)), int(sequence)

    def request_feedback(self):
        """向设备请求新反馈, 不把请求写入当作应答已到达."""
        return self._call('request_feedback')

    def send_pos_vel(self, pos, vlim):
        """发送电机位置 rad 与速度上限 rad/s; 本层单句柄入口不替代上层限位准入."""
        return self._send('send_pos_vel', pos, vlim)

    def send_mit(self, pos, vel, kp, kd, tau):
        """发送 MIT 位置, 速度, PD 增益及前馈力矩; 总力矩约束由调用链计算."""
        return self._send('send_mit', pos, vel, kp, kd, tau)

    def send_vel(self, vel):
        """发送电机速度 rad/s, 模式与资源所有权由上层检查."""
        return self._send('send_vel', vel)

    def enable(self):
        """发送单电机使能, 不在此确认反馈状态."""
        return self._call('enable')

    def disable(self):
        """发送单电机失能, 不在此证明失能完成或姿态安全."""
        return self._call('disable')

    def ensure_mode(self, mode, timeout):
        """按设备超时切换并确认模式, 调用期间保持总线事务锁."""
        return self._call('ensure_mode', mode, timeout)

    def write_register_f32(self, address, value):
        """写指定浮点寄存器, 失败带电机名和地址上抛; 不自动重试整组参数."""
        try:
            return self._call('write_register_f32', address, value)
        except Exception as exc:
            raise RuntimeError(f'{self.name} register {address} write failed: {exc}') from exc

    def set_zero_position(self):
        """写入当前编码器零位; 必须由已完成失能准入的生命周期调用."""
        return self._call('set_zero_position')

    def close(self):
        """在总线锁内释放该句柄并拒绝后续操作, 不替代控制器级安全关闭."""
        if self._owner is not None:
            self._owner()
        with self._lock:
            if not self._closed:
                self._handle.close()
                self._closed = True


class BusPort:
    """共享控制器句柄的受锁访问入口; 关闭后拒绝操作."""

    def __init__(self, handle, lock, owner=None):
        """包装共享控制器句柄与总线锁, 保存可选唯一线程断言."""
        self._handle, self.bus_lock = handle, lock
        self._owner = owner
        self._closed = False

    def _call(self, method):
        """在设备所有权和存活检查后执行控制器调用, 保留失败审计."""
        if self._owner is not None:
            self._owner()
        with self.bus_lock:
            if self._closed: raise RuntimeError('controller handle is closed')
            try:
                result = getattr(self._handle, method)()
            except Exception as exc:
                _AUDIT.info('bus_call_error', extra={'audit': dict(method=method, error=str(exc))})
                raise
            _AUDIT.info('bus_call', extra={'audit': dict(method=method)})
            return result

    def poll_feedback_once(self):
        """轮询一次设备反馈; 是否形成新样本由接收序号决定."""
        return self._call('poll_feedback_once')

    def enable_all(self):
        """调用控制器的批量使能入口, 后续必须读取新帧验证."""
        return self._call('enable_all')

    def disable_all(self):
        """调用控制器的批量失能入口, 发送成功不等于七路状态已确认."""
        return self._call('disable_all')

    def shutdown(self):
        """调用底层 shutdown; 当前实现会向登记电机追加失能帧, 不是只读资源释放."""
        return self._call('shutdown')

    def close_bus(self):
        """调用底层总线关闭, 不证明设备已停稳或失能."""
        return self._call('close_bus')

    def close(self):
        """在锁内释放控制器句柄, 之后拒绝该端口操作."""
        if self._owner is not None:
            self._owner()
        with self.bus_lock:
            if not self._closed:
                self._handle.close()
                self._closed = True


class SdkMotorDriver:
    """持有唯一总线及电机句柄; 构造可注入工厂和等待函数用于离线验证."""

    def __init__(self, config, *, controller_factory=None, sleep=time.sleep):
        """保存冻结配置与工厂, 创建命令/总线锁; 构造不打开串口或注册电机."""
        self.config = config
        self._factory = controller_factory
        self._sleep = sleep
        # 始终先取得命令锁再取得总线锁. 反馈只使用总线锁,
        # 因此较长的稳定等待期间仍能采集反馈.
        self._command_lock = threading.RLock()
        self.bus_lock = threading.RLock()
        self._bus = None
        self._motors = {}
        self.mode = 'unknown'
        self.owner = None
        self.last_receipt = None

    @staticmethod
    def _open_controller(channel, baud):
        """按通道创建 MotorBridge 控制器, 串口与伪终端走达妙串口构造入口."""
        from motorbridge import Controller, Motor
        # 接口缺失必须在建立串口及注册电机前失败, 避免清理时产生设备写入.
        if not callable(getattr(Motor, 'get_state_with_sequence', None)):
            raise RuntimeError('feedback requires patched MotorBridge get_state_with_sequence()')
        if channel.startswith(('/dev/tty', '/dev/pts/')):
            return Controller.from_dm_serial(channel, baud)
        return Controller(channel)

    @property
    def connected(self):
        """查询本驱动是否持有总线端口, 不代表反馈或使能状态已验证."""
        return self._bus is not None

    def connect(self):
        """打开总线并注册七个电机; 注册中途失败时释放已取得的资源."""
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
        """返回已连接会话的指定受管理电机端口, 断连时拒绝."""
        if not self.connected: raise RuntimeError('hardware is not connected')
        return self._motors[name]

    def controllers(self):
        """返回当前唯一总线端口元组; 断连时为空."""
        return () if self._bus is None else (self._bus,)

    def feedback_groups(self):
        """返回共享控制器上的六轴反馈组; 夹爪由 FeedbackReader 合并加入采集组."""
        if not self.connected: raise RuntimeError('hardware is not connected')
        return [(self._bus, [(j.name, self.motor(j.name)) for j in self.config.joints])]

    def _confirm_status(self, names, expected, retries=None, interval=None):
        """以请求前序号为基线, 轮询指定电机的新帧直到全部达到期望状态.

        names 可为六轴子集; expected 为原始状态码. 稳定等待使用注入函数, 允许运行时继续采集.
        序号推进和状态码必须同时满足, 旧缓存不能确认使能或失能; 超时直接报告未确认.
        """
        # 状态码必须来自基线之后的新反馈, 不能用旧缓存确认使能或失能.
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
                    # 按无符号 64 位序号差判断前进, 允许计数器回绕.
                    advanced = 0 < ((seq-baseline[name]) & ((1 << 64)-1)) < (1 << 63)
                    if state is not None and advanced and state.status_code == expected:
                        pending.remove(name)
            if not pending: return
            self._sleep(interval)
        raise RuntimeError(f'fresh status_code={expected} not confirmed: {sorted(pending)}')

    def _set_enabled(self, enabled):
        """在命令锁内发送使能/失能并确认六轴新帧; 等待间隙允许采集总线反馈."""
        # 驱动失能不负责停止或等待控制线程.
        with self._command_lock:
            if not self.connected: raise RuntimeError('hardware is not connected')
            # 端口只在实际写入时持有总线锁; 确认期间保留命令锁,
            # 并在各次总线事务之间允许反馈采集.
            (self._bus.enable_all if enabled else self._bus.disable_all)()
            self._sleep(self.config.timing.command_motor_delay)
            self._confirm_status(tuple(j.name for j in self.config.joints), int(enabled))

    def enable(self):
        """执行驱动级六轴使能与新帧确认, 夹爪验证仍由生命周期负责."""
        self._set_enabled(True)

    def disable(self):
        """执行驱动级六轴失能与新帧确认, 现场卸力条件由调用方保证."""
        self._set_enabled(False)

    def set_mode(self, mode):
        """逐轴设置模式, 任一轴失败时停止并保留部分操作回执."""
        from motorbridge import Mode
        modes = dict(mit=Mode.MIT, pos_vel=Mode.POS_VEL, vel=Mode.VEL)
        if mode not in modes: raise ValueError(f'unsupported mode: {mode}')
        with self._command_lock:
            # 未完成整组六轴切换前, 不能对外声称已进入目标模式.
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
        """验证六轴位置和速度上限后发送; 成功仅表示设备写入成功."""
        q = vector(command.positions, 6, 'position command')
        vlim = vector(command.velocity_limits, 6, 'velocity limits')
        bounds = np.asarray(POSITION_LIMITS)
        if (q < bounds[:,0]).any() or (q > bounds[:,1]).any():
            raise ValueError('position command outside hardware limits')
        if (vlim <= 0).any() or (vlim > [j.vlim for j in self.config.joints]).any():
            raise ValueError('velocity limit exceeds reviewed configuration')
        return self._send_joint_batch('send_pos_vel',q,vlim)

    def send_mit_batch(self, pos, vel, kp, kd, tau):
        """将五类 MIT 输入校验为有限六轴向量, 再按关节顺序批量发送."""
        values = tuple(vector(value,6,label) for value,label in
                       zip((pos,vel,kp,kd,tau),('MIT position','MIT velocity','MIT kp','MIT kd','MIT torque')))
        return self._send_joint_batch('send_mit',*values)

    def _send_joint_batch(self, method, *values):
        """在命令锁和总线锁内逐轴发送, 首个失败即停止并保留已发/未尝试回执."""
        sent = []
        names = tuple(j.name for j in self.config.joints)
        with self._command_lock, self.bus_lock:
            for index, name in enumerate(names):
                try:
                    getattr(self.motor(name),method)(*(float(value[index]) for value in values))
                except Exception as exc:
                    # 已发送的电机可能已响应; 记录回执并立即终止, 不重发整批.
                    self.last_receipt = CommandReceipt(tuple(sent),name,names[index+1:],str(exc))
                    raise CommandTransportError(self.last_receipt) from exc
                sent.append(name)
        self.last_receipt = CommandReceipt(tuple(sent))
        return self.last_receipt

    def set_zero(self, names=None):
        """失能并逐轴确认新状态后写零; 调用前仍须经过生命周期安全门."""
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
        """尝试释放所有句柄和总线并汇总错误; 底层 shutdown 仍可能追加七路失能帧."""
        errors = []
        with self._command_lock, self.bus_lock:
            if self._bus is None: return
            # 先释放所有电机句柄再释放控制器; 逐项尝试全部清理步骤.
            for name,motor in tuple(self._motors.items()):
                try: motor.close()
                except Exception as exc: errors.append(f'{name} free: {exc}')
            self._motors.clear()
            # shutdown 本身带设备副作用, 会追加失能帧; 此入口只能在上层确认可安全释放后使用.
            for method in ('shutdown','close_bus','close'):
                try:
                    getattr(self._bus,method)()
                    if method == 'shutdown': self._sleep(self.config.timing.shutdown_settle)
                except Exception as exc: errors.append(f'{method}: {exc}')
            self._bus = None
            self.mode = 'unknown'
        if errors: raise RuntimeError('; '.join(errors))

    def disconnect(self):
        """执行驱动级失能和释放; 不负责停止控制线程或验证安全基线."""
        with self._command_lock:
            errors = []
            if not self.connected: return
            try: self.disable()
            except Exception as exc: errors.append(f'disable: {exc}')
            self._sleep(self.config.timing.disconnect_settle)
            try: self.close()
            except Exception as exc: errors.append(str(exc))
            if errors: raise RuntimeError('; '.join(errors))
