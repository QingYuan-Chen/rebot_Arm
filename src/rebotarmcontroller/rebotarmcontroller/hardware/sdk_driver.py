"""Project-owned damiao adapter over MotorBridge's opaque handle API.

Construction is pure. One Controller owns all seven Motor handles. Command
operations serialize separately from bus transactions, so feedback can proceed
during mode settling. This is not the final runtime request queue (batch 6).
No legacy RobotArm structures.
"""
from __future__ import annotations
import threading
import time
import numpy as np
from ..contracts.models import CommandReceipt, MotorFeedback
from ..gravity_control import vector, POSITION_LIMITS

class CommandTransportError(RuntimeError):
    def __init__(self, receipt):
        self.receipt = receipt
        super().__init__(f"{receipt.failed}: {receipt.detail}; sent={receipt.sent}; not_attempted={receipt.not_attempted}")

class MotorPort:
    """One owned motor handle. All methods fail after release."""
    def __init__(self, name, handle, bus_lock):
        self.name, self._handle, self._lock = name, handle, bus_lock
        self._closed = False

    def _call(self, method, *args):
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
        with self._lock:
            if not self._closed:
                self._handle.close()
                self._closed = True

class BusPort:
    def __init__(self, handle, lock):
        self._handle, self.bus_lock = handle, lock
        self._closed = False

    def _call(self, method):
        with self.bus_lock:
            if self._closed: raise RuntimeError('controller handle is closed')
            return getattr(self._handle, method)()

    def poll_feedback_once(self): return self._call('poll_feedback_once')
    def enable_all(self): return self._call('enable_all')
    def disable_all(self): return self._call('disable_all')
    def shutdown(self): return self._call('shutdown')
    def close_bus(self): return self._call('close_bus')
    def close(self):
        with self.bus_lock:
            if not self._closed:
                self._handle.close()
                self._closed = True

class SdkMotorDriver:
    def __init__(self, config, *, controller_factory=None, sleep=time.sleep):
        self.config = config
        self._factory = controller_factory
        self._sleep = sleep
        # Always acquire command ownership before the bus. Feedback uses only
        # the bus lock and must remain available during long settling delays.
        self._command_lock = threading.RLock()
        self.bus_lock = threading.RLock()
        self._bus = None
        self._motors = {}
        self.mode = 'unknown'
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
        with self._command_lock, self.bus_lock:
            if self.connected: return
            factory = self._factory or self._open_controller
            self._bus = BusPort(factory(self.config.channel, self.config.timing.serial_baud), self.bus_lock)
            try:
                for cfg in (*self.config.joints, self.config.gripper):
                    handle = self._bus._handle.add_damiao_motor(cfg.motor_id, cfg.feedback_id, cfg.model)
                    self._motors[cfg.name] = MotorPort(cfg.name, handle, self.bus_lock)
            except BaseException:
                self.close()
                raise
            self.mode = 'unknown'

    def motor(self, name):
        if not self.connected: raise RuntimeError('hardware is not connected')
        return self._motors[name]

    def controllers(self): return () if self._bus is None else (self._bus,)
    def feedback_groups(self):
        if not self.connected: raise RuntimeError('hardware is not connected')
        return [(self._bus, [(j.name, self.motor(j.name)) for j in self.config.joints])]

    def _confirm_status(self, names, expected, retries=None, interval=None):
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
                    advanced = 0 < ((seq-baseline[name]) & ((1 << 64)-1)) < (1 << 63)
                    if state is not None and advanced and state.status_code == expected:
                        pending.remove(name)
            if not pending: return
            self._sleep(interval)
        raise RuntimeError(f'fresh status_code={expected} not confirmed: {sorted(pending)}')

    def _set_enabled(self, enabled):
        # Driver disable never stops or joins a control thread.
        with self._command_lock, self.bus_lock:
            if not self.connected: raise RuntimeError('hardware is not connected')
            (self._bus.enable_all if enabled else self._bus.disable_all)()
            self._sleep(self.config.timing.command_motor_delay)
            self._confirm_status(tuple(j.name for j in self.config.joints), int(enabled))

    def enable(self): self._set_enabled(True)
    def disable(self): self._set_enabled(False)

    def set_mode(self, mode):
        from motorbridge import Mode
        modes = dict(mit=Mode.MIT, pos_vel=Mode.POS_VEL, vel=Mode.VEL)
        if mode not in modes: raise ValueError(f'unsupported mode: {mode}')
        with self._command_lock:
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
                    self.last_receipt = CommandReceipt(tuple(sent),name,names[index+1:],str(exc))
                    raise CommandTransportError(self.last_receipt) from exc
                sent.append(name)
        self.last_receipt = CommandReceipt(tuple(sent))
        return self.last_receipt

    def set_zero(self, names=None):
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
        errors = []
        with self._command_lock, self.bus_lock:
            if self._bus is None: return
            # Every Motor is freed before the Controller. Attempt all release paths.
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
        with self._command_lock:
            errors = []
            if not self.connected: return
            try: self.disable()
            except Exception as exc: errors.append(f'disable: {exc}')
            self._sleep(self.config.timing.disconnect_settle)
            try: self.close()
            except Exception as exc: errors.append(str(exc))
            if errors: raise RuntimeError('; '.join(errors))
