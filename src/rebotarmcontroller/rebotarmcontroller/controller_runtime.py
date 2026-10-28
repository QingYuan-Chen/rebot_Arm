"""控制器唯一设备线程、周期调度与请求仲裁。

职责：串行执行连接、生命周期、透传及目标请求；在请求间运行控制周期，
失能时继续采集反馈。对外使用 ``call`` 提交有有效期及资源代次的请求，
``start/stop_control_loop`` 只切换本线程的周期回调，不创建第二个写入者。

普通请求每轮最多处理四个；停止/失能使用保留的优先级入口。较长模式及
使能握手属于独占设备事务，稳定等待通过 ``device_wait`` 继续采集反馈，
不补跑积压运动周期。已开始事务完成状态确认后再处理下一请求；底层单次
ABI 调用的最长阻塞仍受驱动超时约束，软件停止不是硬件急停。
"""
from __future__ import annotations
import math
import threading
import time
from .sdk_driver import SdkMotorDriver
from .arm_execution import ArmExecution
from .arm_position_control import ArmPositionControl
from .device_requests import DeviceRequests

class ControllerRuntime:
    """管理唯一周期线程和六轴执行状态；可注入驱动以进行离线验证。"""
    def __init__(self,config,kinematics,*,driver=None):
        self.config = config
        self.driver = SdkMotorDriver(config) if driver is None else driver
        self.execution = ArmExecution()
        self.position = ArmPositionControl(j.vlim for j in config.joints)
        self.kinematics = kinematics
        self.feedback_reader = None
        self.on_error = None
        self.last_error = None
        self._stop = threading.Event()
        self._stop.set()
        self._thread = None
        self._thread_lock = threading.RLock()
        self.requests = DeviceRequests()
        self._wake = threading.Event()
        self._closing = False
        self._callback = None
        self._period = 1. / self.rate
        self.idle_callback = None
        self._waiting = False
        if isinstance(self.driver, SdkMotorDriver):
            self.driver._sleep = self.device_wait
            self.driver.owner = self.assert_writer

    @property
    def joint_names(self): return [j.name for j in self.config.joints]
    @property
    def rate(self): return self.config.rate
    @property
    def mode(self): return self.driver.mode
    @property
    def control_thread(self): return self._thread
    @property
    def control_loop_active(self):
        return self._callback is not None and self.writer_active
    @property
    def execution_running(self): return self.execution.running
    def set_execution_running(self,value):
        with self.execution.lock: self.execution.running = bool(value)
    @property
    def home_velocity(self): return self.config.timing.home_velocity
    @property
    def arrival_poll_period(self): return self.config.timing.arrival_poll_period
    def set_speed_override(self,value): self.execution.set_speed_override(value)
    def set_target(self,value): self.execution.set_target(value)
    def begin_scoped_target(self, target, velocity):
        return self.execution.begin_scoped_target(target, velocity)
    def finish_scoped_target(self, generation, hold):
        return self.execution.finish_scoped_target(generation, hold)
    @property
    def target_generation(self): return self.execution.generation
    def target(self): return self.execution.target()
    def cancel_trajectory(self): self.execution.cancel()
    def join_sender(self): pass  # 没有独立发送线程，取消操作同步完成。
    @property
    def trajectory_moving(self): return self.execution.moving
    @property
    def trajectory_progress(self): return self.execution.progress
    def joint_device(self,name): return self.driver.motor(name)
    def joint_configuration(self,name): return next(j for j in self.config.joints if j.name==name)
    def controllers(self): return self.driver.controllers()
    def feedback_groups(self): return self.driver.feedback_groups()
    def add_gripper(self,config):
        if config!=self.config.gripper: raise ValueError('gripper configuration changed after connect')
        return self.driver.motor(config.name),self.driver.controllers()[0]
    def connect(self):
        self.driver.connect()
        self.last_error = None
        self.execution.cancel()
    def disconnect(self):
        self.stop_control_loop()
        self.driver.disconnect()
    def release_disabled(self):
        """调用方确认设备已失能后释放句柄，不再次发送失能命令。"""
        self.stop_control_loop()
        self.driver.close()
        self._closing = True
        self.requests.close()
        self._wake.set()
    def enable(self): self.driver.enable()
    def disable(self): self.driver.disable()
    def send_mit_batch(self,**values): return self.driver.send_mit_batch(**values)
    def mode_pos_vel(self): return self.driver.set_mode('pos_vel')
    def mode_mit(self,**kwargs):
        # MIT 增益属于命令值，不写入模式寄存器。
        return self.driver.set_mode('mit')
    def mode_vel(self): return self.driver.set_mode('vel')
    def set_zero_single(self,name): return self.driver.set_zero((name,))
    def set_zero(self): return self.driver.set_zero()
    def get_state(self):
        if self.feedback_reader is None: raise RuntimeError('verified feedback reader unavailable')
        raise RuntimeError('hardware is not connected; waiting for verified first frame')
    def current_pose_values(self,q): return self.kinematics.pose(q)

    def position_tick(self,_runtime,dt):
        """执行一次六轴位置发送；失败时不确认轨迹游标。"""
        # 取消或更新目标要等待正在发送的批次，并使旧代次失效；
        # 取消返回后不会再发送旧快照。
        with self.execution.lock:
            target = self.execution.sample(time.monotonic())
            command = self.position.command(target)
            self.driver.send_positions(command)
            self.execution.acknowledge(target)

    def assert_writer(self):
        if not self.in_writer:
            raise RuntimeError("device access outside runtime writer")

    @property
    def in_writer(self):
        return self._thread is threading.current_thread()

    @property
    def writer_active(self):
        return self._thread is not None and self._thread.is_alive()

    def call(self, operation, **request_options):
        """等待自己的请求；未开始的超时请求撤销，已开始的事务等待真实结果。"""
        if self.in_writer:
            return operation()
        with self._thread_lock:
            if self._closing:
                raise RuntimeError('device runtime is closing')
            if not self.writer_active:
                self._closing = False
                self._thread = threading.Thread(target=self._run, name='rebotarm-hardware', daemon=True)
                self._thread.start()
            request = self.requests.submit(operation, **request_options)
            self._wake.set()
        if not request.event.wait(max(0., request.expires_at-time.monotonic())):
            self.requests.cancel_pending(request)
            # 已开始的操作不能被当作未执行；尤其失能失败不能谎报成功。
            request.event.wait()
        if request.error is not None:
            raise request.error
        return request.result

    def device_wait(self, duration):
        """握手稳定等待期间采集真实新帧；不刷新旧时间戳，也不发送运动目标。"""
        deadline = time.monotonic() + duration
        if not self.in_writer or self._waiting:
            time.sleep(max(0., duration))
            return
        self._waiting = True
        try:
            while time.monotonic() < deadline:
                if self.idle_callback is not None:
                    self.idle_callback()
                time.sleep(min(.005, max(0., deadline-time.monotonic())))
        finally:
            self._waiting = False

    def _run(self):
        deadline = time.monotonic()
        try:
            while not self._closing:
                self._wake.clear()
                for _ in range(4):
                    request = self.requests.take()
                    if request is None:
                        break
                    try:
                        value = request.operation()
                    except Exception as exc:
                        self.requests.finish(request, error=exc)
                    else:
                        self.requests.finish(request, result=value)
                    if self._closing:
                        break
                    # 事务可能包含慢握手；最多补一次周期，不能在此追赶时间轴。
                    if time.monotonic() >= deadline:
                        break
                if self._closing:
                    break
                if time.monotonic() >= deadline:
                    try:
                        if self._callback is not None:
                            self._callback(self, self._period)
                        elif self.idle_callback is not None:
                            self.idle_callback()
                    except Exception as exc:
                        self.last_error = str(exc)
                        self.request_loop_stop()
                        if self.on_error is not None:
                            try:
                                self.on_error(exc)
                            except Exception as protection_error:
                                self.last_error += f'; protection failed: {protection_error}'
                    deadline = time.monotonic() + self._period
                self._wake.wait(max(0., deadline-time.monotonic()))
        finally:
            self.request_loop_stop()
            self.requests.close()

    def request_loop_stop(self):
        """停止周期输出并作废执行，保留设备线程以服务反馈和显式恢复。"""
        self._callback = None
        self._stop.set()
        self.execution.running = False
        self.execution.cancel()

    def stop_control_loop(self):
        """与在途写入串行交接；确认后不再运行旧回调。"""
        if self.in_writer or not self.writer_active:
            self.request_loop_stop()
        else:
            self.call(self.request_loop_stop, priority=0, invalidate=('arm',))

    def start_control_loop(self, callback, rate=None):
        """在唯一线程中安装周期回调；启动不表示已通过使能安全门。"""
        requested_rate = self.rate if rate is None else float(rate)
        if not math.isfinite(requested_rate) or requested_rate <= 0:
            raise ValueError('control rate must be finite and positive')
        def install():
            self.last_error = None
            self._period = 1. / requested_rate
            self._callback = callback
            self._stop.clear()
            self.execution.running = True
        self.call(install)

    def close_worker(self):
        """仅用于无设备测试或已释放设备的收尾，拒绝关闭仍连接的驱动。"""
        with self._thread_lock:
            if self.driver.connected:
                raise RuntimeError('release device before closing writer')
            thread = self._thread
            if thread is threading.current_thread():
                raise RuntimeError('writer cannot join itself; release through shutdown')
            self._closing = True
            self._wake.set()
            if thread is not None:
                thread.join(timeout=self.config.timing.join_timeout)
                if thread.is_alive():
                    raise RuntimeError('hardware writer did not stop')
            # 释放已完成且旧线程已退出才开放重连；关闭回调与新 connect 不能抢线程引用。
            self._closing = False
