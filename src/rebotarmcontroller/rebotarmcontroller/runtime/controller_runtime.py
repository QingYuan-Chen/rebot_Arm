"""控制器唯一设备线程，周期调度与请求仲裁。

职责与位置
    设备访问与周期调度的唯一执行者；SDK 驱动，六轴数值执行器在此衔接。

对外接口
    call 排队并等待；start/stop_control_loop 安装或撤下周期回调；
    position_tick 发送位置批次，_advance_deferred 推进停止确认续体。

主要流程
    处理有界请求批次 -> 一次控制或空闲采集 -> 推进续体 -> 等待下个周期。

安全与并发约束
    构造不连接或开线程。 设备访问须来自 rebotarm-hardware；紧急请求仍受在途底层调用阻塞限制。
    release_disabled 需要上层先确认失能；驱动 close 的底层 shutdown 仍可能额外发送失能帧。

文件结构（按定义顺序）
    ControllerRuntime：管理唯一周期线程和六轴执行状态；可注入驱动以进行离线验证。
"""

from __future__ import annotations
import math
import logging
import threading
import time
from ..hardware.sdk_driver import SdkMotorDriver
from ..arm.arm_execution import ArmExecution
from ..arm.arm_position_control import ArmPositionControl
from .device_requests import DeviceRequests, RequestRejected
from ..arm.arm_stop import CycleResult

_AUDIT = logging.getLogger('rebotarm.audit')


class ControllerRuntime:
    """管理唯一周期线程和六轴执行状态；可注入驱动以进行离线验证。"""

    def __init__(self,config,kinematics,*,driver=None):
        """装配驱动，数值执行器，请求队列与调度状态；不打开设备或启动线程。

        真实驱动的等待函数指向 device_wait，设备端口持有 assert_writer 所有权断言。
        组件通过 call 延迟启动唯一写入线程，不能另建 SDK 写入者。
        """
        self.config = config
        self.driver = SdkMotorDriver(config) if driver is None else driver
        self.execution = ArmExecution()
        self.position = ArmPositionControl(j.vlim for j in config.joints)
        self.kinematics = kinematics
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
        self._deferred = []
        if isinstance(self.driver, SdkMotorDriver):
            self.driver._sleep = self.device_wait
            self.driver.owner = self.assert_writer

    @property
    def joint_names(self):
        """按配置顺序返回六轴名称副本，夹爪不计入六轴向量。"""
        return [j.name for j in self.config.joints]

    @property
    def rate(self):
        """读取配置的硬件周期频率，单位 Hz。"""
        return self.config.rate

    @property
    def mode(self):
        """读取驱动记录的六轴模式；unknown 表示不能确认整组模式。"""
        return self.driver.mode

    @property
    def control_thread(self):
        """返回唯一硬件线程引用，供所有权检查与生命周期收尾。"""
        return self._thread

    @property
    def control_loop_active(self):
        """只有周期回调已安装且唯一线程存活时，才报告控制环活动。"""
        return self._callback is not None and self.writer_active

    @property
    def execution_running(self):
        """查询数值执行器运行标志；该标志不能替代使能和反馈健康检查。"""
        return self.execution.running

    def set_execution_running(self,value):
        """在执行锁内修改数值运行标志，不启动线程或切换设备使能。"""
        with self.execution.lock: self.execution.running = bool(value)

    @property
    def home_velocity(self):
        """返回配置的停靠速度上限 rad/s。"""
        return self.config.timing.home_velocity

    @property
    def arrival_poll_period(self):
        """返回配置的到位轮询间隔 s，供兼容等待入口使用。"""
        return self.config.timing.arrival_poll_period

    def set_speed_override(self,value):
        """委托数值执行器设置任务速度覆盖，硬件发送仍受逐轴配置上限约束。"""
        self.execution.set_speed_override(value)

    def set_target(self,value):
        """委托执行器校验并设置六轴目标，同时使旧目标代次失效。"""
        self.execution.set_target(value)

    def begin_scoped_target(self, target, velocity):
        """接纳带局部速度覆盖的目标，返回供后续所有权比较的代次。"""
        return self.execution.begin_scoped_target(target, velocity)

    def finish_scoped_target(self, generation, hold):
        """仅当代次匹配时结束局部目标并写入保持角，防止旧收尾覆盖新目标。"""
        return self.execution.finish_scoped_target(generation, hold)

    @property
    def target_generation(self):
        """读取数值执行器当前目标代次。"""
        return self.execution.generation

    @property
    def arm_resource_reserved(self):
        """准备、轨迹和停止确认共用的夹爪排他标记，由六轴所有者释放。"""
        return self.execution.reservation is not None

    def target(self):
        """返回执行器当前目标数组副本，不读取电机反馈。"""
        return self.execution.target()

    def cancel_trajectory(self):
        """使数值轨迹及其待确认快照失效；实际停止保持由六轴组件协调。"""
        self.execution.cancel()

    @property
    def trajectory_moving(self):
        """查询数值轨迹是否还有待发送点，不作为实测运动判据。"""
        return self.execution.moving

    @property
    def trajectory_progress(self):
        """返回已确认发送点比例，实际到位由轨迹任务所有者判断。"""
        return self.execution.progress

    def joint_device(self,name):
        """按名称取受所有权断言管理的电机端口；不允许绕过唯一设备线程使用。"""
        return self.driver.motor(name)

    def joint_configuration(self,name):
        """按名称查冻结电机配置，名称不存在时拒绝。"""
        return next(j for j in self.config.joints if j.name==name)

    def controllers(self):
        """返回驱动管理的总线端口集合，不创建新连接。"""
        return self.driver.controllers()

    def feedback_groups(self):
        """返回共享总线上的六轴反馈分组，供采集装配使用。"""
        return self.driver.feedback_groups()

    def add_gripper(self,config):
        """校验夹爪配置与会话一致，返回共享总线中的现有夹爪端口，不另开串口。"""
        if config!=self.config.gripper: raise ValueError('gripper configuration changed after connect')
        return self.driver.motor(config.name),self.driver.controllers()[0]

    def connect(self):
        """在设备线程连接驱动，清除运行时错误并作废旧数值轨迹；默认失能验证由生命周期完成。"""
        self.driver.connect()
        self.last_error = None
        self.execution.cancel()

    def disconnect(self):
        """停止周期后执行驱动级失能与释放；现场基线与七路确认须由上层保证。"""
        self.stop_control_loop()
        self.driver.disconnect()

    def release_disabled(self):
        """释放调用方已确认失能的会话，并关闭队列以结束写入线程。

        此处不主动调用 driver.disable，但 driver.close 的底层 shutdown 仍会额外发送失能帧。
        因而这是带设备副作用的释放入口，不能用于未知使能状态的只读探测收尾。
        """
        self.stop_control_loop()
        # 句柄释放后不能在 shutdown 等待中重入反馈采集；失败时恢复等待状态。
        waiting = self._waiting
        self._waiting = True
        try:
            self.driver.close()
        finally:
            self._waiting = waiting
        self._closing = True
        self.requests.close()
        self._wake.set()

    def enable(self):
        """执行驱动级六轴使能及确认；夹爪和完整回滚事务由生命周期拥有。"""
        self.driver.enable()

    def disable(self):
        """执行驱动级六轴失能及确认；本层不证明安全基线或七路全部失能。"""
        self.driver.disable()

    def send_mit_batch(self,**values):
        """将六轴 MIT 数值交给驱动批量发送，异常保留部分发送回执。"""
        return self.driver.send_mit_batch(**values)

    def mode_pos_vel(self):
        """请求驱动将六轴切到位置速度模式；不在这里开启周期或重设目标。"""
        return self.driver.set_mode('pos_vel')

    def mode_mit(self,**kwargs):
        """请求驱动切到 MIT；增益属于后续命令值，不在模式切换时写成参数寄存器。"""
        # MIT 增益属于命令值，不写入模式寄存器。
        return self.driver.set_mode('mit')

    def mode_vel(self):
        """请求驱动切到速度模式，准入与停止交接由生命周期负责。"""
        return self.driver.set_mode('vel')

    def set_zero_single(self,name):
        """委托驱动置零指定六轴关节；调用前必须经过生命周期的失能准入。"""
        return self.driver.set_zero((name,))

    def set_zero(self):
        """委托驱动逐轴置零全部六轴，不包含夹爪独立零位验证。"""
        return self.driver.set_zero()

    def current_pose_values(self,q):
        """用给定关节角计算 FK 数值，返回位置与旋转副本，不隐式读反馈。"""
        return self.kinematics.pose(q)

    def position_tick(self,_runtime,dt):
        """执行一次六轴位置发送；失败时不确认轨迹游标。"""
        # 取消或更新目标要等待正在发送的批次，并使旧代次失效；
        # 取消返回后不会再发送旧快照。
        with self.execution.lock:
            # 使用当前单调时刻而非 dt 累加，调度延迟不会形成待补发的轨迹积压。
            target = self.execution.sample(time.monotonic())
            self.send_position_target(target)

    def send_position_target(self, target):
        """发送设备周期已经采样的六轴候选；协同执行不在本方法再次读取时钟。"""
        with self.execution.lock:
            if target.generation != self.execution.generation:
                raise RuntimeError('position target superseded before send')
            command = self.position.command(target)
            try:
                receipt = self.driver.send_positions(command)
            except Exception as exc:
                receipt = getattr(exc, 'receipt', None)
                _AUDIT.info('position_receipt', extra={'audit': dict(generation=target.generation,
                    positions=command.positions, velocity_limits=command.velocity_limits,
                    sent=() if receipt is None else receipt.sent, success=False, error=str(exc))})
                raise
            _AUDIT.info('position_receipt', extra={'audit': dict(generation=target.generation,
                positions=command.positions, velocity_limits=command.velocity_limits,
                sent=receipt.sent, success=receipt.success)})
            # 驱动部分发送失败会携带回执抛错；不能推进游标或污染最后成功发送的保持值。
            self.execution.acknowledge(target)

    def assert_writer(self):
        """拒绝来自唯一设备线程以外的 SDK 访问。"""
        if not self.in_writer:
            raise RuntimeError("device access outside runtime writer")

    @property
    def in_writer(self):
        """查询当前调用是否已经位于唯一设备线程，决定是否可安全嵌套直调。"""
        return self._thread is threading.current_thread()

    @property
    def writer_active(self):
        """查询设备线程是否存活；不等同于使能或周期输出活动。"""
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
        # 等待发生在线程锁外，设备线程可继续取请求；已经开始的操作必须等真实结算。
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
        """唯一写入线程的主循环，交替处理请求，周期与停止续体。

        普通请求每轮最多处理四个；慢握手结束后最多补一拍，不追赶积压轨迹时间轴。
        CycleResult 保留原请求等待者，设备线程继续服务反馈和高优先级请求。
        周期异常先撤下输出，再交给错误回调；退出时作废队列并结算未完成续体。
        """
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
                        if isinstance(value, CycleResult):
                            # 事务已接纳，等待者仍等自己的事件；写入线程继续服务周期和紧急请求。
                            # 操作本身可能已作废旧代次；续体绑定操作完成后的代次，后续交接再使其失效。
                            with self.requests.lock:
                                generations = tuple((name, self.requests.generations[name])
                                                    for name, _ in request.generations)
                            self._deferred.append((request, value, generations))
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
                    # 回调耗时计入本周期；错过下一拍时重新排期，避免追赶积压输出。
                    deadline += self._period
                    finished = time.monotonic()
                    if deadline <= finished:
                        deadline = finished + self._period
                # 停止证据以续体逐拍推进，避免在唯一设备线程内睡眠等静止而阻塞显式失能。
                self._advance_deferred()
                self._wake.wait(max(0., deadline-time.monotonic()))
        finally:
            self.request_loop_stop()
            self.requests.close()
            self._advance_deferred(closing=True)

    def _advance_deferred(self, *, closing=False):
        """周期推进已开始的停止事务；代次失效时禁止执行旧续体。"""
        for item in tuple(self._deferred):
            request, value, generations = item
            try:
                with self.requests.lock:
                    obsolete = any(self.requests.generations[name] != generation
                                   for name, generation in generations)
                if closing or obsolete:
                    raise RequestRejected('deferred execution superseded or runtime closed')
                # 在队列锁外推进续体，其他线程可继续提交停止或失能请求。
                done, result = value.advance()
                if not done:
                    continue
            except Exception as exc:
                try:
                    value.cancel(str(exc))
                except Exception as cleanup_error:
                    exc = RuntimeError(f'{exc}; deferred cleanup failed: {cleanup_error}')
                finally:
                    self.requests.finish(request, error=exc)
            else:
                self.requests.finish(request, result=result)
            self._deferred.remove(item)

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
            """在设备线程安装新周期与回调；清运行时错误不代表完成显式使能。"""
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
