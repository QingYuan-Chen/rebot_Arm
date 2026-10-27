"""控制器最小周期运行时及过渡期设备门面。

``ControllerRuntime`` 装配设备驱动、六轴目标执行器和位置命令生成器，
提供连接、目标/取消、轨迹进度、模式和设备操作的转发入口。
``start_control_loop`` 使用调用方提供的周期回调启动唯一写入线程；
``stop_control_loop`` 请求停止并有界等待线程退出。``position_tick``
采样目标、发送位置命令，成功后确认轨迹游标。

当前反馈、夹爪及生命周期保护仍由 ``HardwareManager`` 协调；这些设备
调用尚未全部进入统一请求队列，完整仲裁留到批次 6。启动线程不等于使能，
使能前的现场检查、新鲜反馈与显式服务调用由上层负责。旧位姿入口暂经
``legacy_backend`` 计算，后续逆运动学和路径生成归 motion。
"""
from __future__ import annotations
import math
import threading
import time
from .sdk_driver import SdkMotorDriver
from .arm_execution import ArmExecution
from .arm_position_control import ArmPositionControl

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
        return self._thread is not None and self._thread.is_alive()
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

    def request_loop_stop(self):
        """发出停止请求并取消当前轨迹；此方法不等待线程退出。"""
        self._stop.set()
        with self.execution.lock:
            self.execution.running = False
            self.execution.cancel()

    def stop_control_loop(self):
        """请求停止并等待其他线程退出；超时则报错，不能启动替代线程。"""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=self.config.timing.join_timeout)
            if thread.is_alive(): raise RuntimeError('hardware control loop did not stop')
        self.request_loop_stop()

    def start_control_loop(self,callback,rate=None):
        """以指定频率启动回调；异常会停止循环并交给保护回调处理。"""
        # 更换写入线程前先在调用方校验频率。
        requested_rate = self.rate if rate is None else float(rate)
        if not math.isfinite(requested_rate) or requested_rate <= 0:
            raise ValueError('control rate must be finite and positive')
        # 确认旧写入线程退出后才能启动新线程。
        with self._thread_lock:
            if self._thread is threading.current_thread():
                raise RuntimeError('cannot replace the hardware writer from itself')
            self.stop_control_loop()
            self.last_error = None
            self._stop.clear()
            self.execution.running = True
            period = 1./requested_rate
            def run():
                deadline = time.monotonic()
                try:
                    while not self._stop.is_set():
                        callback(self,period)
                        deadline += period
                        self._stop.wait(max(0.,deadline-time.monotonic()))
                        # 超过一个周期时重设基准，避免连续补跑积压周期。
                        if deadline < time.monotonic()-period: deadline = time.monotonic()
                except Exception as exc:
                    self.last_error = str(exc)
                    self.request_loop_stop()
                    if self.on_error is not None:
                        try: self.on_error(exc)
                        except Exception as protection_error:
                            self.last_error += f'; protection failed: {protection_error}'
                finally:
                    self._stop.set()
                    self.execution.running = False
            self._thread = threading.Thread(target=run,name='rebotarm-hardware',daemon=True)
            self._thread.start()

    def move_to_pose_ik(self,*pose):
        """使用临时计算桥求目标；求解期间被取消则不接纳旧结果。"""
        if not self.execution.running: return False
        from .legacy_backend import solve_pose_ik
        generation = self.execution.generation
        q = self.feedback_reader()[0]
        solution = solve_pose_ik(self.kinematics,q,pose)
        if solution is None: return False
        with self.execution.lock:
            if generation!=self.execution.generation or not self.execution.running: return False
            self.execution.set_target(solution)
        return True

    def move_to_pose_path(self,*args):
        """计算并尝试提交旧位姿路径；返回值表示接纳，不表示执行完成。"""
        if not self.execution.running: return False
        from .legacy_backend import plan_pose_path
        generation = self.execution.generation
        q = self.feedback_reader()[0]
        path = plan_pose_path(self.kinematics,q,args[:6],float(args[6]),self.arrival_poll_period)
        if path is None: return False
        points,times = path
        return self.execution.submit(points,times,generation)
