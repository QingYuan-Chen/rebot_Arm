"""保留旧行为样本的设备替身；实现运行时接口，不改变生产准入条件。"""
import threading
import time
from rebotarmcontroller.arm_execution import ArmExecution
import numpy as np
from rebotarmcontroller.device_requests import DeviceRequests

class FakeBackend:
    def __init__(self, arm, config):
        self._arm=arm
        self.config = config
        self.execution = ArmExecution()
        self.position_callback = lambda _runtime, _dt: None
        self.requests = DeviceRequests()
        self.in_writer = True
        self._waiting = False

    @property
    def joint_names(self): return list(self._arm.joint_names)
    @property
    def mode(self): return self._arm.mode
    @property
    def rate(self): return float(self._arm._rate)
    @property
    def control_loop_active(self): return bool(self._arm.control_loop_active)
    @property
    def control_thread(self): return getattr(self._arm,'_ctrl_thread',None)
    @property
    def execution_running(self): return self.execution.running
    def set_execution_running(self, value): self.execution.running = bool(value)
    @property
    def home_velocity(self): return self.config.timing.home_velocity
    @property
    def arrival_poll_period(self): return self.config.timing.arrival_poll_period
    def set_speed_override(self, value): self.execution.set_speed_override(value)
    def set_target(self, value): self.execution.set_target(value)
    def target(self): return np.asarray(self.execution.target())
    def cancel_trajectory(self): self.execution.cancel()
    @property
    def target_generation(self): return self.execution.generation
    @property
    def trajectory_moving(self): return self.execution.moving
    @property
    def trajectory_progress(self): return self.execution.progress
    def position_tick(self, runtime, dt): self.position_callback(runtime, dt)
    def current_pose_values(self, q): return self.kinematics.pose(q)
    def device_wait(self, duration): time.sleep(duration)
    def joint_device(self,name): return self._arm._motor_map[name]
    def joint_configuration(self,name): return next(j for j in self._arm._joints if j.name==name)
    def controllers(self): return tuple(self._arm._ctrl_map.values())
    def feedback_groups(self):
        groups=[]
        for joint in self._arm._joints:
            ctrl=self._arm._ctrl_map.get(joint.vendor)
            motor=self._arm._motor_map.get(joint.name)
            if motor is None: raise RuntimeError(f'{joint.name} feedback motor unavailable')
            if ctrl is None: raise RuntimeError(f'{joint.name} feedback controller unavailable')
            entries=next((entries for existing,entries in groups if existing is ctrl),None)
            if entries is None: entries=[];groups.append((ctrl,entries))
            entries.append((joint.name,motor))
        return groups
    def add_gripper(self,config):
        ctrl=self._arm._ctrl_map.get(config.vendor)
        if ctrl is None: raise RuntimeError(f'gripper vendor={config.vendor!r} cannot share the arm Controller')
        motor=ctrl.add_damiao_motor(config.motor_id,config.feedback_id,config.model)
        self._patch_controller_bus(ctrl)
        self._wrap_motor_bus(motor,ctrl._bus_lock)
        return motor,ctrl
    def connect(self): self._arm.connect()
    def disconnect(self): self._arm.disconnect()
    def release_disabled(self): self._arm.disconnect()
    def send_mit_batch(self,**values):
        for index,name in enumerate(self.joint_names):
            try:
                self.joint_device(name).send_mit(*(float(values[field][index]) for field in ('pos','vel','kp','kd','tau')))
            except Exception as exc:
                raise RuntimeError(f'{name} MIT send failed: {exc}') from exc
    def enable(self): self._arm.enable()
    def disable(self): self._arm.disable()
    def get_state(self): return self._arm.get_state()
    def mode_pos_vel(self): return self._arm.mode_pos_vel()
    def mode_mit(self,**kwargs): return self._arm.mode_mit(**kwargs)
    def mode_vel(self): return self._arm.mode_vel()
    def set_zero_single(self,name): return self._arm.set_zero_single(name)
    def set_zero(self): return self._arm.set_zero()
    def start_control_loop(self,callback,rate=None):
        if rate is None: self._arm.start_control_loop(callback)
        else: self._arm.start_control_loop(callback,rate=rate)
    def stop_control_loop(self):
        self._arm.stop_control_loop()
        thread = self.control_thread
        if thread is not None and thread.is_alive():
            raise RuntimeError('hardware writer did not stop; refusing a second writer')
    def request_loop_stop(self): self._arm._running=False
    def patch_bus(self):
        for ctrl in self.controllers(): self._patch_controller_bus(ctrl)
        if not hasattr(self._arm,'_bus_lock_patched'):
            for joint in self._arm._joints:
                self._wrap_motor_bus(self._arm._motor_map[joint.name],self._arm._ctrl_map[joint.vendor]._bus_lock)
            self._arm._bus_lock_patched=True

    @staticmethod
    def _patch_controller_bus(ctrl) -> None:
        """给控制器的 poll_feedback_once/enable_all/disable_all 包上可重入锁。

        这几个方法内部会做多次总线读写，必须在同一把锁内完成，否则与其他线程
        的电机命令交错会串帧。重复调用是幂等的。
        """
        if not hasattr(ctrl, "_bus_lock"):
            ctrl._bus_lock = threading.RLock()
        if hasattr(ctrl, "_bus_lock_patched"):
            return
        lock = ctrl._bus_lock

        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in ("poll_feedback_once", "enable_all", "disable_all"):
            if hasattr(ctrl, attr):
                wrapped = _wrap(getattr(ctrl, attr))
                # 标记位用于 _wrap_motor_bus 判定是否已包装，避免重复加锁。
                wrapped._rebotarm_locked = True
                setattr(ctrl, attr, wrapped)
        ctrl._bus_lock_patched = True

    @staticmethod
    def _wrap_motor_bus(mot, lock) -> None:
        """把单个电机的总线类方法包上同一把锁（幂等，已包装的跳过）。"""
        def _wrap(fn, _lock=lock):
            def _locked(*args, **kwargs):
                with _lock:
                    return fn(*args, **kwargs)

            return _locked

        for attr in (
            "send_pos_vel",
            "send_mit",
            "send_vel",
            "request_feedback",
            "enable",
            "disable",
            "ensure_mode",
            "write_register_f32",
            "set_zero_position",
        ):
            if hasattr(mot, attr) and not hasattr(getattr(mot, attr), "_rebotarm_locked"):
                wrapped = _wrap(getattr(mot, attr))
                wrapped._rebotarm_locked = True
                setattr(mot, attr, wrapped)
