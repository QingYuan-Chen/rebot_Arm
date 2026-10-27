"""Temporary boundary for legacy SDK devices and execution. Removed in batch 6.

Only this adapter may inspect RobotArm/ArmEndPos private fields. No delegation
through __getattr__, no SDK object escapes through the manager public facade.
"""
from pathlib import Path
import sys
import tempfile
import threading
import numpy as np
import yaml

def ensure_sdk():
    workspace=Path(__file__).resolve().parents[4]
    candidates=(workspace/'third_party/reBotArm_control_py',Path.cwd()/'third_party/reBotArm_control_py')
    for root in candidates:
        if (root/'reBotArm_control_py').is_dir():
            if str(root) not in sys.path: sys.path.insert(0,str(root))
            return root
    raise FileNotFoundError('Cannot find reBotArm_control_py in third_party')

class LegacyBackend:
    def __init__(self, arm, endpos):
        self._arm=arm
        self._endpos=endpos

    @classmethod
    def create(cls,config):
        ensure_sdk()
        from reBotArm_control_py.actuator import RobotArm
        from reBotArm_control_py.controllers import ArmEndPos
        # Instance-local bridge for the old path-only constructor, removed in batch 2.
        with tempfile.TemporaryDirectory(prefix='rebotarm-config-') as directory:
            path=Path(directory)/'arm.yaml'
            path.write_text(yaml.safe_dump(config.arm_mapping(),sort_keys=False))
            arm=RobotArm(cfg_path=str(path))
        try:
            backend=cls(arm,ArmEndPos(arm))
            backend.patch_bus()
            return backend
        except BaseException:
            arm.disconnect()
            raise

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
    def execution_running(self): return bool(self._endpos._running)
    def set_execution_running(self,value): self._endpos._running=bool(value)
    @property
    def home_velocity(self): return float(self._endpos._home_vel)
    @property
    def arrival_poll_period(self): return float(self._endpos._dt)
    def set_speed_override(self,value): self._endpos._vlim_override=value
    def set_target(self,value): self._endpos._q_target[:]=np.array(value,dtype=float,copy=True)
    def target(self): return self._endpos._q_target.copy()
    def cancel_trajectory(self):
        self._endpos._stop_send.set()
        self._endpos._moving=False
    def join_sender(self):
        thread=self._endpos._send_thread
        if thread is not None: thread.join(timeout=5.)
        if thread is not None and thread.is_alive(): raise RuntimeError('trajectory sender did not stop')
    @property
    def trajectory_moving(self): return bool(self._endpos._moving)
    @property
    def trajectory_progress(self):
        points=getattr(self._endpos,'_traj',())
        return min(1.,float(self._endpos._traj_idx)/len(points)) if points else 1.
    def move_to_pose_path(self,*args): return self._endpos.move_to_traj(*args)
    def move_to_pose_ik(self,*args): return self._endpos.move_to_ik(*args)
    def position_tick(self,arm,dt): self._endpos._loop_cb(arm,dt)
    def current_pose_values(self,q):
        from reBotArm_control_py.kinematics import compute_fk
        position,rotation,_=compute_fk(self._endpos._model,q)
        return position,rotation
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
    def stop_control_loop(self): self._arm.stop_control_loop()
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

