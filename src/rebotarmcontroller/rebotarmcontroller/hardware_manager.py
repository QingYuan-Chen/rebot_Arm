"""Python 控制器装配与公开兼容接口。

职责：创建配置、模型、唯一运行时及独立职责组件，注入明确的组件依赖；
公开方法只转发请求或只读查询，ROS 名称和调用签名沿用既有接口。
约束：构造不打开设备；默认失能、现场确认和显式使能规则由生命周期执行。
所有设备访问归唯一运行时。这里不保存目标、反馈、夹爪或重力状态，也不
提供可写私有状态转发、惰性装配、动态属性代理或控制算法。
"""
from __future__ import annotations
from pathlib import Path
from typing import Optional, Sequence
import numpy as np
import time
import threading
from .configuration import HardwareSettings, load_configuration, default_config
from .feedback_acquisition import _FeedbackValidationError, _JOINT_POSITION_LIMITS_RAD
from .controller_runtime import ControllerRuntime
from .controller_kinematics import ArmKinematics
from .gravity_control import GravitySettings, load_gravity_model, package_file
from .lifecycle import ControllerLifecycle, LifecyclePorts
from .device_requests import device_request
from .arm_controller import ArmController
from .gripper_controller import GripperController
from .feedback_reader import FeedbackReader
from .gravity_controller import GravityController
from .controller_cycle import ControllerCycle
from .controller_protection import ControllerProtection
from .controller_parameters import _HARDWARE_FEEDBACK_RATE_HZ
from .controller_parameters import (
    _G_LARGE_MOVE_MAX_TAU_CAP_NM,
    _G_POSITION_MAX_SPEED_MIN_RAD_S,
    _G_POSITION_MAX_SPEED_MAX_RAD_S,
    _G_POSITION_TIMEOUT_MARGIN_MAX_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_MAX_SEC,
    _HARDWARE_FEEDBACK_RATE_MIN_HZ,
    _HARDWARE_FEEDBACK_RATE_MAX_HZ,
)
# 既有公开导入名称保留；参数模块只包含无状态工具和常量。
from .controller_parameters import (
    GravityCommandError,
    _GC_EE_FRAME,
    _GC_KD,
    _GC_KP,
    _GC_TAU_SCALE,
    _GC_VEL_THRESHOLD,
    _GC_W_VEL_THRESHOLD,
    _G_ANGLE_OPEN,
    _G_CLOSED_FEEDBACK_TOL_RAD,
    _G_COORDINATE_TOL_RAD,
    _G_DEFAULT_FORCE,
    _G_FEEDBACK_STALE_TIMEOUT_SEC,
    _G_GRASP_CLOSE_FORCE_DEFAULT,
    _G_GRASP_CLOSE_FORCE_MAX,
    _G_GRASP_HOLD_FORCE_DEFAULT,
    _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
    _G_GRASP_HOLD_TIMEOUT_MAX_SEC,
    _G_GRASP_MIN_CLOSE_TIME,
    _G_GRASP_MIN_CLOSURE_M,
    _G_GRASP_TIMEOUT,
    _G_GRASP_VEL_THRESHOLD,
    _G_LARGE_MOVE_MAX_TAU_NM,
    _G_MAX_DIST_M,
    _G_POSITION_MAX_SPEED_RAD_S,
    _G_POSITION_TIMEOUT_MARGIN_SEC,
    _G_TAU_MAX,
    _G_VERIFIED_OPEN_LIMIT_M,
    _G_ZERO_VERIFY_SAMPLES,
    _G_ZERO_VERIFY_TIMEOUT_SEC,
    _LOG,
    _SAFE_HOME_ARRIVE_TOL_RAD,
    _SAFE_HOME_JOINT_POSITIONS,
    _SAFE_HOME_TIMEOUT_SEC,
    apply_gravity_compensation_tau_scale,
)


class HardwareManager:
    """唯一硬件会话的装配门面；各组件独立拥有状态。"""

    def __init__(
        self,
        arm_cfg: Optional[str] = None,
        gripper_cfg: Optional[str] = None,
        channel: str = "",
        hardware_feedback_rate_hz: float = _HARDWARE_FEEDBACK_RATE_HZ,
        gripper_position_torque_cap_nm: float = _G_LARGE_MOVE_MAX_TAU_NM,
        gripper_position_max_speed_rad_s: float = _G_POSITION_MAX_SPEED_RAD_S,
        gripper_position_timeout_margin_sec: float = _G_POSITION_TIMEOUT_MARGIN_SEC,
        gripper_feedback_stale_timeout_sec: float = _G_FEEDBACK_STALE_TIMEOUT_SEC,
        grasp_hold_timeout_sec: float = _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
        gravity_config: Optional[str] = None,
        *,
        runtime_factory=ControllerRuntime,
    ) -> None:
        settings = HardwareSettings(hardware_feedback_rate_hz, gripper_position_torque_cap_nm,
            gripper_position_max_speed_rad_s, gripper_position_timeout_margin_sec,
            gripper_feedback_stale_timeout_sec, grasp_hold_timeout_sec)
        configuration = load_configuration(arm_cfg, gripper_cfg, channel)
        gravity_path = (Path(gravity_config).expanduser() if gravity_config else
                        package_file('rebotarmcontroller', 'config/gravity_compensation.yaml'))
        model = load_gravity_model(package_file('rebotarm_moveit_config', 'config/rebotarm.urdf'),
                                   GravitySettings.from_yaml(gravity_path))
        self._backend = runtime_factory(configuration, ArmKinematics(model.model))
        self._lifecycle = ControllerLifecycle()
        self.arm = ArmController(self._backend, self._lifecycle, settings)
        self.gripper = GripperController(self._backend, self._lifecycle, settings,
            configuration, Path(gripper_cfg).expanduser() if gripper_cfg else default_config('gripper'))
        self.feedback = FeedbackReader(self._backend, self._lifecycle, settings)
        self.gravity = GravityController(self._backend, self._lifecycle, settings, model)
        self.cycle = ControllerCycle(self._backend, self._lifecycle, settings)
        self.protection = ControllerProtection(self._backend, self._lifecycle, settings)
        self.feedback.bind(gripper=self.gripper)
        self.gripper.bind(arm=self.arm, feedback=self.feedback, protection=self.protection)
        self.arm.bind(cycle=self.cycle, feedback=self.feedback, gravity=self.gravity, protection=self.protection)
        self.gravity.bind(arm=self.arm, cycle=self.cycle, feedback=self.feedback, protection=self.protection)
        self.cycle.bind(arm=self.arm, feedback=self.feedback, gripper=self.gripper, protection=self.protection)
        self.protection.bind(arm=self.arm, gravity=self.gravity, gripper=self.gripper)
        self._lifecycle.ports = LifecyclePorts(
            device=self._backend,
            joint_feedback=lambda **kw: self.feedback._validated_joint_feedback(**kw),
            gripper_status=lambda status: self.gripper._validated_gripper_status(status),
            gripper_feedback_status=lambda: (None if self.gripper._gripper_mot is None else
                                            int(self.gripper._refresh_gripper_feedback().status_code)),
            gripper_motor=lambda: self.gripper._gripper_mot,
            gripper_coordinate_failure=self.gripper._lifecycle_gripper_coordinate_failure,
            prepare_gripper_enable=self.gripper._prepare_gripper_enable,
            clear_gripper_transport_error=self.gripper.clear_transport_error,
            reset_session=self.cycle._reset_device_session,
            initialize_gripper=lambda: self.gripper.init_gripper(str(self.gripper._gripper_cfg_path)),
            stop_gripper=lambda: self.gripper._stop_gripper_loop(),
            stop_control=lambda: self.arm._stop_control_loop(),
            stop_gravity=lambda: self.gravity.stop_gravity_compensation(),
            clear_passthrough=self.cycle._clear_passthrough,
            hold=lambda: self.arm.hold_current_position(),
            start_position=lambda **kw: self.arm._start_pos_vel_loop(**kw),
            zero_gripper=lambda: self.gripper._set_gripper_zero(),
            confirm_disabled=lambda: self.protection._command_guard.confirm_disabled(),
            protect=lambda *args, **kw: self.protection._protective_disable_from_hardware_loop(*args, **kw),
        )
        self._backend.idle_callback = self.cycle._runtime_feedback_tick
        self._backend.on_error = lambda exc: self.protection._protective_disable_from_hardware_loop(
            str(exc), category="CONTROLLER")

    def default_arm_cfg(self) -> Path:
        return default_config('arm')


    def default_gripper_cfg(self) -> Path:
        return default_config('gripper')


    @property
    def joint_names(self) -> list[str]:
        return list(self._backend.joint_names)


    @property
    def mode(self) -> str:
        return str(self._backend.mode)


    @property
    def enabled(self) -> bool:
        return self._lifecycle.enabled


    @property
    def connected(self) -> bool:
        return self._lifecycle.connected


    @property
    def control_loop_active(self) -> bool:
        return bool(self._backend.control_loop_active)


    @property
    def has_gripper(self) -> bool:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.has_gripper

    @property
    def state_machine(self) -> str:
        return self._lifecycle.execution


    @property
    def lifecycle_state(self) -> str:
        return self._lifecycle.state


    @property
    def ready_for_motion(self) -> bool:
        return self._lifecycle.ready_for_motion


    @property
    def gripper_active(self) -> bool:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_active

    @property
    def gripper_mode(self) -> str:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_mode

    @property
    def error_codes(self) -> list[str]:
        """累计错误码 + 当前反馈健康状态派生出的错误项。

        反馈类错误是在读取时按最新时效性实时派生的，不会写入 ``_error_codes``；
        这样"反馈恢复"后无需清理历史即可自动消失。
        """
        return self.get_feedback_status()[1]


    def get_feedback_status(self) -> tuple[list[int], list[str]]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.get_feedback_status()

    def set_state_machine(self, state: str) -> None:
        self._lifecycle.set_execution_state(state)

    def refresh_feedback_if_due(self, *args, **kwargs):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.refresh_feedback_if_due(*args, **kwargs)

    @device_request(priority=5, ttl=10., invalidate=('device',))
    def connect(self) -> None:
        """建立新会话并确认默认失能，不启动运动输出。"""
        return self._lifecycle.connect()


    @device_request(priority=5, ttl=10., invalidate=('device',))
    def shutdown(self) -> None:
        """关闭已确认失能的会话；仍使能或状态未知时保留设备连接。"""
        return self._lifecycle.shutdown()


    def get_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.get_joint_state()

    def get_cached_joint_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.get_cached_joint_state()

    def get_cached_joint_sample(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.get_cached_joint_sample()

    def hold_current_position(self) -> np.ndarray:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.hold_current_position()

    def stop_active_motion(self, task=None) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.stop_active_motion(task)

    def submit_arm_trajectory(self, joint_names, points, times, *, home=False,
                              velocity=None, timeout=2.):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.submit_arm_trajectory(joint_names, points, times, home=home, velocity=velocity, timeout=timeout)

    def arm_execution_status(self, task):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.arm_execution_status(task)

    def set_joint_target(self, positions) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.set_joint_target(positions)

    def joint_target(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.joint_target()

    def cancel_arm_trajectory(self) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.cancel_arm_trajectory()

    @property
    def trajectory_moving(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.trajectory_moving

    @property
    def trajectory_progress(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.trajectory_progress

    def safe_home_target(self) -> np.ndarray:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.safe_home_target()

    def safe_home(self, target=None, vlim=None, timeout_sec=None) -> np.ndarray:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.safe_home(target, vlim, timeout_sec)

    @device_request(priority=5, ttl=5., invalidate=('device',))
    def enable(self) -> None:
        """显式使能并以当前位置保持；失败由生命周期组件验证回滚。"""
        return self._lifecycle.enable()


    @device_request(priority=1, ttl=10., invalidate=('device',))
    def disable(self) -> None:
        """显式失能并验证新反馈，验证失败不谎报已安全失能。"""
        return self._lifecycle.disable()


    @device_request(priority=5, ttl=5., invalidate=('device',))
    def set_mode(self, mode: str) -> bool:
        """串行交接 mit、pos_vel 或 vel 模式，传输故障进入保护。"""
        return self._lifecycle.set_mode(mode)


    @device_request(priority=5, ttl=5., invalidate=('device',))
    def set_zero(self, joint_name: str = "") -> bool:
        """仅失能时允许置零；空名称为六轴，gripper 为夹爪。"""
        return self._lifecycle.set_zero(joint_name)


    def ensure_pos_vel_control(self) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.ensure_pos_vel_control()

    def send_joint_motor_cmd(self, joint_name: str, cmd, arbitration="reject") -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.send_joint_motor_cmd(joint_name, cmd, arbitration)

    def start_gravity_compensation(self) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gravity.start_gravity_compensation()

    def stop_gravity_compensation(self) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gravity.stop_gravity_compensation()

    def gravity_compensation_active(self) -> bool:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gravity.gravity_compensation_active()

    def gravity_compensation_target(self) -> np.ndarray | None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gravity.gravity_compensation_target()

    def gravity_compensation_diagnostics(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gravity.gravity_compensation_diagnostics()

    def current_pose(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.arm.current_pose()

    def get_joint_status_codes(self) -> list[int]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.feedback.get_joint_status_codes()

    def init_gripper(self, cfg_path: str) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.init_gripper(cfg_path)

    def set_gripper_target(self, position_m: float, max_effort: float = 0.0) -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.set_gripper_target(position_m, max_effort)

    def gripper_target_timeout_sec(self) -> float:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_target_timeout_sec()

    @property
    def gripper_command_error(self) -> str | None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_command_error

    @property
    def gripper_feedback_error(self) -> str | None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_feedback_error

    def gripper_execution_status(self, generation):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_execution_status(generation)

    def wait_gripper_target(self, timeout=None, generation=None) -> bool:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.wait_gripper_target(timeout, generation)

    def set_gripper_position(self, position_m: float, max_effort: float = 0.0) -> tuple[bool, float]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.set_gripper_position(position_m, max_effort)

    def submit_grasp(
        self,
        close_force: float = _G_GRASP_CLOSE_FORCE_DEFAULT,
        hold_force: float = _G_GRASP_HOLD_FORCE_DEFAULT,
        close_timeout_sec: float = _G_GRASP_TIMEOUT,
        min_close_time_sec: float = _G_GRASP_MIN_CLOSE_TIME,
        velocity_threshold: float = _G_GRASP_VEL_THRESHOLD,
        min_closure_distance_m: float = _G_GRASP_MIN_CLOSURE_M,
        hold_timeout_sec: float | None = None,
    ) -> tuple[bool, bool, float, float, float, str]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.submit_grasp(close_force, hold_force, close_timeout_sec, min_close_time_sec, velocity_threshold, min_closure_distance_m, hold_timeout_sec)

    def grasp_gripper(self, **kwargs):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.grasp_gripper(**kwargs)

    def get_cached_gripper_sample(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.get_cached_gripper_sample()

    def get_gripper_state(self) -> tuple[float, float, float, int]:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.get_gripper_state()

    def gripper_position_m(self) -> float:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_position_m()

    def gripper_reached_target(self) -> bool:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.gripper_reached_target()

    def send_gripper_motor_cmd(self, cmd, arbitration="reject") -> None:
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.send_gripper_motor_cmd(cmd, arbitration)

    def cancel_gripper_position_command(self, *args, **kwargs):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.cancel_gripper_position_command(*args, **kwargs)

    def release_grasp_hold(self, *args, **kwargs):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.release_grasp_hold(*args, **kwargs)

    @property
    def grasp_hold_release_reason(self):
        """转发到职责组件，状态和安全规则由组件拥有。"""
        return self.gripper.grasp_hold_release_reason
