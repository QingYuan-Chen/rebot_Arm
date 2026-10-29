"""Python 控制器的组件装配.

职责与位置
    控制器装配根: 创建配置, 模型, 唯一运行时和各独立职责组件.

对外接口
    HardwareManager 构造后仅暴露 runtime, lifecycle, arm, gripper, feedback, gravity,
    cycle, protection 引用; 调用方按职责取用, 不经过业务转发门面.

主要流程
    校验配置与模型 -> 创建状态所有者 -> 构造能力端口 -> bind -> 接入生命周期和周期回调.

安全与并发约束
    构造会加载资源和数值模型, 但不连接设备, 不启动写入线程, 不自动使能.
    bind 只做一次; 回调交接环仍存在, 不代表所有组件耦合均已消除.

文件结构(按定义顺序)
    HardwareManager: 创建唯一硬件运行时并连接组件; 各组件独立拥有状态.
"""

from __future__ import annotations
from pathlib import Path
from typing import Optional

from .configuration import HardwareSettings, load_configuration, default_config
from .controller_runtime import ControllerRuntime
from .controller_kinematics import ArmKinematics
from .gravity_control import GravitySettings, load_gravity_model, package_file
from .lifecycle import ControllerLifecycle, LifecyclePorts
from .arm_controller import ArmController
from .component_ports import (
    ArmFeedbackPort, GripperFeedbackPort, GravityFeedbackPort, CycleFeedbackPort,
    GripperAcquisitionPort, ArmCyclePort, GripperCyclePort, ArmProtectionPort,
    GripperProtectionPort, GravityProtectionPort, GravityArmPort, ArmGravityPort,
    CyclePort, ProtectionPort,
)
from .gripper_controller import GripperController
from .feedback_reader import FeedbackReader
from .gravity_controller import GravityController
from .controller_cycle import ControllerCycle
from .controller_protection import ControllerProtection
from .controller_parameters import (
    _HARDWARE_FEEDBACK_RATE_HZ, _G_LARGE_MOVE_MAX_TAU_NM,
    _G_POSITION_MAX_SPEED_RAD_S, _G_POSITION_TIMEOUT_MARGIN_SEC,
    _G_FEEDBACK_STALE_TIMEOUT_SEC, _G_GRASP_HOLD_TIMEOUT_DEFAULT_SEC,
)


class HardwareManager:
    """创建唯一硬件运行时并连接组件; 各组件独立拥有状态."""

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
        """校验资源并一次性创建, 接线全部组件; 不建立设备会话.

        arm_cfg / gripper_cfg / gravity_config 为空时使用包内资源, channel 仅覆盖内存配置.
        runtime_factory 供离线替身注入; 真实运行时, 生命周期及所有组件必须共享同一设备所有权.
        先创建全部状态所有者, 再构造冻结能力端口, 最后接入生命周期与空闲采集回调.
        调用方通过 lifecycle.connect() 显式连接, manager 不提供业务转发.
        """
        settings = HardwareSettings(hardware_feedback_rate_hz, gripper_position_torque_cap_nm,
            gripper_position_max_speed_rad_s, gripper_position_timeout_margin_sec,
            gripper_feedback_stale_timeout_sec, grasp_hold_timeout_sec)
        configuration = load_configuration(arm_cfg, gripper_cfg, channel)
        gravity_path = (Path(gravity_config).expanduser() if gravity_config else
                        package_file('rebotarmcontroller', 'config/gravity_compensation.yaml'))
        model = load_gravity_model(package_file('rebotarm_moveit_config', 'config/rebotarm.urdf'),
                                   GravitySettings.from_yaml(gravity_path))
        # 先创建状态所有者, 后接线; 构造期间只有资源加载和数值校验, 没有总线会话.
        self.runtime = runtime_factory(configuration, ArmKinematics(model.model))
        self.lifecycle = ControllerLifecycle(self.runtime)
        self.arm = ArmController(self.runtime, self.lifecycle, settings)
        self.gripper = GripperController(self.runtime, self.lifecycle, settings,
            configuration, Path(gripper_cfg).expanduser() if gripper_cfg else default_config('gripper'))
        self.feedback = FeedbackReader(self.runtime, self.lifecycle, settings)
        self.gravity = GravityController(self.runtime, self.lifecycle, settings, model)
        self.cycle = ControllerCycle(self.runtime, self.lifecycle, settings)
        self.protection = ControllerProtection(self.runtime, self.lifecycle, settings)
        # 按消费者职责裁剪能力, 直接绑定公开方法; 不向组件传整个 manager 或完整对端组件.
        arm_feedback_port = ArmFeedbackPort(
            arm_failure=self.feedback.arm_failure,
            get_joint_state=self.feedback.get_joint_state,
            joint_snapshot=self.feedback.joint_snapshot,
            verified_sample=self.feedback.verified_sample,
        )
        gripper_feedback_port = GripperFeedbackPort(
            cached_motor=self.feedback.cached_motor,
            failure_reason=self.feedback.failure_reason,
            refresh_feedback_if_due=self.feedback.refresh_feedback_if_due,
            transaction=self.feedback.transaction,
            verified_sample=self.feedback.verified_sample,
        )
        gravity_feedback_port = GravityFeedbackPort(
            arm_failure=self.feedback.arm_failure,
            refresh_feedback_if_due=self.feedback.refresh_feedback_if_due,
            transaction=self.feedback.transaction,
            validate_sample=self.feedback.validate_sample,
            verified_samples=self.feedback.verified_samples,
        )
        cycle_feedback_port = CycleFeedbackPort(
            arm_failure=self.feedback.arm_failure,
            joint_snapshot=self.feedback.joint_snapshot,
            refresh_active=self.feedback.refresh_active,
            refresh_feedback_if_due=self.feedback.refresh_feedback_if_due,
            reset_session=self.feedback.reset_session,
        )
        gripper_acquisition_port = GripperAcquisitionPort(
            feedback_binding=self.gripper.feedback_binding,
            feedback_values=self.gripper.feedback_values,
            record_feedback=self.gripper.record_feedback,
            record_feedback_error=self.gripper.record_feedback_error,
            cached_feedback=self.gripper.cached_feedback,
            feedback_transaction=self.gripper.feedback_transaction,
        )
        arm_cycle_port = ArmCyclePort(
            reset_session=self.arm.reset_session,
            clear_passthrough=self.arm.clear_passthrough,
            expire_passthrough=self.arm.expire_passthrough,
            check_task_generation=self.arm.check_task_generation,
            passthrough_active=self.arm.passthrough_active,
            update_task=self.arm.update_task,
        )
        gripper_cycle_port = GripperCyclePort(
            clear_passthrough=self.gripper.clear_passthrough,
            expire_passthrough=self.gripper.expire_passthrough,
            reset_session=self.gripper.reset_session,
            tick=self.gripper.tick,
            transport_error=self.gripper.transport_error,
        )
        arm_protection_port = ArmProtectionPort(
            clear_passthrough=self.arm.clear_passthrough,
            abort_task=self.arm.abort_task,
        )
        gripper_protection_port = GripperProtectionPort(
            clear_passthrough=self.gripper.clear_passthrough,
            prepare_protection=self.gripper.prepare_protection,
            feedback_binding=self.gripper.feedback_binding,
        )
        gravity_protection_port = GravityProtectionPort(
            clear_execution=self.gravity.clear_execution,
        )
        gravity_arm_port = GravityArmPort(
            stop_gravity_compensation=self.gravity.stop_gravity_compensation,
            gravity_compensation_active=self.gravity.gravity_compensation_active,
            effort_limit=self.gravity.effort_limit,
        )
        arm_gravity_port = ArmGravityPort(
            clear_passthrough=self.arm.clear_passthrough,
            require_available=self.arm.require_available,
            start_position_loop=self.arm.start_position_loop,
            stop_control_loop=self.arm.stop_control_loop,
        )
        cycle_port = CyclePort(
            control_tick=self.cycle.control_tick,
        )
        protection_port = ProtectionPort(
            protect=self.protection.protect,
        )
        # 组件全部存在后才完成一次绑定, 避免循环依赖要求构造中访问尚未就绪的对端.
        self.feedback.bind(gripper=gripper_acquisition_port)
        self.gripper.bind(feedback=gripper_feedback_port, protection=protection_port)
        self.arm.bind(cycle=cycle_port, feedback=arm_feedback_port, gravity=gravity_arm_port, protection=protection_port)
        self.gravity.bind(arm=arm_gravity_port, cycle=cycle_port, feedback=gravity_feedback_port, protection=protection_port)
        self.cycle.bind(arm=arm_cycle_port, feedback=cycle_feedback_port, gripper=gripper_cycle_port, protection=protection_port)
        self.protection.bind(arm=arm_protection_port, gravity=gravity_protection_port, gripper=gripper_protection_port)
        # 生命周期只接收事务所需能力; 状态与锁仍留在各所有者中, 不建立第二套生命周期门面.
        self.lifecycle.ports = LifecyclePorts(
            device=self.runtime,
            joint_feedback=self.feedback.validated_joints,
            gripper_status=self.gripper.validate_status,
            gripper_feedback_status=self.gripper.feedback_status,
            gripper_motor=lambda: self.gripper.feedback_binding()[1],
            gripper_coordinate_failure=self.gripper.coordinate_failure,
            prepare_gripper_enable=self.gripper.prepare_enable,
            clear_gripper_transport_error=self.gripper.clear_transport_error,
            reset_session=self.cycle.reset_session,
            initialize_gripper=self.gripper.initialize,
            stop_gripper=self.gripper.stop_loop,
            stop_control=self.arm.stop_control_loop,
            stop_gravity=self.gravity.stop_gravity_compensation,
            clear_passthrough=self.cycle.clear_passthrough,
            hold=self.arm.hold_current_position,
            start_position=self.arm.start_position_loop,
            zero_gripper=self.gripper.set_zero,
            confirm_disabled=self.protection.confirm_disabled,
            protect=self.protection.protect,
        )
        self.runtime.idle_callback = self.cycle.feedback_tick
        self.runtime.on_error = lambda exc: self.protection.protect(
            str(exc), category="CONTROLLER")
