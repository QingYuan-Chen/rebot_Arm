"""底层硬件服务适配: 将 ROS 请求交给对应职责组件并回填响应.

职责与位置
    硬件包的显式事务入口, 负责注册服务, 适配参数和刷新 arm_status.
    本模块不做规划, 抓取策略或设备通信; 真实操作由注入的职责端口处理.

对外接口
    服务前缀为 /<arm_namespace>/, 默认命名空间为 rebotarm:
        enable / disable: 显式使能或失能七路电机.
        safe_home: 退出重力补偿, 提交预设或显式配置的停靠目标.
        trajectory_stop: 请求六轴停止保持并等待确认.
        gravity_compensation/start|stop: 开始或结束重力补偿.
        set_zero: 请求六轴, 指定关节或夹爪置零.
        set_mode: 请求模式切换.
        gripper/set: 请求夹爪位置目标并等待结果.
        gripper/grasp: 请求闭合堵转检测及有界保持.

安全与并发约束
    连接成功后默认失能; 显式使能所需的反馈与生命周期准入由组件验证.
    正常失能前的支撑和安全基线由调用方确认, 本适配层不检查现场条件.
    safe_home 不执行路径规划或碰撞检查, 不能用名称代替实际路径与基线验证.
    slow_group 串行处理生命周期, 模式, 置零, 停靠和重力服务.
    reentrant_group 允许停止与夹爪服务并发进入; 所有设备命令仍经唯一运行时仲裁,
    回调并发不代表可以中断已开始的底层事务.
    职责组件调用的异常转换为失败响应; ROS 通信和后续状态发布不在该捕获范围内.

文件结构(按定义顺序)
    ArmServices: 随宿主节点注册服务, 按命令职责调用端口并映射结果.
"""

from __future__ import annotations

import math

from rebotarm_msgs.srv import GraspGripper, SetGripper, SetMode, SetZero
from std_srvs.srv import Trigger

from ..contracts.ros_ports import LifecycleCommandPort, ArmCommandPort, GripperCommandPort, GravityCommandPort


class ArmServices:
    """底层服务适配器, 生命周期与宿主节点相同.

    回调在 ROS 执行器中运行: 调用职责端口 -> 填充 success/message -> 发布状态.
    组件调用异常会写入失败响应; 请求成功的具体条件由组件决定.
    """

    def __init__(self, node, namespace: str, *, lifecycle: LifecycleCommandPort,
                 arm: ArmCommandPort, gripper: GripperCommandPort, gravity: GravityCommandPort) -> None:
        """将服务注册到宿主节点, 保存对应职责端口.

        参数
            node: 提供 slow_group/reentrant_group, 状态发布和停靠参数的宿主节点.
            namespace: 不含首尾斜杠的服务命名空间, 例如 "rebotarm".
            lifecycle/arm/gripper/gravity: 已装配的原组件命令端口, 本类不复制其业务状态.
        """
        self._node = node
        self._lifecycle, self._arm = lifecycle, arm
        self._gripper, self._gravity = gripper, gravity
        self._namespace = namespace

        node.create_service(
            Trigger,
            self._service("enable"),
            self.enable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("disable"),
            self.disable,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("safe_home"),
            self.safe_home,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("trajectory_stop"),
            self.trajectory_stop,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            Trigger,
            self._service("gravity_compensation/start"),
            self.start_gravity_compensation,
            callback_group=node.slow_group,
        )
        node.create_service(
            Trigger,
            self._service("gravity_compensation/stop"),
            self.stop_gravity_compensation,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetZero,
            self._service("set_zero"),
            self.set_zero,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetMode,
            self._service("set_mode"),
            self.set_mode,
            callback_group=node.slow_group,
        )
        node.create_service(
            SetGripper,
            self._service("gripper/set"),
            self.set_gripper,
            callback_group=node.reentrant_group,
        )
        node.create_service(
            GraspGripper,
            self._service("gripper/grasp"),
            self.grasp_gripper,
            callback_group=node.reentrant_group,
        )

    def _service(self, name: str) -> str:
        """拼出 /{namespace}/{name} 绝对服务名; 部署时仍须避免同名服务重复注册."""
        return f"/{self._namespace}/{name}"

    def enable(self, _request, response):
        """使能电机: 成功后电机才具备输出力矩的能力(真机默认失能).

        失败时返回 success=False 并把底层异常文本回给调用方; 无论成败都刷新锁存的
        arm_status, 让操作界面立刻看到 enabled 状态变化.
        """
        try:
            self._lifecycle.enable()
            response.success = True
            response.message = "enabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def disable(self, _request, response):
        """显式请求七路失能并将组件的验证结果映射为服务响应.

        正常卸力前调用方须确认现场支撑和本轮安全基线; 本方法不验证这些条件.
        失败响应不能当作已失能证据, 仍须按生命周期状态处理.
        """
        try:
            self._lifecycle.disable()
            response.success = True
            response.message = "disabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def trajectory_stop(self, _request, response):
        """请求六轴停止保持, 确认成功后才返回 success=True.

        回调使用可重入组, 允许与慢服务并发进入; 请求仍受唯一设备线程和在途事务约束.
        它不取消独立的夹爪目标, 也不等同于硬件急停或失能授权.
        """
        try:
            self._arm.stop_active_motion()
            response.success = True
            response.message = "trajectory stop/hold confirmed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def safe_home(self, _request, response):
        """回到安全停靠位: 先退出重力补偿, 确保位置-速度控制就绪, 再执行硬件层的 safe_home.

        目标关节角来自节点的 safe_home_joint_positions() 参数; 该参数为空时由硬件层回退到
        规范模型的预设停靠位; 该服务不检查现场障碍, 不能用姿态名称代替现场基线. 响应
        message 按关节顺序列出实际到达角度(弧度, 保留 4 位小数), 便于现场核对.
        """
        try:
            self._gravity.stop_gravity_compensation()
            self._arm.ensure_pos_vel_control()
            reached = self._arm.safe_home(self._node.safe_home_joint_positions())
            response.success = True
            response.message = (
                "safe_home complete: "
                + ", ".join(f"{value:.4f}" for value in reached)
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def start_gravity_compensation(self, _request, response):
        """进入重力补偿模式: 电机按重力前馈输出, 操作员可以手动拖动机械臂.

        日志把锁定的目标关节角换算成角度(度)打印, 便于与现场读数对照; 锁定的目标是松开
        后判断机械臂是否回落的参考姿态.
        """
        try:
            self._gravity.start_gravity_compensation()
            target = self._gravity.gravity_compensation_target()
            if target is not None:
                deg = ", ".join(f"{math.degrees(float(v)):+.1f}" for v in target)
                self._node.get_logger().info(
                    f"gravity compensation started, lock target deg=[{deg}]"
                )
            else:
                self._node.get_logger().info("gravity compensation started")
            response.success = True
            response.message = "gravity compensation started"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def stop_gravity_compensation(self, _request, response):
        """退出重力补偿, 回到位置-速度保持状态; 未处于该模式时调用也是安全的(幂等)."""
        try:
            active = self._gravity.gravity_compensation_active()
            self._gravity.stop_gravity_compensation()
            if active:
                self._node.get_logger().info(
                    "gravity compensation stopped, returned to pos_vel hold"
                )
            else:
                self._node.get_logger().info("gravity compensation was not active")
            response.success = True
            response.message = "gravity compensation stopped"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_zero(self, request, response):
        """把当前位置标定为零点(只改编码器坐标, 不产生运动).

        request.joint_name 为空串表示全部关节一起回零; 填 "gripper" 时单独标定夹爪.
        硬件层要求此时处于"已连接但未使能"状态, 所以先退出重力补偿; 若坐标已错乱, 仍需
        允许执行回零以修复, 因此这里不做额外的位姿检查.
        """
        try:
            self._gravity.stop_gravity_compensation()
            ok = self._lifecycle.set_zero(request.joint_name)
            response.success = bool(ok)
            response.message = "set_zero complete" if ok else "set_zero failed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_mode(self, request, response):
        """切换电机控制模式: mode 取 "mit" / "pos_vel" / "vel"(硬件层负责小写化与合法性校验)."""
        try:
            self._gravity.stop_gravity_compensation()
            ok = self._lifecycle.set_mode(request.mode)
            response.success = bool(ok)
            response.message = f"mode set to {request.mode}" if ok else "mode switch incomplete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_gripper(self, request, response):
        """夹爪位置控制: 闭合/张开到指定开口距离并等待到位.

        请求字段:
            position   目标开口距离(米). 硬件层按当前配置常量夹到 [0, 0.09] 的标称行程内;
            max_effort 最大力矩(N·m). 传 <=0 时由硬件层取默认力矩.
        响应字段:
            success          是否在超时内到达目标;
            reached_position 实际到达的开口距离(米), 失败时也回填当前值.
        日志里的 reason 来自硬件层记录的命令失败原因, 无失败时打印 "-".
        """
        try:
            reached, reached_position = self._gripper.set_gripper_position(
                request.position,
                request.max_effort,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
            reason = self._gripper.gripper_command_error
            # 成功走 info, 失败走 error, 便于现场按日志级别快速筛出异常.
            log = (
                self._node.get_logger().info
                if response.success
                else self._node.get_logger().error
            )
            log(
                "gripper set "
                f"target={float(request.position):.3f}m "
                f"max_effort={float(request.max_effort):.3f}Nm "
                f"reached={response.reached_position:.3f}m "
                f"success={response.success} reason={reason or '-'}"
            )
        except Exception as exc:
            response.success = False
            response.reached_position = float(self._gripper.gripper_position_m())
            self._node.get_logger().error(
                "gripper set failed "
                f"target={float(request.position):.3f}m "
                f"max_effort={float(request.max_effort):.3f}Nm "
                f"reached={response.reached_position:.3f}m reason={exc}"
            )
        self._node.publish_arm_status()
        return response

    def grasp_gripper(self, request, response):
        """夹爪力闭合抓取: 闭合到堵转后保持夹持力, 并给出接触(堵转)判定.

        请求字段(字段名沿用历史命名, 力学量实际是夹爪电机力矩, 单位 N·m):
            close_force            闭合阶段力矩, 硬件层夹到 [0.05, 1.0] N·m;
            hold_force             保持阶段前馈力矩, 硬件层夹到 [0.05, 1.5] N·m;
            close_timeout_sec      闭合超时(秒), 超时仍未检测到堵转即判失败;
            min_close_time_sec     最短闭合时间(秒), 防止刚起步就被误判为接触;
            velocity_threshold     判定"已停住"的速度阈值(rad/s);
            min_closure_distance_m 判定接触所需的最小闭合行程(米).
        响应字段:
            success / contact_detected  是否完成闭合, 是否检测到接触;
            contact_position / reached_position 接触位置与实际位置(米);
            hold_force                  实际保持力矩(N·m);
            message                     硬件层说明文本(含保持时长上限).
        该夹爪没有力传感器, "接触"是由闭合行程加速度堵转推断出来的, 不是实测接触力.
        """
        try:
            (
                success,
                contact_detected,
                contact_position,
                reached_position,
                hold_force,
                message,
            ) = self._gripper.grasp_gripper(
                close_force=request.close_force,
                hold_force=request.hold_force,
                close_timeout_sec=request.close_timeout_sec,
                min_close_time_sec=request.min_close_time_sec,
                velocity_threshold=request.velocity_threshold,
                min_closure_distance_m=request.min_closure_distance_m,
            )
            response.success = bool(success)
            response.contact_detected = bool(contact_detected)
            response.contact_position = float(contact_position)
            response.reached_position = float(reached_position)
            response.hold_force = float(hold_force)
            response.message = str(message)
            self._node.get_logger().info(
                "gripper grasp "
                f"success={response.success} "
                f"contact={response.contact_detected} "
                f"contact_position={response.contact_position:.3f}m "
                f"reached={response.reached_position:.3f}m "
                f"hold_force={response.hold_force:.3f}"
            )
        except Exception as exc:
            # 失败时把各字段清零, 避免调用方把上一次的数值误当成本次结果.
            response.success = False
            response.contact_detected = False
            response.contact_position = 0.0
            response.reached_position = 0.0
            response.hold_force = 0.0
            response.message = str(exc)
            self._node.get_logger().error(f"gripper grasp failed: {exc}")
        self._node.publish_arm_status()
        return response
