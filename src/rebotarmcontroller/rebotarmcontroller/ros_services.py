"""底层硬件服务层：把 ROS 2 服务请求翻译成真实的电机操作。

职责与位置
    本模块属于硬件包，是"最后一道执行安全"的服务入口：只做请求转发、调用硬件管理层、
    回填响应，并在每次调用结束后刷新锁存的 arm_status。它不做运动规划、不做抓取策略、
    不管理记录文件，也不直接拼接电机 SDK 调用——所有真实硬件动作都交给 HardwareManager。

对外接口（服务名统一为 /<命名空间>/<名称>，命名空间来自 arm_namespace 参数，默认 rebotarm）
    enable / disable                 标准触发器服务：使能、失能全部电机；
    safe_home                        标准触发器服务：退出重力补偿并回到安全停靠位；
    trajectory_stop                  标准触发器服务：停止当前运动并保持当前位置；
    gravity_compensation/start|stop  标准触发器服务：进入/退出手动拖拽（重力补偿）模式；
    set_zero                         自定义服务：把全部关节或单个关节的当前位置标定为零点；
    set_mode                         自定义服务：切换电机控制模式；
    gripper/set                      自定义服务：夹爪位置控制；
    gripper/grasp                    自定义服务：夹爪力闭合抓取（堵转检测）。

回调分组（决定并发行为，不要随意改动）
    node.slow_group      互斥组。使能/失能、回零、模式切换、安全回零、重力补偿等会改变硬件
                         状态且不允许并发执行的操作放在这里，避免多条命令同时抢串口总线。
    node.reentrant_group 可重入组。停止类（trajectory_stop）与夹爪操作必须能在慢操作
                         进行中被调用，否则操作员无法及时中止运动。

安全约束
    1. 真机上电默认失能；只有显式调用 enable 且反馈新鲜后电机才会输出力矩。
    2. 所有会切换控制模式的入口（safe_home / set_zero / set_mode）都先停止
       重力补偿：重力补偿工作在 MIT 模式下，不先退出会与位置控制互相抢总线。
    3. 回调内部不抛出异常：任何底层异常都翻译成 success=False 加异常文本，保证服务端始终有响应。
"""

from __future__ import annotations

import math

from rebotarm_msgs.srv import GraspGripper, SetGripper, SetMode, SetZero
from std_srvs.srv import Trigger



class ArmServices:
    """注册并实现机械臂的全部底层服务（随节点构造一次，生命周期与节点相同）。

    每个回调都运行在 ROS 2 执行器线程上，流程统一为：调用 HardwareManager -> 填充
    response.success / response.message -> 刷新 arm_status。异常一律被捕获并转成失败响应，
    避免单个失败请求影响服务端存活。
    """

    def __init__(self, node, hardware, namespace: str) -> None:
        """把本类的方法注册为 node 上的服务。

        参数：
            node      宿主节点，需提供 slow_group / reentrant_group 回调组、
                      publish_arm_status() 与 safe_home_joint_positions()；
            hardware  HardwareManager 实例，所有真实电机操作都经它下发；
            namespace 服务名前缀（不含斜杠），例如 "rebotarm"。
        """
        self._node = node
        self._hardware = hardware
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
        """拼出绝对服务名 /{namespace}/{name}，保证与其它节点同名服务不冲突。"""
        return f"/{self._namespace}/{name}"

    def enable(self, _request, response):
        """使能电机：成功后电机才具备输出力矩的能力（真机默认失能）。

        失败时返回 success=False 并把底层异常文本回给调用方；无论成败都刷新锁存的
        arm_status，让操作界面立刻看到 enabled 状态变化。
        """
        try:
            self._hardware.enable()
            response.success = True
            response.message = "enabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def disable(self, _request, response):
        """失能电机：撤销使能，机械臂进入不输出力矩的失能状态。

        注意失能后姿态由重力与机械结构决定，因此调用方应先确认机械臂已处于安全停靠位。
        """
        try:
            self._hardware.disable()
            response.success = True
            response.message = "disabled"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def trajectory_stop(self, _request, response):
        """停止当前运动：清掉运动标志并让电机锁定在当前位置。

        注册在可重入组，使操作员在慢操作（回零、模式切换）进行中仍能及时中止运动；
        这是软件层停止，不等同于硬件急停。
        """
        try:
            self._hardware.stop_active_motion()
            response.success = True
            response.message = "trajectory stop/hold confirmed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def safe_home(self, _request, response):
        """回到安全停靠位：先退出重力补偿、确保位置-速度控制就绪，再执行硬件层的 safe_home。

        目标关节角来自节点的 safe_home_joint_positions() 参数；该参数为空时由硬件层回退到
        内置的无碰撞停靠位（不能停在全部为 0 的折叠姿态，否则规划器判定自碰撞）。响应
        message 按关节顺序列出实际到达角度（弧度，保留 4 位小数），便于现场核对。
        """
        try:
            self._hardware.stop_gravity_compensation()
            self._hardware.ensure_pos_vel_control()
            reached = self._hardware.safe_home(self._node.safe_home_joint_positions())
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
        """进入重力补偿模式：电机按重力前馈输出，操作员可以手动拖动机械臂。

        日志把锁定的目标关节角换算成角度（度）打印，便于与现场读数对照；锁定的目标是松开
        后判断机械臂是否回落的参考姿态。
        """
        try:
            self._hardware.start_gravity_compensation()
            target = self._hardware.gravity_compensation_target()
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
        """退出重力补偿，回到位置-速度保持状态；未处于该模式时调用也是安全的（幂等）。"""
        try:
            active = self._hardware.gravity_compensation_active()
            self._hardware.stop_gravity_compensation()
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
        """把当前位置标定为零点（只改编码器坐标，不产生运动）。

        request.joint_name 为空串表示全部关节一起回零；填 "gripper" 时单独标定夹爪。
        硬件层要求此时处于"已连接但未使能"状态，所以先退出重力补偿；若坐标已错乱，仍需
        允许执行回零以修复，因此这里不做额外的位姿检查。
        """
        try:
            self._hardware.stop_gravity_compensation()
            ok = self._hardware.set_zero(request.joint_name)
            response.success = bool(ok)
            response.message = "set_zero complete" if ok else "set_zero failed"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_mode(self, request, response):
        """切换电机控制模式：mode 取 "mit" / "pos_vel" / "vel"（硬件层负责小写化与合法性校验）。"""
        try:
            self._hardware.stop_gravity_compensation()
            ok = self._hardware.set_mode(request.mode)
            response.success = bool(ok)
            response.message = f"mode set to {request.mode}" if ok else "mode switch incomplete"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._node.publish_arm_status()
        return response

    def set_gripper(self, request, response):
        """夹爪位置控制：闭合/张开到指定开口距离并等待到位。

        请求字段：
            position   目标开口距离（米）。硬件层会夹到 [0, 0.085] 的可信行程内；
            max_effort 最大力矩（N·m）。传 <=0 时由硬件层取默认力矩。
        响应字段：
            success          是否在超时内到达目标；
            reached_position 实际到达的开口距离（米），失败时也回填当前值。
        日志里的 reason 来自硬件层记录的命令失败原因，无失败时打印 "-"。
        """
        try:
            reached, reached_position = self._hardware.set_gripper_position(
                request.position,
                request.max_effort,
            )
            response.success = bool(reached)
            response.reached_position = float(reached_position)
            reason = self._hardware.gripper_command_error
            # 成功走 info、失败走 error，便于现场按日志级别快速筛出异常。
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
            response.reached_position = float(self._hardware.gripper_position_m())
            self._node.get_logger().error(
                "gripper set failed "
                f"target={float(request.position):.3f}m "
                f"max_effort={float(request.max_effort):.3f}Nm "
                f"reached={response.reached_position:.3f}m reason={exc}"
            )
        self._node.publish_arm_status()
        return response

    def grasp_gripper(self, request, response):
        """夹爪力闭合抓取：闭合到堵转后保持夹持力，并给出接触（堵转）判定。

        请求字段（字段名沿用历史命名，力学量实际是夹爪电机力矩，单位 N·m）：
            close_force            闭合阶段力矩，硬件层夹到 [0.05, 1.0] N·m；
            hold_force             保持阶段前馈力矩，硬件层夹到 [0.05, 1.5] N·m；
            close_timeout_sec      闭合超时（秒），超时仍未检测到堵转即判失败；
            min_close_time_sec     最短闭合时间（秒），防止刚起步就被误判为接触；
            velocity_threshold     判定"已停住"的速度阈值（rad/s）；
            min_closure_distance_m 判定接触所需的最小闭合行程（米）。
        响应字段：
            success / contact_detected  是否完成闭合、是否检测到接触；
            contact_position / reached_position 接触位置与实际位置（米）；
            hold_force                  实际保持力矩（N·m）；
            message                     硬件层说明文本（含保持时长上限）。
        该夹爪没有力传感器，"接触"是由闭合行程加速度堵转推断出来的，不是实测接触力。
        """
        try:
            (
                success,
                contact_detected,
                contact_position,
                reached_position,
                hold_force,
                message,
            ) = self._hardware.grasp_gripper(
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
            # 失败时把各字段清零，避免调用方把上一次的数值误当成本次结果。
            response.success = False
            response.contact_detected = False
            response.contact_position = 0.0
            response.reached_position = 0.0
            response.hold_force = 0.0
            response.message = str(exc)
            self._node.get_logger().error(f"gripper grasp failed: {exc}")
        self._node.publish_arm_status()
        return response
