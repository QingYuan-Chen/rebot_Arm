"""机械臂与夹爪的 ROS 2 动作服务端实现（硬件包最外层的执行入口之一）。

本模块把上层下发的动作目标翻译成对硬件管理器的调用，职责包括：

- 注册三个动作服务端：``/{namespace}/move_to_pose``（笛卡尔位姿运动）、
  ``/{namespace}/follow_joint_trajectory``（标准关节轨迹跟踪）、
  ``/{namespace}/gripper/command``（夹爪位置命令）。
- 在目标被接受前执行最后一道安全门：硬件未连接、未显式使能或未处于可动
  生命周期时，一律拒绝运动目标（见 :meth:`ArmActions.goal_callback`）。
- 按轨迹时间点把关节目标写入末端位姿控制器的目标数组，由底层统一控制循环
  周期性地下发给电机；同时发布执行反馈、处理取消与抢占。

安全约束：真机默认失能，只有显式 enable 且反馈新鲜时才会接受并执行运动目标；
关节轨迹终点必须收敛到容差之内，否则中止并保持当前位置（不会自动失能）。

本模块不直接访问电机 SDK，也不负责规划策略、示教文件与界面逻辑。
"""

from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
import numpy as np
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose
from trajectory_msgs.msg import JointTrajectoryPoint

from .conversions import pose_to_xyz_rpy

# ── 关节轨迹执行的三个安全阈值（关节空间，单位弧度 / 秒） ──────────────
# 起始对齐容差：首点与当前实测关节位置的逐关节最大偏差，超出即整条轨迹拒收，
# 避免第一拍就产生跳变冲击。0.10 rad ≈ 5.7°。
_FOLLOW_TRAJECTORY_START_TOL = 0.10
# 终点沉降窗口：最后一点下发后等待关节收敛的最长时间（秒）；超时仍未进入
# 容差即判定 GOAL_TOLERANCE_VIOLATED。
_FOLLOW_TRAJECTORY_SETTLE_TIMEOUT = 2.0
# 终点容差：允许的逐关节最大稳态误差（rad），0.03 rad ≈ 1.7°。
_FOLLOW_TRAJECTORY_GOAL_TOLERANCE = 0.03


class ArmActions:
    """机械臂与夹爪动作服务端的集合。

    生命周期：由控制器节点在硬件连接成功后创建一次，三个动作服务端随之常驻
    直到节点销毁。所有回调都注册在节点的可重入回调组中，因此可能并发执行；
    运动之间的互斥依赖硬件状态机：执行中的目标一旦发现状态机不再是
    ``TRAJ_RUNNING``（例如被停止服务打断，或另一个目标收尾时把它复位为
    ``IDLE``），就会以 preempted 结束，而不是继续下发指令。

    与硬件层的关系：本类只调用硬件管理器暴露的方法（状态机切换、反馈读取、
    使能检查、夹爪命令），把"最后一道执行安全"的判定集中在硬件层。

    参数：
        node: 控制器节点，提供日志、时钟、回调组与状态发布入口。
        hardware: 硬件管理器，唯一允许接触电机与串口的对象。
        namespace: 机械臂命名空间（默认 ``rebotarm``），用于拼接动作名。
    """

    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        # 笛卡尔位姿运动：目标含目标位姿与期望时长，底层走 SE(3) 测地线规划
        # 加 CLIK 跟踪，属于会真实驱动电机的动作。
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_move_to_pose,
            callback_group=node.reentrant_group,
        )
        # 标准关节轨迹跟踪：服务端名字固定为 follow_joint_trajectory，
        # 便于通用规划/执行适配层不经额外映射即可对接。
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group,
        )
        # 夹爪位置命令：position 为开口距离（米），max_effort 为最大夹持力（Nm）。
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group,
        )

    def goal_callback(self, _goal_request):
        """所有动作目标的统一准入闸门。

        只有 ``hardware.ready_for_motion`` 为真（已连接 + 已显式使能 + 生命周期
        处于 ENABLED_HOLD / TRAJECTORY_RUNNING）时才接受目标，否则一律拒绝，
        保证未使能的真机不可能被动作调用驱动。
        """
        if not self._hardware.ready_for_motion:
            self._node.get_logger().warning(
                "Rejecting motion goal: hardware is not explicitly enabled and ready"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_move_to_pose(self, _goal_handle):
        """响应 move_to_pose 的取消请求：置位停止事件并清 ``_moving`` 标志。

        这里只打断底层轨迹发送线程；状态机复位与位置保持交给 execute 循环在
        检测到 ``is_cancel_requested`` 后统一处理，避免两处并发改状态。
        """
        self._hardware.cancel_arm_trajectory()
        return CancelResponse.ACCEPT

    def cancel_follow_joint_trajectory(self, _goal_handle):
        """响应关节轨迹的取消请求：停止下发并保持当前位置。

        ``stop_active_motion`` 会置停止事件、清 ``_moving``、在使能状态下把当前
        位置写回目标（保持位姿）并把状态机切回 IDLE。
        """
        self._hardware.stop_active_motion()
        self._node.publish_arm_status()
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal_handle):
        """响应夹爪命令的取消请求（总是接受）。

        夹爪没有独立的发送线程，真正的取消由 execute 循环轮询
        ``is_cancel_requested`` 后调用硬件层取消接口完成，因此此处不做动作。
        """
        return CancelResponse.ACCEPT

    def execute_move_to_pose(self, goal_handle):
        """执行笛卡尔位姿运动目标：规划后轮询轨迹发送进度并回报反馈。

        流程：状态机置为 TRAJ_RUNNING 并广播状态 → 确保电机处于位置/速度控制
        模式 → 把目标位姿转成位置加欧拉角交给末端控制器规划 → 以 50 ms 周期
        轮询 ``_moving`` 直到整条轨迹下发完毕。

        参数：``goal.target_pose`` 为目标位姿（位置单位米，姿态四元数）；
        ``goal.duration`` 为期望运动时长（秒），``<= 0`` 时由底层按末端移动
        距离自行估算。

        返回：MoveToPose 结果。成功时 ``success=True``；规划失败、异常、取消或
        被抢占时 ``success=False``，``message`` 给出原因，``final_pose`` 为停机
        时的实测位姿，状态机回到 IDLE。

        注意：``_moving`` 变假表示最后一点已下发，并不代表关节已完成跟随，
        本动作不额外等待收敛（关节轨迹动作才有终点沉降检查）。
        """
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            # 先切状态机并广播，使上层与界面能观察到执行开始（使能时同时把
            # 生命周期切到 TRAJECTORY_RUNNING，其它调用据此识别"运动中"）。
            self._hardware.set_state_machine("TRAJ_RUNNING")
            self._node.publish_arm_status()
            # 确保电机处于位置-速度控制模式；不满足会抛异常，由下方 except 收尾。
            self._hardware.ensure_pos_vel_control()
            # 位置单位为米，姿态由四元数转 RPY（弧度）：底层规划接口只接受 RPY。
            x, y, z, roll, pitch, yaw = pose_to_xyz_rpy(goal.target_pose)
            ok = self._hardware.move_to_pose_path(
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
                float(goal.duration),
            )
        except Exception as exc:
            # 规划或硬件异常：回到 IDLE 并以中止结束目标，异常不外泄到回调之外。
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.final_pose = self._hardware.current_pose()
            return result

        if not ok:
            # move_to_traj 返回 False 表示 IK 或轨迹规划失败，未下发任何指令。
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = "trajectory planning failed"
            result.final_pose = self._hardware.current_pose()
            return result

        start = time.monotonic()
        requested_duration = float(goal.duration)
        feedback = MoveToPose.Feedback()
        # 底层发送线程走完轨迹后会把 ``_moving`` 置假，此循环据此判定结束。
        while self._hardware.trajectory_moving:
            if goal_handle.is_cancel_requested:
                # 取消：停止下发并复位状态机，结果按 canceled 语义返回。
                self._hardware.cancel_arm_trajectory()
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                result.final_pose = self._hardware.current_pose()
                return result

            if self._hardware.state_machine != "TRAJ_RUNNING":
                # 状态机被其它请求改写（如底层停止、回零）视为抢占，立即中止本目标。
                goal_handle.abort()
                result.success = False
                result.message = "preempted"
                result.final_pose = self._hardware.current_pose()
                return result

            feedback.current_pose = self._hardware.current_pose()
            elapsed = float(time.monotonic() - start)
            # 进度估算：给了期望时长就按时间比例；否则按已发送轨迹点占比；
            # 两者都不可用时按"已接近完成"上报 1.0。
            if requested_duration > 0.0:
                feedback.progress = max(0.0, min(1.0, elapsed / requested_duration))
            else:
                feedback.progress = self._hardware.trajectory_progress
            feedback.time_elapsed = elapsed
            goal_handle.publish_feedback(feedback)
            # 20 Hz 反馈节流：兼顾上层观测实时性与总线/CPU 开销。
            time.sleep(0.05)

        result.success = True
        result.message = "move_to_pose complete"
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        """执行标准关节轨迹：按时间轴逐点下发关节目标并做终点收敛检查。

        关键约束：

        - 轨迹必须带 joint_names 与至少一个轨迹点，且关节名集合必须与硬件关节
          完全一致（不允许缺项、多项或重名）。
        - 下发前做完整校验（见 :meth:`_validated_trajectory`）：首点必须贴近当前
          实测关节状态，时间戳必须单调不减。
        - 执行时以收到目标后的单调时钟为时间轴零点，按各点 ``time_from_start``
          的绝对时刻写入末端控制器目标数组，由硬件统一控制循环完成插补与限速。
        - 终点必须在 ``_FOLLOW_TRAJECTORY_SETTLE_TIMEOUT`` 秒内收敛到
          ``_FOLLOW_TRAJECTORY_GOAL_TOLERANCE`` 之内，否则判 GOAL_TOLERANCE_VIOLATED
          并保持当前位置；任何异常路径同样先保持位置再中止。

        返回：FollowJointTrajectory 结果，``error_code`` 取标准枚举
        （SUCCESSFUL / INVALID_GOAL / GOAL_TOLERANCE_VIOLATED）。
        """
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory

        # 畸形目标：缺少关节名或轨迹点，直接以 INVALID_GOAL 中止。
        if not trajectory.joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        try:
            current = self._current_positions_for(list(trajectory.joint_names))
            sample_times, sample_positions = self._validated_trajectory(
                list(trajectory.joint_names),
                trajectory.points,
                current,
            )
        except Exception as exc:
            # 校验失败（关节不匹配、时间回退、首点偏离过大等）不驱动任何电机。
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        trajectory_done = False
        start = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)

        self._hardware.set_state_machine("TRAJ_RUNNING")
        self._node.publish_arm_status()
        try:
            self._hardware.ensure_pos_vel_control()

            # 第 0 个采样是合成的"当前状态"起点，从第 1 个点开始才是要下发的目标。
            for target_time, target in zip(sample_times[1:], sample_positions[1:]):
                # 以动作开始时刻为时间轴零点，等到该点的相对时刻再下发。
                if not self._wait_until_time(goal_handle, start + target_time, result):
                    return result
                if self._trajectory_stopped(goal_handle, result):
                    return result

                self._set_endpos_target(list(trajectory.joint_names), target)

                desired = JointTrajectoryPoint()
                desired.positions = [float(v) for v in target]
                feedback.desired = desired
                feedback.actual = self._actual_point(list(trajectory.joint_names))
                # 误差点按"实测 - 期望"计算（单位 rad）；正误差表示实测值大于期望值，
                # 其对运动方向的含义取决于该关节当前的运动方向。
                feedback.error = self._error_point(desired, feedback.actual)
                goal_handle.publish_feedback(feedback)
            trajectory_done = True

            if not trajectory_done:
                # 防御性检查：上方所有中断路径都已提前 return，此处当前不可达。
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory interrupted"
                return result

            # 循环内已写过终点，这里幂等再写一次，确保目标数组稳定停在终点。
            self._set_endpos_target(list(trajectory.joint_names), sample_positions[-1])
            ok, max_error = self._wait_until_goal_reached(
                goal_handle,
                list(trajectory.joint_names),
                sample_positions[-1],
                _FOLLOW_TRAJECTORY_GOAL_TOLERANCE,
                result,
            )
            if not ok:
                # 取消与抢占已由等待函数写好结果，直接透传，不按超差处理。
                if result.error_string in ("canceled", "preempted"):
                    return result
                self._hardware.hold_current_position()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = (
                    "trajectory goal not reached within tolerance "
                    f"(max error {max_error:.3f} rad > "
                    f"{_FOLLOW_TRAJECTORY_GOAL_TOLERANCE:.3f} rad)"
                )
                return result
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "follow_joint_trajectory complete"
            return result
        except Exception as exc:
            # 未预期异常必须先把当前位姿锁为目标再中止，不能让电机停在未知目标上。
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result
        finally:
            # 成功、中止、异常统一收尾：状态机复位并广播，供上层与界面观察。
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()

    def _set_endpos_target(self, joint_names: list[str], positions: np.ndarray) -> None:
        """把一组关节目标按硬件关节顺序写入末端位姿控制器的目标数组。

        轨迹里的关节顺序允许与硬件顺序不同，这里按名字重排；集合必须完全一致，
        否则抛 ValueError——缺失关节会让对应电机继续跟踪旧目标，属于危险状态。
        切片赋值 ``[:]`` 用于原地更新控制循环持有的同一个数组对象。
        """
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(f"trajectory joints must match {self._hardware.joint_names}")
        by_name = {name: float(pos) for name, pos in zip(joint_names, positions)}
        ordered = np.array(
            [by_name[name] for name in self._hardware.joint_names],
            dtype=np.float64,
        )
        self._hardware.set_joint_target(ordered)

    def _trajectory_stopped(self, goal_handle, result) -> bool:
        """检查轨迹是否需要提前终止；返回 True 表示终止语义已处理完毕。

        取消：停止下发并保持位置，结果标记 "canceled"（调用方应原样返回结果）；
        抢占（状态机被外部改写）：保持当前位置并中止，结果标记 "preempted"。
        """
        if goal_handle.is_cancel_requested:
            self._hardware.stop_active_motion()
            self._node.publish_arm_status()
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "canceled"
            return True
        if self._hardware.state_machine != "TRAJ_RUNNING":
            self._hardware.hold_current_position()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "preempted"
            return True
        return False

    def _current_positions_for(self, joint_names: list[str]) -> np.ndarray:
        """读取实测关节位置，并按请求给出的关节顺序重排返回（单位 rad）。

        先校验请求关节名无重复且与硬件关节集合一致，避免"只控制部分关节"的
        轨迹让其余关节继续保持旧目标。
        """
        if len(joint_names) != len(set(joint_names)):
            raise ValueError("joint_names must not contain duplicates")
        if set(joint_names) != set(self._hardware.joint_names):
            raise ValueError(
                f"trajectory joints must match {self._hardware.joint_names}"
            )
        current, _, _ = self._hardware.get_joint_state()
        by_name = {
            name: float(pos)
            for name, pos in zip(self._hardware.joint_names, current)
        }
        return np.array([by_name[name] for name in joint_names], dtype=np.float64)

    def _validated_trajectory(
        self,
        joint_names: list[str],
        points: list[JointTrajectoryPoint],
        current: np.ndarray,
    ) -> tuple[list[float], list[np.ndarray]]:
        """校验并展开轨迹点，返回 (相对时间列表[秒], 关节位置列表[rad])。

        会在最前面插入一个 t=0、位置等于当前实测状态的采样点，使时间轴与
        "从当前状态出发"的语义一致。

        校验规则（违反即抛 ValueError，由调用方转换成 INVALID_GOAL）：

        - 每个点的 positions 长度必须等于关节数；
        - ``time_from_start`` 必须单调不减（允许 1e-9 秒的浮点回差）；
        - 首个点与当前实测状态的逐关节偏差不得超过
          ``_FOLLOW_TRAJECTORY_START_TOL``；若其 ``time_from_start`` 为 0，
          则视为起点占位点直接跳过，不参与下发。
        """
        # 合成 t=0 的起点采样，保证轨迹从实测状态平滑出发。
        sample_times = [0.0]
        sample_positions = [current.copy()]
        last_time = 0.0
        # 注意：last_positions 只被赋值、未被返回值使用，这里保留原有写法不做改动。
        last_positions = current.copy()

        for index, point in enumerate(points, start=1):
            if len(point.positions) != len(joint_names):
                raise ValueError("point.positions length must match joint_names")

            # sec + nanosec 合成秒；nanosec 换算系数 1e-9。
            point_time = float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            if point_time < last_time - 1e-9:
                # 允许 1e-9 秒的回差，避免等时间点因浮点误差被判为时间回退。
                raise ValueError("trajectory time_from_start must be nondecreasing")
            positions = np.array(point.positions, dtype=np.float64)

            if index == 1:
                # 只对首点做起始对齐检查：后续点本应与首点连续。
                start_delta = float(np.max(np.abs(positions - current)))
                if start_delta > _FOLLOW_TRAJECTORY_START_TOL:
                    raise ValueError(
                        "first trajectory point is too far from current joint state "
                        f"(max delta {start_delta:.3f} rad)"
                    )
                if point_time <= 1e-9:
                    # 首点时间为 0：仅作为占位，不下发以免与实测状态重复。
                    last_positions = current.copy()
                    continue

            sample_times.append(point_time)
            sample_positions.append(positions)
            last_time = point_time
            last_positions = positions

        return sample_times, sample_positions

    def _wait_until_time(self, goal_handle, target_time: float, result) -> bool:
        """忙等到单调时钟到达 ``target_time``，期间可响应取消与抢占。

        轮询周期 10 ms。返回 True 表示可以继续下发该时刻的目标；返回 False 表示
        已取消/被抢占（result 已写好），调用方应立即原样返回。
        """
        while time.monotonic() < target_time:
            if goal_handle.is_cancel_requested:
                self._hardware.stop_active_motion()
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "canceled"
                return False
            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "preempted"
                return False
            time.sleep(0.01)
        return True

    def _wait_until_goal_reached(
        self,
        goal_handle,
        joint_names: list[str],
        target: np.ndarray,
        goal_tolerance: float,
        result,
    ) -> tuple[bool, float]:
        """等待所有关节进入终点容差，返回 (是否收敛, 最大逐关节误差[rad])。

        最多等待 ``_FOLLOW_TRAJECTORY_SETTLE_TIMEOUT`` 秒，50 ms 轮询一次；
        误差取各关节绝对误差的最大值。取消或抢占时返回 (False, 0.0)，调用方
        依据 ``result.error_string`` 区分，不当作超差处理。
        """
        deadline = time.monotonic() + _FOLLOW_TRAJECTORY_SETTLE_TIMEOUT
        max_error = float("inf")
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._hardware.stop_active_motion()
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "canceled"
                return False, 0.0
            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "preempted"
                return False, 0.0
            actual = self._current_positions_for(joint_names)
            max_error = float(np.max(np.abs(actual - target)))
            if max_error <= goal_tolerance:
                return True, max_error
            time.sleep(0.05)
        return False, max_error

    def _actual_point(self, joint_names: list[str] | None = None) -> JointTrajectoryPoint:
        """构造反馈用的实测轨迹点（位置 rad、速度 rad/s）。

        ``joint_names`` 为 None 时使用硬件原始关节顺序，否则按给定顺序重排。
        """
        pos, vel, _ = self._hardware.get_joint_state()
        if joint_names is not None:
            by_name = {
                name: i
                for i, name in enumerate(self._hardware.joint_names)
            }
            pos = [pos[by_name[name]] for name in joint_names]
            vel = [vel[by_name[name]] for name in joint_names]
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in pos]
        point.velocities = [float(v) for v in vel]
        return point

    @staticmethod
    def _error_point(desired: JointTrajectoryPoint, actual: JointTrajectoryPoint) -> JointTrajectoryPoint:
        """逐关节计算 ``实测 - 期望`` 误差点（位置 rad、速度 rad/s）。

        正误差表示实测值大于期望值（方向含义取决于该关节的运动方向）；
        ``desired`` 未给出速度时不输出速度误差。
        """
        point = JointTrajectoryPoint()
        point.positions = [
            float(a - d) for d, a in zip(desired.positions, actual.positions)
        ]
        if desired.velocities and actual.velocities:
            point.velocities = [
                float(a - d) for d, a in zip(desired.velocities, actual.velocities)
            ]
        return point

    def execute_gripper_command(self, goal_handle):
        """执行夹爪位置命令，并以 50 ms 周期上报反馈直到到位/堵转/超时。

        参数（取 ``goal_handle.request.command``）：``position`` 为目标开口距离
        （米，硬件层会按已验证的开合范围裁剪）；``max_effort`` 为最大夹持力
        （Nm），``<= 0`` 表示使用硬件层默认力。

        结束条件：到达目标、检测到堵转、硬件层报错，或超过硬件层为本次命令
        计算的超时时间。未真正到位时目标以中止（abort）结束，并显式取消硬件层
        的位置命令，避免夹爪持续以目标力顶住工件。
        """
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            # 硬件层会校验使能状态、控制循环与反馈新鲜度，不满足时抛异常。
            self._hardware.set_gripper_target(goal.position, goal.max_effort)
        except Exception as exc:
            # 命令被拒绝：立即中止，结果各项清零，不残留上一次命令的状态。
            self._node.get_logger().error(f"gripper command rejected: {exc}")
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        # 超时由硬件层按 |目标角 - 起始角| / 最大角速度 + 余量算出。
        timeout_sec = self._hardware.gripper_target_timeout_sec()
        last_pos = self._hardware.gripper_position_m()
        stalled = False
        failure_reason = None
        while time.monotonic() - start < timeout_sec:
            if goal_handle.is_cancel_requested:
                # 操作员取消：取消位置命令并如实回填当前状态。
                self._hardware.cancel_gripper_position_command(
                    "gripper action canceled by operator"
                )
                goal_handle.canceled()
                result.position = self._hardware.gripper_position_m()
                result.effort = self._hardware.get_gripper_state()[2]
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos = self._hardware.gripper_position_m()
            effort = self._hardware.get_gripper_state()[2]
            reached = self._hardware.gripper_reached_target()
            if reached:
                # 位置到位后再做一次短确认，等硬件层完成目标释放（原子切换为空闲）。
                try:
                    reached = self._hardware.wait_gripper_target(timeout=0.05)
                except Exception as exc:
                    failure_reason = f"gripper completion release failed: {exc}"
                    reached = False
            # 堵转判定：相邻两拍位移小于 0.1 mm，且实际力矩已达到本次请求的
            # max_effort；max_effort<=0（改用硬件层默认力）时该条件恒真，
            # 退化为只看位移是否停止。
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            # 硬件层异步检测到的错误（如反馈失效）优先作为本次失败原因。
            failure_reason = failure_reason or self._hardware.gripper_command_error
            if failure_reason is not None:
                self._node.get_logger().error(
                    "gripper action failed "
                    f"target={float(goal.position):.6f}m "
                    f"feedback={pos:.6f}m effort={effort:.6f}Nm "
                    f"reason={failure_reason}"
                )
                break
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        result.position = self._hardware.gripper_position_m()
        result.effort = self._hardware.get_gripper_state()[2]
        result.stalled = stalled
        # 只有全程无错误且当前确实到位才算成功。
        result.reached_goal = (
            failure_reason is None and self._hardware.gripper_reached_target()
        )
        if not result.reached_goal:
            # 失败或超时必须显式取消位置命令，否则夹爪会持续顶住目标位置。
            self._hardware.cancel_gripper_position_command(
                failure_reason or "gripper action target timeout"
            )
        if result.reached_goal:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result
