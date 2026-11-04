"""点到点位姿运动节点: 调用 MoveIt 规划并通过 FJT 请求执行.

职责与位置
    rebotarm_motion 中基于 MoveIt 的位姿执行入口. 不直接访问电机 SDK.
    规划交给 MoveItMotionPlanner, 轨迹执行和设备安全事务交给控制器.

对外接口
    服务前缀为 /<arm_namespace>/motion_execution/:
        execute_pose: 规划位姿目标, execute=False 时只返回规划结果.
        publish_trajectory_preview: 将多段规划轨迹作为一个 RViz 显示序列发布.
        stop: 取消活动目标并请求控制器 trajectory_stop, 再确认停止证据.
    动作客户端为 /<arm_namespace>/follow_joint_trajectory.

主要流程
    位姿请求 -> MoveIt 规划 -> 纯规划返回或提交 FJT -> 等待动作终态.
    预览消息只用于 RViz 显示, 不触发 FJT; 多段预览须先完成顺序与端点检查.

安全与并发约束
    本节点不使能或失能, 不独立执行全路径碰撞预检或运行限位守卫.
    规划约束由 MoveIt 处理, 底层准入和设备故障由控制器处理; 规划成功不代表执行成功.
    默认速度/加速度缩放为 0.10 / 0.08; 请求给 0.0 时回落到默认值.
    stop 成功要求旧请求排空, 动作终态, 控制器停止成功和新鲜反馈持续静止.
    真机另需新鲜, 无故障的 IDLE 状态; 未确认时阻断新执行, 可再次 stop 检查.
    业务服务在可重入组和四线程执行器中等待; 反馈使用独立接收节点和短事务快照.
    回调分组允许并发处理停止, 不能保证外部服务或在途设备事务的响应时间.

文件结构(按定义顺序)
    PoseExecutionNode: 持有请求代次, 动作句柄和停止状态, 适配规划与执行服务.
    main: 创建业务执行器, 退出时关闭业务执行器, 再经节点销毁关闭反馈接收器.
"""

from __future__ import annotations

import time
import math
import threading
import json

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from .feedback_receiver import MotionFeedbackReceiver

from rebotarm_msgs.srv import ExecutePose, PublishTrajectoryPreview, RetimeArmTrajectory

from .moveit_planner import MoveItMotionPlanner


class PoseExecutionNode(Node):
    """位姿服务的规划与执行编排, 持有请求代次, 动作句柄和停止状态.

    执行请求独占本层运动入口, stop 推进代次并阻断旧规划下发, 持续处理迟到接纳和终态.
    服务和 Action 客户端使用可重入组; 规划客户端响应由业务执行器的其他线程处理.
    反馈由独立接收器更新; 业务锁不带入接收回调, 停止未确认时拒绝新执行.
    """

    def __init__(self) -> None:
        """声明规划参数并创建可重入服务/Action 客户端, 状态锁和独立反馈接收器; 不使能设备."""
        super().__init__("motion_execution")
        # 可重入回调组: execute_pose 会长时间阻塞等待轨迹结果, stop 服务必须能在
        # 同一时刻被处理, 否则停止请求会被排在后面.
        self._callback_group = ReentrantCallbackGroup()

        # arm_namespace: 话题/服务/动作的公共前缀, 去掉首尾斜杠后拼接.
        self.declare_parameter("arm_namespace", "rebotarm")
        # frame_id: 位姿目标所在的世界坐标系(MoveIt 规划时的参考系).
        self.declare_parameter("frame_id", "base_link")
        # ee_frame_id: 末端执行器 link 名, MoveIt 位置/姿态约束作用在该 link 上.
        self.declare_parameter("ee_frame_id", "end_link")
        # moveit_group_name: SRDF 中的规划组名(six-axis 手臂组).
        self.declare_parameter("moveit_group_name", "arm")
        # moveit_planning_service: MoveIt 规划服务名(GetMotionPlan 接口).
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        # moveit_planning_pipeline: 规划管线 id, 默认 ompl.
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        # moveit_planner_id: 具体规划器 id, 空串表示用管线默认规划器.
        self.declare_parameter("moveit_planner_id", "")
        # moveit_planning_time: 单次规划允许的最长时间, 单位 s.
        self.declare_parameter("moveit_planning_time", 2.0)
        # moveit_num_planning_attempts: 规划尝试次数, 越大越可能成功但更慢.
        self.declare_parameter("moveit_num_planning_attempts", 1)
        # goal_position_tolerance: 末端位置约束容差, 单位 m(约束盒边长).
        self.declare_parameter("goal_position_tolerance", 0.005)
        # goal_orientation_tolerance: 末端姿态各轴容差, 单位 rad.
        self.declare_parameter("goal_orientation_tolerance", 0.02)
        # default_velocity_scaling: 请求未指定速度缩放时的默认值, 无量纲 (0, 1].
        self.declare_parameter("default_velocity_scaling", 0.10)
        # default_acceleration_scaling: 请求未指定加速度缩放时的默认值, 无量纲 (0, 1].
        self.declare_parameter("default_acceleration_scaling", 0.08)
        # 仅视觉纯规划入口启用; 其他 ExecutePose 调用不会产生预览副作用.
        self.declare_parameter("publish_plan_only_preview", False)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("trajectory_execution_mode", "legacy")
        self.declare_parameter("stop_confirmation_timeout_sec", 8.0)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._trajectory_mode = self.get_parameter('trajectory_execution_mode').value
        self._capability = None
        self._capability_at = 0.
        self.create_subscription(String, f'/{self._arm_namespace}/trajectory_capabilities',
            self._receive_capability, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self._callback_group)
        self._retime_client = self.create_client(RetimeArmTrajectory,
            f'/{self._arm_namespace}/retime_arm_trajectory', callback_group=self._callback_group)
        # 当前请求的动作句柄保留到下一次请求准入时重置, 供停止流程补发取消并核对终态.
        self._active_goal_handle = None
        # 可重入回调会并行修改运动状态; 运动锁保护共享字段, 停止锁只允许一轮停止确认.
        self._motion_lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._request_active = False
        self._stopping = False
        self._generation = 0
        self._send_future = None
        self._result_future = None
        self._preview_publisher = (
            self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
            if bool(self.get_parameter("publish_plan_only_preview").value)
            else None
        )
        # 轨迹执行动作客户端(FollowJointTrajectory): 与控制器包约定的执行通道.
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            callback_group=self._callback_group,
        )
        # 控制器侧的轨迹停止服务, 作为取消动作之外的兜底停止通道.
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        # 规划适配器: 负责把位姿目标翻译成 MoveIt 请求并解析响应.
        self._planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id=str(self.get_parameter("ee_frame_id").value),
            frame_id=str(self.get_parameter("frame_id").value),
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=('ompl_reference' if self._trajectory_mode == 'prepared_v1'
                else str(self.get_parameter("moveit_planning_pipeline").value)),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_orientation_tolerance=float(self.get_parameter("goal_orientation_tolerance").value),
            start_state_provider=self._planning_start_state,
        )
        self.create_service(
            ExecutePose,
            f"/{self._arm_namespace}/motion_execution/execute_pose",
            self._execute_pose,
            callback_group=self._callback_group,
        )
        self.create_service(
            PublishTrajectoryPreview,
            f"/{self._arm_namespace}/motion_execution/publish_trajectory_preview",
            self._publish_trajectory_preview,
            callback_group=self._callback_group,
        )
        # 停止服务: 有界等待动作终态和新鲜静止反馈, 失败时保持执行门控.
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            self._stop,
            callback_group=self._callback_group,
        )
        self._feedback_receiver = MotionFeedbackReceiver(self, self._arm_namespace, gripper=True)
        self._stop_feedback = self._feedback_receiver.stop_feedback
        self.get_logger().info(
            f"pose motion execution ready: /{self._arm_namespace}/motion_execution/execute_pose"
        )

    def destroy_node(self):
        """停止专用反馈接收器后销毁业务节点; 关闭失败不提前释放接收句柄."""
        self._feedback_receiver.close()
        return super().destroy_node()

    def _execute_pose(self, request, response):
        """串行接纳位姿请求, 并在异常后要求重新确认停止."""
        with self._motion_lock:
            if self._request_active or self._stopping:
                response.success = False
                response.message = "motion busy or stop not confirmed"
                return response
            self._request_active = True
            # 记住本次请求的代次; 停止会递增全局代次, 使旧请求失去继续下发的资格.
            self._run_generation = self._generation
            self._send_future = self._result_future = self._active_goal_handle = None
        try:
            return self._execute_pose_impl(request, response)
        except Exception as exc:
            with self._motion_lock:
                self._stopping = True
            response.success = False
            response.message = f"motion failed; stop confirmation required: {exc}"
            return response
        finally:
            with self._motion_lock:
                # 请求退出只释放请求占用; 动作是否结束仍由停止流程单独核对.
                self._request_active = False

    def _interrupted(self):
        """判断本次请求是否已被停止标记或代次变化中断; 调用方须持有运动锁."""
        return self._stopping or self._run_generation != self._generation

    def _execute_pose_impl(self, request: ExecutePose.Request, response: ExecutePose.Response):
        """规划并可选执行一个末端位姿目标.

        请求字段: `target_pose`(位姿目标), `velocity_scaling`/`acceleration_scaling`
        (无量纲缩放, 0.0 表示用参数默认值), `timeout_sec`(本步骤超时, s),
        `execute`(False 时只规划不下发), `preview_start_joint_state`(可选虚拟起点,
        仅允许 execute=False), `suppress_preview`(多阶段预览收集期间禁止逐段发布).
        响应字段: `success`, `stage`("planning" 或 "execution", 用于区分失败发生在
        哪个阶段), `message`, `planned_trajectory`.
        """
        response.stage = "planning"
        preview_start = request.preview_start_joint_state
        if preview_start.name:
            if request.execute:
                response.success = False
                response.message = "explicit preview start is forbidden for execution"
                return response
            if (
                len(preview_start.name) != 6
                or set(preview_start.name) != {f"joint{i}" for i in range(1, 7)}
                or len(preview_start.position) != len(preview_start.name)
                or not all(math.isfinite(value) for value in preview_start.position)
            ):
                response.success = False
                response.message = "invalid preview start joint state"
                return response
        # 请求里 0.0 视为"未指定": 回落节点参数, 避免上位传 0 变成满速或非法值.
        velocity = float(request.velocity_scaling) or float(self.get_parameter("default_velocity_scaling").value)
        acceleration = float(request.acceleration_scaling) or float(
            self.get_parameter("default_acceleration_scaling").value
        )
        plan = self._planner.plan_pose(
            request.target_pose,
            velocity_scaling=velocity,
            acceleration_scaling=acceleration,
            **({"start_joint_state": preview_start} if preview_start.name else {}),
        )
        if not plan.success or plan.trajectory is None:
            response.success = False
            response.message = plan.message
            return response

        if self._trajectory_mode == 'prepared_v1':
            # GetMotionPlan 的 RobotTrajectory 不保留 header；独立重定时服务返回权威消息。
            # 虚拟阶段只核定参考；真实执行阶段在前一阶段结束后绑定当前后端起点。
            try:
                retime = RetimeArmTrajectory.Request()
                retime.trajectory = plan.trajectory
                retime.velocity_scaling, retime.acceleration_scaling = velocity, acceleration
                retime.planning_purpose = (RetimeArmTrajectory.Request.FINAL_EXECUTION
                    if request.execute else RetimeArmTrajectory.Request.REFERENCE)
                if not self._retime_client.service_is_ready():
                    raise RuntimeError('trajectory retiming service unavailable')
                future = self._retime_client.call_async(retime)
                self._wait_future(future, 2.)
                if not future.done():
                    future.cancel()
                    raise TimeoutError('trajectory retiming exceeded 2 s')
                final = future.result()
                if not final.success:
                    raise ValueError(final.message)
                from dataclasses import replace
                plan = replace(plan, trajectory=final.trajectory)
            except (ValueError, RuntimeError) as exc:
                response.success, response.message = False, str(exc)
                return response

        # 即使随后不执行, 也把规划结果回填, 便于调用方审查轨迹.
        response.planned_trajectory = plan.trajectory
        if not bool(request.execute):
            if self._preview_publisher is not None and not bool(request.suppress_preview):
                trajectory = plan.trajectory
                # DisplayTrajectory 的初态必须与路径起点一致; 不能将当前假关节状态
                # 错当成上一阶段的终点. 此消息仅用于 RViz, 绝不发给轨迹 Action.
                if len(trajectory.joint_names) == len(trajectory.points[0].positions):
                    display = DisplayTrajectory()
                    display.model_id = "reBot-DevArm_fixend"
                    display.trajectory_start.joint_state.name = list(trajectory.joint_names)
                    display.trajectory_start.joint_state.position = list(trajectory.points[0].positions)
                    robot_trajectory = RobotTrajectory()
                    robot_trajectory.joint_trajectory = trajectory
                    display.trajectory = [robot_trajectory]
                    self._preview_publisher.publish(display)
            response.success = True
            response.stage = "planning"
            response.message = plan.message
            return response

        response.stage = "execution"
        with self._motion_lock:
            # 规划等待期间可能收到停止请求, 规划成功后仍需重新检查执行资格.
            if self._interrupted():
                response.success = False
                response.message = "stopped during planning"
                return response
        # 请求超时至少按 1 s 处理: 动作服务器发现/连接握手需要时间, 给 0 会立刻误判不可用.
        if not self._trajectory_client.wait_for_server(timeout_sec=max(float(request.timeout_sec), 1.0)):
            response.success = False
            response.message = "follow_joint_trajectory action unavailable"
            return response

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = plan.trajectory
        with self._motion_lock:
            # 服务器就绪等待期间也可能发生停止; 检查与发送在同一把锁内完成.
            if self._interrupted():
                response.success = False
                response.message = "stopped before trajectory dispatch"
                return response
            send_future = self._trajectory_client.send_goal_async(goal)
            self._send_future = send_future
        self._wait_future(send_future, max(float(request.timeout_sec), 1.0))
        if not send_future.done():
            # 接收响应超时不能证明控制器未接纳目标. 保持停止门控, 并取消晚到的目标.
            with self._motion_lock:
                self._stopping = True
            send_future.add_done_callback(self._cancel_late_goal)
            response.success = False
            response.message = "trajectory acceptance timeout; stop confirmation required"
            return response
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            response.success = False
            response.message = "trajectory goal rejected"
            return response

        # 停止流程也跟踪目标接收请求, 响应晚到时仍能取得句柄, 取消并等待终态.
        with self._motion_lock:
            self._active_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            self._result_future = result_future
            if self._interrupted():
                goal_handle.cancel_goal_async()
        # 结果等待窗口至少 30 s: 轨迹本身可能远长于规划超时, 必须给它跑完的机会.
        self._wait_future(result_future, max(float(request.timeout_sec), 30.0))
        if not result_future.done():
            # 取消请求发出不等于机械臂已停稳; 后续请求仍需等待停止确认.
            with self._motion_lock:
                self._stopping = True
            goal_handle.cancel_goal_async()
            response.success = False
            response.message = "trajectory result timeout"
            return response

        wrapped = result_future.result()
        # 动作状态码 4 表示成功终态, error_code 为控制器返回的执行结果码.
        status = int(getattr(wrapped, "status", 0))
        result = getattr(wrapped, "result", None)
        error_string = str(getattr(result, "error_string", "") or "")
        error_code = int(getattr(result, "error_code", 0) or 0)
        # 动作成功终态和控制器成功结果码必须同时满足, 取消或中止均不能算执行成功.
        if status != 4 or error_code != int(FollowJointTrajectory.Result.SUCCESSFUL):
            response.success = False
            response.message = (
                f"trajectory failed: status={status}, error_code={error_code}, {error_string}".strip()
            )
            return response

        response.success = True
        response.message = "trajectory executed"
        return response

    def _receive_capability(self, message):
        """只缓存版本化声明；配置或会话改变后旧规划绑定会由后端拒绝。"""
        from rebotarm_trajectory.capability import decode_capability
        try:
            capability = decode_capability(message.data)
        except (KeyError, TypeError, ValueError):
            self._capability, self._capability_at = None, 0.
            return
        self._capability, self._capability_at = capability, time.monotonic()


    def _publish_trajectory_preview(
        self,
        request: PublishTrajectoryPreview.Request,
        response: PublishTrajectoryPreview.Response,
    ):
        """把多段轨迹放进同一条显示消息, 供 RViz 连续播放.

        `DisplayTrajectory.trajectory` 本身就是有序轨迹数组: 第一段从
        `trajectory_start` 开始, 后续段从前一段终点继续. 这里严格检查关节顺序,
        点维度与相邻端点连续性; 任何异常都会整组拒绝, 避免显示一条拼接错误的路径.
        本函数只通过 RViz 发布器发送显示消息, 不调用轨迹执行动作客户端.
        """

        if self._preview_publisher is None:
            response.success = False
            response.message = "plan-only preview publisher is disabled"
            return response
        trajectories = list(request.trajectories)
        if not trajectories:
            response.success = False
            response.message = "no trajectories supplied for preview"
            return response

        expected_names = list(trajectories[0].joint_names)
        if not expected_names:
            response.success = False
            response.message = "preview trajectory has no joint names"
            return response

        previous_end = None
        display = DisplayTrajectory()
        display.model_id = "reBot-DevArm_fixend"
        for index, trajectory in enumerate(trajectories):
            # 各段必须采用相同的关节顺序, 每个位置向量须完整且数值有限.
            points = list(trajectory.points)
            if list(trajectory.joint_names) != expected_names or not points:
                response.success = False
                response.message = f"preview trajectory {index} has incompatible joints or no points"
                return response
            for point in points:
                positions = list(point.positions)
                if len(positions) != len(expected_names) or not all(
                    math.isfinite(value) for value in positions
                ):
                    response.success = False
                    response.message = f"preview trajectory {index} has invalid positions"
                    return response
            start = list(points[0].positions)
            # 相邻段在关节空间的端点偏差不得超过 1e-4 rad, 避免预览拼接时发生跳变.
            if previous_end is not None and any(
                abs(current - previous) > 1e-4
                for current, previous in zip(start, previous_end)
            ):
                response.success = False
                response.message = f"preview trajectory {index} is discontinuous"
                return response
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = trajectory
            display.trajectory.append(robot_trajectory)
            previous_end = list(points[-1].positions)

        from .gripper_preview import with_gripper_preview
        try:
            # 按阶段加入夹爪显示动画; 事件无效时整组拒绝, 不发布部分预览.
            if 'left_finger_joint' in expected_names:
                if request.gripper_before_trajectory or request.gripper_openings_m:
                    raise ValueError('joint coordinated previews cannot contain independent gripper animations')
            else:
                trajectories = with_gripper_preview(
                    trajectories, request.gripper_before_trajectory, request.gripper_openings_m
                )
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response
        # 重新构造显示序列, 包含开合动画和保持开口的机械臂段.
        display.trajectory = []
        for trajectory in trajectories:
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = trajectory
            display.trajectory.append(robot_trajectory)
        expected_names = list(trajectories[0].joint_names)
        first_point = trajectories[0].points[0]
        # 动画可能新增手指关节, 显示初态应从扩展后的第一段提取.
        display.trajectory_start.joint_state.name = expected_names
        display.trajectory_start.joint_state.position = list(first_point.positions)
        self._preview_publisher.publish(display)
        response.success = True
        response.message = f"published {len(trajectories)} preview trajectories"
        return response

    def _cancel_late_goal(self, future):
        """取消接收响应超时后才获知已接纳的目标, 并保留其结果供停止确认."""
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                with self._motion_lock:
                    self._active_goal_handle = handle
                    self._result_future = handle.get_result_async()
                # 此处只发起取消; 是否终止仍由结果响应和停止确认流程判断.
                handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"late goal cancellation failed: {exc}")

    def _action_terminal(self):
        """检查动作是否终结; 调用方须持有运动锁, 接收状态不明时不视为空闲."""
        if self._send_future is None:
            return True
        if not self._send_future.done() or self._send_future.cancelled():
            # 本地异步对象被取消并不证明服务器拒绝了目标, 仍按状态不明处理.
            return False
        handle = self._send_future.result()
        if handle is None:
            return False
        if not handle.accepted:
            # 明确拒绝的目标不会执行, 无需再等待动作结果.
            return True
        if self._result_future is None:
            # 接纳响应可能先于执行回调被停止线程观察到, 此处补建结果跟踪并请求取消.
            self._active_goal_handle = handle
            self._result_future = handle.get_result_async()
            handle.cancel_goal_async()
        future = self._result_future
        # 4, 5, 6 分别为成功, 已取消, 已中止; 三者仅表示动作终结, 不代表反馈已静止.
        return (future.done() and not future.cancelled()
                and int(future.result().status) in (4, 5, 6))

    def _planning_start_state(self, arm_start=None):
        """规划使用两路新鲜实际反馈；虚拟预览仅替换六轴，不覆盖真实夹爪或附着物。"""
        from sensor_msgs.msg import JointState
        snapshot = self._feedback_receiver.snapshot()
        now = self.get_clock().now().nanoseconds
        for message in (snapshot.joints, snapshot.gripper):
            if message is None:
                raise ValueError('fresh arm and gripper feedback required for planning')
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if stamp <= 0 or not -.05 <= (now-stamp)/1e9 <= .15:
                raise ValueError('arm or gripper feedback expired before planning')
        values = dict(zip(snapshot.joints.name,snapshot.joints.position))
        if arm_start is not None:
            values.update(zip(arm_start.name,arm_start.position))
        width = float(snapshot.gripper.position)
        if not math.isfinite(width) or not 0 <= width <= .09:
            raise ValueError('invalid actual gripper opening')
        values.update(left_finger_joint=width/2.,right_finger_joint=-width/2.)
        return JointState(name=list(values),position=list(values.values()))

    def _hardware_idle(self, now):
        """真机需有新鲜, 无故障的控制器空闲状态; 仿真由动作和关节反馈确认停止."""
        if not bool(self.get_parameter("use_hardware").value):
            return True
        snapshot = self._feedback_receiver.snapshot()
        if snapshot.status is None:
            return False
        received, status = snapshot.status_at, snapshot.status
        stamp = status.header.stamp.sec * 1_000_000_000 + status.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        # 同时检查本地接收年龄和消息源时间戳; 状态码 0/1 均可接受, 不要求失能.
        return (now-received <= 2.0 and stamp > 0 and -.25 <= age <= 2.0
                and not status.error_codes and status.state_machine == "IDLE"
                and len(status.per_joint_status_code) >= 6
                and all(code in (0, 1) for code in status.per_joint_status_code))

    def _stop(self, _request, response):
        """旧请求退出, 动作终结, 控制器停止成功且新鲜反馈持续静止后, 才确认停止.

        确认成功仅解除本节点的停止门控, 不执行使能或失能; 超时和异常保留门控,
        待问题解决后可再次调用停止服务重新检查.
        """
        # 同时只允许一轮停止确认, 避免多个调用相互重置代次或静止观察窗口.
        if not self._stop_lock.acquire(blocking=False):
            response.success = False
            response.message = "stop confirmation already in progress"
            return response
        try:
            with self._motion_lock:
                # 先阻断新请求并使旧规划失效, 再向动作和控制器发送停止请求.
                self._stopping = True
                self._generation += 1
                handle = self._active_goal_handle
            if handle is not None:
                handle.cancel_goal_async()
            timeout = max(float(self.get_parameter("stop_confirmation_timeout_sec").value), .1)
            deadline = time.monotonic() + timeout
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout, .5)):
                raise RuntimeError("trajectory_stop unavailable")
            # 动作取消与控制器停止分别确认; 只收到其中一个响应不足以判定停稳.
            stop_future = self._trajectory_stop_client.call_async(Trigger.Request())
            quiet_since = None
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic()
                with self._motion_lock:
                    terminal = self._action_terminal()
                    stopped = (stop_future.done() and not stop_future.cancelled()
                               and stop_future.result() is not None and stop_future.result().success)
                    # 旧请求必须退出, 防止其规划结果或晚到的目标接收响应重新触发运动.
                    idle = stopped and terminal and not self._request_active and self._hardware_idle(now)
                    if not idle:
                        # 任一前提失效都重新开始观察, 不能沿用此前累计的静止证据.
                        quiet_since = None
                    elif quiet_since is None:
                        # 新显式停止可等待健康流后建立新证据; 已开始的观察若断流,
                        # 保留原起点并超时, 不能用恢复帧把同一次失败变成成功.
                        if self._stop_feedback.ready(now, self.get_clock().now().nanoseconds):
                            quiet_since = now
                    elif self._stop_feedback.stationary(quiet_since, now, self.get_clock().now().nanoseconds):
                        # 只接受观察起点之后的新鲜静止样本, 确认成功后才解除停止门控.
                        self._stopping = False
                        response.success = True
                        response.message = "motion stop confirmed; holding current position"
                        return response
                time.sleep(.02)
            raise RuntimeError("stop confirmation timed out: pending request/action, stale or moving feedback, or controller fault")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        finally:
            # 无论成功与否都释放停止锁; 失败时保留停止标记, 允许后续重新检查.
            self._stop_lock.release()

    def _wait_future(self, future, timeout_sec: float) -> None:
        """在四线程执行器中同步等待异步结果完成.

        当前回调短周期休眠, 结果响应和停止请求由其他线程处理, 本函数不自行自旋.
        超时下限为 0.1 秒; 返回后由调用方检查是否完成, 超时不会自动取消请求.
        """
        deadline = time.monotonic() + max(float(timeout_sec), 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)


def main(args=None) -> None:
    """创建四线程业务执行器, 支持同步等待与停止回调并发; 反馈由独立接收器处理."""
    rclpy.init(args=args)
    node = PoseExecutionNode()
    # 4 线程: execute_pose 阻塞等轨迹结果时, stop 服务与动作反馈仍能被处理.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
