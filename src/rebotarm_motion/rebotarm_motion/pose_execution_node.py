"""点到点（Pose）运动执行节点。

职责与位置
----------
本模块位于运动层，是"位姿目标 -> 关节轨迹 -> 控制器动作"的唯一执行入口，
不直接访问电机 SDK（硬件访问一律留在控制器包内）。流程为：

1. 收到 ExecutePose 请求后交给规划适配器 `MoveItMotionPlanner`，由它调用
   MoveIt 的规划服务求出关节空间轨迹；
2. 若请求 `execute=false`，只做规划并返回轨迹；单段请求可直接发布 RViz 幻影；
   多阶段视觉入口则先收集各段结果，再通过 `publish_trajectory_preview` 一次发布；
3. 若需要执行，则把轨迹作为 FollowJointTrajectory 目标发给控制器动作服务
   `/<arm_namespace>/follow_joint_trajectory`，并同步等待结果。

对外接口
--------
服务（都在 `/<arm_namespace>/motion_execution/` 下）：
- `execute_pose`（ExecutePose）：规划并在需要时执行一个末端位姿目标；
- `publish_trajectory_preview`（PublishTrajectoryPreview）：把多段已规划轨迹作为一个
  RViz 显示序列发布，不会下发控制器动作；
- `stop`（Trigger）：取消当前动作目标，并请求控制器的 trajectory_stop。

动作客户端：`/<arm_namespace>/follow_joint_trajectory`。

安全语义
--------
- 本节点不做使能/失能操作，也不做碰撞或限位校验；它假定控制器侧已处于使能
  状态并由上层（视觉抓取执行器等）决定何时下发；
- 速度/加速度缩放参数默认很小（0.10 / 0.08），请求中给 0.0 时回落到这些默认
  值，避免上位随手传 0 导致按满速执行；
- `stop` 成功要求旧请求退出、动作终态、控制器停止成功及新鲜关节反馈持续静止；
  真机还要求新鲜且无故障的 IDLE 状态。确认失败时阻止后续执行，可再次 stop 重新检查。
"""

from __future__ import annotations

import time
import math
import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus
from .stop_feedback import StopFeedback

from rebotarm_msgs.srv import ExecutePose, PublishTrajectoryPreview

from .moveit_planner import MoveItMotionPlanner


class PoseExecutionNode(Node):
    """位姿执行节点：规划 + 下发轨迹 + 提供停止入口。

    生命周期：由 `main()` 创建后交给 4 线程的多线程执行器自旋，直到进程收到
    中断。执行请求独占运动入口；stop 增加代次、阻止旧规划下发，并跟踪延迟的
    动作接受与终态。停止尚未确认时拒绝新执行。

    线程模型：所有服务、动作客户端与规划器共用同一个可重入回调组，允许执行中
    的 `execute_pose` 与 `stop` 并行进入。
    """

    def __init__(self) -> None:
        super().__init__("motion_execution")
        # 可重入回调组：execute_pose 会长时间阻塞等待轨迹结果，stop 服务必须能在
        # 同一时刻被处理，否则急停请求会被排在后面。
        self._callback_group = ReentrantCallbackGroup()

        # arm_namespace：话题/服务/动作的公共前缀，去掉首尾斜杠后拼接。
        self.declare_parameter("arm_namespace", "rebotarm")
        # frame_id：位姿目标所在的世界坐标系（MoveIt 规划时的参考系）。
        self.declare_parameter("frame_id", "base_link")
        # ee_frame_id：末端执行器 link 名，MoveIt 位置/姿态约束作用在该 link 上。
        self.declare_parameter("ee_frame_id", "end_link")
        # moveit_group_name：SRDF 中的规划组名（six-axis 手臂组）。
        self.declare_parameter("moveit_group_name", "arm")
        # moveit_planning_service：MoveIt 规划服务名（GetMotionPlan 接口）。
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        # moveit_planning_pipeline：规划管线 id，默认 ompl。
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        # moveit_planner_id：具体规划器 id，空串表示用管线默认规划器。
        self.declare_parameter("moveit_planner_id", "")
        # moveit_planning_time：单次规划允许的最长时间，单位 s。
        self.declare_parameter("moveit_planning_time", 2.0)
        # moveit_num_planning_attempts：规划尝试次数，越大越可能成功但更慢。
        self.declare_parameter("moveit_num_planning_attempts", 1)
        # goal_position_tolerance：末端位置约束容差，单位 m（约束盒边长）。
        self.declare_parameter("goal_position_tolerance", 0.005)
        # goal_orientation_tolerance：末端姿态各轴容差，单位 rad。
        self.declare_parameter("goal_orientation_tolerance", 0.02)
        # default_velocity_scaling：请求未指定速度缩放时的默认值，无量纲 (0,1]。
        self.declare_parameter("default_velocity_scaling", 0.10)
        # default_acceleration_scaling：请求未指定加速度缩放时的默认值，无量纲 (0,1]。
        self.declare_parameter("default_acceleration_scaling", 0.08)
        # 仅视觉纯规划入口启用；其他 ExecutePose 调用不会产生预览副作用。
        self.declare_parameter("publish_plan_only_preview", False)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("stop_confirmation_timeout_sec", 8.0)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        # 正在执行的动作目标句柄，仅用于 stop 时发起取消；执行结束后置回 None。
        self._active_goal_handle = None
        self._motion_lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._request_active = False
        self._stopping = False
        self._generation = 0
        self._send_future = None
        self._result_future = None
        self._stop_feedback = StopFeedback()
        self._arm_status = None
        self.create_subscription(JointState, f"/{self._arm_namespace}/joint_states",
                                 self._on_stop_feedback, qos_profile_sensor_data,
                                 callback_group=self._callback_group)
        self.create_subscription(ArmStatus, f"/{self._arm_namespace}/arm_status",
                                 self._on_arm_status, 10, callback_group=self._callback_group)
        self._preview_publisher = (
            self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
            if bool(self.get_parameter("publish_plan_only_preview").value)
            else None
        )
        # 轨迹执行动作客户端（FollowJointTrajectory）：与控制器包约定的执行通道。
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
            callback_group=self._callback_group,
        )
        # 控制器侧的轨迹停止服务，作为取消动作之外的兜底停止通道。
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
            callback_group=self._callback_group,
        )
        # 规划适配器：负责把位姿目标翻译成 MoveIt 请求并解析响应。
        self._planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id=str(self.get_parameter("ee_frame_id").value),
            frame_id=str(self.get_parameter("frame_id").value),
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_orientation_tolerance=float(self.get_parameter("goal_orientation_tolerance").value),
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
        # 停止服务：有界等待动作终态和新鲜静止反馈，失败时保持执行门控。
        self.create_service(
            Trigger,
            f"/{self._arm_namespace}/motion_execution/stop",
            self._stop,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"pose motion execution ready: /{self._arm_namespace}/motion_execution/execute_pose"
        )

    def _on_stop_feedback(self, msg):
        self._stop_feedback.update(msg, time.monotonic(), self.get_clock().now().nanoseconds)

    def _on_arm_status(self, msg):
        with self._motion_lock:
            self._arm_status = (time.monotonic(), msg)

    def _execute_pose(self, request, response):
        with self._motion_lock:
            if self._request_active or self._stopping:
                response.success = False
                response.message = "motion busy or stop not confirmed"
                return response
            self._request_active = True
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
                self._request_active = False

    def _interrupted(self):
        return self._stopping or self._run_generation != self._generation

    def _execute_pose_impl(self, request: ExecutePose.Request, response: ExecutePose.Response):
        """规划并可选执行一个末端位姿目标。

        请求字段：`target_pose`（位姿目标）、`velocity_scaling`/`acceleration_scaling`
        （无量纲缩放，0.0 表示用参数默认值）、`timeout_sec`（本步骤超时，s）、
        `execute`（False 时只规划不下发）、`preview_start_joint_state`（可选虚拟起点，
        仅允许 execute=False）、`suppress_preview`（多阶段预览收集期间禁止逐段发布）。
        响应字段：`success`、`stage`（"planning" 或 "execution"，用于区分失败发生在
        哪个阶段）、`message`、`planned_trajectory`。
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
                or len(preview_start.position) != 6
                or not all(math.isfinite(value) for value in preview_start.position)
            ):
                response.success = False
                response.message = "invalid preview start joint state"
                return response
        # 请求里 0.0 视为"未指定"：回落节点参数，避免上位传 0 变成满速或非法值。
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

        # 即使随后不执行，也把规划结果回填，便于调用方审查轨迹。
        response.planned_trajectory = plan.trajectory
        if not bool(request.execute):
            if self._preview_publisher is not None and not bool(request.suppress_preview):
                trajectory = plan.trajectory
                # DisplayTrajectory 的初态必须与路径起点一致；不能将当前假关节状态
                # 错当成上一阶段的终点。此消息仅用于 RViz，绝不发给轨迹 Action。
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
            if self._interrupted():
                response.success = False
                response.message = "stopped during planning"
                return response
        # 请求超时至少按 1 s 处理：动作服务器发现/连接握手需要时间，给 0 会立刻误判不可用。
        if not self._trajectory_client.wait_for_server(timeout_sec=max(float(request.timeout_sec), 1.0)):
            response.success = False
            response.message = "follow_joint_trajectory action unavailable"
            return response

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = plan.trajectory
        with self._motion_lock:
            if self._interrupted():
                response.success = False
                response.message = "stopped before trajectory dispatch"
                return response
            send_future = self._trajectory_client.send_goal_async(goal)
            self._send_future = send_future
        self._wait_future(send_future, max(float(request.timeout_sec), 1.0))
        if not send_future.done():
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

        # stop 也跟踪 send_future，接受响应晚到时仍会取消并等待终态。
        with self._motion_lock:
            self._active_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            self._result_future = result_future
            if self._interrupted():
                goal_handle.cancel_goal_async()
        # 结果等待窗口至少 30 s：轨迹本身可能远长于规划超时，必须给它跑完的机会。
        self._wait_future(result_future, max(float(request.timeout_sec), 30.0))
        if not result_future.done():
            with self._motion_lock:
                self._stopping = True
            goal_handle.cancel_goal_async()
            response.success = False
            response.message = "trajectory result timeout"
            return response

        wrapped = result_future.result()
        # status 为动作终态（4 = STATUS_SUCCEEDED），error_code 为控制器返回的执行结果码。
        status = int(getattr(wrapped, "status", 0))
        result = getattr(wrapped, "result", None)
        error_string = str(getattr(result, "error_string", "") or "")
        error_code = int(getattr(result, "error_code", 0) or 0)
        # 两个条件都满足才算执行成功：动作被取消/中止时 status 不会是 4。
        if status != 4 or error_code != int(FollowJointTrajectory.Result.SUCCESSFUL):
            response.success = False
            response.message = (
                f"trajectory failed: status={status}, error_code={error_code}, {error_string}".strip()
            )
            return response

        response.success = True
        response.message = "trajectory executed"
        return response

    def _publish_trajectory_preview(
        self,
        request: PublishTrajectoryPreview.Request,
        response: PublishTrajectoryPreview.Response,
    ):
        """把多段轨迹放进同一条 DisplayTrajectory，供 RViz 连续播放。

        `DisplayTrajectory.trajectory` 本身就是有序轨迹数组：第一段从
        `trajectory_start` 开始，后续段从前一段终点继续。这里严格检查关节顺序、
        点维度与相邻端点连续性；任何异常都会整组拒绝，避免显示一条拼接错误的路径。
        本函数只调用 RViz publisher，不接触 FollowJointTrajectory 动作客户端。
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
            trajectories = with_gripper_preview(
                trajectories, request.gripper_before_trajectory, request.gripper_openings_m
            )
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response
        # 重新构造显示序列，包含开合动画和保持开口的机械臂段。
        display.trajectory = []
        for trajectory in trajectories:
            robot_trajectory = RobotTrajectory()
            robot_trajectory.joint_trajectory = trajectory
            display.trajectory.append(robot_trajectory)
        expected_names = list(trajectories[0].joint_names)
        first_point = trajectories[0].points[0]
        display.trajectory_start.joint_state.name = expected_names
        display.trajectory_start.joint_state.position = list(first_point.positions)
        self._preview_publisher.publish(display)
        response.success = True
        response.message = f"published {len(trajectories)} preview trajectories"
        return response

    def _cancel_late_goal(self, future):
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                with self._motion_lock:
                    self._active_goal_handle = handle
                    self._result_future = handle.get_result_async()
                handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"late goal cancellation failed: {exc}")

    def _action_terminal(self):
        """Caller holds motion lock. Unknown/late acceptance is never idle."""
        if self._send_future is None:
            return True
        if not self._send_future.done() or self._send_future.cancelled():
            return False
        handle = self._send_future.result()
        if handle is None:
            return False
        if not handle.accepted:
            return True
        if self._result_future is None:
            self._active_goal_handle = handle
            self._result_future = handle.get_result_async()
            handle.cancel_goal_async()
        future = self._result_future
        return (future.done() and not future.cancelled()
                and int(future.result().status) in (4, 5, 6))

    def _hardware_idle(self, now):
        if not bool(self.get_parameter("use_hardware").value):
            return True
        if self._arm_status is None:
            return False
        received, status = self._arm_status
        stamp = status.header.stamp.sec * 1_000_000_000 + status.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        return (now-received <= 2.0 and stamp > 0 and -.25 <= age <= 2.0
                and not status.error_codes and status.state_machine == "IDLE"
                and len(status.per_joint_status_code) >= 6
                and all(code in (0, 1) for code in status.per_joint_status_code))

    def _stop(self, _request, response):
        """Acknowledge only after cancellation, terminal action and fresh quiet feedback."""
        if not self._stop_lock.acquire(blocking=False):
            response.success = False
            response.message = "stop confirmation already in progress"
            return response
        try:
            with self._motion_lock:
                self._stopping = True
                self._generation += 1
                handle = self._active_goal_handle
            if handle is not None:
                handle.cancel_goal_async()
            timeout = max(float(self.get_parameter("stop_confirmation_timeout_sec").value), .1)
            deadline = time.monotonic() + timeout
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout, .5)):
                raise RuntimeError("trajectory_stop unavailable")
            stop_future = self._trajectory_stop_client.call_async(Trigger.Request())
            quiet_since = None
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic()
                with self._motion_lock:
                    terminal = self._action_terminal()
                    stopped = (stop_future.done() and not stop_future.cancelled()
                               and stop_future.result() is not None and stop_future.result().success)
                    idle = stopped and terminal and not self._request_active and self._hardware_idle(now)
                    if not idle:
                        quiet_since = None
                    elif quiet_since is None:
                        quiet_since = now
                    elif self._stop_feedback.stationary(quiet_since, now, self.get_clock().now().nanoseconds):
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
            # Failure intentionally keeps _stopping set; another stop can
            # recheck evidence after the underlying problem is resolved.
            self._stop_lock.release()

    def _wait_future(self, future, timeout_sec: float) -> None:
        """在 4 线程执行器中同步等待一个 future 完成。

        采用短周期轮询而不是 future.add_done_callback + 事件，是为了让回调语义
        保持"串行阻塞"这一简单模型；超时下限 0.1 s 避免调用方传 0 时直接空转返回。
        """
        deadline = time.monotonic() + max(float(timeout_sec), 0.1)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseExecutionNode()
    # 4 线程：execute_pose 阻塞等轨迹结果时，stop 服务与动作反馈仍能被处理。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
