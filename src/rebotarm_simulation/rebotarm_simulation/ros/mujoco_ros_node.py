"""MuJoCo 后端的 ROS 执行、物理步进及停止保持适配。

文件职责
    提供仿真 FJT、夹爪、控制模式及关节阻抗服务，驱动物理定时器并发布实际状态。
对外接口
    create_node_class 延迟加载 ROS 并返回节点类；main 创建执行器。
    trajectory_execution_mode 选择 legacy 或 continuous；准备服务只返回检查结果，
    execution 组件拥有目标令牌、资源占用及原子接纳；节点驱动物理时钟。
安全约束
    不导入真实硬件后端或运动层实现；参考求值来自共享数值包。
    锁序固定为活动目标锁 → 仿真锁；准备服务等待、停止等待均不能占住物理步进锁。
    continuous 用仿真时间采样，用主机单调时间控制准备期限和反馈新鲜度。
    固定夹爪或场景变化后停止保持；停止未确认继续占用资源，不能因 Action 退出而放行。
    正常成功保持曲线终点；Action 成功条件与独立物理验收指标分开。
文件结构
    延迟创建节点 → 参数/执行组件/ROS 装配 → 协议转换与服务等待
    → Action 反馈和终态映射 → 物理调度及状态发布 → 关闭和进程入口。
"""

from __future__ import annotations

import math
import json
import threading
import time

from rebotarm_simulation.ros.message_codec import DEFAULT_MAX_TRAJECTORY_DURATION_SEC
from rebotarm_simulation.ros.message_codec import DEFAULT_MAX_TRAJECTORY_POINTS
from rebotarm_simulation.execution.feedback_timing import FeedbackRateLimiter
from rebotarm_simulation.execution.trajectory_state import GateOutcome
from rebotarm_simulation.execution.goal_policy import GoalSettlingPolicy
from rebotarm_simulation.ros.message_codec import MonotonicStamp
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.ros.message_codec import duration_to_seconds
from rebotarm_simulation.execution.trajectory_state import terminal_disposition
from rebotarm_simulation.ros.message_codec import trajectory_to_sampler
from rebotarm_simulation.ros.message_codec import validate_gripper_width
from rebotarm_simulation.execution.trajectory_sampler import ARM_JOINT_NAMES
from rebotarm_trajectory.polynomial import TrajectoryPoint
from rebotarm_trajectory.joint_contract import canonical_input
from rebotarm_trajectory.capability import make_capability, constraint_revision
from rebotarm_simulation.execution.trajectory_execution import TrajectoryExecution
from pathlib import Path


from rebotarm_simulation.execution.control_api import SimulationControlApi, normalize_ros_control_mode
from rebotarm_simulation.ros.message_codec import validate_gripper_force

DEFAULT_HOME_JOINT_POSITIONS = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)

def create_node_class():
    """延迟导入 ROS 并返回具体节点类；仅导入本模块不会创建节点或仿真实例。"""
    import rclpy
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from rebotarm_msgs.action import ExecuteTeleopMotion
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rebotarm_msgs.msg import JointMotorState, ArmStatus
    from rebotarm_msgs.srv import SetGripper, SetMode, SetJointImpedance, GetArmPlanningState
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.ros.ros_diagnostics import build_control_diagnostic

    class RebotArmMujocoNode(Node):
        def __init__(self) -> None:
            super().__init__("rebotarm_mujoco_node")
            self.declare_parameter("backend", "mujoco")
            self.declare_parameter("headless", True)
            self.declare_parameter("show_viewer", False)
            self.declare_parameter("model_path", "")
            self.declare_parameter("arm_namespace", "rebotarm")
            self.declare_parameter("trajectory_execution_mode", "legacy")
            self.declare_parameter('trajectory_trace_path', '')
            from rebotarm_simulation.diagnostics.trajectory_trace import TrajectoryTrace
            self._trace = TrajectoryTrace(self.get_parameter('trajectory_trace_path').value)
            self._trajectory_mode = self.get_parameter("trajectory_execution_mode").value
            if self._trajectory_mode not in ('legacy', 'continuous'):
                raise ValueError('trajectory_execution_mode must be legacy or continuous')
            self.declare_parameter("publish_rate_hz", 30.0)
            self.declare_parameter("max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS)
            self.declare_parameter("max_trajectory_duration_sec", DEFAULT_MAX_TRAJECTORY_DURATION_SEC)
            self.declare_parameter("initial_joint_positions", list(DEFAULT_HOME_JOINT_POSITIONS))
            self.declare_parameter("goal_position_tolerance", 0.02)
            self.declare_parameter("goal_velocity_tolerance", 0.05)
            self.declare_parameter("goal_time_tolerance_sec", 5.0)
            self.declare_parameter("feedback_rate_hz", 20.0)
            self.declare_parameter("diagnostic_rate_hz", 1.0)
            self.declare_parameter("max_contact_force_n", 200.0)
            self.declare_parameter("max_contact_penetration_m", 0.005)
            for name in ("diagnostic_rate_hz", "max_contact_force_n", "max_contact_penetration_m"):
                value = float(self.get_parameter(name).value)
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError(f"{name} must be positive and finite")
            # 以下三项是启动前的硬门：参数不合法直接抛错终止，绝不带着错误配置跑仿真。
            if self.get_parameter("backend").value != "mujoco":
                raise ValueError("simulation backend must be mujoco")
            if self.get_parameter("headless").value is not True:
                raise ValueError("ROS adapter is headless; run the viewer separately")

            namespace = str(self.get_parameter("arm_namespace").value).strip("/")
            if not namespace or any(part in namespace for part in ("//", " ")):
                raise ValueError("arm namespace is invalid")
            self._arm_namespace = namespace
            rate = float(self.get_parameter("publish_rate_hz").value)
            if not math.isfinite(rate) or rate <= 0.0 or rate > 1000.0:
                raise ValueError("publish rate must be finite and in (0, 1000]")
            self._max_points = int(self.get_parameter("max_trajectory_points").value)
            self._max_duration = float(self.get_parameter("max_trajectory_duration_sec").value)
            # Per-goal JointTolerance arrays are intentionally not interpreted:
            # this simulation backend uses these bounded node-wide defaults so
            # clients cannot weaken settling guarantees on individual goals.
            settling_policy = GoalSettlingPolicy(
                float(self.get_parameter("goal_position_tolerance").value),
                float(self.get_parameter("goal_velocity_tolerance").value),
                float(self.get_parameter("goal_time_tolerance_sec").value),
            )
            self._feedback_rate_hz = float(self.get_parameter("feedback_rate_hz").value)
            # Constructor performs finite/positive/range validation.
            FeedbackRateLimiter(self._feedback_rate_hz)
            self._execute_wait_sec = min(0.01, 1.0 / self._feedback_rate_hz)
            initial = tuple(float(v) for v in self.get_parameter("initial_joint_positions").value)
            if len(initial) != 6 or any(not math.isfinite(v) for v in initial):
                raise ValueError("initial positions must contain six finite values")
            diagnostic_rate = float(self.get_parameter("diagnostic_rate_hz").value)
            self._max_contact_force = float(self.get_parameter("max_contact_force_n").value)
            self._max_contact_penetration = float(
                self.get_parameter("max_contact_penetration_m").value
            )
            if (
                not math.isfinite(diagnostic_rate)
                or diagnostic_rate <= 0.0
                or not math.isfinite(self._max_contact_force)
                or self._max_contact_force <= 0.0
                or not math.isfinite(self._max_contact_penetration)
                or self._max_contact_penetration <= 0.0
            ):
                raise ValueError("diagnostic rates and contact thresholds must be positive finite values")

            model_path = str(self.get_parameter("model_path").value).strip()
            self._sim = RebotArmMujoco(model_path or None)
            self._control = SimulationControlApi(self._sim)
            self._retain_simulation = False
            self._lock = threading.RLock()
            self._sim_access = SerializedSimulationAccess(self._sim, self._lock)
            def initialize(_sim) -> None:
                self._control.reset_home_and_hold()
                if initial != DEFAULT_HOME_JOINT_POSITIONS:
                    self._control.command_joint_positions(initial)

            self._sim_access.run(initialize)
            self._callback_group = ReentrantCallbackGroup()
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._capability_publisher = self.create_publisher(String,
                f'/{namespace}/trajectory_execution_mode', qos)
            self._capability_publisher.publish(String(data=self._trajectory_mode))
            self.declare_parameter('gripper_position_max_speed_rad_s', .5)
            self.declare_parameter('gripper_position_torque_cap_nm', 1.)
            import secrets
            from ament_index_python.packages import get_package_share_directory
            directory = Path(get_package_share_directory('rebotarm_moveit_config'))/'config'
            revision = constraint_revision(*(directory.joinpath(name).read_bytes()
                for name in ('rebotarm.urdf','joint_limits.yaml')))
            capability = make_capability(self._trajectory_mode, secrets.randbits(63),
                float(self.get_parameter('gripper_position_max_speed_rad_s').value),
                float(self.get_parameter('gripper_position_torque_cap_nm').value), revision)
            self._execution = TrajectoryExecution(self._sim_access, self._control,
                capability, self._trajectory_mode, settling_policy)
            # 仅保存 ROS Goal UUID 与执行令牌的传输关联；任务状态仍全部归 execution。
            self._goal_tokens = {}
            self._versioned_capabilities = self.create_publisher(String,
                f'/{namespace}/trajectory_capabilities', qos)
            self.create_service(GetArmPlanningState, f'/{namespace}/get_arm_planning_state',
                self._planning_state_service, callback_group=self._callback_group)
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
            self._capability_timer = self.create_timer(.1, self._publish_admission, clock=self._physics_clock)
            if self._trajectory_mode == 'continuous':
                from rebotarm_msgs.srv import PrepareArmTrajectory
                self._prepare_type = PrepareArmTrajectory
                self._prepare_client = self.create_client(PrepareArmTrajectory,
                    f'/{namespace}/prepare_arm_trajectory', callback_group=self._callback_group)
                self.create_subscription(String, f'/{namespace}/trajectory_scene_revision',
                    self._scene_changed, qos, callback_group=self._callback_group)
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            self._stamp = MonotonicStamp()
            # 只读反馈话题：关节状态、夹爪状态与仿真时钟。队列深度 10 足以吸收
            # 短暂抖动，仿真时间由 /clock 统一对外，供 use_sim_time 的节点对齐。
            self._joint_pub = self.create_publisher(
                JointState, f"/{self._arm_namespace}/joint_states", 10
            )
            self._gripper_pub = self.create_publisher(
                JointMotorState, f"/{self._arm_namespace}/gripper/state", 10
            )
            self._clock_pub = self.create_publisher(Clock, "/clock", 10)
            self._status_pub = self.create_publisher(ArmStatus, f'/{namespace}/arm_status', qos)
            self._diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diagnostic_last_wall = time.monotonic()
            self._diagnostic_last_sim = 0.0
            # 与真实控制器同名的轨迹动作接口：上层运动/示教代码无需区分真机与仿真。
            self._action_server = ActionServer(
                self,
                FollowJointTrajectory,
                f"/{self._arm_namespace}/follow_joint_trajectory",
                goal_callback=self._goal_callback,
                handle_accepted_callback=self._handle_accepted,
                cancel_callback=self._cancel_callback,
                execute_callback=self._execute_goal,
                callback_group=self._callback_group,
            )
            self._gripper_servers = []
            for suffix,moveit in (('command',False),('moveit_command',True)):
                self._gripper_servers.append(ActionServer(self,GripperCommand,
                    f'/{namespace}/gripper/{suffix}',
                    goal_callback=self._gripper_goal_callback,
                    cancel_callback=lambda goal: CancelResponse.ACCEPT,
                    execute_callback=lambda goal,m=moveit:self._execute_gripper(goal,moveit=m),
                    callback_group=self._callback_group))
            if self._trajectory_mode == 'continuous':
                self._teleop_server = ActionServer(self,ExecuteTeleopMotion,
                    f'/{namespace}/execute_teleop_motion',
                    goal_callback=lambda goal:self._goal_callback(goal,web=True),
                    handle_accepted_callback=self._handle_accepted,
                    cancel_callback=self._cancel_callback,
                    execute_callback=lambda goal:self._execute_goal(goal,web=True),
                    callback_group=self._callback_group)
            self.create_service(
                Trigger,
                f"/{self._arm_namespace}/trajectory_stop",
                self._stop_service,
                callback_group=self._callback_group,
            )
            self.create_service(
                SetGripper,
                f"/{self._arm_namespace}/gripper/set",
                self._gripper_service,
                callback_group=self._callback_group,
            )
            self.create_service(
                SetMode,
                f"/{self._arm_namespace}/sim/set_mode",
                self._mode_service,
                callback_group=self._callback_group,
            )
            self.create_service(
                SetJointImpedance,
                f"/{self._arm_namespace}/sim/set_joint_impedance",
                self._impedance_service,
                callback_group=self._callback_group,
            )
            # Each callback advances enough fixed physics steps to match the
            # configured publication period; simulation time remains the
            # authoritative trajectory clock.
            self._steps_per_tick = max(1, round((1.0 / rate) / self._sim.timestep))
            self.create_timer(
                1.0 / rate,
                self._timer_callback,
                callback_group=self._timer_callback_group,
                clock=self._physics_clock,
            )
            self.create_timer(
                1.0 / float(self.get_parameter("diagnostic_rate_hz").value),
                self._publish_diagnostics,
                callback_group=self._timer_callback_group,
                clock=self._physics_clock,
            )

        def _current_arm_positions(self) -> tuple[float, ...]:
            return self._sim_access.run(
                lambda sim: tuple(sim.get_state().joint_positions[:6])
            )

        @property
        def show_viewer(self) -> bool:
            return bool(self.get_parameter("show_viewer").value)

        def viewer_handles(self):
            return self._sim_access.run(lambda sim: sim.borrow_viewer_handles())

        def sync_viewer(self, viewer) -> None:
            self._sim_access.run(lambda _sim: viewer.sync())

        def close(self) -> None:
            self._sim_access.run(lambda sim: sim.close())



        def _goal_callback(self, goal_request, *, web=False):
            received = time.monotonic()
            token = object()
            try:
                names = tuple(goal_request.trajectory.joint_names)
                if len(names) != 6 or set(names) != set(ARM_JOINT_NAMES):
                    raise ValueError('FJT requires exactly six arm joints; use independent gripper command')
                if self._trajectory_mode == 'continuous':
                    if not self._prepare_client.service_is_ready():
                        raise RuntimeError('preparation service unavailable')
                    command = None
                    if web and goal_request.move_gripper:
                        command = (validate_gripper_width(goal_request.gripper_position),
                            float(goal_request.gripper_max_effort))
                        validate_gripper_force(command[1])
                    points = tuple(TrajectoryPoint(duration_to_seconds(p.time_from_start),
                        p.positions, p.velocities, p.accelerations) for p in goal_request.trajectory.points)
                    canonical_input(names, points)
                    if not 0 < len(points) <= self._max_points or points[-1].time > self._max_duration:
                        raise ValueError('trajectory point or duration budget exceeded')
                    self._execution.reserve(token, names, points, goal_request.trajectory.header.frame_id,
                        web=web, gripper_command=command, received=received)
                else:
                    sampler = trajectory_to_sampler(goal_request.trajectory,
                        initial_positions=self._current_arm_positions(),
                        max_points=self._max_points, max_duration_sec=self._max_duration)
                    self._execution.admit_legacy(token, sampler)
                return GoalResponse.ACCEPT
            except (TypeError, ValueError, RuntimeError) as exc:
                self.get_logger().warning(f"FJT goal rejected: {exc}")
                return GoalResponse.REJECT

        def _planning_state_service(self, _request, response):
            try:
                response.state_json = json.dumps(self._execution.planning_snapshot(), allow_nan=False)
                response.success, response.message = True, 'ready'
            except (ValueError, RuntimeError) as exc:
                response.success, response.message = False, str(exc)
            return response

        def _publish_admission(self):
            self._versioned_capabilities.publish(String(data=json.dumps(self._execution.capability, allow_nan=False)))
            reasons = self._execution.admission_reasons()
            status = self._execution.status_snapshot()
            if self._trajectory_mode == 'continuous':
                if not status.scene_revision:
                    reasons.append('scene revision unavailable')
                if not self._prepare_client.service_is_ready():
                    reasons.append('preparation service unavailable')
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [DiagnosticStatus(name=f'{self._arm_namespace}/trajectory_admission',hardware_id='simulation',
                level=DiagnosticStatus.WARN if reasons else DiagnosticStatus.OK,
                message='; '.join(reasons) if reasons else 'ready',
                values=[KeyValue(key='scene_revision',value=status.scene_revision)])]
            self._diagnostic_pub.publish(message)

        def _scene_changed(self, message):
            self._execution.scene_changed(message.data)

        def _prepare_sampler(self, goal_handle, token):
            """ROS 工作线程仅编码请求及等待 Future；执行组件完成上下文复核与原子接纳。"""
            pending = self._execution.preparation_for(token)
            request = self._prepare_type.Request()
            request.trajectory = goal_handle.request.trajectory
            for field in ('session', 'generation', 'feedback_observed_at', 'gripper_generation',
                          'gripper_collision_policy', 'scene_revision', 'deadline_monotonic'):
                setattr(request, field, getattr(pending, field))
            for field in ('hold_positions', 'feedback_positions', 'gripper_positions'):
                setattr(request, field, list(getattr(pending, field)))
            request.stationary, request.enabled = False, True
            request.capability_json = json.dumps(self._execution.capability, allow_nan=False)
            self.get_logger().info(f'trajectory preparation dispatch age={time.monotonic()-pending.feedback_observed_at:.4f}s')
            future = self._prepare_client.call_async(request)
            while not future.done():
                if (goal_handle.is_cancel_requested or self._execution.active.cancel_requested
                        or time.monotonic() >= pending.deadline_monotonic):
                    future.cancel()
                    raise RuntimeError('trajectory preparation canceled or timed out')
                time.sleep(.002)
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            sampler = self._execution.admit(token, json.loads(response.prepared_json),
                response.scene_revision, lambda: goal_handle.is_cancel_requested)
            self.get_logger().info(f'prepared curve accepted degree={sampler.curve.degree} points={len(sampler.curve.points)}')
            return sampler

        def _wait_prepared_stop(self, token):
            wait = self._execution.begin_stop_wait(token)
            while not self._execution.poll_stop(wait):
                time.sleep(.01)

        def _handle_accepted(self, goal_handle):
            """协议句柄绑定本次令牌；取消与执行回调都使用同一关联。"""
            with self._execution.lock:
                self._goal_tokens[bytes(goal_handle.goal_id.uuid)] = self._execution.active.token
            goal_handle.execute()

        def _cancel_callback(self, goal_handle):
            with self._execution.lock:
                token = self._goal_tokens.get(bytes(goal_handle.goal_id.uuid))
                if token is None or not goal_handle.is_active:
                    return CancelResponse.REJECT
                stopped = self._execution.stop(token)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            with self._execution.lock:
                token = self._execution.active.token
                stopped = self._execution.stop()
            if self._trajectory_mode == 'continuous' and stopped:
                try:
                    self._wait_prepared_stop(token)
                except RuntimeError as exc:
                    response.success, response.message = False, str(exc)
                    return response
            response.success = True
            response.message = "simulation trajectory stop requested" if stopped else "no active trajectory"
            return response

        def _publish_diagnostics(self):
            state, contacts, status = self._sim_access.run(
                lambda sim: (
                    sim.get_state(),
                    sim.get_contacts(),
                    self._control.get_control_status(),
                )
            )
            now = time.monotonic()
            elapsed = max(now - self._diagnostic_last_wall, 1e-9)
            physics_rate = (state.simulation_time - self._diagnostic_last_sim) / elapsed / self._sim.timestep
            self._diagnostic_last_wall, self._diagnostic_last_sim = now, state.simulation_time
            summary = build_control_diagnostic(
                arm_namespace=self._arm_namespace,
                configured_rate_hz=1.0 / self._sim.timestep,
                measured_rate_hz=physics_rate,
                state=state,
                status=status,
                contacts=contacts,
                max_contact_force_n=self.get_parameter("max_contact_force_n").value,
                max_contact_penetration_m=self.get_parameter("max_contact_penetration_m").value,
            )
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            item = DiagnosticStatus()
            item.level = DiagnosticStatus.WARN if summary.warning else DiagnosticStatus.OK
            item.name, item.hardware_id, item.message = summary.name, summary.hardware_id, summary.message
            item.values = [KeyValue(key=value.key, value=value.value) for value in summary.values]
            msg.status = [item]
            self._diagnostic_pub.publish(msg)


        def _gripper_goal_callback(self, request):
            try:
                with self._execution.lock:
                    self._sim_access.run(lambda _sim: self._execution.gripper.reserve(object()))
                return GoalResponse.ACCEPT
            except RuntimeError:
                return GoalResponse.REJECT

        def _execute_gripper(self, goal, *, moveit=False):
            result = GripperCommand.Result()
            owner = self._execution.gripper.reservation
            task = None
            try:
                width = validate_gripper_width(goal.request.command.position*(2. if moveit else 1.))
                effort = float(goal.request.command.max_effort)
                validate_gripper_force(effort)
                with self._execution.lock:
                    task = self._sim_access.run(lambda sim: self._execution.gripper.start(owner, width, effort))
                while rclpy.ok():
                    with self._execution.lock:
                        if goal.is_cancel_requested:
                            self._sim_access.run(lambda sim: self._execution.gripper.cancel(task, 'gripper action canceled'))
                        state = self._sim_access.run(lambda sim: sim.get_state())
                        result.position = state.gripper_width/(2. if moveit else 1.)
                        result.reached_goal = task.state == 'succeeded'
                        goal.publish_feedback(GripperCommand.Feedback(
                            position=result.position, reached_goal=result.reached_goal))
                        if task.state in ('succeeded', 'failed'):
                            if result.reached_goal: goal.succeed()
                            elif goal.is_cancel_requested: goal.canceled()
                            else: goal.abort()
                            return result
                    time.sleep(.01)
                raise RuntimeError('simulation shutting down')
            except Exception as exc:
                self.get_logger().error(f'gripper execution failed: {exc}')
                with self._execution.lock:
                    if task is not None:
                        self._sim_access.run(lambda sim: self._execution.gripper.cancel(task, str(exc)))
                goal.abort()
                return result
            finally:
                with self._execution.lock:
                    self._execution.gripper.release_reservation(owner)

        def _gripper_service(self, request, response):
            try:
                width = validate_gripper_width(request.position)
                effort = float(request.max_effort)
                validate_gripper_force(effort)
                with self._execution.lock:
                    if self._execution.active.busy or self._execution.gripper.busy:
                        raise RuntimeError('gripper resources reserved')
                    task = self._sim_access.run(lambda sim: self._execution.gripper.start(object(), width, effort))
                while rclpy.ok():
                    with self._execution.lock:
                        if task.state not in ('active', 'releasing'):
                            response.success = task.state == 'succeeded'
                            break
                    time.sleep(.01)
                else:
                    response.success = False
            except (TypeError, ValueError, RuntimeError):
                response.success = False
            response.reached_position = self._sim_access.run(lambda sim: sim.get_state().gripper_width)
            return response


        def _mode_service(self, request, response):
            try:
                reached = self._execution.change_mode(normalize_ros_control_mode(request.mode))
            except (TypeError, ValueError, RuntimeError) as exc:
                response.success, response.message = False, f"mode rejected: {exc}"
                return response
            response.success, response.message = True, f"simulation mode: {reached}"
            return response

        def _impedance_service(self, request, response):
            """薄协议适配；资源检查、锁序与规划失效统一归执行组件。"""
            try:
                self._execution.command_joint_impedance(request.positions, request.stiffness,
                    request.damping, timeout_s=request.timeout_s)
            except (TypeError, ValueError, RuntimeError) as exc:
                response.success, response.message = False, f"joint impedance rejected: {exc}"
                return response
            response.success, response.message = True, "simulation mode: joint_impedance"
            return response

        @staticmethod
        def _terminate_goal(goal_handle, result, outcome):
            disposition = terminal_disposition(outcome, goal_handle.is_cancel_requested)
            if disposition == "canceled":
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "simulation trajectory canceled"
                return result
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            if outcome is GateOutcome.SERVICE_STOP:
                result.error_string = "simulation trajectory stopped by service"
            else:
                result.error_string = "simulation trajectory is no longer active"
            return result


        def _execute_goal(self, goal_handle, *, web=False):
            """协调传输与等待；曲线采样、到位、组合终态和清理由执行组件拥有。"""
            execution = self._execution
            goal_key = bytes(goal_handle.goal_id.uuid)
            with execution.lock:
                token = self._goal_tokens[goal_key]
            result = ExecuteTeleopMotion.Result() if web else FollowJointTrajectory.Result()
            grip = None
            completed = False
            sampler = execution.take_sampler()
            if sampler is None and self._trajectory_mode == 'legacy':
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                execution.cleanup(token)
                with execution.lock:
                    del self._goal_tokens[goal_key]
                return result
            try:
                if self._trajectory_mode == 'continuous':
                    if web:
                        goal_handle.publish_feedback(ExecuteTeleopMotion.Feedback(stage='preparing',
                            gripper_state='not_started', gripper_position=self._sim_access.run(
                                lambda sim: sim.get_state().gripper_width)))
                    sampler = self._prepare_sampler(goal_handle, token)
                    start_time, grip = execution.accepted_context(token)
                else:
                    start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    outcome, observation = execution.observe(token, sampler, start_time,
                        lambda: goal_handle.is_cancel_requested)
                    if outcome is not GateOutcome.APPLIED:
                        if self._trajectory_mode == 'continuous':
                            self._wait_prepared_stop(token)
                        return self._terminate_goal(goal_handle, result, outcome)
                    state, elapsed, desired = observation.state, observation.elapsed, observation.desired
                    settling = observation.settling
                    feedback = ExecuteTeleopMotion.Feedback() if web else FollowJointTrajectory.Feedback()
                    if web:
                        feedback.stage = 'executing'
                        feedback.arm_progress = min(1., elapsed/sampler.duration) if sampler.duration else 1.
                        feedback.gripper_state = observation.gripper_state
                        feedback.gripper_position = state.gripper_width
                        result.arm_state, result.gripper_state = observation.arm_state, observation.gripper_state
                    else:
                        feedback.joint_names = list(ARM_JOINT_NAMES)
                    feedback.desired.positions = list(desired)
                    elapsed_ns = round(min(sampler.duration, max(0., elapsed))*1e9)
                    feedback.desired.time_from_start.sec, feedback.desired.time_from_start.nanosec = divmod(elapsed_ns, 1_000_000_000)
                    actual = tuple(state.joint_positions[:6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:6])
                    if not web:
                        feedback.error.positions = [d-a for d,a in zip(desired,actual)]
                    if feedback_limiter.should_publish(time.monotonic(), final=settling in ('succeeded', 'timed_out')):
                        goal_handle.publish_feedback(feedback)
                    if settling == 'succeeded':
                        completion = execution.complete(token, sampler,
                            lambda: goal_handle.is_cancel_requested, goal_handle.succeed)
                        if completion is not GateOutcome.SUCCEEDED:
                            if self._trajectory_mode == 'continuous':
                                self._wait_prepared_stop(token)
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        completed = True
                        return result
                    if settling == 'timed_out':
                        execution.stop(token)
                        if self._trajectory_mode == 'continuous':
                            self._wait_prepared_stop(token)
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = "simulation goal did not settle within configured tolerance"
                        return result
                    time.sleep(self._execute_wait_sec)
                execution.stop(token)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation shutting down"
                return result
            except Exception as exc:
                self.get_logger().error(f"trajectory execution exception: {type(exc).__name__}: {exc}")
                if self._trajectory_mode == 'continuous':
                    execution.stop(token)
                    try:
                        self._wait_prepared_stop(token)
                    except RuntimeError as stop_error:
                        exc = RuntimeError(f'{exc}; {stop_error}')
                    if goal_handle.is_cancel_requested and not execution.status_snapshot().stop_unconfirmed:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                else:
                    execution.fail_legacy(token,
                        lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None)
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"simulation trajectory execution failed: {exc}"
                return result
            finally:
                if web:
                    if completed:
                        result.arm_state = 'succeeded'
                    elif result.arm_state != 'succeeded':
                        result.arm_state = 'canceled' if goal_handle.is_cancel_requested else 'failed'
                    result.gripper_state = (grip.state if grip else
                        'not_started' if goal_handle.request.move_gripper else 'not_requested')
                execution.cleanup(token)
                with execution.lock:
                    del self._goal_tokens[goal_key]

        def _timer_callback(self) -> None:
            """物理定时器串行推进仿真；一个定时回调含多步时也在每个物理步重采参考。"""
            state = self._execution.step(self._steps_per_tick, self._trace.record)
            stamp = Clock()
            seconds, nanoseconds = self._stamp.update(state.simulation_time)
            stamp.clock.sec = seconds
            stamp.clock.nanosec = nanoseconds
            self._clock_pub.publish(stamp)

            joint = JointState()
            joint.header.stamp = stamp.clock
            joint.name = list(state.joint_names)
            joint.position = list(state.joint_positions)
            joint.velocity = list(state.joint_velocities)
            joint.effort = list(state.actuator_forces)
            self._joint_pub.publish(joint)

            gripper = JointMotorState()
            gripper.header.stamp = stamp.clock
            gripper.joint_name = "gripper"
            gripper.position = float(state.gripper_width)
            gripper.velocity = 0.0
            gripper.torque = float(sum(abs(v) for v in state.actuator_forces[-2:]))
            gripper.status_code = 1
            self._gripper_pub.publish(gripper)
            status = ArmStatus()
            status.header.stamp = stamp.clock
            status.mode, status.enabled, status.control_loop_active = 'pos_vel', True, True
            execution_status = self._execution.status_snapshot()
            status.state_machine = execution_status.phase
            status.joint_names = list(ARM_JOINT_NAMES)
            status.per_joint_status_code = [1]*6
            status.error_codes = ['SIM_STOP_UNCONFIRMED'] if execution_status.stop_unconfirmed else []
            self._status_pub.publish(status)


        def destroy_node(self):
            """释放动作服务端与仿真实例。"""
            self._action_server.destroy()
            for server in self._gripper_servers:
                server.destroy()
            if hasattr(self,'_teleop_server'):
                self._teleop_server.destroy()
            self._trace.close()
            if not self._retain_simulation:
                self.close()
            return super().destroy_node()

    return RebotArmMujocoNode


def main(args=None) -> None:
    import importlib
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rebotarm_simulation.apps.viewer_lifecycle import (
        close_passive_viewer_safely, retain_unsafe_resources, stop_executor,
    )

    rclpy.init(args=args)
    node = create_node_class()()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    viewer = None
    executor_thread = None
    model = data = None
    worker_errors = []

    def spin_viewer_executor():
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except BaseException as exc:
            worker_errors.append(exc)

    try:
        if node.show_viewer:
            model, data = node.viewer_handles()
            viewer = importlib.import_module("mujoco.viewer").launch_passive(model, data)
            executor_thread = threading.Thread(target=spin_viewer_executor, daemon=True)
            executor_thread.start()
            while rclpy.ok() and viewer.is_running():
                if worker_errors:
                    raise RuntimeError("simulation executor failed") from worker_errors[0]
                if not executor_thread.is_alive():
                    raise RuntimeError("simulation executor stopped unexpectedly")
                node.sync_viewer(viewer)
                time.sleep(0.01)
        else:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            stop_executor(executor, executor_thread, multi_threaded=True)
        except BaseException:
            retain_unsafe_resources(viewer, node, model, data, executor, executor_thread)
            if rclpy.ok():
                rclpy.shutdown()
            raise
        try:
            close_passive_viewer_safely(viewer, node, model, data)
        except BaseException:
            node._retain_simulation = True
            raise
        finally:
            try:
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    if worker_errors:
        raise RuntimeError("simulation executor failed") from worker_errors[0]


if __name__ == "__main__":
    main()
