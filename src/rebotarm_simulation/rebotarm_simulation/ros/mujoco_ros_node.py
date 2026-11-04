"""MuJoCo 后端的 ROS 执行、物理步进及停止保持适配。

文件职责
    提供仿真 FJT、夹爪和控制模式服务，驱动物理定时器，并发布实际状态与可选验收记录。
对外接口
    create_node_class 延迟加载 ROS 并返回节点类；main 创建执行器。
    trajectory_execution_mode 选择 legacy 或 prepared_v1；准备服务只返回检查结果，
    仿真节点自己拥有目标令牌、资源占用、物理时钟和原子接纳。
安全约束
    不导入真实硬件后端或运动层实现；参考求值来自共享数值包。
    锁序固定为活动目标锁 → 仿真锁；准备服务等待、停止等待均不能占住物理步进锁。
    prepared_v1 用仿真时间采样，用主机单调时间控制准备期限和反馈新鲜度。
    固定夹爪或场景变化后停止保持；停止未确认继续占用资源，不能因 Action 退出而放行。
    正常成功保持曲线终点；Action 成功条件与独立物理验收指标分开。
文件结构
    延迟创建节点 → 参数/仿真/ROS 装配 → Goal 预约与准备 → 停止及控制服务
    → Action 终态协调 → 物理周期与状态发布 → 关闭和进程入口。
"""

from __future__ import annotations

import math
import json
from collections import deque
import threading
import time
from typing import Any, Sequence

from rebotarm_simulation.ros.message_codec import DEFAULT_MAX_TRAJECTORY_DURATION_SEC
from rebotarm_simulation.ros.message_codec import DEFAULT_MAX_TRAJECTORY_POINTS
from rebotarm_simulation.execution.trajectory_state import ActiveTrajectory
from rebotarm_simulation.execution.trajectory_state import ExecutionLifecycle
from rebotarm_simulation.execution.feedback_timing import FeedbackRateLimiter
from rebotarm_simulation.execution.trajectory_state import GateOutcome
from rebotarm_simulation.execution.goal_policy import GoalSettlingPolicy
from rebotarm_simulation.ros.message_codec import MonotonicStamp
from rebotarm_simulation.execution.simulation_access import SerializedSimulationAccess
from rebotarm_simulation.execution.trajectory_state import TrajectoryCommandGate
from rebotarm_simulation.ros.message_codec import duration_to_seconds
from rebotarm_simulation.ros.message_codec import seconds_to_stamp_parts
from rebotarm_simulation.execution.trajectory_state import terminal_disposition
from rebotarm_simulation.ros.message_codec import trajectory_to_sampler
from rebotarm_simulation.ros.message_codec import validate_gripper_width
from rebotarm_simulation.execution.trajectory_sampler import ARM_JOINT_NAMES, NamedTrajectoryPoint, TrajectorySampler
from rebotarm_simulation.execution.trajectory_sampler import PreparedTrajectorySampler
from rebotarm_trajectory.polynomial import curve_from_payload, TrajectoryPoint, ordered_points
from rebotarm_trajectory.joint_contract import canonical_input
from rebotarm_trajectory.capability import make_capability, constraint_revision
from pathlib import Path


from rebotarm_simulation.execution.control_api import SimulationControlApi, normalize_ros_control_mode
from rebotarm_simulation.ros.message_codec import validate_gripper_force

DEFAULT_HOME_JOINT_POSITIONS = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)

def create_node_class():
    """延迟导入 ROS 并返回具体节点类；仅导入本模块不会创建节点或仿真实例。"""
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rebotarm_msgs.msg import JointMotorState, ArmStatus
    from rebotarm_msgs.srv import SetGripper, SetMode
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
            if self._trajectory_mode not in ('legacy', 'prepared_v1'):
                raise ValueError('trajectory_execution_mode must be legacy or prepared_v1')
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
            self._settling_policy = GoalSettlingPolicy(
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
            self._lock = threading.RLock()
            self._sim_access = SerializedSimulationAccess(self._sim, self._lock)
            def initialize(_sim) -> None:
                self._control.reset_home_and_hold()
                if initial != DEFAULT_HOME_JOINT_POSITIONS:
                    self._control.command_joint_positions(initial)

            self._sim_access.run(initialize)
            # Gate/active lock is intentionally distinct. Lock order is always
            # gate first, then simulation; the timer only takes simulation.
            self._active = ActiveTrajectory()
            self._command_gate = TrajectoryCommandGate(self._active)
            self._lifecycle = ExecutionLifecycle(self._active, self._command_gate)
            self._pending_sampler: TrajectorySampler | None = None
            self._callback_group = ReentrantCallbackGroup()
            self._scene_revision = ''
            # 待准备请求与已接纳曲线分开保存，服务返回之前物理周期仍保持当前位置。
            self._preparation = None
            self._coordinated_active = False
            self._sample_history = deque(maxlen=512)
            self._last_physics_wall = time.monotonic()
            self._stop_unconfirmed = False
            self._retained_stop_token = None
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
            self._execution_capability = make_capability(self._trajectory_mode, secrets.randbits(63),
                float(self.get_parameter('gripper_position_max_speed_rad_s').value),
                float(self.get_parameter('gripper_position_torque_cap_nm').value), revision)
            self._versioned_capabilities = self.create_publisher(String,
                f'/{namespace}/trajectory_capabilities', qos)
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
            self._capability_timer = self.create_timer(.1, self._publish_admission, clock=self._physics_clock)
            if self._trajectory_mode == 'prepared_v1':
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
                cancel_callback=self._cancel_callback,
                execute_callback=self._execute_goal,
                callback_group=self._callback_group,
            )
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

        def _hold_current_position(self) -> None:
            self._sim_access.run(lambda _sim: self._hold_current_position_unlocked())

        def _hold_current_position_unlocked(self) -> None:
            self._control.hold_current_position()
            if self._coordinated_active:
                self._sim.release_gripper_position()
                self._coordinated_active = False

        def _apply_target_threadsafe(self, operation) -> None:
            self._sim_access.run(lambda _sim: operation())

        def _goal_callback(self, goal_request):
            token = goal_request
            try:
                if self._trajectory_mode == 'prepared_v1':
                    return self._reserve_prepared_goal(goal_request)
                sampler = trajectory_to_sampler(
                    goal_request.trajectory,
                    initial_positions=self._current_arm_positions(),
                    max_points=self._max_points,
                    max_duration_sec=self._max_duration,
                )
            except (TypeError, ValueError, RuntimeError) as exc:
                self.get_logger().warning(f"FJT goal rejected: {exc}")
                return GoalResponse.REJECT
            if not self._active.try_start(token):
                self.get_logger().warning("rejected trajectory goal while controller is busy")
                return GoalResponse.REJECT
            self._pending_sampler = sampler
            return GoalResponse.ACCEPT

        def _stationary_snapshot(self, *, after=None):
            """调用方持仿真锁，核验六轴及两指的完整静止窗口并返回实际状态和保持命令。

            物理窗口按仿真时间覆盖 0.2 s，墙上单调时钟另查物理循环是否停滞；
            after 用于要求停止发起之后的全新窗口，重复发布旧状态不能满足条件。
            """
            state = self._sim.get_state()
            cutoff = state.simulation_time-.2
            samples = [item for item in self._sample_history if item.simulation_time >= cutoff-.0021]
            if (time.monotonic()-self._last_physics_wall > .15 or len(samples) < 3
                    or samples[-1].simulation_time-samples[0].simulation_time < .2-1e-9
                    or (after is not None and samples[0].simulation_time <= after)):
                raise RuntimeError('fresh complete stationary window required')
            speeds = (.05,)*6+(.00045,)*2
            for index, speed in enumerate(speeds):
                if any(abs(item.joint_velocities[index]) > speed for item in samples):
                    raise RuntimeError(f'{state.joint_names[index]} stationary evidence:'
                        f' speed={max(abs(item.joint_velocities[index]) for item in samples):.9f}'
                        f' > {speed:.9f}')
            hold = tuple(self._sim.get_control_status().joint_targets)
            # 静止窗口比较实际反馈；实际位置与命令保持值的残差不单独阻断下一次请求。
            return state, hold

        def _publish_admission(self):
            """发布只读准入证据；物理状态和能力不代表任何一次 REJECT 的准确原因。"""
            self._versioned_capabilities.publish(String(data=json.dumps(self._execution_capability, allow_nan=False)))
            reasons = []
            with self._active._lock:
                if self._active.busy or self._stop_unconfirmed:
                    reasons.append('arm/gripper resources reserved or stop unconfirmed')
                with self._lock:
                    try:
                        self._stationary_snapshot()
                    except RuntimeError as exc:
                        reasons.append(str(exc))
                    if self._sim.control_mode not in ('position','hold'):
                        reasons.append(f'mode={self._sim.control_mode}, required=position/hold')
                if self._trajectory_mode == 'prepared_v1':
                    if not self._scene_revision:
                        reasons.append('scene revision unavailable')
                    if not self._prepare_client.service_is_ready():
                        reasons.append('preparation service unavailable')
            message = DiagnosticArray()
            message.header.stamp = self.get_clock().now().to_msg()
            message.status = [DiagnosticStatus(name=f'{self._arm_namespace}/trajectory_admission',hardware_id='simulation',
                level=DiagnosticStatus.WARN if reasons else DiagnosticStatus.OK,
                message='; '.join(reasons) if reasons else 'ready',
                values=[KeyValue(key='scene_revision',value=self._scene_revision)])]
            self._diagnostic_pub.publish(message)

        def _reserve_prepared_goal(self, goal):
            """先校验输入，再按固定锁序预约空闲资源；忙时拒绝，不累积键盘或网页目标。"""
            received = time.monotonic()
            if not self._prepare_client.service_is_ready():
                raise RuntimeError('preparation service unavailable')
            if not self._scene_revision:
                raise RuntimeError('scene revision unavailable')
            points = tuple(TrajectoryPoint(duration_to_seconds(p.time_from_start), p.positions,
                p.velocities, p.accelerations) for p in goal.trajectory.points)
            canonical_input(goal.trajectory.joint_names, points, self._current_arm_positions())
            if not 0 < len(points) <= self._max_points or points[-1].time > self._max_duration:
                raise ValueError('trajectory point or duration budget exceeded')
            with self._active._lock:
                if self._active.busy:
                    raise RuntimeError('arm/gripper resources reserved')
                with self._lock:
                    state, hold = self._stationary_snapshot()
                    if self._sim.control_mode not in ('position', 'hold'):
                        raise RuntimeError(f'mode={self._sim.control_mode}, required=position/hold')
                    if not self._active.try_start(goal):
                        return GoalResponse.REJECT
                    request = self._prepare_type.Request()
                    request.trajectory = goal.trajectory
                    # 本进程保留 goal 对象作为身份；仿真无设备重连会话，仍须逐项核对回传值。
                    request.session = request.generation = id(goal)
                    request.hold_positions, request.feedback_positions = list(hold), list(state.joint_positions[:6])
                    request.feedback_observed_at = self._last_physics_wall
                    request.stationary = request.enabled = True
                    request.gripper_positions = list(state.joint_positions[-2:])
                    request.gripper_generation = 0
                    request.scene_revision = self._scene_revision
                    request.capability_json = json.dumps(self._execution_capability, allow_nan=False)
                    request.deadline_monotonic = received+min(2., .2*points[-1].time+.25)
                    self._preparation = (goal, request)
                    self._pending_sampler = None
                    self._stop_unconfirmed = False
            return GoalResponse.ACCEPT

        def _scene_changed(self, message):
            """收到不同碰撞版本时使准备/执行失效；由命令闸门接管保持，不重算活动曲线。"""
            with self._active._lock:
                self._scene_revision = message.data
                expected = (self._preparation[1].scene_revision if self._preparation is not None
                            else self._active.prepared['revision'] if self._active.prepared else None)
                if expected is not None and expected != message.data:
                    self._command_gate.stop_and_hold(self._hold_current_position)

        def _prepare_sampler(self, goal_handle, token):
            """Action 工作线程等待服务并解码，最终由命令闸门原子核验身份和仿真快照。"""
            pending = self._preparation
            if pending is None or pending[0] is not token:
                raise RuntimeError('preparation ownership lost')
            request = pending[1]
            self.get_logger().info(f'trajectory preparation dispatch age={time.monotonic()-request.feedback_observed_at:.4f}s')
            future = self._prepare_client.call_async(request)
            while not future.done():
                if (goal_handle.is_cancel_requested or self._active.cancel_requested
                        or time.monotonic() >= request.deadline_monotonic):
                    future.cancel()
                    raise RuntimeError('trajectory preparation canceled or timed out')
                time.sleep(.002)
            response = future.result()
            if not response.success:
                raise ValueError(response.message)
            payload = json.loads(response.prepared_json)
            curve = curve_from_payload(payload['curve'])
            def admit():
                """在目标锁内再取得仿真锁，复核期间不能插入物理状态或控制模式修改。"""
                with self._lock:
                    state, hold = self._stationary_snapshot()
                    context = payload['context']
                    if (self._sim.control_mode not in ('position', 'hold')
                            or time.monotonic() >= request.deadline_monotonic
                            or self._scene_revision != response.scene_revision
                            or payload['scene_revision'] != request.scene_revision
                            or context['session'] != request.session or context['generation'] != request.generation
                            or tuple(context['hold']) != tuple(request.hold_positions)
                            or hold != tuple(request.hold_positions) or curve.points[0].positions[:6] != hold
                            or payload['gripper_generation'] != request.gripper_generation
                            or tuple(payload['gripper_positions']) != tuple(request.gripper_positions)):
                        raise RuntimeError('prepared simulation context changed')
                    if len(curve.joint_names) == 7 and (
                            payload.get('capability_revision') != self._execution_capability['revision']
                            or abs(curve.points[0].positions[6]-request.gripper_positions[0]) > 1e-9):
                        raise RuntimeError('prepared coordinated capability or initial finger changed')
                    # 执行起时在接纳时捕获；不能用准备开始时间让曲线跳过起始运动。
                    self._active.prepared = dict(curve=curve, start=state.simulation_time,
                        revision=response.scene_revision, fingers=tuple(request.gripper_positions))
                    self._coordinated_active = len(curve.joint_names) == 7
                    self._preparation = None
            outcome = self._command_gate.apply_with_reason(token,
                lambda: goal_handle.is_cancel_requested, admit, self._hold_current_position)
            if outcome is not GateOutcome.APPLIED:
                raise RuntimeError('prepared trajectory canceled before atomic admission')
            return PreparedTrajectorySampler(curve)

        def _wait_prepared_stop(self):
            """取消不是连续轨迹保证；资源仍占用至新物理帧确认静止。"""
            after = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
            deadline = time.monotonic()+5.
            # 每轮只短暂读取物理状态，等待在锁外进行，物理定时器才能产生新的静止证据。
            while time.monotonic() < deadline:
                try:
                    self._sim_access.run(lambda _sim: self._stationary_snapshot(after=after))
                    self._stop_unconfirmed = False
                    return
                except RuntimeError:
                    time.sleep(.01)
            self._stop_unconfirmed = True
            raise RuntimeError('simulation stop/hold unconfirmed')

        def _finish_prepared(self, sampler):
            """末点已由物理周期送出；原命令终点保持，协同夹爪中性释放后才完成。"""
            def finish(sim):
                self._control.command_joint_positions(sampler.sample(sampler.duration)[:6])
                if len(sampler.curve.joint_names) == 7:
                    sim.release_gripper_position()
                    self._coordinated_active = False
            self._sim_access.run(finish)

        def _cancel_callback(self, _goal_handle):
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            token = self._active.token
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            if not stopped:
                self._hold_current_position()
            if self._trajectory_mode == 'prepared_v1' and stopped:
                try:
                    self._wait_prepared_stop()
                    with self._active._lock:
                        if self._retained_stop_token is token:
                            self._preparation = None
                            self._active.finish(token)
                            self._retained_stop_token = None
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

        def _gripper_service(self, request, response):
            try:
                width = validate_gripper_width(request.position)
                max_force = validate_gripper_force(request.max_effort)
                with self._active._lock:
                    # 从准备到停止确认都固定夹爪；不能只在“正在采样曲线”时才拒绝夹爪运动。
                    if self._trajectory_mode == 'prepared_v1' and self._active.busy:
                        raise ValueError('fixed gripper reserved by arm trajectory')
                    reached = self._sim_access.run(lambda _sim:
                        self._control.command_gripper_width(width, max_force_n=max_force))
            except (TypeError, ValueError):
                response.success = False
                response.reached_position = 0.0
                return response
            response.success = True
            response.reached_position = float(reached)
            return response

        def _mode_service(self, request, response):
            try:
                mode = normalize_ros_control_mode(request.mode)
            except ValueError as exc:
                response.success = False
                response.message = str(exc)
                return response
            try:
                # 与准备占用使用同一锁，避免检查空闲后被新 Goal 抢入再切换模式。
                with self._active._lock:
                    if self._active.busy:
                        response.success = False
                        response.message = "trajectory active; stop or cancel it before changing mode"
                        return response
                    reached = self._sim_access.run(lambda _sim: self._control.set_mode(mode))
            except (TypeError, ValueError) as exc:
                response.success = False
                response.message = f"mode rejected: {exc}"
                return response
            response.success = True
            response.message = f"simulation mode: {reached}"
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

        def _execute_goal(self, goal_handle):
            """协调反馈和终态；prepared_v1 的实际目标采样由物理定时器唯一推进。"""
            # rclpy does not promise that the goal-request wrapper passed to
            # admission and the one exposed by the goal handle are identical.
            # Preserve the admission token so cleanup cannot leave the server
            # permanently busy after an otherwise valid goal.
            token = self._active.token
            result = FollowJointTrajectory.Result()
            sampler, self._pending_sampler = self._pending_sampler, None
            if sampler is None and self._trajectory_mode == 'legacy':
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                self._active.finish(token)
                return result
            try:
                if self._trajectory_mode == 'prepared_v1':
                    sampler = self._prepare_sampler(goal_handle, token)
                    start_time = self._active.prepared['start']
                else:
                    start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    command: dict[str, Any] = {}

                    def apply_target() -> None:
                        state = self._sim.get_state()
                        elapsed = max(0.0, state.simulation_time - start_time)
                        desired = sampler.sample(min(elapsed, sampler.duration))
                        # prepared 路径这里只构造反馈，避免 Action 线程另行写目标造成双重推进。
                        if self._trajectory_mode == 'legacy':
                            self._control.command_joint_positions(desired)
                        command.update(state=state, elapsed=elapsed, desired=desired)

                    outcome = self._command_gate.apply_with_reason(
                        token,
                        lambda: goal_handle.is_cancel_requested,
                        lambda: self._apply_target_threadsafe(apply_target),
                        self._hold_current_position,
                    )
                    if outcome is not GateOutcome.APPLIED:
                        if self._trajectory_mode == 'prepared_v1':
                            self._wait_prepared_stop()
                        return self._terminate_goal(goal_handle, result, outcome)
                    state = command["state"]
                    elapsed = command["elapsed"]
                    desired = command["desired"]
                    feedback = FollowJointTrajectory.Feedback()
                    coordinated = isinstance(sampler, PreparedTrajectorySampler) and len(sampler.curve.joint_names) == 7
                    feedback.joint_names = list(sampler.curve.joint_names if isinstance(sampler, PreparedTrajectorySampler) else ARM_JOINT_NAMES)
                    feedback.desired = JointTrajectoryPoint()
                    feedback.actual = JointTrajectoryPoint()
                    feedback.error = JointTrajectoryPoint()
                    feedback.desired.positions = list(desired)
                    elapsed_ns = round(min(sampler.duration,max(0.,elapsed))*1e9)
                    feedback.desired.time_from_start.sec, feedback.desired.time_from_start.nanosec = divmod(elapsed_ns,1_000_000_000)
                    actual = tuple(state.joint_positions[:7 if coordinated else 6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:len(actual)])
                    feedback.error.positions = [d - a for d, a in zip(desired, actual)]
                    settling = self._settling_policy.evaluate(
                        sampler.sample(sampler.duration)[:6],
                        actual[:6],
                        tuple(state.joint_velocities[:6]),
                        max(0.0, elapsed - sampler.duration),
                    ) if elapsed >= sampler.duration else "tracking"
                    if coordinated and settling == 'succeeded':
                        if abs(actual[-1]-sampler.sample(sampler.duration)[-1])/.009 > .12:
                            settling = ('timed_out' if elapsed-sampler.duration >= self._settling_policy.time_tolerance_sec else 'settling')
                    if self._trajectory_mode == 'prepared_v1' and settling == 'succeeded':
                        active = self._active.prepared
                        if active is None or active.get('sent_at') is None or state.simulation_time <= active['sent_at']:
                            settling = 'settling'
                    if feedback_limiter.should_publish(
                        time.monotonic(), final=settling in ("succeeded", "timed_out")
                    ):
                        goal_handle.publish_feedback(feedback)
                    if settling == "succeeded":
                        # prepared 正常结束再明确写入最终命令；不能捕获瞬时实际位置代替它。
                        completion = self._command_gate.complete_with_reason(
                            token,
                            lambda: goal_handle.is_cancel_requested,
                            self._hold_current_position,
                            lambda: (self._finish_prepared(sampler)
                                if self._trajectory_mode == 'prepared_v1' else self._hold_current_position(),
                                goal_handle.succeed()),
                        )
                        if completion is not GateOutcome.SUCCEEDED:
                            if self._trajectory_mode == 'prepared_v1':
                                self._wait_prepared_stop()
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        return result
                    if settling == "timed_out":
                        self._command_gate.stop_and_hold(self._hold_current_position)
                        if self._trajectory_mode == 'prepared_v1':
                            self._wait_prepared_stop()
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = "simulation goal did not settle within configured tolerance"
                        return result
                    time.sleep(self._execute_wait_sec)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation shutting down"
                self._hold_current_position()
                return result
            except Exception as exc:
                self.get_logger().error(
                    f"trajectory execution exception: {type(exc).__name__}: {exc}"
                )
                if self._trajectory_mode == 'prepared_v1':
                    self._command_gate.stop_and_hold(self._hold_current_position)
                    try:
                        self._wait_prepared_stop()
                    except RuntimeError as stop_error:
                        exc = RuntimeError(f'{exc}; {stop_error}')
                    if goal_handle.is_cancel_requested and not self._stop_unconfirmed:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                else:
                    self._lifecycle.fail(token, self._hold_current_position,
                        lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None)
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = f"simulation trajectory execution failed: {exc}"
                return result
            finally:
                # 停止证据不足时保留 token；只有后续显式停止确认才能恢复资源可用。
                if self._stop_unconfirmed:
                    self._retained_stop_token = token
                if not self._stop_unconfirmed:
                    with self._active._lock:
                        if self._preparation is not None and self._preparation[0] is token:
                            self._preparation = None
                        self._active.finish(token)

        def _timer_callback(self) -> None:
            """物理定时器串行推进仿真；一个定时回调含多步时也在每个物理步重采参考。"""
            if self._trajectory_mode == 'legacy':
                state = self._sim_access.run(lambda sim: sim.step(self._steps_per_tick))
            else:
                # 锁序固定为目标闸门 -> 仿真；每个物理步按当前仿真时间采样，不积压旧目标。
                with self._active._lock:
                    def step(sim):
                        for _ in range(self._steps_per_tick):
                            active = self._active.prepared
                            if active is not None and not self._active.cancel_requested:
                                current = sim.get_state()
                                curve = active['curve']
                                elapsed = current.simulation_time-active['start']
                                target = curve.sample(elapsed)[0]
                                coordinated = len(curve.joint_names) == 7
                                invalid = (max(abs(a-b) for a,b in zip(current.joint_positions[:6],target[:6])) > .03
                                    or (coordinated and abs(current.joint_positions[6]-target[6])/.009 > .12))
                                if invalid:
                                    self._command_gate.stop_and_hold(self._hold_current_position)
                                else:
                                    self._control.command_joint_positions(target[:6])
                                    if coordinated:
                                        sim.command_coordinated_gripper(target[6],self._execution_capability['gripper_torque_cap_nm'])
                                    if elapsed >= curve.duration:
                                        active.setdefault('sent_at',current.simulation_time)
                            state = sim.step()
                            # 记录与静止窗口都使用真正推进后的物理帧，而不是 ROS 重复发布频率。
                            self._trace.record(state, sim.get_control_status(), active)
                            self._sample_history.append(state)
                        self._last_physics_wall = time.monotonic()
                        return state
                    state = self._sim_access.run(step)
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
            status.state_machine = ('TRAJ_PREPARING' if self._preparation else
                'TRAJ_RUNNING' if self._active.busy else 'IDLE')
            status.joint_names = list(ARM_JOINT_NAMES)
            status.per_joint_status_code = [1]*6
            status.error_codes = ['SIM_STOP_UNCONFIRMED'] if self._stop_unconfirmed else []
            self._status_pub.publish(status)


        def destroy_node(self):
            """释放动作服务端与仿真实例。"""
            self._action_server.destroy()
            self._trace.close()
            self._sim_access.run(lambda sim: sim.close())
            return super().destroy_node()

    return RebotArmMujocoNode


def main(args=None) -> None:
    import importlib
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = create_node_class()()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    viewer = None
    executor_thread = None
    try:
        if node.show_viewer:
            model, data = node.viewer_handles()
            viewer = importlib.import_module("mujoco.viewer").launch_passive(model, data)
            executor_thread = threading.Thread(target=executor.spin, daemon=True)
            executor_thread.start()
            while rclpy.ok() and viewer.is_running():
                node.sync_viewer(viewer)
                time.sleep(0.01)
            return
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if viewer is not None:
            viewer.close()
        executor.shutdown()
        if executor_thread is not None:
            executor_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
