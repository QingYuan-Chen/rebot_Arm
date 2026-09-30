"""Safe ROS 2 adapter for the headless reBotArm MuJoCo simulation.

The validation helpers in this module intentionally have no ROS imports so
trajectory inputs can be fuzzed and unit tested on development hosts without
a ROS installation. ROS types are imported only when the node is constructed.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Sequence

from .trajectory_sampler import ARM_JOINT_NAMES, NamedTrajectoryPoint, TrajectorySampler


DEFAULT_MAX_TRAJECTORY_POINTS = 10_000
DEFAULT_MAX_TRAJECTORY_DURATION_SEC = 300.0
DEFAULT_HOME_JOINT_POSITIONS = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)
ROS_CONTROL_MODES = ("position", "hold", "gravity_comp")


def normalize_ros_control_mode(mode: Any) -> str:
    """Validate the deliberately small, non-torque ROS control surface."""
    value = str(mode).strip().lower()
    if value not in ROS_CONTROL_MODES:
        raise ValueError(
            "ROS simulation mode must be position, hold, or gravity_comp; "
            "raw_torque is available only through the local diagnostic API"
        )
    return value


class SimulationControlApi:
    """Compatibility boundary between ROS and the evolving simulation core."""

    def __init__(self, simulation: Any) -> None:
        self.simulation = simulation

    def reset_home_and_hold(self) -> None:
        self.simulation.reset_home()
        self.set_mode("hold")

    def set_mode(self, mode: str) -> str:
        public_mode = normalize_ros_control_mode(mode)
        if hasattr(self.simulation, "set_mode"):
            return str(self.simulation.set_mode(public_mode))
        legacy = "pos_vel" if public_mode == "position" else public_mode
        return str(self.simulation.set_control_mode(legacy))

    def command_joint_positions(self, values: Sequence[float]) -> tuple[float, ...]:
        if hasattr(self.simulation, "command_joint_positions"):
            reached = self.simulation.command_joint_positions(values)
        else:
            reached = self.simulation.set_joint_position_targets(values)
        self.set_mode("position")
        return tuple(float(value) for value in reached)

    def hold_current_position(self) -> None:
        if hasattr(self.simulation, "set_mode"):
            self.simulation.set_mode("hold")
            return
        current = tuple(self.simulation.get_state().joint_positions[:6])
        self.simulation.set_joint_position_targets(current)
        self.simulation.set_control_mode("hold")

    def command_gripper_width(
        self, width: float, max_force_n: float | None = None
    ) -> float:
        if hasattr(self.simulation, "command_gripper_width"):
            return float(
                self.simulation.command_gripper_width(
                    width, max_force_n=max_force_n
                )
            )
        return float(self.simulation.set_gripper_width(width))

    def get_control_status(self) -> Any:
        if hasattr(self.simulation, "get_control_status"):
            return self.simulation.get_control_status()
        return {
            "mode": "position" if self.simulation.control_mode == "pos_vel" else self.simulation.control_mode,
            "joint_targets": tuple(self.simulation.control_targets[:6]),
            "saturated": False,
            "watchdog_remaining_s": 0.0,
        }


@dataclass(frozen=True)
class GoalSettlingPolicy:
    position_tolerance: float = 0.02
    velocity_tolerance: float = 0.05
    time_tolerance_sec: float = 5.0

    def __post_init__(self) -> None:
        values = (self.position_tolerance, self.velocity_tolerance, self.time_tolerance_sec)
        if any(not math.isfinite(float(value)) or float(value) <= 0.0 for value in values):
            raise ValueError("goal tolerances must be positive finite values")

    def evaluate(self, desired, actual, velocities, settle_elapsed: float) -> str:
        vectors = (tuple(desired), tuple(actual), tuple(velocities))
        if any(len(vector) != len(ARM_JOINT_NAMES) for vector in vectors):
            raise ValueError("goal state must contain six arm values")
        numeric = tuple(tuple(float(value) for value in vector) for vector in vectors)
        elapsed = float(settle_elapsed)
        if any(not math.isfinite(value) for vector in numeric for value in vector) or not math.isfinite(elapsed):
            raise ValueError("goal state must be finite")
        if elapsed < 0.0:
            raise ValueError("settling time must be non-negative")
        position_error = max(abs(target - reached) for target, reached in zip(numeric[0], numeric[1]))
        max_velocity = max(abs(value) for value in numeric[2])
        if position_error <= self.position_tolerance and max_velocity <= self.velocity_tolerance:
            return "succeeded"
        if elapsed >= self.time_tolerance_sec:
            return "timed_out"
        return "settling"


def duration_to_seconds(duration: Any) -> float:
    try:
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("trajectory time is invalid") from exc
    if not math.isfinite(value):
        raise ValueError("trajectory time must be finite")
    return value


def trajectory_to_sampler(
    trajectory: Any,
    *,
    initial_positions: Sequence[float],
    max_points: int = DEFAULT_MAX_TRAJECTORY_POINTS,
    max_duration_sec: float = DEFAULT_MAX_TRAJECTORY_DURATION_SEC,
) -> TrajectorySampler:
    """Validate an untrusted JointTrajectory-like object and build a sampler."""
    if isinstance(max_points, bool) or int(max_points) <= 0:
        raise ValueError("max points must be positive")
    duration_cap = float(max_duration_sec)
    if not math.isfinite(duration_cap) or duration_cap <= 0.0:
        raise ValueError("max duration must be finite and positive")
    try:
        names = tuple(trajectory.joint_names)
        raw_points = tuple(trajectory.points)
    except (AttributeError, TypeError) as exc:
        raise ValueError("trajectory structure is invalid") from exc
    if len(raw_points) > int(max_points):
        raise ValueError("trajectory contains too many points")

    points = tuple(
        NamedTrajectoryPoint(duration_to_seconds(point.time_from_start), point.positions)
        for point in raw_points
    )
    # TrajectorySampler owns canonical-name, duplicate, length, finite-value,
    # non-empty, and strictly-increasing-time validation.
    sampler = TrajectorySampler(names, points, initial_positions=initial_positions)
    if sampler.duration > duration_cap:
        raise ValueError("trajectory duration exceeds configured limit")
    return sampler


def validate_gripper_width(width: Any) -> float:
    try:
        value = float(width)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper width must be finite") from exc
    if not math.isfinite(value):
        raise ValueError("gripper width must be finite")
    return value


def validate_gripper_force(max_effort: Any) -> float | None:
    try:
        value = float(max_effort)
    except (TypeError, ValueError) as exc:
        raise ValueError("gripper max effort must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("gripper max effort must be finite and non-negative")
    return None if value == 0.0 else value


def seconds_to_stamp_parts(simulation_time: Any) -> tuple[int, int]:
    try:
        value = float(simulation_time)
    except (TypeError, ValueError) as exc:
        raise ValueError("simulation time must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("simulation time must be finite and non-negative")
    seconds = math.floor(value)
    nanoseconds = int(round((value - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return int(seconds), nanoseconds


class MonotonicStamp:
    def __init__(self) -> None:
        self._nanoseconds = 0
        self._lock = threading.Lock()

    def update(self, simulation_time: Any) -> tuple[int, int]:
        seconds, nanoseconds = seconds_to_stamp_parts(simulation_time)
        candidate = seconds * 1_000_000_000 + nanoseconds
        with self._lock:
            self._nanoseconds = max(self._nanoseconds, candidate)
            return divmod(self._nanoseconds, 1_000_000_000)


class FeedbackRateLimiter:
    def __init__(self, rate_hz: Any) -> None:
        try:
            rate = float(rate_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError("feedback rate must be finite and in (0, 200]") from exc
        if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
            raise ValueError("feedback rate must be finite and in (0, 200]")
        self.rate_hz = rate
        self._period = 1.0 / rate
        self._last_publish: float | None = None

    def should_publish(self, monotonic_time: Any, *, final: bool = False) -> bool:
        now = float(monotonic_time)
        if not math.isfinite(now):
            raise ValueError("feedback clock must be finite")
        due = self._last_publish is None or now - self._last_publish >= self._period
        if final or due:
            self._last_publish = now
            return True
        return False


class ActiveTrajectory:
    """Thread-safe single-goal admission and cooperative cancellation gate."""

    def __init__(self, lock: threading.RLock | None = None) -> None:
        self._lock = lock or threading.RLock()
        self._token: object | None = None
        self._cancel_requested = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._token is not None

    @property
    def cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    @property
    def token(self) -> object | None:
        with self._lock:
            return self._token

    def try_start(self, token: object) -> bool:
        with self._lock:
            if self._token is not None:
                return False
            self._token = token
            self._cancel_requested = False
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._token is None:
                return False
            self._cancel_requested = True
            return True

    def finish(self, token: object) -> None:
        with self._lock:
            if self._token is token:
                self._token = None
                self._cancel_requested = False


class GateOutcome(Enum):
    APPLIED = auto()
    ACTION_CANCEL = auto()
    SERVICE_STOP = auto()
    INACTIVE = auto()
    SUCCEEDED = auto()


def terminal_disposition(outcome: GateOutcome, action_cancel_requested: bool) -> str:
    if outcome is GateOutcome.ACTION_CANCEL and bool(action_cancel_requested):
        return "canceled"
    return "aborted"


class TrajectoryCommandGate:
    """Atomically arbitrate trajectory commands and stop/hold operations."""

    def __init__(self, active: ActiveTrajectory) -> None:
        self._active = active

    def apply_if_active(self, token, cancel_requested, apply, hold) -> bool:
        return self.apply_with_reason(token, cancel_requested, apply, hold) is GateOutcome.APPLIED

    def apply_with_reason(self, token, action_cancel_requested, apply, hold) -> GateOutcome:
        with self._active._lock:
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            apply()
            return GateOutcome.APPLIED

    def stop_and_hold(self, hold) -> bool:
        with self._active._lock:
            stopped = self._active._token is not None
            if stopped:
                self._active._cancel_requested = True
                hold()
            return stopped

    def complete_if_active(self, token, cancel_requested, hold, succeed) -> bool:
        return self.complete_with_reason(
            token, cancel_requested, hold, succeed
        ) is GateOutcome.SUCCEEDED

    def complete_with_reason(self, token, action_cancel_requested, hold, succeed) -> GateOutcome:
        """Linearize cancellation versus the terminal success transition."""
        with self._active._lock:
            # Evaluate the external hook first, then re-read internal state: a
            # cancel callback may mark the goal through a reentrant test hook
            # or immediately before this critical section.
            action_cancel = bool(action_cancel_requested())
            if self._active._token is not token:
                return GateOutcome.INACTIVE
            if action_cancel:
                self._active._cancel_requested = True
                hold()
                return GateOutcome.ACTION_CANCEL
            if self._active._cancel_requested:
                hold()
                return GateOutcome.SERVICE_STOP
            # No simulation lock is held while calling the ROS transition.
            succeed()
            self._active._token = None
            self._active._cancel_requested = False
            return GateOutcome.SUCCEEDED


class ExecutionLifecycle:
    """Best-effort failure cleanup that never leaks an admitted goal token."""

    def __init__(self, active: ActiveTrajectory, gate: TrajectoryCommandGate) -> None:
        self._active = active
        self._gate = gate

    def fail(self, token, hold, abort) -> None:
        try:
            self._gate.stop_and_hold(hold)
        except Exception:
            pass
        try:
            abort()
        finally:
            self._active.finish(token)


class SerializedSimulationAccess:
    """Serialize every operation touching one simulation instance."""

    def __init__(self, simulation: Any, lock: threading.RLock | None = None) -> None:
        self._simulation = simulation
        self._lock = lock or threading.RLock()

    def run(self, operation):
        with self._lock:
            return operation(self._simulation)


def create_node_class():
    """Import ROS lazily and return the concrete node class."""
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rebotarm_msgs.msg import JointMotorState
    from rebotarm_msgs.srv import SetGripper, SetMode
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint

    from .mujoco_sim import RebotArmMujoco
    from .ros_diagnostics import build_control_diagnostic

    class RebotArmMujocoNode(Node):
        def __init__(self) -> None:
            super().__init__("rebotarm_mujoco_node")
            self.declare_parameter("backend", "mujoco")
            self.declare_parameter("headless", True)
            self.declare_parameter("show_viewer", False)
            self.declare_parameter("model_path", "")
            self.declare_parameter("arm_namespace", "rebotarm")
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
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
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
            return self._sim_access.run(lambda sim: sim._unsafe_viewer_handles())

        def sync_viewer(self, viewer) -> None:
            self._sim_access.run(lambda _sim: viewer.sync())

        def _hold_current_position(self) -> None:
            self._sim_access.run(lambda _sim: self._hold_current_position_unlocked())

        def _hold_current_position_unlocked(self) -> None:
            self._control.hold_current_position()

        def _apply_target_threadsafe(self, operation) -> None:
            self._sim_access.run(lambda _sim: operation())

        def _goal_callback(self, goal_request):
            token = goal_request
            try:
                sampler = trajectory_to_sampler(
                    goal_request.trajectory,
                    initial_positions=self._current_arm_positions(),
                    max_points=self._max_points,
                    max_duration_sec=self._max_duration,
                )
            except (TypeError, ValueError):
                self.get_logger().warning("rejected invalid trajectory goal")
                return GoalResponse.REJECT
            if not self._active.try_start(token):
                self.get_logger().warning("rejected trajectory goal while controller is busy")
                return GoalResponse.REJECT
            self._pending_sampler = sampler
            return GoalResponse.ACCEPT

        def _cancel_callback(self, _goal_handle):
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            if not stopped:
                self._hold_current_position()
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
                reached = self._sim_access.run(
                    lambda _sim: self._control.command_gripper_width(
                        width, max_force_n=max_force
                    )
                )
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
            if self._active.busy:
                response.success = False
                response.message = "trajectory active; stop or cancel it before changing mode"
                return response
            try:
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
            # rclpy does not promise that the goal-request wrapper passed to
            # admission and the one exposed by the goal handle are identical.
            # Preserve the admission token so cleanup cannot leave the server
            # permanently busy after an otherwise valid goal.
            token = self._active.token
            result = FollowJointTrajectory.Result()
            sampler, self._pending_sampler = self._pending_sampler, None
            if sampler is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                self._active.finish(token)
                return result
            try:
                start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    command: dict[str, Any] = {}

                    def apply_target() -> None:
                        state = self._sim.get_state()
                        elapsed = max(0.0, state.simulation_time - start_time)
                        desired = sampler.sample(min(elapsed, sampler.duration))
                        self._control.command_joint_positions(desired)
                        command.update(state=state, elapsed=elapsed, desired=desired)

                    outcome = self._command_gate.apply_with_reason(
                        token,
                        lambda: goal_handle.is_cancel_requested,
                        lambda: self._apply_target_threadsafe(apply_target),
                        self._hold_current_position,
                    )
                    if outcome is not GateOutcome.APPLIED:
                        return self._terminate_goal(goal_handle, result, outcome)
                    state = command["state"]
                    elapsed = command["elapsed"]
                    desired = command["desired"]
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.joint_names = list(ARM_JOINT_NAMES)
                    feedback.desired = JointTrajectoryPoint()
                    feedback.actual = JointTrajectoryPoint()
                    feedback.error = JointTrajectoryPoint()
                    feedback.desired.positions = list(desired)
                    actual = tuple(state.joint_positions[:6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:6])
                    feedback.error.positions = [d - a for d, a in zip(desired, actual)]
                    settling = self._settling_policy.evaluate(
                        sampler.sample(sampler.duration),
                        actual,
                        tuple(state.joint_velocities[:6]),
                        max(0.0, elapsed - sampler.duration),
                    ) if elapsed >= sampler.duration else "tracking"
                    if feedback_limiter.should_publish(
                        time.monotonic(), final=settling in ("succeeded", "timed_out")
                    ):
                        goal_handle.publish_feedback(feedback)
                    if settling == "succeeded":
                        completion = self._command_gate.complete_with_reason(
                            token,
                            lambda: goal_handle.is_cancel_requested,
                            self._hold_current_position,
                            lambda: (self._hold_current_position(), goal_handle.succeed()),
                        )
                        if completion is not GateOutcome.SUCCEEDED:
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        return result
                    if settling == "timed_out":
                        self._command_gate.stop_and_hold(self._hold_current_position)
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
                self._lifecycle.fail(
                    token,
                    self._hold_current_position,
                    lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None,
                )
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation trajectory execution failed"
                return result
            finally:
                self._active.finish(token)

        def _timer_callback(self) -> None:
            state = self._sim_access.run(lambda sim: sim.step(self._steps_per_tick))
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
            gripper.status_code = 0
            self._gripper_pub.publish(gripper)


        def destroy_node(self):
            """释放动作服务端与仿真实例。"""
            self._action_server.destroy()
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
