"""无头 reBotArm MuJoCo 仿真的 ROS 2 安全适配节点。

模块职责：把 MuJoCo 物理仿真包装成符合 ROS 2 约定的仿真控制器后端，向上层提供
与真实控制器一致的接口，使运动规划、示教回放等上层代码无需区分真机与仿真：

- 动作 `/<namespace>/follow_joint_trajectory`：接收关节轨迹并在仿真时间轴上执行；
- 服务 `/<namespace>/trajectory_stop`：请求停止当前轨迹并原地保持；
- 服务 `/<namespace>/gripper/set`：设置仿真夹爪开口宽度；
- 话题 `/<namespace>/joint_states`、`/<namespace>/gripper/state`、`/clock`；
- 可选的虚拟 RGB-D 相机话题（只有显式开启虚拟相机时才创建）。

安全约束：本节点只驱动 MuJoCo 仿真，不导入也不调用任何真实电机 SDK；仿真启动
不会打开硬件通道。所有外部输入（轨迹、夹爪宽度、仿真时间）先经本模块顶部的
校验助手做有限性、范围与规模检查，非法输入一律拒绝而不是静默接受；属于物理
行程范围的裁剪（如夹爪开口）由仿真侧按模型限位完成。

本模块顶部的校验助手刻意不导入任何 ROS 依赖，因此可以在未安装 ROS 的开发主机上
对轨迹输入做模糊测试与单元测试；ROS 类型只在构造节点时延迟导入。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

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


def create_node_class():
    """延迟导入 ROS 依赖并返回具体的节点类。

    之所以做成工厂函数而不是模块级类定义：本模块顶部的校验逻辑需要在没有 ROS
    的环境里被导入和测试，因此所有 ROS 消息、执行器与 QoS 类型都推迟到真正
    构造节点时才导入。
    """
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.clock import Clock as RclpyClock
    from rclpy.clock import ClockType
    from rclpy.node import Node
    from rebotarm_msgs.msg import JointMotorState
    from rebotarm_msgs.srv import SetGripper
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectoryPoint

    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.ros.ros_diagnostics import build_control_diagnostic

    class RebotArmMujocoNode(Node):
        """仿真控制器节点：唯一持有 MuJoCo 实例并对外提供假硬件接口。

        线程模型：
        - 定时器回调（互斥回调组）负责按固定周期步进物理、发布 `/clock`、
          关节状态与夹爪状态，并触发虚拟相机渲染；
        - 动作执行回调（可重入回调组）在自己的线程里按仿真时间采样轨迹；
        - 虚拟相机工作线程负责离屏渲染，其生命周期完全在单独线程上；
        - 三者对仿真的访问统一经 `_sim_access` 串行化，锁序固定为
          "命令闸门锁 → 仿真锁"，避免死锁。

        安全语义：只接受 backend=mujoco 且 headless=true；同一时刻只服务一条
        轨迹；取消/停止/异常都会保持当前位置，绝不继续下发目标。
        """

        def __init__(self) -> None:
            super().__init__("rebotarm_mujoco_node")
            # 后端选择：本节点只实现 mujoco，其他取值（例如指向真实硬件）必须走别的节点。
            self.declare_parameter("backend", "mujoco")
            # 必须为 true：ROS 适配只允许无头模式，查看器由启动脚本按需另开进程。
            self.declare_parameter("headless", True)
            # 是否在执行线程之外同步一个 MuJoCo 被动查看器窗口（桌面调试用）。
            self.declare_parameter("show_viewer", False)
            # 场景模型路径；空字符串表示用包内默认场景。
            self.declare_parameter("model_path", "")
            # 话题与服务命名空间前缀（去除首尾斜杠后使用），决定所有接口的名字。
            self.declare_parameter("arm_namespace", "rebotarm")
            # 物理步进定时器频率，单位 Hz；决定 /clock 与关节状态的发布周期。
            self.declare_parameter("publish_rate_hz", 30.0)
            # 单条轨迹点数上限与时长上限（单位 s），见模块顶部常量说明。
            self.declare_parameter("max_trajectory_points", DEFAULT_MAX_TRAJECTORY_POINTS)
            self.declare_parameter("max_trajectory_duration_sec", DEFAULT_MAX_TRAJECTORY_DURATION_SEC)
            # 六个手臂关节的初始角，单位 rad；启动即把仿真复位到该位形。
            self.declare_parameter("initial_joint_positions", [0.0] * 6)
            # 到位判定阈值：位置单位 rad、速度单位 rad/s、等待时间单位 s。
            self.declare_parameter("goal_position_tolerance", 0.02)
            self.declare_parameter("goal_velocity_tolerance", 0.05)
            self.declare_parameter("goal_time_tolerance_sec", 5.0)
            # 动作反馈发布频率，单位 Hz（上限 200，见 FeedbackRateLimiter）。
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
                raise ValueError("ROS adapter requires headless=true")
            self._show_viewer = bool(self.get_parameter("show_viewer").value)

            # 命名空间只允许去掉首尾斜杠的普通标识；含 "//" 或空格会拼出非法接口名。
            namespace = str(self.get_parameter("arm_namespace").value).strip("/")
            if not namespace or any(part in namespace for part in ("//", " ")):
                raise ValueError("arm namespace is invalid")
            self._arm_namespace = namespace
            # 步进频率上限 1000 Hz：再高只会空转 CPU 且不改善物理精度（精度由 timestep 决定）。
            rate = float(self.get_parameter("publish_rate_hz").value)
            if not math.isfinite(rate) or rate <= 0.0 or rate > 1000.0:
                raise ValueError("publish rate must be finite and in (0, 1000]")
            self._max_points = int(self.get_parameter("max_trajectory_points").value)
            self._max_duration = float(self.get_parameter("max_trajectory_duration_sec").value)
            # 刻意不解释单个目标自带的 JointTolerance 数组：本仿真后端只使用
            # 上述有界的节点级默认值，防止客户端在单次目标里放宽到位保证。
            self._settling_policy = GoalSettlingPolicy(
                float(self.get_parameter("goal_position_tolerance").value),
                float(self.get_parameter("goal_velocity_tolerance").value),
                float(self.get_parameter("goal_time_tolerance_sec").value),
            )
            self._feedback_rate_hz = float(self.get_parameter("feedback_rate_hz").value)
            # 构造函数内部完成有限/正数/范围校验，非法值在此直接终止启动。
            FeedbackRateLimiter(self._feedback_rate_hz)
            # 执行循环的休眠步长：上限 10 ms，避免忙等占满 CPU，同时保证反馈与
            # 取消请求的响应延迟不超过一个反馈周期。
            self._execute_wait_sec = min(0.01, 1.0 / self._feedback_rate_hz)
            # 初始位形必须是六个有限值，否则拒绝启动（不静默补零）。
            initial = tuple(float(v) for v in self.get_parameter("initial_joint_positions").value)
            if len(initial) != 6 or any(not math.isfinite(v) for v in initial):
                raise ValueError("initial positions must contain six finite values")

            model_path = str(self.get_parameter("model_path").value).strip()
            # 空路径交给仿真类去解析包内默认场景，语义与显式传路径一致。
            self._sim = RebotArmMujoco(model_path or None)
            self._lock = threading.RLock()
            self._sim_access = SerializedSimulationAccess(self._sim, self._lock)
            self._sim_access.run(lambda sim: sim.reset_joint_positions(initial))
            # 命令闸门锁与仿真锁刻意分开：加锁顺序恒为"先闸门、后仿真"，而定时器
            # 只取仿真锁，因此不存在反向持锁路径，不会死锁。
            self._active = ActiveTrajectory()
            self._command_gate = TrajectoryCommandGate(self._active)
            self._lifecycle = ExecutionLifecycle(self._active, self._command_gate)
            # 准入时校验好的轨迹采样器，由目标回调暂存、执行回调取走（一次性交接）。
            self._pending_sampler: TrajectorySampler | None = None
            # 动作/服务回调放在可重入组里：执行回调自身会长时间运行，且停止服务
            # 必须在执行期间仍能被派发，否则"停止"会被执行回调阻塞住。
            self._callback_group = ReentrantCallbackGroup()
            # 定时器组互斥：避免多次步进回调重入同一个物理实例。
            self._timer_callback_group = MutuallyExclusiveCallbackGroup()
            # 物理时钟用 STEADY_TIME，保证定时器周期不受系统时间跳变影响。
            self._physics_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
            # 对外时间戳单调递增（见 MonotonicStamp 说明）。
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
            # 停止服务：让上层以"服务调用"的方式软停（保持当前位置），不依赖动作取消。
            self.create_service(
                Trigger,
                f"/{self._arm_namespace}/trajectory_stop",
                self._stop_service,
                callback_group=self._callback_group,
            )
            # 夹爪服务沿用 SetGripper 的调用约定（开口宽度单位 m），便于仿真链路复用。
            self.create_service(
                SetGripper,
                f"/{self._arm_namespace}/gripper/set",
                self._gripper_service,
                callback_group=self._callback_group,
            )
            # 每次定时器回调推进足够多的固定物理步，使其时长与配置的发布周期一致；
            # 轨迹采样始终以仿真时间（而非墙钟）为准，因此慢机器上轨迹不会"变慢"。
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

        @property
        def show_viewer(self) -> bool:
            return self._show_viewer

        def viewer_handles(self):
            """在仿真锁内取出 MuJoCo 原生 model/data 句柄供查看器使用。

            句柄可在 Viewer 生命周期内持有；调用方不得绕过本节点改动物理状态。
            sync 必须走仿真锁，关闭 Viewer 后才能释放仿真。
            """
            return self._sim_access.run(lambda sim: sim.borrow_viewer_handles())

        def sync_viewer(self, viewer) -> None:
            """在仿真锁内把最新物理状态同步给查看器（避免读到半更新状态）。"""
            self._sim_access.run(lambda _sim: viewer.sync())

        def _current_arm_positions(self) -> tuple[float, ...]:
            """读取当前六个手臂关节角（rad），供轨迹准入时补齐缺省关节。"""
            return self._sim_access.run(
                lambda sim: tuple(sim.get_state().joint_positions[:6])
            )

        def _hold_current_position(self) -> None:
            """线程安全地"原地保持"：把当前位置同时设为目标位置。"""
            self._sim_access.run(lambda _sim: self._hold_current_position_unlocked())

        def _hold_current_position_unlocked(self) -> None:
            """已持有仿真锁时的保持实现，供闸门在同一临界区内调用。"""
            current = tuple(self._sim.get_state().joint_positions[:6])
            self._sim.set_joint_position_targets(current)

        def _apply_target_threadsafe(self, operation) -> None:
            """在仿真锁内执行一个会改写物理目标的操作。"""
            self._sim_access.run(lambda _sim: operation())

        def _goal_callback(self, goal_request):
            """动作目标准入：先校验轨迹，再抢占执行权；任一步失败都返回 REJECT。

            校验在此完成而不是等到执行阶段，是为了让非法目标尽快得到明确的
            拒绝响应，也避免执行线程被畸形输入拖住。通过后把采样器暂存，由
            执行回调一次性取走。
            """
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
            # 单目标闸门：忙时直接拒绝新目标，不做排队，避免隐式覆盖正在执行的轨迹。
            if not self._active.try_start(token):
                self.get_logger().warning("rejected trajectory goal while controller is busy")
                return GoalResponse.REJECT
            self._pending_sampler = sampler
            return GoalResponse.ACCEPT

        def _cancel_callback(self, _goal_handle):
            """动作取消回调：能停下活动目标才 ACCEPT，否则 REJECT（不改动仿真）。"""
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            return CancelResponse.ACCEPT if stopped else CancelResponse.REJECT

        def _stop_service(self, _request, response):
            """停止服务：软停（保持当前位置），始终返回 success=True 以区分"无活动目标"。"""
            stopped = self._command_gate.stop_and_hold(self._hold_current_position)
            response.success = True
            response.message = "simulation trajectory stop requested" if stopped else "no active trajectory"
            return response

        def _publish_diagnostics(self):
            state, contacts, status = self._sim_access.run(
                lambda sim: (
                    sim.get_state(),
                    sim.get_contacts(),
                    {"mode": sim.control_mode, "joint_targets": sim.control_targets[:6]},
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
            """夹爪服务：校验开口宽度后交给仿真裁剪到可信行程并返回实际到位值。"""
            try:
                width = validate_gripper_width(request.position)
                reached = self._sim_access.run(lambda sim: sim.set_gripper_width(width))
            except (TypeError, ValueError):
                # 请求非法：明确报失败，并回填 0.0 而不是伪造一个"已到位"的位置。
                response.success = False
                response.reached_position = 0.0
                return response
            response.success = True
            response.reached_position = float(reached)
            return response

        @staticmethod
        def _terminate_goal(goal_handle, result, outcome):
            """按闸门裁决把目标终止为 canceled 或 aborted 并填好错误码。

            注意：取消成功时动作错误码仍填 SUCCESSFUL——取消是客户端主动行为，
            不是执行故障，错误信息只用于日志区分。
            """
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
            """动作执行回调：在仿真时间轴上跟踪轨迹、发反馈并判定终态。

            每轮循环先经命令闸门下发目标（闸门会同时处理取消与 token 失效），
            再按反馈频率发布 desired/actual/error；轨迹走完后进入到位判定，
            只有位置与速度都收敛才 succeed，超时则 abort 并保持当前位置。
            """
            # rclpy 不保证"传给准入门的目标请求包装对象"与"目标句柄暴露的对象"
            # 是同一个。这里保留准入时的 token，确保清理阶段不会因为对象不一致
            # 而漏释放执行权、让动作服务器在一个合法目标之后永久 busy。
            token = self._active.token
            result = FollowJointTrajectory.Result()
            # 一次性交接：取走准入时校验好的采样器并清空暂存位。
            sampler, self._pending_sampler = self._pending_sampler, None
            if sampler is None:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "trajectory was not admitted"
                goal_handle.abort()
                self._active.finish(token)
                return result
            try:
                # 以目标开始执行时的仿真时间为轨迹时间原点；仿真时间只随物理步进前进，
                # 因此慢机器不会让轨迹被执行得更快或更慢。
                start_time = self._sim_access.run(lambda sim: sim.get_state().simulation_time)
                # 每个目标一个限流器实例：首个反馈立即发出，之后按反馈频率节流。
                feedback_limiter = FeedbackRateLimiter(self._feedback_rate_hz)
                while rclpy.ok():
                    # 闭包无法直接返回结果，用一个临时字典把本轮的仿真状态与采样值
                    # 从 apply_target 里带出来。
                    command: dict[str, Any] = {}

                    def apply_target() -> None:
                        state = self._sim.get_state()
                        elapsed = max(0.0, state.simulation_time - start_time)
                        # 超过轨迹时长后夹到终点，保持终点位姿直到判定结束。
                        desired = sampler.sample(min(elapsed, sampler.duration))
                        self._sim.set_joint_position_targets(desired)
                        command.update(state=state, elapsed=elapsed, desired=desired)

                    # 闸门回调里再查一次取消标志：取消可能刚好在上一轮循环之后到达。
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
                    # 反馈统一使用规范的六个关节名与名称顺序，保证订阅端能按名对齐。
                    feedback = FollowJointTrajectory.Feedback()
                    feedback.joint_names = list(ARM_JOINT_NAMES)
                    feedback.desired = JointTrajectoryPoint()
                    feedback.actual = JointTrajectoryPoint()
                    feedback.error = JointTrajectoryPoint()
                    feedback.desired.positions = list(desired)
                    actual = tuple(state.joint_positions[:6])
                    feedback.actual.positions = list(actual)
                    feedback.actual.velocities = list(state.joint_velocities[:6])
                    # error 定义为"期望 - 实际"，符号与上层监控约定一致。
                    feedback.error.positions = [d - a for d, a in zip(desired, actual)]
                    # 只有在轨迹走完后才开始到位判定；在此之前统一报告 "tracking"
                    # （非终态伪状态），settle_elapsed 也才从 0 起算，不会把跟踪
                    # 过程中的减速误判为"已到位"。
                    settling = self._settling_policy.evaluate(
                        sampler.sample(sampler.duration),
                        actual,
                        tuple(state.joint_velocities[:6]),
                        max(0.0, elapsed - sampler.duration),
                    ) if elapsed >= sampler.duration else "tracking"
                    # 终态反馈强制发布（final=True），确保订阅端不会漏掉最后一次状态。
                    if feedback_limiter.should_publish(
                        time.monotonic(), final=settling in ("succeeded", "timed_out")
                    ):
                        goal_handle.publish_feedback(feedback)
                    if settling == "succeeded":
                        # 成功迁移与取消判定在同一临界区完成：若期间到达取消请求，
                        # 则按取消/停止终止，绝不把已取消的目标报成成功。
                        completion = self._command_gate.complete_with_reason(
                            token,
                            lambda: goal_handle.is_cancel_requested,
                            self._hold_current_position,
                            goal_handle.succeed,
                        )
                        if completion is not GateOutcome.SUCCEEDED:
                            return self._terminate_goal(goal_handle, result, completion)
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "simulation trajectory finished"
                        return result
                    if settling == "timed_out":
                        # 超时视为失败：先软停并保持当前位置，再以容差违规错误码
                        # 上报，便于上层区分"没走到位"与"轨迹本身非法"。
                        self._command_gate.stop_and_hold(self._hold_current_position)
                        goal_handle.abort()
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = "simulation goal did not settle within configured tolerance"
                        return result
                    time.sleep(self._execute_wait_sec)
                # 退出 while 说明 rclpy 已停止（进程正在关闭）。
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation shutting down"
                return result
            except Exception as exc:
                self.get_logger().error(
                    f"trajectory execution exception: {type(exc).__name__}: {exc}"
                )
                # 兜底清理：先保持当前位置，再在句柄仍活动时 abort，最后一定会
                # 释放 token（见 ExecutionLifecycle），避免异常后节点永久 busy。
                self._lifecycle.fail(
                    token,
                    self._hold_current_position,
                    lambda: goal_handle.abort() if getattr(goal_handle, "is_active", True) else None,
                )
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "simulation trajectory execution failed"
                return result
            finally:
                # 正常/取消/异常路径都从这里释放执行权；finish 只在 token 匹配时生效，
                # 因此不会误释放后续目标。
                self._active.finish(token)

        def _timer_callback(self) -> None:
            """固定周期回调：步进物理、发布时钟/关节/夹爪状态并驱动虚拟相机。

            物理步数与发布周期对齐（见 _steps_per_tick），因此仿真时间大致与墙钟
            同步；本回调在互斥回调组中执行，不会与自身重入。
            """
            state = self._sim_access.run(lambda sim: sim.step(self._steps_per_tick))
            # 先发布 /clock：使用 use_sim_time 的订阅端据此驱动自己的定时器，
            # 因此时钟必须先于同批次的状态消息发出。
            stamp = Clock()
            seconds, nanoseconds = self._stamp.update(state.simulation_time)
            stamp.clock.sec = seconds
            stamp.clock.nanosec = nanoseconds
            self._clock_pub.publish(stamp)

            # 关节状态包含八个关节：joint1..joint6 单位 rad；effort 取执行器输出力
            # （手臂为 N·m，直线手指关节为 N），供上层监控但不可当作真机测量值。
            joint = JointState()
            joint.header.stamp = stamp.clock
            joint.name = list(state.joint_names)
            joint.position = list(state.joint_positions)
            joint.velocity = list(state.joint_velocities)
            joint.effort = list(state.actuator_forces)
            self._joint_pub.publish(joint)

            # 夹爪状态：position 承载开口宽度（m，0 = 完全闭合），velocity 不建模固定为 0；
            # torque 用两个手指执行器出力绝对值之和近似夹持力（N）；status_code 固定 0
            # 表示"正常"，不代表真机意义的电机使能位。
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
    """进程入口：创建节点并驱动执行器（可选同步 MuJoCo 被动查看器）。

    线程数取 3：定时器回调、动作执行回调、以及动作/服务回调各占一路，避免
    长时间运行的执行回调把步进定时器饿死。开启查看器时，MuJoCo 的窗口循环
    必须占用主线程，因此把执行器放到后台线程，主线程只做 viewer.sync()。
    """
    import importlib
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = create_node_class()()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor_thread = None
    viewer = None
    try:
        if node.show_viewer:
            # 被动查看器直接复用仿真内的 model/data 句柄，不做物理推进（由定时器负责）。
            model, data = node.viewer_handles()
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive
            viewer = launch_passive(model, data)
            executor_thread = threading.Thread(
                target=executor.spin,
                name="rebotarm-mujoco-ros-executor",
                daemon=True,
            )
            executor_thread.start()
            # 主线程以约 100 Hz 同步渲染画面，直到窗口关闭或收到关闭信号。
            while rclpy.ok() and viewer.is_running():
                node.sync_viewer(viewer)
                time.sleep(0.01)
            return
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 与外部关闭都按正常退出处理，不打印堆栈。
        pass
    finally:
        # 关闭顺序：查看器 → 执行线程 → 执行器 → 节点 → rclpy，确保没有线程
        # 仍在访问已被销毁的节点或仿真对象。
        if viewer is not None:
            viewer.close()
        if executor_thread is not None:
            executor_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
