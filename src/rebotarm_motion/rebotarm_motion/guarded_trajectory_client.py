"""真机与仿真轨迹对照工作流的 ROS 客户端。

职责与位置
    轨迹对照工作流的客户端适配；持有观察记录，实际验收策略由调用方制定。

对外接口
    GuardedTrajectoryNode 提供预检，Trigger 调用，execute_leg 和保持采样。

安全与并发约束
    单线程主动自旋，不能与另一个执行器同时驱动该节点。 此工具使用接收年龄检查，
    不等同生产位姿链路的 150 ms 源时间连续性守卫，也不替代独立现场监测。
    客户端具备 enable/disable 入口，但现场条件，基线和调用授权由上层流程保证。

文件结构（按定义顺序）
    _stamp_ns：把 ROS Time 转成源时间纳秒整数，仅用于记录，不刷新消息身份。
    _duration：把相对秒转换为 ROS Duration 的整秒和纳秒字段。
    GuardedTrajectoryNode：由调用流程主动自旋的轨迹对照客户端，保存执行样本和停止原因。
"""

from __future__ import annotations

from collections import deque
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from rebotarm_motion.paired_trajectory_protocol import (
    ARM_JOINT_NAMES,
    sample_command,
    validate_command,
)


def _stamp_ns(stamp) -> int:
    """把 ROS Time 转成源时间纳秒整数，仅用于记录，不刷新消息身份。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _duration(seconds: float):
    """把相对秒转换为 ROS Duration 的整秒和纳秒字段。"""
    total_nanoseconds = int(round(float(seconds) * 1_000_000_000))
    whole, nanoseconds = divmod(total_nanoseconds, 1_000_000_000)
    from builtin_interfaces.msg import Duration

    return Duration(sec=whole, nanosec=nanoseconds)


class GuardedTrajectoryNode(Node):
    """由调用流程主动自旋的轨迹对照客户端，保存执行样本和停止原因。"""

    def __init__(
        self,
        *,
        namespace: str,
        backend: str,
    ) -> None:
        """创建反馈订阅，FJT 与显式使能/失能/停止客户端，构造不发送运动。"""
        super().__init__(f"rebotarm_guarded_trajectory_{backend}")
        self.namespace = namespace.strip("/")
        self.backend = backend
        self.latest_joint_state: JointState | None = None
        self.latest_status: ArmStatus | None = None
        self.latest_gripper: JointMotorState | None = None
        self.last_joint_monotonic: float | None = None
        self.active_command: dict[str, object] | None = None
        self.active_start_ns: int | None = None
        self.active_samples: list[dict[str, object]] = []
        self.guard_stop: dict[str, object] | None = None
        self.position_history: deque[tuple[float, tuple[float, ...]]] = deque()
        self.action_feedback: list[dict[str, object]] = []
        self.create_subscription(
            JointState,
            f"/{self.namespace}/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        if backend == "real":
            self.create_subscription(
                ArmStatus,
                f"/{self.namespace}/arm_status",
                self._status_callback,
                latched,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self.namespace}/gripper/state",
            self._gripper_callback,
            qos_profile_sensor_data,
        )
        self.action = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self.namespace}/follow_joint_trajectory",
        )
        self.enable_client = self.create_client(Trigger, f"/{self.namespace}/enable")
        self.disable_client = self.create_client(Trigger, f"/{self.namespace}/disable")
        self.stop_client = self.create_client(Trigger, f"/{self.namespace}/trajectory_stop")

    def _status_callback(self, message: ArmStatus) -> None:
        """保存控制器状态；活动执行中观察到失能，错误或非正常电机码时记录停止原因。"""
        self.latest_status = message
        if self.backend != "real" or self.active_command is None:
            return
        if not message.enabled or not message.control_loop_active:
            self.guard_stop = {"reason": "controller_left_enabled_control_state"}
        elif any(int(value) != 1 for value in message.per_joint_status_code):
            self.guard_stop = {
                "reason": "motor_status_changed",
                "per_joint_status_code": [int(value) for value in message.per_joint_status_code],
            }
        elif message.error_codes:
            self.guard_stop = {
                "reason": "controller_error",
                "error_codes": list(message.error_codes),
            }

    def _gripper_callback(self, message: JointMotorState) -> None:
        """保存最新夹爪消息供轨迹样本附带记录，不主动生成夹爪命令。"""
        self.latest_gripper = message

    def _joint_state_callback(self, message: JointState) -> None:
        """按规范名称取六轴有限值，记录期望/实测，速度，力矩和状态，不在回调中直接停止设备。"""
        now = time.monotonic()
        now_ns = time.monotonic_ns()
        self.latest_joint_state = message
        self.last_joint_monotonic = now
        if self.active_command is None or self.active_start_ns is None:
            return
        by_name = {name: index for index, name in enumerate(message.name)}
        if any(name not in by_name for name in ARM_JOINT_NAMES):
            self.guard_stop = {"reason": "missing_joint", "names": list(message.name)}
            return
        if any(
            by_name[name] >= len(values)
            for values in (message.position, message.velocity, message.effort)
            for name in ARM_JOINT_NAMES
        ):
            self.guard_stop = {
                "reason": "incomplete_joint_state_arrays",
                "position_count": len(message.position),
                "velocity_count": len(message.velocity),
                "effort_count": len(message.effort),
            }
            return
        elapsed = max(0.0, (now_ns - self.active_start_ns) * 1e-9)
        desired = sample_command(self.active_command, elapsed)
        observed = tuple(float(message.position[by_name[name]]) for name in ARM_JOINT_NAMES)
        velocities = tuple(float(message.velocity[by_name[name]]) for name in ARM_JOINT_NAMES)
        efforts = tuple(float(message.effort[by_name[name]]) for name in ARM_JOINT_NAMES)
        if not all(math.isfinite(value) for values in (observed, velocities, efforts) for value in values):
            self.guard_stop = {"reason": "non_finite_joint_state"}
            return
        self.position_history.append((now, observed))
        while self.position_history and now - self.position_history[0][0] > 0.30:
            self.position_history.popleft()
        window_velocities = self._window_velocities(now, observed)
        tracking = tuple(target - actual for target, actual in zip(desired, observed))
        sample = {
            "monotonic_ns": now_ns,
            "ros_stamp_ns": _stamp_ns(message.header.stamp),
            "elapsed_sec": elapsed,
            "desired_positions": list(desired),
            "observed_positions": list(observed),
            "velocities": list(velocities),
            "window_velocities": list(window_velocities),
            "efforts": list(efforts),
            "tracking_errors": list(tracking),
            "action_state": "executing",
            "gripper": self._gripper_payload(),
            "contact": {"available": False, "reason": "no common real/sim contact topic"},
            "arm_status": self._status_payload(),
        }
        self.active_samples.append(sample)

    def _window_velocities(self, now: float, observed: tuple[float, ...]) -> tuple[float, ...]:
        """用约 0.20 s 前位置计算诊断平均速度，历史不足时填零；不是瞬时实测速度。"""
        candidate = None
        for timestamp, positions in self.position_history:
            if now - timestamp >= 0.20:
                candidate = (timestamp, positions)
            else:
                break
        if candidate is None:
            return (0.0,) * len(ARM_JOINT_NAMES)
        timestamp, positions = candidate
        delta = max(now - timestamp, 1e-9)
        return tuple((current - previous) / delta for current, previous in zip(observed, positions))

    def _gripper_payload(self) -> dict[str, object]:
        """把已有夹爪消息压缩为报告字段，无消息时标记不可用。"""
        if self.latest_gripper is None:
            return {"available": False}
        message = self.latest_gripper
        return {
            "available": True,
            "position": float(message.position),
            "velocity": float(message.velocity),
            "torque": float(message.torque),
            "status_code": int(message.status_code),
        }

    def _status_payload(self) -> dict[str, object]:
        """把已有控制器状态压缩为报告字段，不在此检查源年龄或访问设备。"""
        if self.latest_status is None:
            return {"available": False}
        message = self.latest_status
        return {
            "available": True,
            "enabled": bool(message.enabled),
            "control_loop_active": bool(message.control_loop_active),
            "state_machine": str(message.state_machine),
            "per_joint_status_code": [int(value) for value in message.per_joint_status_code],
            "error_codes": [str(value) for value in message.error_codes],
        }

    def wait_for_preflight(self, timeout_sec: float = 10.0) -> None:
        """等待首批反馈并检查唯一发布者；真机要求初始失能和无报告错误，不替代现场检查。"""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            status_ready = self.backend != "real" or self.latest_status is not None
            if self.latest_joint_state is not None and status_ready:
                break
        if self.latest_joint_state is None:
            raise RuntimeError("joint feedback unavailable")
        publishers = self.get_publishers_info_by_topic(f"/{self.namespace}/joint_states")
        if len(publishers) != 1:
            raise RuntimeError(f"expected one joint-state publisher, got {len(publishers)}")
        if self.backend == "real":
            assert self.latest_status is not None
            if self.latest_status.enabled or self.latest_status.control_loop_active:
                raise RuntimeError("real controller must start disabled")
            if any(int(value) != 0 for value in self.latest_status.per_joint_status_code):
                raise RuntimeError("real controller has non-zero motor status")
            if self.latest_status.error_codes:
                raise RuntimeError(f"real controller reports errors: {list(self.latest_status.error_codes)}")

    def canonical_positions(self) -> tuple[float, ...]:
        """按 joint1..joint6 顺序返回最新消息位置；新鲜度和完整性由调用流程另行保证。"""
        if self.latest_joint_state is None:
            raise RuntimeError("joint state unavailable")
        by_name = {name: index for index, name in enumerate(self.latest_joint_state.name)}
        return tuple(float(self.latest_joint_state.position[by_name[name]]) for name in ARM_JOINT_NAMES)

    def call_trigger(self, client, label: str, timeout_sec: float = 10.0):
        """等待 Trigger 服务并自旋等待本次回执，失败或无响应抛异常；不会把超时解释为未执行。"""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{label} service unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"{label} failed: {message}")
        return {"success": True, "message": str(response.message)}

    def _feedback_callback(self, feedback_message) -> None:
        """记录 FJT 的期望，实际及误差反馈，独立于 joint_states 采样。"""
        feedback = feedback_message.feedback
        self.action_feedback.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "joint_names": list(feedback.joint_names),
                "desired_positions": list(feedback.desired.positions),
                "actual_positions": list(feedback.actual.positions),
                "error_positions": list(feedback.error.positions),
            }
        )

    def execute_leg(self, command: dict[str, object]) -> dict[str, object]:
        """发送已校验轨迹并记录整段结果，成功要求动作终态，结果码和本地停止原因同时通过。

        单线程主动自旋处理回执；状态异常或接收超时则取消，真机另请求 trajectory_stop。
        此处的接收超时为工具判据，不能替代生产源时间检查或独立现场监测；恢复策略由调用方决定。
        """
        validate_command(command)
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("FollowJointTrajectory action unavailable")
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINT_NAMES)
        for source in command["points"]:
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in source["positions"]]
            point.time_from_start = _duration(float(source["elapsed_sec"]))
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.active_command = command
        self.active_samples = []
        self.action_feedback = []
        self.guard_stop = None
        self.position_history.clear()
        # 本工具按本地发送时刻对齐报告，不能把此时间轴当成电机实际执行起点的精确测量。
        self.active_start_ns = time.monotonic_ns()
        send_future = self.action.send_goal_async(goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.active_command = None
            raise RuntimeError("trajectory goal rejected")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + float(command["duration_sec"]) + 12.0
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.guard_stop is not None:
                goal_handle.cancel_goal_async()
                if self.backend == "real":
                    self.call_trigger(self.stop_client, "trajectory_stop")
                break
            if self.last_joint_monotonic is None or time.monotonic() - self.last_joint_monotonic > 0.5:
                self.guard_stop = {"reason": "joint_state_stale"}
                goal_handle.cancel_goal_async()
                if self.backend == "real":
                    self.call_trigger(self.stop_client, "trajectory_stop")
                break
        if not result_future.done():
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        wrapped = result_future.result()
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.02)
        self.active_command = None
        self.active_start_ns = None
        if wrapped is None:
            raise RuntimeError("trajectory result unavailable")
        result_payload = {
            "status": int(wrapped.status),
            "error_code": int(wrapped.result.error_code),
            "error_string": str(wrapped.result.error_string),
            "guard_stop": self.guard_stop,
        }
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            and self.guard_stop is None
        )
        return {
            "command": command,
            "samples": self.active_samples,
            "action_feedback": self.action_feedback,
            "result": result_payload,
            "success": succeeded,
        }

    def hold_and_collect(self, seconds: float) -> None:
        """在指定秒数内继续主动自旋接收；本方法不另发保持目标，也不证明静止。"""
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
