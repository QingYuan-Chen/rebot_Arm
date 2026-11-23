"""键盘遥操作节点:分模式将终端单键翻译为关节增量轨迹,重力补偿或关节阻抗调参请求.

在系统中的位置:属于"操作者意图 → ROS 命令"的适配层,订阅 `/<ns>/arm_status` 按模式路由:
- `IDLE`: ``1..6`` / ``q w e r t y`` 走运动层重定时 + FJT 关节增量点动; ``g`` 进重力补偿; ``i`` 进关节阻抗;
- `GRAVITY_COMP`: 仅允许 ``h``/空格退出与 ``p`` 打印状态,拦截运动和调参按键;
- `JOINT_IMPEDANCE`: 拦截 FJT 点动; ``1..6`` 选轴, ``[``/``]`` 调 Kp, ``,``/``.`` 调 Kd,
  ``-``/``=`` 调 q_ref, ``0``/``l`` 对齐实时位置, ``h``/空格退出, ``p`` 打印六轴实时遥测.

对外接口(``<ns>`` 取自参数 arm_namespace,默认 rebotarm):
- 动作 ``/<ns>/follow_joint_trajectory``;
- 服务 ``/<ns>/retime_arm_trajectory`` / ``/<ns>/trajectory_stop`` /
  ``/<ns>/gravity_compensation/start|stop`` / ``/<ns>/joint_impedance/start|stop``;
- 订阅 ``/<ns>/joint_states``(BEST_EFFORT), ``/<ns>/arm_status``(TRANSIENT_LOCAL),
  ``/<ns>/trajectory_execution_mode``(TRANSIENT_LOCAL);
- 发布 ``/<ns>/teleop/status``:JSON 字符串,字段 source/state/message/last_key/joint_positions.

安全约束:退出时恢复终端属性;所有位置目标夹紧到关节限位;
绝不内置默认刚度/阻尼,仿真与真机阻抗配置通过显式 profile 强隔离;
在途异步请求未结束前丢弃新触发按键, ``h``/空格停机始终放行.

文件结构:参数与配置隔离校验 → 订阅与服务装配 → 分模式轮询路由 → 异步回调/遥测/退出.
"""

from __future__ import annotations

import json
import select
import sys
import termios
import time
import tty
from contextlib import suppress

from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus
from rebotarm_msgs.srv import SetJointImpedance
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .keyboard_force_tuning import (
    ImpedanceTuningDraft,
    KeyboardImpedanceProfile,
    KeyboardModeRouter,
    apply_draft_tuning,
    format_telemetry,
)
from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .teleop_core import TeleopTargetPlanner


class TeleopKeyboardNode(Node):
    """键盘遥操作节点:单线程 executor 下由定时器驱动分模式按键轮询与异步请求下发."""

    def __init__(self) -> None:
        super().__init__("teleop_keyboard_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        self.declare_parameter("joint_step_rad", 0.02)
        from .trajectory_capability import TrajectoryCapability
        self._capability = TrajectoryCapability()
        self.declare_parameter("trajectory_duration", .4)
        self._busy = False
        self._step_generation = 0
        self._retime_pending = None
        self._service_pending = None
        self._feedback_at = 0.
        self._arm_status = None
        self._arm_status_at = 0.0
        self._last_arm_status_key = None
        self._active_draft: ImpedanceTuningDraft | None = None
        self.declare_parameter("poll_period", 0.05)
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("deadman_required", False)
        self.declare_parameter("deadman_key", "z")
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("impedance_profile", "")
        dynamic_array = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("impedance_initial_stiffness", [], dynamic_array)
        self.declare_parameter("impedance_initial_damping", [], dynamic_array)
        self.declare_parameter("impedance_max_stiffness", [], dynamic_array)
        self.declare_parameter("impedance_max_damping", [], dynamic_array)
        self.declare_parameter("impedance_kp_step", 0.0)
        self.declare_parameter("impedance_kd_step", 0.0)
        self.declare_parameter("impedance_q_step_rad", 0.0)

        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        self._joint_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        self._trajectory_duration = float(self.get_parameter("trajectory_duration").value)
        self._input_timeout = float(self.get_parameter("input_timeout").value)
        self._deadman_required = bool(self.get_parameter("deadman_required").value)
        self._deadman_key = str(self.get_parameter("deadman_key").value)
        self._use_hardware = bool(self.get_parameter("use_hardware").value)
        self._impedance_profile = KeyboardImpedanceProfile.from_parameters(
            use_hardware=self._use_hardware,
            profile_name=str(self.get_parameter("impedance_profile").value),
            initial_stiffness=self.get_parameter("impedance_initial_stiffness").value,
            initial_damping=self.get_parameter("impedance_initial_damping").value,
            max_stiffness=self.get_parameter("impedance_max_stiffness").value,
            max_damping=self.get_parameter("impedance_max_damping").value,
            kp_step=float(self.get_parameter("impedance_kp_step").value),
            kd_step=float(self.get_parameter("impedance_kd_step").value),
            q_step_rad=float(self.get_parameter("impedance_q_step_rad").value),
        )
        # monotonic 时间戳:死锁授权截止时刻与最近一次有效输入时刻,均不受系统时钟调整影响.
        self._deadman_until = 0.0
        self._last_input_time = 0.0
        # 关节角/速度/力矩基准,收到 joint_states 后按声明关节名实时刷新.
        self._current_positions = {name: 0.0 for name in self._joint_names}
        self._current_velocities = {name: 0.0 for name in self._joint_names}
        self._current_efforts = {name: 0.0 for name in self._joint_names}
        self._terminal_settings = None
        self._router = KeyboardModeRouter(joint_names=self._joint_names)
        self._planner = TeleopTargetPlanner(
            joint_names=self._joint_names,
            joint_limits=self._joint_limits,
            joint_step_rad=float(self.get_parameter("joint_step_rad").value),
        )
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        from rebotarm_msgs.srv import RetimeArmTrajectory
        self._retime_type = RetimeArmTrajectory
        self._retime_client = self.create_client(RetimeArmTrajectory,
            f'/{self._arm_namespace}/retime_arm_trajectory')
        self._stop_client = self.create_client(
            Trigger, f"/{self._arm_namespace}/trajectory_stop"
        )
        self._gravity_start_client = self.create_client(
            Trigger, f"/{self._arm_namespace}/gravity_compensation/start"
        )
        self._gravity_stop_client = self.create_client(
            Trigger, f"/{self._arm_namespace}/gravity_compensation/stop"
        )
        self._impedance_start_client = self.create_client(
            SetJointImpedance, f"/{self._arm_namespace}/joint_impedance/start"
        )
        self._impedance_stop_client = self.create_client(
            Trigger, f"/{self._arm_namespace}/joint_impedance/stop"
        )
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/status",
            10,
        )
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String,
            f'/{self._arm_namespace}/trajectory_execution_mode',
            self._capability.update,
            latched_qos,
        )
        self.create_subscription(
            ArmStatus,
            f"/{self._arm_namespace}/arm_status",
            self._on_arm_status,
            latched_qos,
        )
        # 传感器数据用 BEST_EFFORT:与控制器发布端 QoS 匹配,宁可丢帧也不要阻塞.
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        # 逐字符读取需要把终端切到 cbreak(保留信号处理,关闭行缓冲);
        # 非 tty(如被 launch 重定向)时保持默认,按键轮询自然读不到数据.
        stdin_is_tty = False
        try:
            if sys.stdin.isatty():
                stdin_is_tty = True
                self._terminal_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
        except Exception as exc:
            self.get_logger().warn(f"keyboard raw mode unavailable: {exc}")

        # 使用主机单调时钟轮询按键,不受 use_sim_time=/clock 启停或仿真速率影响.
        self._poll_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(
            float(self.get_parameter("poll_period").value),
            self._poll_key,
            clock=self._poll_clock,
        )
        profile_name = (
            self._impedance_profile.profile_name
            if self._impedance_profile is not None
            else "unconfigured"
        )
        self.get_logger().info(
            f"[keyboard] ready (tty={stdin_is_tty}, profile={profile_name}): "
            "IDLE: 1..6/q..y jog, g gravity_comp, i joint_impedance | "
            "IMPEDANCE: 1..6 select, [/] Kp, ,/. Kd, -/= q_ref, 0/l align | "
            "GLOBAL: h/space stop->hold, p telemetry"
        )
        self._publish_status("idle", "keyboard teleop ready", log=False)

    def _on_joint_state(self, msg: JointState) -> None:
        """用最新关节反馈刷新位置/速度/力矩;未声明或长度不足的关节保持原值."""
        for index, name in enumerate(msg.name):
            key = str(name)
            if key not in self._current_positions:
                continue
            if index < len(msg.position):
                self._current_positions[key] = float(msg.position[index])
            if index < len(msg.velocity):
                self._current_velocities[key] = float(msg.velocity[index])
            if index < len(msg.effort):
                self._current_efforts[key] = float(msg.effort[index])
        if set(self._joint_names).issubset(msg.name) and len(msg.position) >= len(msg.name):
            self._feedback_at = time.monotonic()

    def _on_arm_status(self, msg: ArmStatus) -> None:
        """同步后端真实状态机;仅在离开 JOINT_IMPEDANCE 时清空本地草稿,避免旧帧误清."""
        previous_mode = (
            str(self._arm_status.state_machine).strip().upper()
            if self._arm_status is not None
            else None
        )
        current_mode = str(msg.state_machine).strip().upper()
        self._arm_status = msg
        self._arm_status_at = time.monotonic()
        if previous_mode == "JOINT_IMPEDANCE" and current_mode != "JOINT_IMPEDANCE":
            self._active_draft = None
        status_key = (current_mode, str(msg.mode), bool(msg.enabled))
        if status_key != self._last_arm_status_key:
            self._last_arm_status_key = status_key
            self.get_logger().info(
                f"[arm_status] state_machine={current_mode} mode={msg.mode} enabled={msg.enabled}"
            )

    def _poll_key(self) -> None:
        """定时器回调:清理超时挂起请求 → 读键 → 分模式路由与异步发送."""
        now = time.monotonic()
        if self._retime_pending is not None:
            future, generation, deadline = self._retime_pending
            if now >= deadline or generation != self._step_generation:
                self._retime_pending = None
                self._step_generation += 1
                future.cancel()
        if self._service_pending is not None:
            future, generation, deadline, label = self._service_pending
            if now >= deadline or generation != self._step_generation:
                self._service_pending = None
                future.cancel()
                if generation == self._step_generation and now >= deadline:
                    self._publish_status("rejected", f"{label} timed out")
        key = self._read_key()
        now = time.monotonic()
        if key is None:
            # 长时间无输入只提示一次超时,随后清空时间戳避免反复刷状态.
            if self._last_input_time and now - self._last_input_time > self._input_timeout:
                self._publish_status("timeout", "keyboard input timeout")
                self._last_input_time = 0.0
            return
        self._last_input_time = now
        if key == self._deadman_key:
            # 死锁键把授权窗口向后推 input_timeout,需要周期性重按以保持有效.
            self._deadman_until = now + self._input_timeout
            self._publish_status("deadman", "deadman refreshed", key=key)
            return

        state_machine = self._arm_status.state_machine if self._arm_status is not None else None
        action = self._router.classify_key(
            key,
            state_machine=state_machine,
            has_impedance_profile=self._impedance_profile is not None,
        )
        if action.kind == "ignore":
            return
        if action.kind == "stop":
            self._handle_stop(key, state_machine)
            return
        if action.kind == "print_telemetry":
            self._print_telemetry(key)
            return
        if action.kind == "reject":
            self._publish_status("rejected", action.message, key=key)
            return
        if action.kind == "select_joint":
            if self._active_draft is None:
                self._publish_status(
                    "rejected",
                    "no active keyboard impedance draft; press h/space then i to re-enter",
                    key=key,
                )
                return
            self._active_draft = self._active_draft.select_joint(action.joint_index)
            idx = self._active_draft.selected_index
            sel_name = self._active_draft.selected_joint_name
            detail = (
                f"selected {sel_name} "
                f"[Kp={self._active_draft.stiffness[idx]:.2f}, "
                f"Kd={self._active_draft.damping[idx]:.2f}, "
                f"q_ref={self._active_draft.positions[idx]:+.4f} rad]"
            )
            self._publish_status("active", detail, key=key)
            return

        if action.requires_deadman and self._deadman_required and now > self._deadman_until:
            self._publish_status("blocked", "deadman key required", key=key)
            return
        if self._busy or self._service_pending is not None:
            self._publish_status("rejected", "keyboard request busy; input discarded", key=key)
            return

        if action.kind == "jog_fjt":
            target = self._planner.apply_delta(
                current_positions=self._current_positions,
                joint_name=action.joint_name,
                direction=action.direction,
            )
            if not target.accepted:
                self._publish_status("rejected", target.message, key=key)
                return
            self._send_target(target.joint_names, target.positions, target.message, key)
            return

        if self._arm_status is None:
            self._publish_status("rejected", "arm_status unavailable", key=key)
            return
        if not self._arm_status.enabled:
            self._publish_status("rejected", "arm disabled; enable arm first", key=key)
            return

        if action.kind == "start_gravity_comp":
            self._call_trigger_async(
                self._gravity_start_client, "gravity_compensation/start", key
            )
            return

        if time.monotonic() - self._feedback_at > 0.15:
            self._publish_status("rejected", "fresh joint feedback required", key=key)
            return

        if action.kind == "start_impedance":
            try:
                candidate = ImpedanceTuningDraft.create_initial(
                    joint_names=self._joint_names,
                    current_positions=self._current_positions,
                    joint_limits=self._joint_limits,
                    profile=self._impedance_profile,
                )
            except ValueError as exc:
                self._publish_status("rejected", str(exc), key=key)
                return
            kp_text = ", ".join(f"{v:.2f}" for v in candidate.stiffness)
            kd_text = ", ".join(f"{v:.2f}" for v in candidate.damping)
            label = (
                f"joint impedance started [sel={candidate.selected_joint_name}, "
                f"Kp=[{kp_text}], Kd=[{kd_text}]]"
            )
            self._call_impedance_async(candidate, label, key)
            return

        if action.kind == "tune_impedance":
            if self._active_draft is None:
                self._publish_status(
                    "rejected",
                    "no active keyboard impedance draft; press h/space then i to re-enter",
                    key=key,
                )
                return
            try:
                candidate, note = apply_draft_tuning(
                    self._active_draft,
                    tune_op=action.tune_op,
                    sign=action.sign,
                    profile=self._impedance_profile,
                    current_positions=self._current_positions,
                    joint_limits=self._joint_limits,
                )
            except ValueError as exc:
                self._publish_status("rejected", str(exc), key=key)
                return
            self._call_impedance_async(candidate, note, key)

    def _handle_stop(self, key: str, state_machine: str | None) -> None:
        """h / 空格全局退出:使未决规划失效并按当前状态机异步退出对应后端模式."""
        self._step_generation += 1
        mode = str(state_machine or "").strip().upper()
        if mode == "GRAVITY_COMP":
            self._dispatch_stop_service(
                self._gravity_stop_client, "gravity_compensation/stop", key, clear_draft=False
            )
            return
        if mode == "JOINT_IMPEDANCE":
            self._dispatch_stop_service(
                self._impedance_stop_client, "joint_impedance/stop", key, clear_draft=True
            )
            return
        if self._stop_client.service_is_ready():
            future = self._stop_client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda res: self._on_stop_reply(res, "trajectory_stop", key, clear_draft=False)
            )
        self._publish_status("stopped", "keyboard stop requested", key=key)

    def _dispatch_stop_service(self, client, label: str, key: str, *, clear_draft: bool) -> None:
        if not client.service_is_ready():
            self._publish_status("unavailable", f"{label} service unavailable", key=key)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda res: self._on_stop_reply(res, label, key, clear_draft=clear_draft)
        )

    def _on_stop_reply(self, future, label: str, key: str, *, clear_draft: bool) -> None:
        try:
            response = future.result()
            if response.success:
                if clear_draft:
                    self._active_draft = None
                self._publish_status("stopped", response.message or f"{label} ok", key=key)
            else:
                self._publish_status("rejected", response.message or f"{label} rejected", key=key)
        except Exception as exc:
            self._publish_status("failed", str(exc), key=key)

    def _call_trigger_async(self, client, label: str, key: str) -> None:
        if not client.service_is_ready():
            self._publish_status("unavailable", f"{label} service unavailable", key=key)
            return
        generation = self._step_generation
        deadline = time.monotonic() + 2.0
        future = client.call_async(Trigger.Request())
        self._service_pending = (future, generation, deadline, label)
        future.add_done_callback(
            lambda res: self._on_trigger_reply(res, generation, deadline, label, key)
        )

    def _on_trigger_reply(self, future, generation: int, deadline: float, label: str, key: str) -> None:
        if generation != self._step_generation:
            return
        self._service_pending = None
        if time.monotonic() > deadline:
            self._publish_status("rejected", f"{label} timed out", key=key)
            return
        try:
            response = future.result()
            if response.success:
                self._publish_status("active", response.message or label, key=key)
            else:
                self._publish_status("rejected", response.message or f"{label} rejected", key=key)
        except Exception as exc:
            self._publish_status("failed", str(exc), key=key)

    def _call_impedance_async(self, candidate: ImpedanceTuningDraft, label: str, key: str) -> None:
        if not self._impedance_start_client.service_is_ready():
            self._publish_status("unavailable", "joint_impedance/start service unavailable", key=key)
            return
        request = SetJointImpedance.Request()
        request.positions = list(candidate.positions)
        request.stiffness = list(candidate.stiffness)
        request.damping = list(candidate.damping)
        generation = self._step_generation
        deadline = time.monotonic() + 2.0
        future = self._impedance_start_client.call_async(request)
        self._service_pending = (future, generation, deadline, label)
        future.add_done_callback(
            lambda res: self._on_impedance_reply(res, generation, deadline, candidate, label, key)
        )

    def _on_impedance_reply(
        self,
        future,
        generation: int,
        deadline: float,
        candidate: ImpedanceTuningDraft,
        label: str,
        key: str,
    ) -> None:
        if generation != self._step_generation:
            return
        self._service_pending = None
        if time.monotonic() > deadline:
            self._publish_status("rejected", f"{label} timed out", key=key)
            return
        try:
            response = future.result()
            if response.success:
                self._active_draft = candidate
                self._publish_status("active", label, key=key)
            else:
                self._publish_status("rejected", response.message or "joint impedance rejected", key=key)
        except Exception as exc:
            self._publish_status("failed", str(exc), key=key)

    def _print_telemetry(self, key: str) -> None:
        text = format_telemetry(
            joint_names=self._joint_names,
            state_machine=self._arm_status.state_machine if self._arm_status is not None else None,
            arm_mode=self._arm_status.mode if self._arm_status is not None else None,
            enabled=self._arm_status.enabled if self._arm_status is not None else None,
            current_positions=self._current_positions,
            current_velocities=self._current_velocities,
            current_efforts=self._current_efforts,
            draft=self._active_draft,
        )
        self.get_logger().info("\n" + text)
        self._publish_status(
            "active" if self._active_draft else "idle",
            "telemetry printed",
            key=key,
            log=False,
        )

    def _read_key(self) -> str | None:
        """非阻塞读取一个字符;无输入或读取异常时返回 None(不消耗输入缓冲)."""
        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                return sys.stdin.read(1)
        except Exception:
            return None
        return None

    def _send_target(
        self,
        joint_names: tuple[str, ...],
        positions: tuple[float, ...],
        message: str,
        key: str,
    ) -> None:
        """把一次按键结果生成完整约束并发出,并占用单步状态直到 Action 终态.

        只等 0.05 s 服务端:控制器不在线时立刻报 unavailable 并丢弃本次目标,
        不排队,不阻塞定时器,避免按键积压成突然的大幅运动.
        目标被接受只发布 active;后续结果回调才区分 done/failed 并解除本地 busy.
        """
        if self._busy:
            self._publish_status('rejected', 'keyboard step busy; input discarded', key=key)
            return
        try:
            self._capability.require()
            if time.monotonic()-self._feedback_at > .15:
                raise ValueError('fresh joint feedback required')
        except ValueError as exc:
            self._publish_status('rejected', str(exc), key=key)
            return
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            self._publish_status("unavailable", "follow_joint_trajectory action unavailable")
            return
        # 前端不实现插值数学;运动层给出完整约束,后端准备后逐周期采样.
        from rebotarm_motion.trajectory_messages import point_to_point_message
        trajectory = point_to_point_message(joint_names,
            tuple(self._current_positions[name] for name in joint_names), positions,
            self._trajectory_duration, trajectory_factory=JointTrajectory,
            point_factory=JointTrajectoryPoint)
        if not self._retime_client.service_is_ready():
            self._publish_status('unavailable', 'trajectory retiming service unavailable', key=key)
            return
        request = self._retime_type.Request()
        request.trajectory, request.fixed_duration = trajectory, True
        request.velocity_scaling = request.acceleration_scaling = 1.
        self._busy = True
        generation = self._step_generation
        deadline = time.monotonic()+2.
        future = self._retime_client.call_async(request)
        self._retime_pending = (future, generation, deadline)
        future.add_done_callback(lambda result:self._retimed_step(result,generation,deadline,message,key))

    def _retimed_step(self, future, generation, deadline, message, key):
        """固定时长只检查不延长;异步等待不阻塞反馈,旧按键响应不能触发发送."""
        try:
            self._retime_pending = None
            if generation != self._step_generation or time.monotonic() > deadline:
                raise ValueError('keyboard planning canceled or exceeded 2 s')
            result = future.result()
            if not result.success or not result.executable:
                raise ValueError(result.message if not result.success else 'enable and re-plan keyboard step')
            self._dispatch_step(result.trajectory,message,key)
        except Exception as exc:
            self._busy = False
            self._publish_status('rejected', str(exc), key=key)

    def _dispatch_step(self, trajectory, message, key):
        """只发送原最终消息;后端仍原子核验会话与代次."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._busy = True
        try:
            future = self._action_client.send_goal_async(goal)
            future.add_done_callback(lambda value: self._goal_response(value, message, key))
        except Exception as exc:
            self._busy = False
            self._publish_status('failed', str(exc), key=key)

    def _goal_response(self, future, message, key):
        """处理接纳或拒绝;接纳后仍保持 busy,避免运动中连续按键形成排队轨迹."""
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                raise ValueError('keyboard trajectory goal rejected')
            self._publish_status('active', message, key=key)
            handle.get_result_async().add_done_callback(lambda value: self._goal_result(value, key))
        except Exception as exc:
            self._busy = False
            self._publish_status('rejected', str(exc), key=key)

    def _goal_result(self, future, key):
        """释放本地单步占用,并同时核对 ROS 终态及 FJT 错误码."""
        self._busy = False
        try:
            response = future.result()
            state = 'done' if response.status == 4 and response.result.error_code == 0 else 'failed'
            self._publish_status(state, response.result.error_string, key=key)
        except Exception as exc:
            self._publish_status('failed', str(exc), key=key)

    def _publish_status(
        self,
        state: str,
        message: str,
        *,
        key: str | None = None,
        log: bool = True,
    ) -> None:
        """发布一条 JSON 状态并在终端输出对应 INFO 反馈;字典键名与 state 取值是面板依赖的对外契约."""
        msg = String()
        payload = {
            "source": "keyboard",
            "state": state,
            "message": message,
            "last_key": key,
            "joint_positions": [self._current_positions[name] for name in self._joint_names],
        }
        # 紧凑分隔符:去掉空格,减小高频状态消息体积.
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)
        if log and state != "timeout":
            key_label = "space" if key == " " else repr(key) if key is not None else "-"
            self.get_logger().info(f"[keyboard] key={key_label} state={state}: {message}")

    def destroy_node(self) -> bool:
        """退出前把终端属性恢复原状;tcsetattr 失败也不能跳过父类的资源释放."""
        try:
            if self._terminal_settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    """节点入口:初始化 → spin → 退出时保证 destroy_node/shutdown 都会执行."""
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
