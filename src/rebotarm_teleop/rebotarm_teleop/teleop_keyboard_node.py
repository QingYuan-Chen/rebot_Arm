"""键盘遥操作节点:把终端单键输入翻译成关节增量轨迹目标.

在系统中的位置:属于"操作者意图 → ROS 命令"的适配层,只负责读键,限位,组轨迹,
关节限位之外的保护(碰撞,回放质量等)由运动层与硬件层负责.

工作方式:以 ``poll_period`` 为周期轮询终端(非阻塞),每个有效按键由
``KeyboardCommandMapper`` 映射为"某关节 ±1 步",再由 ``TeleopTargetPlanner``
在当前反馈角上叠加 ``joint_step_rad`` 并夹紧到关节限位,再发送 FJT.
运动层生成完整五次首末约束,默认单步 0.02 rad / 0.4 s.

对外话题/动作(``<ns>`` 取自参数 arm_namespace,默认 rebotarm):
- 动作 ``/<ns>/follow_joint_trajectory``:下发目标关节角;
- 订阅 ``/<ns>/joint_states``:取当前角作为增量基准,使用 BEST_EFFORT 传感器 QoS;
- 发布 ``/<ns>/teleop/status``:JSON 字符串,字段 source/state/message/last_key/joint_positions,
  供操作面板显示当前遥操作状态,其取值与文本是对外契约.

状态机(发布在 status 的 state 字段):idle → active;按下空格为 stopped;
配置 deadman_required 时,未在 input_timeout 内按过 deadman_key 的按键会被 blocked,
只有 deadman_key 能刷新授权时间窗;超时未按键回落到 timeout,此后按键需重新按 deadman_key;
动作服务端在 50 ms 内不可达则 unavailable,校验失败为 rejected;Action 成功终结后发布 done.

安全约束:终端被切到 cbreak 模式以逐字符读取输入,退出时必须恢复原终端设置;
所有目标角都经关节限位夹紧;请求前核对执行能力和完整六轴反馈的本地接收年龄.
反馈接收及时不代替后端源数据验证;忙时明确丢弃本次按键,不累积下一步目标.

文件结构:节点参数与终端设置 → 反馈/读键 → 规划绑定与单步发送 → 终态回调,状态发布和退出.
"""

from __future__ import annotations

import json
import select
import sys
import termios
import time
import tty
from contextlib import suppress

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .parameter_helpers import build_joint_limits, sensor_qos_kwargs
from .teleop_core import KeyboardCommandMapper, TeleopTargetPlanner


class TeleopKeyboardNode(Node):
    """键盘遥操作节点:单线程 executor 下由定时器驱动按键轮询与动作下发.

    参数(全部由上层 yaml 覆盖,默认值对应"无死锁键保护,6 关节 ±π 限位"):
    - ``arm_namespace``:话题/动作命名空间,前后斜杠会被去掉;
    - ``joint_names`` / ``joint_lower_limits`` / ``joint_upper_limits``:平行数组参数,
      经 ``build_joint_limits`` 组装成关节名 → (下限, 上限) 映射,单位 rad;
    - ``joint_step_rad``:单次按键的关节增量(rad),默认 0.02;
    - ``trajectory_duration``:单步完整轨迹时长(s),默认 0.4,越小动作越急;
    - ``poll_period``:键盘轮询周期(s),默认 0.05;
    - ``input_timeout``:按键静默判超时,以及死锁键授权窗口的时长(s),默认 0.5;
    - ``deadman_required`` / ``deadman_key``:是否要求先按住式确认键,默认 false / "z".

    动作结果只用于发布状态,不做重试:一次按键即一次目标,不会累积队列.
    """

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
        self._feedback_at = 0.
        self.declare_parameter("poll_period", 0.05)
        self.declare_parameter("input_timeout", 0.5)
        self.declare_parameter("deadman_required", False)
        self.declare_parameter("deadman_key", "z")

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
        # monotonic 时间戳:死锁授权截止时刻与最近一次有效输入时刻,均不受系统时钟调整影响.
        self._deadman_until = 0.0
        self._last_input_time = 0.0
        # 关节角基准值,初值 0.0;收到 joint_states 后被实时反馈覆盖,只认参数里声明过的关节名.
        self._current_positions = {name: 0.0 for name in self._joint_names}
        self._terminal_settings = None
        self._mapper = KeyboardCommandMapper(joint_names=self._joint_names)
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
        self._status_pub = self.create_publisher(
            String,
            f"/{self._arm_namespace}/teleop/status",
            10,
        )
        from rclpy.qos import DurabilityPolicy
        self.create_subscription(String, f'/{self._arm_namespace}/trajectory_execution_mode',
            self._capability.update, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
        try:
            if sys.stdin.isatty():
                self._terminal_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
        except Exception as exc:
            self.get_logger().warn(f"keyboard raw mode unavailable: {exc}")

        self.create_timer(float(self.get_parameter("poll_period").value), self._poll_key)
        self._publish_status("idle", "keyboard teleop ready")

    def _on_joint_state(self, msg: JointState) -> None:
        """用最新关节反馈刷新增量基准;未声明或长度不足的关节保持原值."""
        for name, position in zip(msg.name, msg.position):
            if name in self._current_positions:
                self._current_positions[str(name)] = float(position)
        if set(self._joint_names).issubset(msg.name) and len(msg.position) >= len(msg.name):
            self._feedback_at = time.monotonic()

    def _poll_key(self) -> None:
        """定时器回调:读取一个按键并完成"死锁检查 → 映射 → 限位 → 下发"流程."""
        if self._retime_pending is not None:
            future, generation, deadline = self._retime_pending
            if time.monotonic() >= deadline or generation != self._step_generation:
                self._retime_pending = None
                self._step_generation += 1
                future.cancel()
        key = self._read_key()
        now = time.monotonic()
        if key is None:
            # 长时间无输入只提示一次超时,随后清空时间戳避免反复刷状态.
            if self._last_input_time and now - self._last_input_time > self._input_timeout:
                self._publish_status("timeout", "keyboard input timeout")
                self._last_input_time = 0.0
            return
        self._last_input_time = now
        if key == " ":
            self._step_generation += 1  # 使尚在规划的按键响应失效;不产生后继运动.
            # 空格是软件级停止:本节点不再下发新目标,是否停止现有轨迹由控制器决定.
            self._publish_status("stopped", "keyboard stop requested")
            return
        if key == self._deadman_key:
            # 死锁键把授权窗口向后推 input_timeout,需要周期性重按以保持有效.
            self._deadman_until = now + self._input_timeout
            self._publish_status("deadman", "deadman refreshed")
            return
        if self._deadman_required and now > self._deadman_until:
            self._publish_status("blocked", "deadman key required")
            return
        command = self._mapper.command_for_key(key)
        if command is None:
            return
        target = self._planner.apply_delta(
            current_positions=self._current_positions,
            joint_name=command.joint_name,
            direction=command.direction,
        )
        if not target.accepted:
            self._publish_status("rejected", target.message)
            return
        self._send_target(target.joint_names, target.positions, target.message, key)

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

    def _publish_status(self, state: str, message: str, *, key: str | None = None) -> None:
        """发布一条 JSON 状态;字典键名与 state 取值都是面板依赖的对外契约."""
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
