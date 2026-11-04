"""机械臂与夹爪状态发布器.

职责与位置
    将所有者的缓存快照转换为 ROS 状态, 不发起任何设备读写.

对外接口
    JointStatePublisher 发布聚合六轴, 电机, 夹爪及锁存 ArmStatus.

安全与并发约束
    同一反馈身份沿用源时间映射; 状态心跳时间不代表电机产生了新反馈.
    非阻塞发布锁防止堆积, 状态锁保护定时器与服务回调并发发布.

文件结构(按定义顺序)
    JointStatePublisher: 按固定速率发布关节, 电机, 夹爪与机械臂状态话题.
"""

from __future__ import annotations

import logging
import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

from ..contracts.ros_ports import RuntimeStatePort, LifecycleStatePort, JointFeedbackPort, GripperStatePort


class JointStatePublisher:
    """按固定速率发布关节, 电机, 夹爪与机械臂状态话题.

    构造时创建全部发布器并立即发布一次机械臂状态(锁存), 之后由定时器周期性调用
    ``publish``. 对外话题:

    - ``joint_states``: 传感器 QoS 的聚合 6 轴反馈, 位置(rad)/速度(rad/s)/力矩;
    - ``joints/<name>/state``: 每关节单独一路电机状态, 含电机状态码;
    - ``arm_status``: 锁存且可靠, 晚加入的订阅者立刻拿到最后一帧;
    - ``gripper/state``: 仅当硬件配置了夹爪时才创建.

    线程模型: ``publish`` 用非阻塞锁串行化, 重入时跳过本轮; 状态发布用独立锁
    串行化定时器与服务回调. 状态变化立即发布, 未变化时每 0.2 s 发一次心跳.
    """

    def __init__(self, node, namespace: str, rate_hz: float, *, runtime: RuntimeStatePort,
                 lifecycle: LifecycleStatePort, feedback: JointFeedbackPort, gripper: GripperStatePort) -> None:
        """创建缓存发布器, 独立发布/状态锁及心跳定时器, 立即发布一次锁存状态; 不访问设备."""
        self._node = node
        self._runtime, self._lifecycle = runtime, lifecycle
        self._feedback, self._gripper = feedback, gripper
        # 反馈批次身份(各关节的接收序号元组)与首次观测到该批次时的 ROS 时间戳:
        # 同一批次被重复发布时沿用旧时间戳, 避免"数据没变但时间在走".
        self._last_feedback_identity = None
        self._last_feedback_stamp = None
        self._last_gripper_identity = None
        self._last_gripper_stamp = Time()
        # 非阻塞发布锁: 重入回调组下定时器与外部调用可能并发进入 publish.
        self._publish_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._last_status_identity = None
        self._last_status_published_at = None
        self._status_heartbeat_sec = 0.2
        self._publisher = node.create_publisher(
            JointState,
            f"/{namespace}/joint_states",
            node.sensor_qos,
            callback_group=node.reentrant_group,
        )
        self._joint_state_publishers = {
            name: node.create_publisher(
                JointMotorState,
                f"/{namespace}/joints/{name}/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
            for name in runtime.joint_names
        }
        # arm_status 用锁存 QoS: 晚加入的订阅者(例如 Web 状态面板)立即收到最后一帧,
        # 不必等到状态变化; 深度 1 只保留最新状态, 避免旧故障状态反复回放.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = node.create_publisher(
            ArmStatus,
            f"/{namespace}/arm_status",
            latched_qos,
            callback_group=node.reentrant_group,
        )
        self._gripper_state_publisher = None
        if gripper.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        # 发布频率下限 1 Hz: 参数为 0 或负数时兜住除法, 发布周期最长 1 s.
        period = 1.0 / max(float(rate_hz), 1.0)
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.reentrant_group,
        )
        # 建好即发布一次: 锁存语义保证订阅者随时都能读到机械臂状态.
        self.publish_status()

    def publish(self) -> None:
        """定时器回调入口: 抢到锁才发布, 抢不到就放弃本轮."""
        if not self._publish_lock.acquire(blocking=False):
            return
        try:
            self._publish_feedback()
        finally:
            self._publish_lock.release()

    def _publish_feedback(self) -> None:
        """读取同一六轴缓存快照, 按样本身份保留源时间并发布; 失败仍更新夹爪与状态诊断."""
        try:
            # 使能和失能期间均由运行时采集; 发布不能主动刷新总线.
            # 一次原子读取拿到"最近一批已校验反馈"及其接收序号, 两者必须配套使用.
            pos, vel, effort, status_codes, sequences, observed_at = self._feedback.get_cached_joint_sample()
            identity = (sequences, observed_at)
        except Exception as exc:
            self._node.get_logger().warn(f"joint state read failed: {exc}")
            # 不把陈旧的关节位置重新打时间戳当作当前值, 但仍刷新锁存状态, 让 Web UI
            # 报告通信故障, 而不是看起来停在健康的最后一帧.
            self._publish_gripper_feedback()
            self.publish_status(force=False)
            return

        msg = JointState()
        if identity != self._last_feedback_identity:
            self._last_feedback_identity = identity
            self._last_feedback_stamp = self._feedback_stamp(observed_at)
            logging.getLogger('rebotarm.audit').info('joint_source', extra={'audit': dict(
                sequences=sequences, observed_at=observed_at,
                stamp_ns=self._last_feedback_stamp.sec * 1_000_000_000 + self._last_feedback_stamp.nanosec)})
        # 同一批已校验反馈被重复发布时, 不应看起来像一份新样本.
        msg.header.stamp = self._last_feedback_stamp
        msg.name = self._runtime.joint_names
        msg.position = [float(v) for v in pos]
        msg.velocity = [float(v) for v in vel]
        msg.effort = [float(v) for v in effort]
        self._publisher.publish(msg)

        for i, name in enumerate(self._runtime.joint_names):
            motor_msg = JointMotorState()
            motor_msg.header = msg.header
            motor_msg.joint_name = name
            motor_msg.position = float(pos[i])
            motor_msg.velocity = float(vel[i])
            motor_msg.torque = float(effort[i])
            # 电机状态码由驱动给出: 失能时 0, 使能后 1; 255 表示取不到样本(不可用).
            motor_msg.status_code = int(status_codes[i])
            self._joint_state_publishers[name].publish(motor_msg)

        self._publish_gripper_feedback()

        # 失败和成功路径都检查状态变化: 恢复时立即覆盖锁存的 255/stale,
        # 稳定时只发低频心跳, 不随 joint_states 以 100 Hz 重复发相同状态.
        self.publish_status(force=False)

    def _feedback_stamp(self, observed_at: float) -> Time:
        """把缓存的单调时钟年龄映射到 ROS 时间, 保留首次发布前的等待时间."""
        stamp = self._node.get_clock().now().to_msg()
        age_ns = int(max(time.monotonic() - observed_at, 0.0) * 1_000_000_000)
        stamp_ns = max(stamp.sec * 1_000_000_000 + stamp.nanosec - age_ns, 0)
        return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)

    def _publish_gripper_feedback(self) -> None:
        """独立发布夹爪缓存身份, 米制开口和可用性; 六轴故障不伪造夹爪时间."""
        # 夹爪有独立的样本与健康范围; 六轴失效时仍发布其自身诊断状态.
        if self._gripper_state_publisher is None:
            return
        position, velocity, torque, status, sequence, observed_at = self._gripper.get_cached_gripper_sample()
        identity = (sequence, observed_at)
        if identity != self._last_gripper_identity:
            self._last_gripper_identity = identity
            self._last_gripper_stamp = Time() if observed_at is None else self._feedback_stamp(observed_at)
        msg = JointMotorState()
        msg.header.stamp = self._last_gripper_stamp
        msg.joint_name = "gripper"
        msg.position = float(position)
        msg.velocity = float(velocity)
        msg.torque = float(torque)
        msg.status_code = int(status)
        self._gripper_state_publisher.publish(msg)

    def publish_status(self, *, force: bool = True) -> None:
        """发布锁存的机械臂运行状态(模式, 使能位, 控制环占用, 状态机, 错误码).

        定时器使用 ``force=False``: 内容变化立即发布, 否则最多 5 Hz 心跳.
        服务/动作回调使用默认 ``force=True``, 请求立即发布本次操作后的状态.
        仅读取对应所有者的状态与反馈缓存, 不触发串口事务.
        """
        with self._status_lock:
            msg = ArmStatus()
            msg.mode = self._runtime.mode
            msg.enabled = self._lifecycle.enabled
            msg.control_loop_active = self._runtime.control_loop_active
            msg.state_machine = self._lifecycle.execution
            msg.joint_names = list(self._runtime.joint_names)
            statuses, errors = self._feedback.get_feedback_status()
            msg.per_joint_status_code = statuses
            msg.error_codes = errors
            identity = (
                msg.mode,
                msg.enabled,
                msg.control_loop_active,
                msg.state_machine,
                tuple(msg.joint_names),
                tuple(msg.per_joint_status_code),
                tuple(msg.error_codes),
            )
            now = time.monotonic()
            if (
                not force
                and identity == self._last_status_identity
                and self._last_status_published_at is not None
                and now - self._last_status_published_at < self._status_heartbeat_sec
            ):
                return
            msg.header.stamp = self._node.get_clock().now().to_msg()
            self._status_publisher.publish(msg)
            self._last_status_identity = identity
            self._last_status_published_at = now
