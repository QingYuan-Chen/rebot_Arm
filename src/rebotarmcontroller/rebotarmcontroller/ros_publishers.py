"""机械臂与夹爪状态发布器。

职责：把硬件管理器维护的**已校验反馈缓存**转成 ROS 话题对外发布——聚合关节状态、
每关节电机状态、夹爪状态，以及锁存（latched）的机械臂运行状态。发布由定时器按
关节状态发布速率（默认 100 Hz）驱动。

与硬件环的分工（安全要点）：

- 本模块只"读缓存 + 发布"，绝不发起同步串口事务。使能后总线由配置频率的统一硬件
  控制环独占，定时器退化为纯缓存读取，避免与命令写入者抢总线；
- 反馈读取失败时不重新打时间戳：宁可不发新帧，也不把陈旧位置伪装成当前样本，
  同时检查并发布状态变化，让上层看到通信故障而不是"停在健康的最后一帧"；
- 上一轮发布未结束（回调组可重入，定时器与外部调用可能并发）时直接跳过本轮，
  不排队，避免状态话题积压。
"""

from __future__ import annotations

import threading
import time

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState


class JointStatePublisher:
    """按固定速率发布关节、电机、夹爪与机械臂状态话题。

    构造时创建全部发布器并立即发布一次机械臂状态（锁存），之后由定时器周期性调用
    ``publish``。对外话题：

    - ``joint_states``：传感器 QoS 的聚合 6 轴反馈，位置（rad）/速度（rad/s）/力矩；
    - ``joints/<name>/state``：每关节单独一路电机状态，含电机状态码；
    - ``arm_status``：锁存且可靠，晚加入的订阅者立刻拿到最后一帧；
    - ``gripper/state``：仅当硬件配置了夹爪时才创建。

    线程模型：``publish`` 用非阻塞锁串行化，重入时跳过本轮；状态发布用独立锁
    串行化定时器与服务回调。状态变化立即发布，未变化时每 0.2 s 发一次心跳。
    """

    def __init__(self, node, hardware, namespace: str, rate_hz: float) -> None:
        self._node = node
        self._hardware = hardware
        # 反馈批次身份（各关节的接收序号元组）与首次观测到该批次时的 ROS 时间戳：
        # 同一批次被重复发布时沿用旧时间戳，避免"数据没变但时间在走"。
        self._last_feedback_identity = None
        self._last_feedback_stamp = None
        # 非阻塞发布锁：重入回调组下定时器与外部调用可能并发进入 publish。
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
            for name in hardware.joint_names
        }
        # arm_status 用锁存 QoS：晚加入的订阅者（例如 Web 状态面板）立即收到最后一帧，
        # 不必等到状态变化；深度 1 只保留最新状态，避免旧故障状态反复回放。
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
        if hardware.has_gripper:
            self._gripper_state_publisher = node.create_publisher(
                JointMotorState,
                f"/{namespace}/gripper/state",
                node.sensor_qos,
                callback_group=node.reentrant_group,
            )
        # 发布频率下限 1 Hz：参数为 0 或负数时兜住除法，发布周期最长 1 s。
        period = 1.0 / max(float(rate_hz), 1.0)
        self._timer = node.create_timer(
            period,
            self.publish,
            callback_group=node.reentrant_group,
        )
        # 建好即发布一次：锁存语义保证订阅者随时都能读到机械臂状态。
        self.publish_status()

    def publish(self) -> None:
        """定时器回调入口：抢到锁才发布，抢不到就放弃本轮。"""
        if not self._publish_lock.acquire(blocking=False):
            return
        try:
            self._publish_feedback()
        finally:
            self._publish_lock.release()

    def _publish_feedback(self) -> None:
        try:
            # 使能时，配置频率的统一硬件循环独占总线，本调用是空操作；失能时没有命令
            # 写入者，它自己执行同样限速的 50 Hz 批量刷新。
            self._hardware.refresh_feedback_if_due()
            # 一次原子读取拿到"最近一批已校验反馈"及其接收序号，两者必须配套使用。
            pos, vel, effort, status_codes, identity = self._hardware.get_cached_joint_sample()
        except Exception as exc:
            self._node.get_logger().warn(f"joint state read failed: {exc}")
            # 不把陈旧的关节位置重新打时间戳当作当前值，但仍刷新锁存状态，让 Web UI
            # 报告通信故障，而不是看起来停在健康的最后一帧。
            self.publish_status(force=False)
            return

        msg = JointState()
        if identity != self._last_feedback_identity:
            self._last_feedback_identity = identity
            self._last_feedback_stamp = self._node.get_clock().now().to_msg()
        # 同一批已校验反馈被重复发布时，不应看起来像一份新样本。
        msg.header.stamp = self._last_feedback_stamp
        msg.name = self._hardware.joint_names
        msg.position = [float(v) for v in pos]
        msg.velocity = [float(v) for v in vel]
        msg.effort = [float(v) for v in effort]
        self._publisher.publish(msg)

        for i, name in enumerate(self._hardware.joint_names):
            motor_msg = JointMotorState()
            motor_msg.header = msg.header
            motor_msg.joint_name = name
            motor_msg.position = float(pos[i])
            motor_msg.velocity = float(vel[i])
            motor_msg.torque = float(effort[i])
            # 电机状态码由驱动给出：失能时 0、使能后 1；255 表示取不到样本（不可用）。
            motor_msg.status_code = int(status_codes[i])
            self._joint_state_publishers[name].publish(motor_msg)

        if self._gripper_state_publisher is not None:
            g_pos, g_vel, g_torque, g_status = self._hardware.get_gripper_state()
            gripper_msg = JointMotorState()
            gripper_msg.header = msg.header
            gripper_msg.joint_name = "gripper"
            # 位置字段单独取 gripper_position_m()（米制开合距离，坐标校验失败时为 NaN），
            # 而不是直接用上面的电机角度 g_pos。
            gripper_msg.position = float(self._hardware.gripper_position_m())
            gripper_msg.velocity = float(g_vel)
            gripper_msg.torque = float(g_torque)
            gripper_msg.status_code = int(g_status)
            self._gripper_state_publisher.publish(gripper_msg)

        # 失败和成功路径都检查状态变化：恢复时立即覆盖锁存的 255/stale，
        # 稳定时只发低频心跳，不随 joint_states 以 100 Hz 重复发相同状态。
        self.publish_status(force=False)

    def publish_status(self, *, force: bool = True) -> None:
        """发布锁存的机械臂运行状态（模式、使能位、控制环占用、状态机、错误码）。

        定时器使用 ``force=False``：内容变化立即发布，否则最多 5 Hz 心跳。
        服务/动作回调使用默认 ``force=True``，保证操作结果及时可见。
        仅读取硬件管理器快照，不触发串口事务。
        """
        with self._status_lock:
            msg = ArmStatus()
            msg.mode = self._hardware.mode
            msg.enabled = self._hardware.enabled
            msg.control_loop_active = self._hardware.control_loop_active
            msg.state_machine = self._hardware.state_machine
            msg.joint_names = list(self._hardware.joint_names)
            msg.per_joint_status_code = self._hardware.get_joint_status_codes()
            msg.error_codes = list(self._hardware.error_codes)
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
