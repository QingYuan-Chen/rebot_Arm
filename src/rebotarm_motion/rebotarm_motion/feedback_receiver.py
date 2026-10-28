"""运动消费端的独立反馈接收与快照交接。

职责：专用单线程执行器只接收关节、夹爪与控制器状态，不执行运动请求。
接口：snapshot 返回脱离接收缓存的消息副本，stop_feedback 提供原有停止证据。
约束：接收锁不与规划/Action 的业务锁共享；保留默认 QoS 事件及原始源时间。
关节反馈超过 150 ms 或状态中断/模式变化会递增代次，即使恢复后新帧很新鲜，
已开始的位姿请求也不能跨过空档继续。关闭先唤醒并等待线程，再销毁 ROS 句柄。
"""
from copy import deepcopy
from dataclasses import dataclass, replace
import threading
import time

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import JointState

from .stop_feedback import StopFeedback


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


@dataclass(frozen=True)
class MotionFeedbackSnapshot:
    joints: object = None
    gripper: object = None
    status: object = None
    status_at: float = 0.
    epoch: int = 0
    joint_gaps: int = 0
    maximum_joint_age: float = 0.


class MotionFeedbackStore:
    """只保存接收器拥有的消息；快照副本不允许调用方改写缓存。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.value = MotionFeedbackSnapshot()
        self.joint_lost = False
        self.status_lost = False
        self.error = ''
        self.stop_feedback = StopFeedback()

    def inspect(self, received, now_ns):
        """调用方持接收锁；替换旧帧前检查，避免恢复新帧掩盖接收空档。"""
        value = self.value
        if value.joints is not None:
            age = (now_ns - stamp_ns(value.joints)) / 1e9
            lost = not -.05 <= age <= .15
            value = replace(value, maximum_joint_age=max(value.maximum_joint_age, age))
            if lost and not self.joint_lost:
                value = replace(value, epoch=value.epoch + 1, joint_gaps=value.joint_gaps + 1)
            self.joint_lost = lost
        lost = value.status is not None and received - value.status_at > .6
        if lost and not self.status_lost:
            value = replace(value, epoch=value.epoch + 1)
        self.status_lost = lost
        self.value = value

    def receive(self, kind, msg, received, now_ns):
        with self.lock:
            self.inspect(received, now_ns)
            value = self.value
            if kind == 'joints':
                # 消息由 ROS 回调移交，之后只读；StopFeedback 有自己的短事务锁。
                self.stop_feedback.update(msg, received, now_ns)
                self.value = replace(value, joints=msg)
                self.joint_lost = not -.05 <= (now_ns-stamp_ns(msg))/1e9 <= .15
            elif kind == 'gripper':
                self.value = replace(value, gripper=msg)
            elif kind == 'status':
                old = value.status
                changed = old is not None and (old.mode, old.enabled, old.control_loop_active) != (
                    msg.mode, msg.enabled, msg.control_loop_active)
                self.value = replace(value, status=msg, status_at=received,
                                     epoch=value.epoch + int(changed))
                self.status_lost = False
            else:
                raise ValueError(kind)

    def snapshot(self, received, now_ns):
        with self.lock:
            if self.error:
                raise RuntimeError('feedback receiver failed: ' + self.error)
            self.inspect(received, now_ns)
            value = self.value
        return deepcopy(value)


class MotionFeedbackReceiver(Node):
    def __init__(self, owner, namespace, *, gripper=False):
        super().__init__(owner.get_name() + '_feedback', context=owner.context,
                         namespace=owner.get_namespace(), use_global_arguments=False,
                         cli_args=['--ros-args', '-r', '/clock:=' + owner.resolve_topic_name('/clock')],
                         parameter_overrides=[Parameter('use_sim_time', value=
                             owner.get_parameter('use_sim_time').value)])
        self.store = MotionFeedbackStore()
        self.stop_feedback = self.store.stop_feedback
        self._closed = False
        # 仿真时钟也由本接收执行器推进，不能读取可能被同步业务等待阻塞的父节点时钟。
        self._now_ns = lambda: self.get_clock().now().nanoseconds
        prefix = '/' + namespace
        for kind, message, topic, qos in (
            ('joints', JointState, prefix + '/joint_states', qos_profile_sensor_data),
            ('status', ArmStatus, prefix + '/arm_status', 10),
            *([('gripper', JointMotorState, prefix + '/gripper/state', qos_profile_sensor_data)]
              if gripper else []),
        ):
            self.create_subscription(message, owner.resolve_topic_name(topic), self._callback(kind), qos)
        self._feedback_executor = SingleThreadedExecutor(context=owner.context)
        self._feedback_executor.add_node(self)
        self.thread = threading.Thread(target=self._spin, name=self.get_name(), daemon=True)
        self.thread.start()

    def _callback(self, kind):
        def receive(msg):
            self.store.receive(kind, msg, time.monotonic(), self._now_ns())
        return receive

    def _spin(self):
        try:
            self._feedback_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            with self.store.lock:
                self.store.error = str(exc)
            self.get_logger().error('反馈接收线程退出：' + str(exc))

    def snapshot(self):
        return self.store.snapshot(time.monotonic(), self._now_ns())

    def close(self):
        if self._closed:
            return
        self._feedback_executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            raise RuntimeError('feedback receiver did not stop; preserving ROS handles')
        self._feedback_executor.remove_node(self)
        self.destroy_node()
        self._closed = True
