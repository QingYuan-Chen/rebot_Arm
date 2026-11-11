"""运动消费端的独立反馈接收与快照交接。

职责与位置
    运动消费端的独立接收器，让同步规划/Action 等待不占用反馈执行器。

对外接口
    MotionFeedbackStore 管消息与连续性代次；MotionFeedbackReceiver 持单线程执行器；
    snapshot 返回消息深副本，stop_feedback 提供跨线程停止证据。

安全与并发约束
    锁序为接收缓存 -> 停止证据；业务锁不能带入接收回调。 先检查旧帧再替换恢复帧。
    源时间与本地接收时间分开使用；关闭失败保留 ROS 句柄，不在活线程下销毁节点。

文件结构（按定义顺序）
    stamp_ns：读取消息原始源时间戳并换算为整数纳秒，不改写时间。
    MotionFeedbackSnapshot：接收器的一致性快照：消息，状态接收时刻，连续性代次和历史年龄诊断。
    MotionFeedbackStore：只保存接收器拥有的消息；快照副本不允许调用方改写缓存。
    MotionFeedbackReceiver：只接收状态的独立 ROS 节点；自有单线程执行器与线程，不执行业务请求。
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
    """读取消息原始源时间戳并换算为整数纳秒，不改写时间。"""
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


@dataclass(frozen=True)
class MotionFeedbackSnapshot:
    """接收器的一致性快照：消息，状态接收时刻，连续性代次和历史年龄诊断。"""

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
        """建立短事务缓存锁，连续性标记与停止证据；不创建 ROS 句柄。"""
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
            stamp = stamp_ns(value.joints)
            age = (now_ns - stamp) / 1e9
            lost = stamp <= 0 or not -.05 <= age <= .15
            value = replace(value, maximum_joint_age=max(value.maximum_joint_age, age))
            if lost and not self.joint_lost:
                value = replace(value, epoch=value.epoch + 1, joint_gaps=value.joint_gaps + 1)
                self.stop_feedback.invalidate(received)
            self.joint_lost = lost
        lost = value.status is not None and received - value.status_at > .6
        if lost and not self.status_lost:
            value = replace(value, epoch=value.epoch + 1)
            self.stop_feedback.invalidate(received)
        self.status_lost = lost
        self.value = value

    def receive(self, kind, msg, received, now_ns):
        """在接收锁内检查旧缓存及新帧连续性；模式变化也会作废旧停止证据。"""
        with self.lock:
            self.inspect(received, now_ns)
            # 检查旧缓存发生在替换前，才能让恢复后的新帧保留此前空档带来的代次失效。
            value = self.value
            if kind == 'joints':
                # 消息由 ROS 回调移交，之后只读；StopFeedback 有自己的短事务锁。
                self.stop_feedback.update(msg, received, now_ns)
                stamp = stamp_ns(msg)
                age = (now_ns - stamp) / 1e9
                regressed = value.joints is not None and stamp < stamp_ns(value.joints)
                lost = stamp <= 0 or not -.05 <= age <= .15 or regressed
                # 新帧本身也能破坏连续性；即使业务只看到恢复帧，旧动作仍必须失效。
                if lost and not self.joint_lost:
                    value = replace(value, epoch=value.epoch + 1, joint_gaps=value.joint_gaps + 1)
                    self.stop_feedback.invalidate(received)
                self.value = replace(value, joints=msg,
                                     maximum_joint_age=max(value.maximum_joint_age, age))
                self.joint_lost = lost
            elif kind == 'gripper':
                self.value = replace(value, gripper=msg)
            elif kind == 'status':
                old = value.status
                changed = old is not None and (old.mode, old.enabled, old.control_loop_active) != (
                    msg.mode, msg.enabled, msg.control_loop_active)
                if changed or msg.error_codes:
                    self.stop_feedback.invalidate(received)
                self.value = replace(value, status=msg, status_at=received,
                                     epoch=value.epoch + int(changed))
                self.status_lost = False
            else:
                raise ValueError(kind)

    def snapshot(self, received, now_ns):
        """检查接收线程错误与源年龄后取得一致状态，锁外深复制消息以免业务修改缓存。"""
        with self.lock:
            if self.error:
                raise RuntimeError('feedback receiver failed: ' + self.error)
            self.inspect(received, now_ns)
            value = self.value
        return deepcopy(value)


class MotionFeedbackReceiver(Node):
    """只接收状态的独立 ROS 节点；自有单线程执行器与线程，不执行业务请求。"""

    def __init__(self, owner, namespace, *, gripper=False):
        """复制父节点时间与话题配置，注册状态订阅并启动专用接收执行器；不共用业务回调锁。"""
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
        """生成按消息类型分类的短回调，源时间保留在消息内，另记本地接收时刻。"""
        def receive(msg):
            """仅把消息及两类时间交给缓存，不运行规划，Action 等待或设备命令。"""
            self.store.receive(kind, msg, time.monotonic(), self._now_ns())
        return receive

    def _spin(self):
        """自旋专用单线程执行器；异常保存到缓存，后续查询必须能观察到接收器失败。"""
        try:
            self._feedback_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            with self.store.lock:
                self.store.error = str(exc)
            self.get_logger().error('反馈接收线程退出：' + str(exc))

    def snapshot(self):
        """从接收缓存取得独立副本，并按接收器自身时钟检查连续性。"""
        return self.store.snapshot(time.monotonic(), self._now_ns())

    def close(self):
        """先关闭执行器并等待线程退出，再移除节点和释放 ROS 句柄；线程未停则拒绝销毁。"""
        if self._closed:
            return
        self._feedback_executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            raise RuntimeError('feedback receiver did not stop; preserving ROS handles')
        self._feedback_executor.remove_node(self)
        self.destroy_node()
        self._closed = True
