"""抓取执行端的计划、候选与 TF 接收。

专用单线程执行器只更新输入邮箱和 TF Buffer，不访问执行状态锁或下游服务。
邮箱只保留每类最新消息及最近一次空候选撤销，避免限界缓存吞掉撤销事件。
业务线程通过 take 取得消息所有权；停止期间丢弃输入，确认停止后须接收新计划。
原始采集时间和默认 QoS 事件保持，规划/TF 查询与日志不放入接收回调。
"""
import threading

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rebotarm_msgs.msg import GraspCandidateArray, GraspPlan
from tf2_ros import Buffer, TransformListener


class GraspInputMailbox:
    def __init__(self):
        self.lock = threading.Lock()
        self.sequence = 0
        self.pending = {}
        self.blocked = False
        self.error = ''

    def receive(self, kind, message):
        with self.lock:
            if self.blocked:
                return
            self.sequence += 1
            event = (self.sequence, kind, message)
            self.pending[kind] = event
            if kind == 'candidates' and not message.candidates:
                self.pending['revoke'] = event

    def take(self):
        with self.lock:
            if self.error:
                raise RuntimeError('grasp input receiver failed: ' + self.error)
            # 同一条空候选可能占两格，只交接一次，并保留不同消息的接收次序。
            events = sorted({event[0]: event for event in self.pending.values()}.values())
            self.pending.clear()
            return events

    def block(self, blocked):
        with self.lock:
            self.blocked = blocked
            self.pending.clear()


class GraspInputReceiver(Node):
    def __init__(self, owner, plan_topic, candidates_topic):
        name = owner.get_name() + '_inputs'
        super().__init__(name, context=owner.context, namespace=owner.get_namespace(),
                         use_global_arguments=False, cli_args=['--ros-args',
                             '-r', '/tf:=' + owner.resolve_topic_name('/tf'),
                             '-r', '/tf_static:=' + owner.resolve_topic_name('/tf_static'),
                             '-r', '/clock:=' + owner.resolve_topic_name('/clock')],
                         parameter_overrides=[Parameter('use_sim_time', value=
                             owner.get_parameter('use_sim_time').value)])
        self.mailbox = GraspInputMailbox()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(GraspPlan, owner.resolve_topic_name(plan_topic),
                                 self._plan, 10)
        self.create_subscription(GraspCandidateArray, owner.resolve_topic_name(candidates_topic),
                                 self._candidates, 10)
        self._closed = False
        self._input_executor = SingleThreadedExecutor(context=owner.context)
        self._input_executor.add_node(self)
        self.thread = threading.Thread(target=self._spin, name=name, daemon=True)
        self.thread.start()

    def _plan(self, message):
        self.mailbox.receive('plan', message)

    def _candidates(self, message):
        self.mailbox.receive('candidates', message)

    def _spin(self):
        try:
            self._input_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            with self.mailbox.lock:
                self.mailbox.error = str(exc)
            self.get_logger().error('抓取输入线程退出：' + str(exc))

    def close(self):
        if self._closed:
            return
        self._input_executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            raise RuntimeError('grasp input receiver did not stop; preserving ROS handles')
        self._input_executor.remove_node(self)
        self.listener.unregister()
        self.destroy_node()
        self._closed = True
