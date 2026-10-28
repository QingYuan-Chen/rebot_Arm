"""标定图像、内参与 TF 的独立单线程接收。

接收回调只更新有界缓存；检测、稳定性判断、会话落盘和服务等待由标定节点拥有。
snapshot 复制消息并保留采集时间；configure 经 guard condition 在接收线程执行，
新订阅就绪后才交换代次并清空缓存，旧回调不能污染新话题。关闭先结束线程，
再注销 TF 和销毁节点，不能在等待集中销毁仍被使用的订阅。
"""
from collections import deque
from concurrent.futures import Future
from copy import deepcopy
from dataclasses import dataclass
from queue import Empty, Queue
import threading

from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


@dataclass(frozen=True)
class CaptureSnapshot:
    image: object
    infos: tuple
    generation: int


class CaptureInputReceiver(Node):
    def __init__(self, owner):
        name = owner.get_name() + '_inputs'
        super().__init__(name, context=owner.context, namespace=owner.get_namespace(),
                         use_global_arguments=False, cli_args=['--ros-args',
                             '-r', '/tf:=' + owner.resolve_topic_name('/tf'),
                             '-r', '/tf_static:=' + owner.resolve_topic_name('/tf_static'),
                             '-r', '/clock:=' + owner.resolve_topic_name('/clock')],
                         parameter_overrides=[Parameter('use_sim_time', value=
                             owner.get_parameter('use_sim_time').value)])
        self.lock = threading.Lock()
        self.frames = deque(maxlen=40)
        self.infos = deque(maxlen=40)
        self.generation = 0
        self.topics = None
        self.error = ''
        self._camera_subscriptions = []
        self._requests = Queue(maxsize=4)
        self._configure_guard = self.create_guard_condition(self._configure_pending)
        self._resolve_topic = owner.resolve_topic_name
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self._closed = False
        self._input_executor = SingleThreadedExecutor(context=owner.context)
        self._input_executor.add_node(self)
        self.thread = threading.Thread(target=self._spin, name=name, daemon=True)
        self.thread.start()

    def configure(self, topics):
        if self._closed or not self.thread.is_alive():
            raise RuntimeError('calibration input receiver is stopped')
        future = Future()
        self._requests.put_nowait((tuple(self._resolve_topic(t) for t in topics), future))
        self._configure_guard.trigger()
        try:
            future.result(timeout=3.)
        except Exception:
            future.cancel()
            raise

    def _configure_pending(self):
        while True:
            try:
                topics, future = self._requests.get_nowait()
            except Empty:
                return
            if not future.set_running_or_notify_cancel():
                continue
            try:
                self._configure(topics)
                future.set_result(None)
            except Exception as exc:
                future.set_exception(exc)

    def _callback(self, generation, kind):
        def receive(message):
            with self.lock:
                if generation == self.generation:
                    (self.frames if kind == 'image' else self.infos).append(message)
        return receive

    def _configure(self, topics):
        if topics == self.topics:
            return
        generation = self.generation + 1
        new_subscriptions = []
        try:
            for message, topic, kind in zip((Image, CameraInfo), topics, ('image', 'info')):
                new_subscriptions.append(self.create_subscription(message, topic,
                    self._callback(generation, kind), qos_profile_sensor_data))
        except Exception:
            for subscription in new_subscriptions:
                self.destroy_subscription(subscription)
            raise
        with self.lock:
            self.generation = generation
            self.frames.clear()
            self.infos.clear()
        old = self._camera_subscriptions
        self._camera_subscriptions, self.topics = new_subscriptions, topics
        for subscription in old:
            self.destroy_subscription(subscription)

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError('calibration input receiver failed: ' + self.error)
            snapshot = CaptureSnapshot(self.frames[-1] if self.frames else None,
                                       tuple(self.infos), self.generation)
        return deepcopy(snapshot)

    def _spin(self):
        try:
            self._input_executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            self.get_logger().error('标定输入线程退出：' + str(exc))

    def close(self):
        if self._closed:
            return
        self._input_executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=3.)
        if self.thread.is_alive():
            raise RuntimeError('calibration input receiver did not stop; preserving ROS handles')
        self._input_executor.remove_node(self)
        self.listener.unregister()
        self.destroy_node()
        self._closed = True
