"""设备请求的有界队列与代次检查.

职责与位置
    组件命令边界与唯一设备线程之间的排队层, 集中保存 REQUEST_POLICIES.

对外接口
    device_request 复制可变输入后调用运行时; DeviceRequests 管理请求, TTL, 优先级和资源代次.

主要流程
    组件调用 -> 按策略复制并排队 -> 检查会话/资源代次 -> 执行 -> 事件返回结果.

安全与并发约束
    入队发生在生命周期锁外; 设备线程内的嵌套调用直调, 避免自等死锁.
    TTL 只限制尚未开始的请求; 已经开始的设备事务必须回报实际结果.

文件结构(按定义顺序)
    RequestRuntime: 组件入队所需的运行时能力; 生命周期不接收整个装配门面.
    RequestPolicy: 一个命令入口的调度策略; 关闭回收仅在调用线程确认释放后执行.
    RequestRejected: 请求尚未执行即被拒绝; 没有发生对应的设备写入.
    DeviceRequest: 单次请求记录: 会话/资源代次, 开始期限, 操作, 专属事件及实际结果.
    DeviceRequests: 持锁管理有界队列, 紧急保留容量及资源代次; 不直接执行设备操作.
    device_request: 组件命令入口: 按集中策略复制输入后排队, 设备线程内的嵌套调用直接执行.
"""

from dataclasses import dataclass, field
from functools import wraps
from types import MappingProxyType
from typing import Callable, Protocol
import copy
import math
import threading
import time


class RequestRuntime(Protocol):
    """组件入队所需的运行时能力; 生命周期不接收整个装配门面."""

    @property
    def in_writer(self) -> bool:
        """声明是否处于唯一写入线程, 供嵌套命令避免再次排队."""
        ...

    def call(self, operation: Callable, **request_options):
        """声明按策略排队并返回本请求实际结果的能力."""
        ...

    def close_worker(self):
        """声明已释放设备后的写入线程回收能力."""
        ...


@dataclass(frozen=True)
class RequestPolicy:
    """一个命令入口的调度策略; 关闭回收仅在调用线程确认释放后执行."""

    resource: str
    priority: int = 10
    ttl: float = 2.
    invalidate: tuple[str, ...] = ()
    close_worker: bool = False


# 组件入口与策略在此集中核对; 装饰器直接引用这些冻结值, 禁止另一份默认策略漂移.
REQUEST_POLICIES = MappingProxyType({
    'ControllerLifecycle.connect': RequestPolicy('device', 5, 10., ('device',)),
    'ControllerLifecycle.shutdown': RequestPolicy('device', 5, 10., ('device',), close_worker=True),
    'ControllerLifecycle.enable': RequestPolicy('device', 5, 5., ('device',)),
    'ControllerLifecycle.disable': RequestPolicy('device', 1, 10., ('device',)),
    'ControllerLifecycle.set_mode': RequestPolicy('device', 5, 5., ('device',)),
    'ControllerLifecycle.set_zero': RequestPolicy('device', 5, 5., ('device',)),
    'ArmController.hold_current_position': RequestPolicy('arm', 0, invalidate=('arm',)),
    'ArmController.stop_active_motion': RequestPolicy('device', 0, 5.),
    'ArmController.submit_home': RequestPolicy('arm'),
    # 预约/接纳按普通六轴请求排序;取消和场景变更走设备级高优先级,不能被准备阻塞.
    'ArmController.reserve_arm_preparation': RequestPolicy('arm'),
    'ArmController.planning_snapshot': RequestPolicy('arm'),
    'ArmController.admit_prepared_arm_trajectory': RequestPolicy('arm'),
    'ArmController.cancel_arm_preparation': RequestPolicy('device', 0, 5.),
    'ArmController.update_trajectory_scene': RequestPolicy('device', 0, 5.),
    'ArmController.set_joint_target': RequestPolicy('arm'),
    'ArmController.cancel_arm_trajectory': RequestPolicy('device', 0),
    'ArmController.ensure_pos_vel_control': RequestPolicy('arm', 5),
    'ArmController.send_joint_motor_cmd': RequestPolicy('arm', ttl=.25),
    'GripperController.set_gripper_target': RequestPolicy('gripper'),
    'GripperController.send_gripper_motor_cmd': RequestPolicy('gripper', ttl=.25),
    'GripperController.cancel_gripper_position_command': RequestPolicy('gripper', 0),
    'GripperController._release_gripper_position_target': RequestPolicy('gripper', 0),
    'GravityController.start_gravity_compensation': RequestPolicy('arm', 5, invalidate=('arm',)),
    'GravityController.stop_gravity_compensation': RequestPolicy('arm', 5, invalidate=('arm',)),
})


class RequestRejected(RuntimeError):
    """请求尚未执行即被拒绝; 没有发生对应的设备写入."""


@dataclass
class DeviceRequest:
    """单次请求记录: 会话/资源代次, 开始期限, 操作, 专属事件及实际结果."""

    sequence: int
    resource: str
    priority: int
    generations: tuple
    expires_at: float
    operation: object
    invalidate: tuple = ()
    event: threading.Event = field(default_factory=threading.Event)
    started: bool = False
    result: object = None
    error: Exception | None = None


class DeviceRequests:
    """持锁管理有界队列, 紧急保留容量及资源代次; 不直接执行设备操作."""

    def __init__(self, capacity=64, reserved=4, clock=time.monotonic):
        """注入单调时钟, 分别设置普通和紧急容量并初始化设备, 六轴, 夹爪代次."""
        self.capacity, self.reserved, self.clock = capacity, reserved, clock
        self.lock = threading.RLock()
        self.pending = []
        self.generations = dict(device=0, arm=0, gripper=0)
        self.sequence = 0

    def submit(self, operation, *, resource='device', priority=10, ttl=2., invalidate=()):
        """校验资源与 TTL, 在队列锁内保留当前会话/资源代次并返回请求; 不代表已执行."""
        if resource not in self.generations or not math.isfinite(ttl) or ttl <= 0:
            raise ValueError('invalid request resource or lifetime')
        with self.lock:
            self._expire()
            # 普通请求和紧急请求分别计容量, 普通队列饱和时仍有停止/失能的保留槽位.
            urgent = priority <= 1
            count = sum((r.priority <= 1) == urgent for r in self.pending)
            if count >= (self.reserved if urgent else self.capacity):
                raise RequestRejected('device request queue full')
            self.sequence += 1
            # 资源请求同时绑定设备会话; 独立取消夹爪不影响六轴队列.
            names = ('device',) if resource == 'device' else ('device', resource)
            request = DeviceRequest(self.sequence, resource, priority,
                tuple((name, self.generations[name]) for name in names),
                self.clock() + ttl, operation, tuple(invalidate))
            self.pending.append(request)
            return request

    def _expire(self):
        """淘汰尚未开始且已到期的请求, 并唤醒它们各自的等待者; 调用方持队列锁."""
        for request in tuple(self.pending):
            if request.expires_at <= self.clock():
                self.cancel_pending(request, 'device request expired before execution')

    def cancel_pending(self, request, reason='request wait timed out before execution'):
        """仅撤销尚未开始的请求; 已开始或已经结算时返回 False, 不能假称取消设备事务."""
        with self.lock:
            if request.started or request.event.is_set():
                return False
            if request in self.pending:
                self.pending.remove(request)
            self.finish(request, error=RequestRejected(reason))
            return True

    def invalidate(self, *resources):
        """在队列锁内递增指定资源代次, 使旧排队请求或续体失去执行资格."""
        with self.lock:
            for name in resources:
                self.generations[name] += 1

    def take(self):
        """按优先级, 序号取出一项有效请求, 拒绝旧代次并在执行前应用其失效策略."""
        with self.lock:
            self._expire()
            while self.pending:
                request = min(self.pending, key=lambda r: (r.priority, r.sequence))
                self.pending.remove(request)
                if any(self.generations[name] != generation
                       for name, generation in request.generations):
                    self.finish(request, error=RequestRejected('obsolete device/resource generation'))
                    continue
                # 从此刻起不能以本地等待超时伪称撤销, 必须回传设备实际结果或续体失败.
                request.started = True
                self.invalidate(*request.invalidate)
                return request
        return None

    def finish(self, request, result=None, error=None):
        """在锁内保存本请求实际结果或异常, 再唤醒等待者."""
        with self.lock:
            request.result, request.error = result, error
            request.event.set()

    def close(self):
        """使所有资源代次失效并拒绝未开始请求; 在途事务由运行时单独结算."""
        with self.lock:
            self.invalidate('device', 'arm', 'gripper')
            for request in tuple(self.pending):
                self.cancel_pending(request, 'device runtime closed')


def device_request(method):
    """组件命令入口: 按集中策略复制输入后排队, 设备线程内的嵌套调用直接执行.

    装饰器必须位于生命周期锁外, 避免调用者持锁等待而写入线程反向等待该锁.
    所有装配必须显式提供同一运行时; 含同步等待的上层编排不在此入队.
    """
    policy = REQUEST_POLICIES[method.__qualname__]

    @wraps(method)
    def wrapped(self, *args, **kwargs):
        """锁外复制输入并入队; 设备线程内直调, 关闭请求仅在释放确认后回收线程."""
        runtime = self._backend
        if runtime.in_writer:
            return method(self, *args, **kwargs)
        # 请求可能延后执行,冻结调用时的输入可防止调用方随后修改列表或消息而改变待发命令.
        values, options = copy.deepcopy(args), copy.deepcopy(kwargs)
        result = runtime.call(lambda: method(self, *values, **options),
            resource=policy.resource, priority=policy.priority, ttl=policy.ttl, invalidate=policy.invalidate)
        if policy.close_worker and not self.connected:
            runtime.close_worker()
        return result

    wrapped.request_policy = policy
    return wrapped
