"""设备请求的有界队列与代次检查。

职责：保存尚未执行的请求，按优先级和提交顺序交给唯一运行时线程。
对外接口：``submit/take/finish/cancel_pending/invalidate``；请求结果通过事件传回。
约束：普通队列满时仍保留停止入口；有效期只约束开始执行的时刻。已开始的
设备事务不能因调用方等待超时而伪称取消，必须继续报告实际回执。资源代次
在停止、模式交接和会话结束时失效，六轴与夹爪的取消互不混淆。
"""
from dataclasses import dataclass, field
from functools import wraps
import copy
import math
import threading
import time


class RequestRejected(RuntimeError):
    """请求尚未执行即被拒绝；没有发生对应的设备写入。"""


@dataclass
class DeviceRequest:
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
    def __init__(self, capacity=64, reserved=4, clock=time.monotonic):
        self.capacity, self.reserved, self.clock = capacity, reserved, clock
        self.lock = threading.RLock()
        self.pending = []
        self.generations = dict(device=0, arm=0, gripper=0)
        self.sequence = 0

    def submit(self, operation, *, resource='device', priority=10, ttl=2., invalidate=()):
        if resource not in self.generations or not math.isfinite(ttl) or ttl <= 0:
            raise ValueError('invalid request resource or lifetime')
        with self.lock:
            self._expire()
            urgent = priority <= 1
            count = sum((r.priority <= 1) == urgent for r in self.pending)
            if count >= (self.reserved if urgent else self.capacity):
                raise RequestRejected('device request queue full')
            self.sequence += 1
            # 资源请求同时绑定设备会话；独立取消夹爪不影响六轴队列。
            names = ('device',) if resource == 'device' else ('device', resource)
            request = DeviceRequest(self.sequence, resource, priority,
                tuple((name, self.generations[name]) for name in names),
                self.clock() + ttl, operation, tuple(invalidate))
            self.pending.append(request)
            return request

    def _expire(self):
        for request in tuple(self.pending):
            if request.expires_at <= self.clock():
                self.cancel_pending(request, 'device request expired before execution')

    def cancel_pending(self, request, reason='request wait timed out before execution'):
        with self.lock:
            if request.started or request.event.is_set():
                return False
            if request in self.pending:
                self.pending.remove(request)
            self.finish(request, error=RequestRejected(reason))
            return True

    def invalidate(self, *resources):
        with self.lock:
            for name in resources:
                self.generations[name] += 1

    def take(self):
        with self.lock:
            self._expire()
            while self.pending:
                request = min(self.pending, key=lambda r: (r.priority, r.sequence))
                self.pending.remove(request)
                if any(self.generations[name] != generation
                       for name, generation in request.generations):
                    self.finish(request, error=RequestRejected('obsolete device/resource generation'))
                    continue
                request.started = True
                self.invalidate(*request.invalidate)
                return request
        return None

    def finish(self, request, result=None, error=None):
        with self.lock:
            request.result, request.error = result, error
            request.event.set()

    def close(self):
        with self.lock:
            self.invalidate('device', 'arm', 'gripper')
            for request in tuple(self.pending):
                self.cancel_pending(request, 'device runtime closed')


def device_request(*, resource='device', priority=10, ttl=2., invalidate=()):
    """兼容门面的同步入口：复制输入后排队，运行时内部的嵌套调用直接执行。

    装饰器必须位于生命周期锁外，避免调用者持锁等待而写入线程反向等待该锁。
    所有装配必须提供同一运行时契约；算法测试应直接使用对应组件，不能绕过准入。
    """
    def decorate(method):
        @wraps(method)
        def wrapped(self, *args, **kwargs):
            runtime = self._backend
            if runtime.in_writer:
                return method(self, *args, **kwargs)
            values, options = copy.deepcopy(args), copy.deepcopy(kwargs)
            result = runtime.call(lambda: method(self, *values, **options),
                resource=resource, priority=priority, ttl=ttl, invalidate=invalidate)
            if method.__name__ == 'shutdown' and not self.connected:
                runtime.close_worker()
            return result
        return wrapped
    return decorate
