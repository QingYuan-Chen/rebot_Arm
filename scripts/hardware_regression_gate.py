"""现场回归的单一监测与目标准入状态，不连接 ROS 或设备。

会话只允许一个动作在途；故障锁存不可复位，所有后续目标均被拒绝。
监测与发送使用同一锁；故障发生在目标接纳前时，迟到接纳必须取消。
调用者负责执行停止保持、处理未知终态，并在结束后另行检查现场。
"""
import threading
import time


class MonitorFault(RuntimeError):
    """本轮监测失败或监测未就绪，禁止发送新目标。"""


class SustainedSpeed:
    """按独立源帧累计；夹爪动作期间和静止期间分别采用调用者给出的上限。"""
    def __init__(self, stationary_limit, gripper_limit, duration=0.10):
        self._stationary_limit = stationary_limit
        self._gripper_limit = gripper_limit
        self._duration = duration
        self._history = {}

    def observe(self, name, stamp, velocity, gripper_active):
        limit = self._gripper_limit if name == 'gripper' and gripper_active else self._stationary_limit
        item = self._history.setdefault(name, {'stamp': None, 'since': None, 'limit': limit})
        if item['limit'] != limit:
            item['limit'] = limit
            item['since'] = None
        if item['stamp'] is not None and stamp < item['stamp']:
            return True
        if item['stamp'] is None or stamp > item['stamp']:
            item['stamp'] = stamp
            if abs(velocity) > limit:
                if item['since'] is None:
                    item['since'] = stamp
            else:
                item['since'] = None
        return item['since'] is not None and (item['stamp'] - item['since']) / 1e9 >= self._duration


class RegressionGate:
    def __init__(self, stop_hold, *, on_cancel=None, clock=time.monotonic, max_monitor_age=0.15):
        self._lock = threading.RLock()
        self._fault_lock = threading.RLock()
        self._clock = clock
        self._max_age = max_monitor_age
        self._stop_hold = stop_hold
        self._on_cancel = on_cancel or (lambda future, error: None)
        self._last_check = None
        self._fault = None
        self._pending = None
        self._submitted = {}
        self._gripper_motion = False

    @property
    def fault(self):
        with self._fault_lock:
            return self._fault

    @property
    def gripper_motion(self):
        with self._lock:
            return self._gripper_motion

    @property
    def ready(self):
        return self._last_check is not None and self.fault is None

    @property
    def has_pending(self):
        with self._lock:
            return self._pending is not None or bool(self._submitted)

    def trip(self, issue):
        """异常路径锁存，禁止后续目标并取消已接纳目标。"""
        if not self._store_fault((issue,)):
            return
        # 监测检查若卡住，停止请求不等待它持有的锁。
        try:
            self._stop_hold()
        finally:
            # 检查可能卡在主锁内；已接纳夹爪的取消不等待该锁。
            entry = self._pending
            if entry is not None and entry.get('handle') is not None:
                self._cancel_locked(entry)

    def _store_fault(self, issues):
        with self._fault_lock:
            if self._fault is not None:
                return False
            self._fault = tuple(issues)
            return True

    def monitor_check(self, inspect):
        """在同一锁内检查反馈与活动状态，故障在释放锁之前锁存。"""
        stop = False
        with self._lock:
            try:
                issues = tuple(inspect(self._gripper_motion))
            except Exception as exc:
                issues = (f'monitor_exception: {exc}',)
            self._last_check = self._clock()
            if issues:
                stop = self._latch_locked(issues)
        if stop:
            self._stop_hold()
        return issues

    def _latch_locked(self, issues):
        if not self._store_fault(issues):
            return False
        if self._pending is not None:
            handle = self._pending.get('handle')
            if handle is not None:
                self._cancel_locked(self._pending)
        return True

    def _cancel_locked(self, entry):
        with entry['cancel_lock']:
            if entry['cancel_requested']:
                return
            entry['cancel_requested'] = True
        try:
            future = entry['handle'].cancel_goal_async()
            self._on_cancel(future, None)
        except Exception as exc:
            # 锁存不因取消失败被清除；现场仍须核实动作终态。
            try:
                self._on_cancel(None, str(exc))
            except Exception:
                pass

    def watchdog(self):
        """监测线程失活或长时间未完成检查时锁存并请求停止保持。"""
        last_check = self._last_check
        if last_check is None or self._clock() - last_check > self._max_age:
            self.trip('monitor_stale')
        return self.fault

    def guarded_call(self, send):
        """显式使能等非动作请求也必须经过同一故障与监测准入。"""
        with self._lock, self._fault_lock:
            if self._fault is not None:
                raise MonitorFault(f'本轮监测已锁存: {self._fault}')
            if self._last_check is None or self._clock() - self._last_check > self._max_age:
                raise MonitorFault('监测未就绪或已过期')
            return send()

    def submit(self, send, *, gripper=False):
        """send 为非阻塞的发送函数，返回接纳 future；迟到的句柄仍受监控。"""
        stop = False
        error = None
        with self._lock, self._fault_lock:
            if self._fault is not None:
                raise MonitorFault(f'本轮监测已锁存: {self._fault}')
            if self._last_check is None or self._clock() - self._last_check > self._max_age:
                raise MonitorFault('监测未就绪或已过期')
            if self._pending is not None:
                raise MonitorFault('上一个动作终态未知')
            entry = {'handle': None, 'result': None, 'processed': False,
                     'cancel_requested': False, 'cancel_lock': threading.Lock()}
            self._pending = entry
            self._gripper_motion = bool(gripper)
            try:
                future = send()
                self._submitted[future] = entry
                future.add_done_callback(lambda f: self._accepted(entry, f))
            except Exception as exc:
                stop = self._latch_locked(('goal_send_unknown',))
                error = exc
        if stop:
            self._stop_hold()
        if error is not None:
            raise error
        return future

    def _accepted(self, entry, future):
        # 先公开已接纳的句柄；若监测检查卡住，看门狗仍可取消夹爪。
        try:
            handle = future.result()
            if handle.accepted:
                entry['handle'] = handle
                if self.fault is not None:
                    self._cancel_locked(entry)
        except Exception:
            self.trip('goal_acceptance_unknown')
            return
        stop = False
        with self._lock:
            if entry['processed']:
                return
            entry['processed'] = True
            try:
                if not handle.accepted:
                    self._pending = None
                    self._gripper_motion = False
                    return
                result = handle.get_result_async()
                entry['result'] = result
                result.add_done_callback(lambda f: self._terminal(entry, f))
                if self.fault is not None:
                    self._cancel_locked(entry)
            except Exception:
                stop = self._latch_locked(('goal_acceptance_unknown',))
        if stop:
            self._stop_hold()

    def result_for(self, future):
        """接纳完成时同步合并回调，避免 executor 延迟调度造成句柄缺口。"""
        with self._lock:
            entry = self._submitted.pop(future, None)
            if entry is None:
                raise MonitorFault('动作未接纳或终态已经结束')
        self._accepted(entry, future)
        with self._lock:
            if entry['handle'] is None or entry['result'] is None:
                raise MonitorFault('动作未接纳或接纳状态未知')
            return entry['handle'], entry['result']

    def _terminal(self, entry, future):
        stop = False
        with self._lock:
            try:
                future.result()
            except Exception:
                stop = self._latch_locked(('goal_terminal_unknown',))
            else:
                if self._pending is entry:
                    self._pending = None
                    self._gripper_motion = False
        if stop:
            self._stop_hold()
