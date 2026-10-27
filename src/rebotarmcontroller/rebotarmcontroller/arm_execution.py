"""持有六轴目标和按时间推进的轨迹状态。

``ArmExecution`` 提供目标设置、速度覆盖、轨迹提交、取消和进度查询。
周期线程调用 ``sample`` 取得当前应发送的 ``ArmTarget``；实际发送成功后
再调用 ``acknowledge`` 提交游标。``ArmTarget`` 是携带目标、速度覆盖、
代次和候选游标的不可变快照。

状态由可重入锁保护；取消递增代次，使旧轨迹和旧快照不能继续推进。
目标必须是六个有限且在硬件限位内的关节值，轨迹时间必须非负且严格递增。
本模块不读取反馈、不插值、不访问设备，也不证明机械臂已经到位；
进度表示已确认发送的轨迹点比例，而非物理到达比例。
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time
import numpy as np
from .gravity_control import vector, POSITION_LIMITS

@dataclass(frozen=True)
class ArmTarget:
    """一次待发送的六轴目标；``cursor`` 是发送成功后应确认的位置。"""
    positions: tuple[float, ...]
    velocity_override: tuple[float, ...] | None
    generation: int
    cursor: int

class ArmExecution:
    """管理一个六轴目标及其轨迹游标，由运行时协调发送与取消。"""
    def __init__(self):
        self.lock = threading.RLock()
        self._target = (0.,) * 6
        self._override = None
        self._generation = 0
        self._points = ()
        self._times = ()
        self._cursor = 0
        self._started = 0.
        self.running = False
        self.moving = False

    @staticmethod
    def validate_target(value):
        values = vector(value,6,'joint target')
        bounds = np.asarray(POSITION_LIMITS)
        if (values < bounds[:,0]).any() or (values > bounds[:,1]).any():
            raise ValueError('joint target outside hardware limits')
        return tuple(float(v) for v in values)

    @property
    def generation(self):
        with self.lock: return self._generation

    def set_target(self, values):
        """取消旧轨迹并设置新目标；返回前先检查维度和位置范围。"""
        target = self.validate_target(values)
        with self.lock:
            self.cancel()
            self._target = target

    def target(self):
        with self.lock: return np.array(self._target,copy=True)

    def set_speed_override(self, value):
        """设置六轴任务速度覆盖；传入 ``None`` 清除覆盖。"""
        override = None if value is None else tuple(vector(value,6,'velocity override'))
        if override is not None and min(override)<=0: raise ValueError('velocity override must be positive')
        with self.lock: self._override = override

    def cancel(self):
        """使旧代次失效并清空轨迹，保留最后目标和速度覆盖。"""
        with self.lock:
            # 代次变化使取消前产生的轨迹和待发送快照失效。
            self._generation += 1
            self._points = self._times = ()
            self._cursor = 0
            self.moving = False
            return self._generation

    def submit(self, points, times, generation, *, now=None):
        """在当前代次仍有效且运行时活动时接收轨迹，否则返回 ``False``。"""
        values = tuple(self.validate_target(q) for q in points)
        stamps = tuple(float(t) for t in times)
        if not values or len(values)!=len(stamps) or not all(math.isfinite(t) and t>=0 for t in stamps):
            raise ValueError('trajectory needs finite nonnegative times and joint targets')
        if any(b<=a for a,b in zip(stamps,stamps[1:])):
            raise ValueError('trajectory times must increase')
        with self.lock:
            if generation!=self._generation or not self.running: return False
            self._points,self._times = values,stamps
            self._cursor = 0
            self._started = time.monotonic() if now is None else float(now)
            self.moving = True
            return True

    def sample(self, now):
        """取指定单调时间已到期的最新目标；游标仍等待发送确认。"""
        with self.lock:
            index = self._cursor
            # 一个周期内若多个点已到期，只输出其中最新目标；不在此处确认发送。
            while self.moving and index<len(self._points) and self._times[index]<=now-self._started:
                self._target = self._points[index]
                index += 1
            return ArmTarget(self._target,self._override,self._generation,index)

    def acknowledge(self, target):
        """确认已发送快照对应的游标；过期代次不会改变当前轨迹。"""
        with self.lock:
            # 只有当前代次的命令成功发送后才能推进游标。
            if target.generation!=self._generation: return
            self._cursor = target.cursor
            if self._points and self._cursor==len(self._points): self.moving = False

    @property
    def progress(self):
        with self.lock: return self._cursor/len(self._points) if self._points else 1.
