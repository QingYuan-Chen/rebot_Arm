"""持有六轴目标和按时间推进的轨迹状态.

职责与位置
    运行时使用的六轴目标, 时间游标和目标代次所有者; 没有设备或 ROS 依赖.

对外接口
    ArmExecution 接纳数值轨迹; sample 返回 ArmTarget 候选, acknowledge 在发送成功后提交游标.

安全与并发约束
    锁内快照与代次防止旧发送确认污染接替目标; 发送进度不能用作物理到位证明.

统一约定
    六轴固定 joint1..joint6; 位置 rad, 速度覆盖 rad/s, 轨迹时间为相对单调秒.

文件结构(按定义顺序)
    ArmTarget: 一次待发送的六轴目标; ``cursor`` 是发送成功后应确认的位置.
    ArmExecution: 管理一个六轴目标及其轨迹游标, 由运行时协调发送与取消.
"""

from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time
import numpy as np
from .gravity_control import vector, POSITION_LIMITS
from rebotarm_trajectory.sampling import step_index, validate_samples


@dataclass(frozen=True)
class ArmTarget:
    """一次待发送的六轴目标; ``cursor`` 是发送成功后应确认的位置."""

    positions: tuple[float, ...]
    velocity_override: tuple[float, ...] | None
    generation: int
    cursor: int


class ArmExecution:
    """管理一个六轴目标及其轨迹游标, 由运行时协调发送与取消."""

    def __init__(self):
        """建立独立的目标, 代次, 轨迹游标和可重入锁; 初始状态不运行, 不发送."""
        self.lock = threading.RLock()
        self._target = (0.,) * 6
        self._override = None
        self._generation = 0
        self._points = ()
        self._times = ()
        self._cursor = 0
        self._started = 0.
        self._curve = None
        self.last_sent_target = None
        self.reservation = None
        self.running = False
        self.moving = False

    @staticmethod
    def validate_target(value):
        """校验六个有限关节角及规范位置边界, 返回不可变目标元组."""
        values = vector(value,6,'joint target')
        bounds = np.asarray(POSITION_LIMITS)
        if (values < bounds[:,0]).any() or (values > bounds[:,1]).any():
            raise ValueError('joint target outside hardware limits')
        return tuple(float(v) for v in values)

    @property
    def generation(self):
        """在执行锁内读取当前目标代次, 供任务所有权和发送确认比较."""
        with self.lock: return self._generation

    def set_target(self, values):
        """取消旧轨迹并设置新目标; 返回前先检查维度和位置范围."""
        target = self.validate_target(values)
        with self.lock:
            self.cancel()
            self._target = target

    def target(self):
        """在执行锁内复制当前目标数组, 调用方修改副本不会改变执行器."""
        with self.lock: return np.array(self._target,copy=True)

    def set_speed_override(self, value):
        """设置六轴任务速度覆盖; 传入 ``None`` 清除覆盖."""
        override = None if value is None else tuple(vector(value,6,'velocity override'))
        if override is not None and min(override)<=0: raise ValueError('velocity override must be positive')
        with self.lock: self._override = override

    def begin_scoped_target(self, values, velocity):
        """原子接纳停靠目标与速度, 返回其代次; 接替时速度覆盖随目标失效."""
        target = self.validate_target(values)
        override = tuple(vector(velocity, 6, 'velocity override'))
        if min(override) <= 0:
            raise ValueError('velocity override must be positive')
        with self.lock:
            self.cancel()
            self._target, self._override = target, override
            return self._generation

    def finish_scoped_target(self, generation, hold):
        """仅当任务仍拥有当前代次时切换到反馈保持, 避免覆盖接替目标."""
        target = self.validate_target(hold)
        with self.lock:
            if generation != self._generation:
                return False
            self.cancel()
            self._target = target
            return True

    def cancel(self):
        """使旧代次失效并清空轨迹和任务速度覆盖, 保留最后目标."""
        with self.lock:
            # 代次变化使取消前产生的轨迹和待发送快照失效.
            self._generation += 1
            self._points = self._times = ()
            self._curve = None
            self._override = None
            self._cursor = 0
            self.moving = False
            return self._generation

    def submit(self, points, times, generation, *, now=None, velocity=None):
        """先校验轨迹及速度覆盖, 再在同一事务内接纳; 代次失效或未运行时返回 False."""
        values, stamps = validate_samples(points, times)
        values = tuple(self.validate_target(q) for q in values)
        override = None if velocity is None else tuple(vector(velocity, 6, 'velocity override'))
        if override is not None and min(override) <= 0:
            raise ValueError('velocity override must be positive')
        with self.lock:
            if generation!=self._generation or not self.running: return False
            # 校验失败不能留下无任务轨迹; 目标, 速度覆盖及代次在同一锁内一起接替.
            self._generation += 1
            self._override = override
            self._curve = None
            self._points,self._times = values,stamps
            self._cursor = 0
            self._started = time.monotonic() if now is None else float(now)
            self.moving = True
            return True

    def submit_prepared(self, curve, generation, *, now):
        """接纳原样系数；不再次规范化起点、合并节点或拟合。"""
        self.validate_target(curve.points[0].positions)
        self.validate_target(curve.points[-1].positions)
        with self.lock:
            if generation != self._generation or not self.running:
                return False
            self._generation += 1
            self._curve = curve
            self._points, self._times = curve.positions, curve.times
            self._cursor, self._started, self._override = 0, float(now), None
            self.moving = True
            return True

    def sample(self, now):
        """取指定单调时间已到期的最新目标; 游标仍等待发送确认."""
        with self.lock:
            index = self._cursor
            if self._curve is not None and self.moving:
                elapsed = now-self._started
                q = self._curve.sample(elapsed)[0]
                return ArmTarget(q, self._override, self._generation,
                                 step_index(self._times, elapsed))
            # 一个周期内若多个点已到期, 只输出其中最新目标; 不在此处确认发送.
            if self.moving:
                index = step_index(self._times, now-self._started, index)
                if index:
                    self._target = self._points[index-1]
            return ArmTarget(self._target,self._override,self._generation,index)

    def acknowledge(self, target):
        """确认已发送快照对应的游标; 过期代次不会改变当前轨迹."""
        with self.lock:
            # 只有当前代次的命令成功发送后才能推进游标.
            if target.generation!=self._generation: return
            self.last_sent_target = tuple(target.positions)
            if self._curve is not None:
                self._target = self.last_sent_target
            self._cursor = target.cursor
            if self._points and self._cursor==len(self._points): self.moving = False

    @property
    def progress(self):
        """返回已成功发送的点比例; 空轨迹为 1, 不代表物理到位."""
        with self.lock: return self._cursor/len(self._points) if self._points else 1.
