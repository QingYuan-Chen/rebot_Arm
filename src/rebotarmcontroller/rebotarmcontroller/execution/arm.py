"""Minimal arm target and timed cursor; owns no device or sender thread."""
from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time
import numpy as np
from ..gravity_control import vector, POSITION_LIMITS

@dataclass(frozen=True)
class ArmTarget:
    positions: tuple[float, ...]
    velocity_override: tuple[float, ...] | None
    generation: int
    cursor: int

class ArmExecution:
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
        target = self.validate_target(values)
        with self.lock:
            self.cancel()
            self._target = target

    def target(self):
        with self.lock: return np.array(self._target,copy=True)

    def set_speed_override(self, value):
        override = None if value is None else tuple(vector(value,6,'velocity override'))
        if override is not None and min(override)<=0: raise ValueError('velocity override must be positive')
        with self.lock: self._override = override

    def cancel(self):
        with self.lock:
            self._generation += 1
            self._points = self._times = ()
            self._cursor = 0
            self.moving = False
            return self._generation

    def submit(self, points, times, generation, *, now=None):
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
        with self.lock:
            index = self._cursor
            while self.moving and index<len(self._points) and self._times[index]<=now-self._started:
                self._target = self._points[index]
                index += 1
            return ArmTarget(self._target,self._override,self._generation,index)

    def acknowledge(self, target):
        with self.lock:
            if target.generation!=self._generation: return
            self._cursor = target.cursor
            if self._points and self._cursor==len(self._points): self.moving = False

    @property
    def progress(self):
        with self.lock: return self._cursor/len(self._points) if self._points else 1.
