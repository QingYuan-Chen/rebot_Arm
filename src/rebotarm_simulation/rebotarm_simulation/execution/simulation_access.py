"""execution simulation_access components; preserve existing execution contracts."""
from __future__ import annotations
import threading
from typing import Any


class SerializedSimulationAccess:
    """串行化所有触碰同一仿真实例的操作。

    物理步进（定时器线程）、轨迹目标下发（动作执行线程）和虚拟相机渲染
    （相机工作线程）都会访问同一个 MuJoCo data，必须通过这把锁互斥；否则
    会出现读写竞态导致的非物理跳变。
    """

    def __init__(self, simulation: Any, lock: threading.RLock | None = None) -> None:
        self._simulation = simulation
        self._lock = lock or threading.RLock()

    def run(self, operation):
        with self._lock:
            return operation(self._simulation)
