"""通过同一仿真锁串行访问 MuJoCo;run 回调借用实例,结果由调用者持有."""
from __future__ import annotations
import threading
from typing import Any


class SerializedSimulationAccess:
    """串行化所有触碰同一仿真实例的操作.

    物理定时器,动作执行与状态查询共享同一 MuJoCo data;目标写入,步进和
    快照提取均在锁内完成.涉及目标身份时,执行组件先持目标锁再进入本层,
    回调须保持该锁序,返回快照供锁外发布.
    """

    def __init__(self, simulation: Any, lock: threading.RLock | None = None) -> None:
        self._simulation = simulation
        self._lock = lock or threading.RLock()

    def run(self, operation):
        with self._lock:
            return operation(self._simulation)
