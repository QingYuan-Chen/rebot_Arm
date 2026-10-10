"""execution feedback_timing components; preserve existing execution contracts."""
from __future__ import annotations
import math
from typing import Any


class FeedbackRateLimiter:
    """按固定频率限制动作反馈发布，避免执行循环每个物理步都发反馈。

    上限 200 Hz 是刻意的保护：反馈频率若超过真实控制周期只会制造无意义的总线
    与订阅端压力，这里直接把配置错误拒绝掉。`should_publish` 使用调用方传入的
    单调时钟（time.monotonic），与仿真时间无关。
    """

    def __init__(self, rate_hz: Any) -> None:
        try:
            rate = float(rate_hz)
        except (TypeError, ValueError) as exc:
            raise ValueError("feedback rate must be finite and in (0, 200]") from exc
        if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
            raise ValueError("feedback rate must be finite and in (0, 200]")
        self.rate_hz = rate
        self._period = 1.0 / rate
        self._last_publish: float | None = None

    def should_publish(self, monotonic_time: Any, *, final: bool = False) -> bool:
        """判断此刻是否应发布反馈。

        final=True 用于终态反馈（成功/超时），无论是否已到周期都强制放行，
        确保订阅端一定收到最后一条状态；首次调用（_last_publish 为 None）
        也一定放行。
        """
        now = float(monotonic_time)
        if not math.isfinite(now):
            raise ValueError("feedback clock must be finite")
        due = self._last_publish is None or now - self._last_publish >= self._period
        if final or due:
            self._last_publish = now
            return True
        return False
