"""运动停止确认的线程安全证据；保留源时间、连续性与固定请求锚点。

接收线程 update 只拷贝新样本；业务线程 stationary 检查 0.30 s 全窗口位置
极差及固定锚点位移（三个位置编码）。单帧速度越限延迟确认，持续越限始终阻断。
150 ms 断流、无效值和时间倒退使跨空档的旧请求失效；恢复帧不洗掉空档。
"""
from collections import deque
import math
import threading


class StopFeedback:
    def __init__(self, *, window=.3, max_age=.15, velocity_limit=.03, position_span=3*25/65535):
        self.window = window
        self.max_age = max_age
        self.velocity_limit = velocity_limit
        self.position_span = position_span
        self.samples = deque(maxlen=600)
        self.lock = threading.Lock()
        self.broken_at = float('-inf')
        self._since = None
        self._anchor = None

    def invalidate(self, received):
        """接收空档/状态变化使旧请求失效；调用方不得持有本对象的锁。"""
        with self.lock:
            self.broken_at = max(self.broken_at, received)
            self.samples.clear()

    def ready(self, now, now_ns):
        """仅供新停止请求建立观察起点；已开始的窗口不能靠此接口跨过空档。"""
        with self.lock:
            return (bool(self.samples) and now-self.samples[-1][0] <= self.max_age
                    and -.05 <= (now_ns-self.samples[-1][1])/1e9 <= self.max_age)

    def update(self, msg, received, now_ns):
        names = [f'joint{i}' for i in range(1, 7)]
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            indices = [list(msg.name).index(name) for name in names]
            q = tuple(float(msg.position[i]) for i in indices)
            v = tuple(float(msg.velocity[i]) for i in indices)
            valid = (stamp > 0 and -.05 <= (now_ns-stamp)/1e9 <= self.max_age
                     and all(math.isfinite(x) for x in q+v))
        except (ValueError, IndexError, TypeError):
            valid = False
        with self.lock:
            if not valid:
                self.broken_at = max(self.broken_at, received)
                self.samples.clear()
                return
            if self.samples:
                last = self.samples[-1]
                if stamp == last[1]:
                    return
                if (stamp < last[1] or received-last[0] > self.max_age
                        or (stamp-last[1])/1e9 > self.max_age):
                    self.broken_at = max(self.broken_at, received)
                    self.samples.clear()
            self.samples.append((received, stamp, q, max(abs(x) for x in v)))

    def stationary(self, since, now, now_ns):
        with self.lock:
            samples = [s for s in self.samples if s[0] >= since]
            if since != self._since:
                self._since, self._anchor = since, None
            if samples and self._anchor is None:
                if samples[0][0]-since > self.max_age:
                    return False  # 请求起点证据已丢失，不能把较晚位置重新当作锚点。
                self._anchor = samples[0][2]
            if since <= self.broken_at or len(samples) < 3:
                return False
            latest = samples[-1]
            if (now-latest[0] > self.max_age
                    or not -.05 <= (now_ns-latest[1])/1e9 <= self.max_age):
                # 即使查询时发现空档，后续恢复也不能确认同一请求。
                self.broken_at = max(self.broken_at, now)
                return False
            # 保留窗口左侧一份样本；全窗口任意速度越限须等后续正常窗口覆盖。
            begin = len(samples)-1
            while begin > 0 and latest[0]-samples[begin][0] < self.window-1e-9:
                begin -= 1
            selected = samples[begin:]
            if len(selected) < 3 or latest[0]-selected[0][0] < self.window-1e-9:
                return False
            if any(s[3] > self.velocity_limit for s in selected):
                return False
            return all(max(s[2][j] for s in selected)-min(s[2][j] for s in selected) <= self.position_span+1e-12
                       and max(abs(s[2][j]-self._anchor[j]) for s in selected) <= self.position_span+1e-12
                       for j in range(6))
