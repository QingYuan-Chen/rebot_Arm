"""运动停止确认的线程安全证据；保留源时间、连续性与速度窗口。

职责与位置
    接收线程与停止编排之间共享的证据容器, 不读取设备, 不发出控制命令.

对外接口
    update 保存去重样本, ready 供新请求建立起点, stationary 验证固定请求的连续静止.

安全与并发约束
    重复源时间戳不累计，断流锁存为该请求失败；不按位置变化设置门限。
    内部锁只保护短事务, 禁止持此锁反向调用接收缓存或同步等待 ROS.

文件结构(按定义顺序)
    StopFeedback: 用独立短事务锁保存去重六轴反馈, 为固定停止请求提供连续静止证据.
"""

from collections import deque
import math
import threading


class StopFeedback:
    """用独立短事务锁保存去重六轴反馈, 为固定停止请求提供连续静止证据."""

    def __init__(self, *, window=.3, max_age=.15, velocity_limit=.03):
        """保存窗口与阈值, 初始化样本队列和失效时间; 不执行任何控制动作."""
        self.window = window
        self.max_age = max_age
        self.velocity_limit = velocity_limit
        self.samples = deque(maxlen=600)
        self.lock = threading.Lock()
        self.broken_at = float('-inf')
        self._since = None
        self._first_received = None

    def invalidate(self, received):
        """接收空档/状态变化使旧请求失效; 调用方不得持有本对象的锁."""
        with self.lock:
            self.broken_at = max(self.broken_at, received)
            self.samples.clear()

    def ready(self, now, now_ns):
        """仅供新停止请求建立观察起点; 已开始的窗口不能靠此接口跨过空档."""
        with self.lock:
            return (bool(self.samples) and now-self.samples[-1][0] <= self.max_age
                    and -.05 <= (now_ns-self.samples[-1][1])/1e9 <= self.max_age)

    def update(self, msg, received, now_ns):
        """校验六轴数组与源年龄, 只保存推进的源时间; 无效, 回退和接收/源断流使旧请求失效."""
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
        """查询 since 之后是否已有连续静止全窗口, 不以恢复帧消除旧请求失败.

        since / now 为单调秒, now_ns 与消息时间戳处于同一 ROS 时钟域.
        至少三帧覆盖窗口，逐帧速度需达标；不检查位置极差或首帧偏差。
        发现新的过期也锁存 broken_at, 同一请求之后不能因新帧恢复而确认.
        """
        with self.lock:
            samples = [s for s in self.samples if s[0] >= since]
            if since != self._since:
                self._since, self._first_received = since, None
            if samples and self._first_received is None:
                if samples[0][0]-since > self.max_age:
                    return False  # 请求起点证据已丢失，不能用较晚的新帧跨过空档。
                self._first_received = samples[0][0]
            if since <= self.broken_at or len(samples) < 3:
                return False
            latest = samples[-1]
            if (now-latest[0] > self.max_age
                    or not -.05 <= (now_ns-latest[1])/1e9 <= self.max_age):
                # 即使查询时发现空档, 后续恢复也不能确认同一请求.
                self.broken_at = max(self.broken_at, now)
                return False
            # 保留窗口左侧一份样本; 全窗口任意速度越限须等后续正常窗口覆盖.
            begin = len(samples)-1
            while begin > 0 and latest[0]-samples[begin][0] < self.window-1e-9:
                begin -= 1
            selected = samples[begin:]
            if len(selected) < 3 or latest[0]-selected[0][0] < self.window-1e-9:
                return False
            if any(s[3] > self.velocity_limit for s in selected):
                return False
            return True
