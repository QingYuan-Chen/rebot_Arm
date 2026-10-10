"""Fresh, stationary feedback evidence for motion-stop acknowledgement."""
from collections import deque
import math
import threading


class StopFeedback:
    def __init__(self, *, window=.3, max_age=.5, velocity_limit=.03, position_span=.01):
        self.window = window
        self.max_age = max_age
        self.velocity_limit = velocity_limit
        self.position_span = position_span
        self.samples = deque(maxlen=200)
        self.lock = threading.Lock()

    def update(self, msg, received, now_ns):
        names = [f'joint{i}' for i in range(1, 7)]
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        try:
            indices = [list(msg.name).index(name) for name in names]
            q = tuple(float(msg.position[i]) for i in indices)
            v = tuple(float(msg.velocity[i]) for i in indices)
            valid = (stamp > 0 and -.25 <= (now_ns-stamp)/1e9 <= self.max_age
                     and all(math.isfinite(x) for x in q+v))
        except (ValueError, IndexError, TypeError):
            valid = False
        with self.lock:
            if not valid:
                self.samples.clear()
                return
            if self.samples and stamp <= self.samples[-1][1]:
                # Republishing a cached sample is not fresh feedback.
                return
            if max(abs(x) for x in v) > self.velocity_limit:
                self.samples.clear()
                return
            if self.samples and received-self.samples[-1][0] > self.max_age:
                self.samples.clear()
            self.samples.append((received, stamp, q))

    def stationary(self, since, now, now_ns):
        with self.lock:
            samples = [s for s in self.samples if s[0] >= since and now-s[0] <= self.window+self.max_age]
        if len(samples) < 3:
            return False
        if now-samples[-1][0] > self.max_age or not -.25 <= (now_ns-samples[-1][1])/1e9 <= self.max_age:
            return False
        if samples[-1][0]-samples[0][0] < self.window:
            return False
        return all(max(s[2][j] for s in samples)-min(s[2][j] for s in samples) <= self.position_span
                   for j in range(6))
