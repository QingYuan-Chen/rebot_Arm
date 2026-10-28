"""设备线程拥有的静止证据；不访问总线、不等待或发送目标。

接口：reset 固定本次保持锚点，observe 接收复制数值，confirmed 查询连续窗口。
使用全窗口位置极差及相对固定锚点的位移，防止同端点振荡和缓慢漂移被洗掉。
新帧须逐轴序号推进，重复缓存不累计时间。单帧速度越限只重建静止窗口；
失效/断流另行报告，不能以恢复后的新帧覆盖旧停止事务的证据缺口。
"""
from collections import deque
import numpy as np

POSITION_CODE = 25. / 65535
POSITION_SPAN = 3 * POSITION_CODE


class StationaryEvidence:
    def __init__(self, *, window=.20, velocity_limit=.05, span=POSITION_SPAN, max_age=.15, joints=6):
        if not np.isfinite([window, velocity_limit, span, max_age]).all() or min(window, velocity_limit, span, max_age) <= 0:
            raise ValueError('stationary policy requires positive finite bounds')
        self.window, self.velocity_limit, self.span, self.max_age = window, velocity_limit, span, max_age
        self.joints = joints
        self.reset()

    def reset(self, anchor=None):
        self.anchor = None if anchor is None else np.array(anchor, dtype=float, copy=True)
        if self.anchor is not None and (self.anchor.shape != (self.joints,) or not np.isfinite(self.anchor).all()):
            raise ValueError('stationary anchor must contain finite positions')
        self.samples = deque(maxlen=256)
        self.last_stamp = None
        self.last_sequences = None
        self.reason = 'waiting'

    def observe(self, positions, velocities, stamp, now, *, sequences=None):
        q, v = np.asarray(positions, dtype=float), np.asarray(velocities, dtype=float)
        if (q.shape != (self.joints,) or v.shape != (self.joints,) or not np.isfinite(q).all()
                or not np.isfinite(v).all() or not np.isfinite([stamp, now]).all()
                or not -.005 <= now-stamp <= self.max_age):
            self.samples.clear()
            self.reason = 'invalid_or_stale'
            return self.reason
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.samples.clear()
            self.reason = 'source_regressed'
            return self.reason
        seq = None if sequences is None else np.array(sequences, dtype=np.uint64, copy=True)
        if seq is not None and seq.shape != (self.joints,):
            raise ValueError('feedback sequence count mismatch')
        if self.last_stamp == stamp:
            return 'duplicate'
        if seq is not None and self.last_sequences is not None:
            delta = seq-self.last_sequences
            if ((delta >= (1 << 63)) | ((seq == 0) & (self.last_sequences != 0))).any():
                self.samples.clear()
                self.reason = 'source_regressed'
                return self.reason
            if not ((delta > 0) & (delta < (1 << 63))).all():
                return 'duplicate'
        gap = self.last_stamp is not None and stamp-self.last_stamp > self.max_age
        self.last_stamp, self.last_sequences = stamp, seq
        if gap:
            self.samples.clear()
            self.reason = 'source_gap'
            return self.reason
        if self.anchor is None:
            self.anchor = q.copy()
        if np.max(np.abs(q-self.anchor)) > self.span + 1e-12:
            self.samples.clear()
            self.reason = 'anchor_displacement'
        elif np.max(np.abs(v)) > self.velocity_limit:
            self.samples.clear()
            self.reason = 'velocity'
        else:
            self.samples.append((stamp, q.copy()))
            # 保留窗口左边界的一份样本，使覆盖时长不会因不整齐采样缩水。
            while len(self.samples) > 2 and self.samples[1][0] <= stamp-self.window:
                self.samples.popleft()
            self.reason = 'quiet'
        return self.reason

    @property
    def quiet_since(self):
        return self.samples[0][0] if self.samples else None

    def confirmed(self, now=None):
        return (len(self.samples) >= 3
                and (now is None or -.005 <= now-self.last_stamp <= self.max_age)
                and self.samples[-1][0]-self.samples[0][0] >= self.window-1e-9
                and np.max(np.ptp(np.asarray([q for _, q in self.samples]), axis=0)) <= self.span+1e-12)


class DisabledTransitionEvidence:
    """现场诊断策略；不改变生产失能事务，不自行发出任何设备命令。

    七路输入均为原始电机 rad/rad/s。需显式服务成功、请求后七路新鲜失能帧、
    全过渡位置界和之后连续一秒静止；孤立速度峰保留事件并延迟确认。
    断流/未知状态/位移超界锁存失败，之后恢复不能改写该次验收结果。
    """
    def __init__(self, positions, sequences, requested_at):
        self.requested_at = requested_at
        self.baseline_sequences = np.array(sequences, dtype=np.uint64, copy=True)
        self.anchor = np.array(positions, dtype=float, copy=True)
        if self.anchor.shape != (7,) or self.baseline_sequences.shape != (7,) or not np.isfinite(self.anchor).all():
            raise ValueError('seven finite raw motor baselines required')
        self.evidence = StationaryEvidence(window=1., velocity_limit=.07, joints=7)
        self.evidence.reset(self.anchor)
        self.evidence.last_stamp = requested_at
        self.evidence.last_sequences = self.baseline_sequences
        self.receipt = False
        self.disabled = False
        self.failure = ''
        self.velocity_events = 0

    def service_result(self, success):
        self.receipt = bool(success)
        if not success:
            self.failure = 'disable service failed'

    def observe(self, positions, velocities, statuses, sequences, observed_at, now):
        if self.failure:
            return
        if observed_at <= self.requested_at:
            if now-self.requested_at > self.evidence.max_age:
                self.failure = 'source_gap'
            return
        q = np.asarray(positions, dtype=float)
        codes = np.asarray(statuses)
        seq = np.asarray(sequences, dtype=np.uint64)
        if (q.shape != (7,) or codes.shape != (7,) or seq.shape != (7,)
                or not np.isfinite(q).all() or not np.isin(codes, [0, 1]).all()):
            self.failure = 'invalid transition feedback'
            return
        if np.max(np.abs(q-self.anchor)) > POSITION_SPAN+1e-12:
            self.failure = 'transition position displacement'
            return
        reason = self.evidence.observe(q, velocities, observed_at, now, sequences=seq)
        if reason in ('invalid_or_stale', 'source_gap', 'source_regressed'):
            self.failure = reason
        if reason == 'velocity':
            self.velocity_events += 1
        advanced = ((seq-self.baseline_sequences > 0)
                    & (seq-self.baseline_sequences < (1 << 63))).all()
        disabled = observed_at > self.requested_at and advanced and (codes == 0).all()
        if not disabled:
            self.evidence.samples.clear()
        if self.disabled and not disabled:
            self.failure = 'disabled status lost'
        self.disabled = bool(disabled)

    def confirmed(self, now):
        return not self.failure and self.receipt and self.disabled and self.evidence.confirmed(now)
