"""设备线程拥有的静止证据; 不访问总线, 不等待或发送目标.

职责与位置
    纯数值静止判定, 供生产停止确认及独立现场失能诊断使用.

对外接口
    StationaryEvidence 用新鲜独立帧的速度与完整时间窗口确认;
    DisabledTransitionEvidence 额外组合七路状态与失能服务回执.

安全与并发约束
    单帧速度峰会延后确认, 不能单凭它证明真实位移; 断流, 回退需交给事务锁存失败.
    现场诊断的 0.07 rad/s 与生产停止的 0.05 rad/s 分属不同策略, 不在这里统一或放宽.

文件结构(按定义顺序)
    StationaryEvidence: 速度与时间窗口证据; 不持有锁, 由调用方串行访问.
    DisabledTransitionEvidence: 现场诊断策略; 不改变生产失能事务, 不自行发出任何设备命令.
"""

from collections import deque
import numpy as np

class StationaryEvidence:
    """只按新帧、速度与完整窗口确认；位置锚点仅供诊断记录。"""

    def __init__(self, *, window=.20, velocity_limit=.05, max_age=.15, joints=6):
        """保留各调用方的窗口、速度和新鲜度策略，不以位置偏差判定静止。"""
        if not np.isfinite([window, velocity_limit, max_age]).all() or min(window, velocity_limit, max_age) <= 0:
            raise ValueError('stationary policy requires positive finite bounds')
        self.window, self.velocity_limit, self.max_age = window, velocity_limit, max_age
        self.joints = joints
        self.reset()

    def reset(self, anchor=None):
        """保存可选诊断起点，清除样本、序号与源时间；不设置位置门限。"""
        self.anchor = None if anchor is None else np.array(anchor, dtype=float, copy=True)
        if self.anchor is not None and (self.anchor.shape != (self.joints,) or not np.isfinite(self.anchor).all()):
            raise ValueError('stationary anchor must contain finite positions')
        self.samples = deque(maxlen=256)
        self.last_stamp = None
        self.last_sequences = None
        self.reason = 'waiting'

    def observe(self, positions, velocities, stamp, now, *, sequences=None):
        """接收同一批位置, 速度, 源时间和可选逐轴序号, 返回本次证据原因.

        stamp / now 必须同属单调时钟. 重复时间或未全部推进的序号不累计窗口.
        无效值, 断流和回退清空窗口并报告原因; 上层事务须保留连续性失败.
        速度峰清空静止窗口；位置仍校验维度和有限性，不检查位置极差或锚点偏差。
        """
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
        # 每个电机独立计序号; 必须六轴都推进才算下一份证据, 重读局部更新批次不累计静止.
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
        # 只保留初始位置用于诊断输出，不用它限制累计位移。
        if self.anchor is None:
            self.anchor = q.copy()
        if np.max(np.abs(v)) > self.velocity_limit:
            self.samples.clear()
            self.reason = 'velocity'
        else:
            self.samples.append((stamp, q.copy()))
            # 保留窗口左边界的一份样本, 使覆盖时长不会因不整齐采样缩水.
            while len(self.samples) > 2 and self.samples[1][0] <= stamp-self.window:
                self.samples.popleft()
            self.reason = 'quiet'
        return self.reason

    @property
    def quiet_since(self):
        """返回当前连续静止窗口的首帧源时间, 无窗口时为 None."""
        return self.samples[0][0] if self.samples else None

    def confirmed(self, now=None):
        """要求至少三份速度合格的新帧覆盖完整窗口，可选同时检查最新年龄。"""
        return (len(self.samples) >= 3
                and (now is None or -.005 <= now-self.last_stamp <= self.max_age)
                and self.samples[-1][0]-self.samples[0][0] >= self.window-1e-9)


class DisabledTransitionEvidence:
    """现场诊断策略; 不改变生产失能事务, 不自行发出任何设备命令.

    七路输入均为原始电机 rad/rad/s. 需显式服务成功, 请求后七路新鲜失能帧,
    之后连续一秒速度合格; 孤立速度峰保留事件并延迟确认.
    断流/未知状态锁存失败，位置变化不再单独使诊断失败。
    """

    def __init__(self, positions, sequences, requested_at):
        """固定七路原始位置, 序号及失能请求时刻, 建立独立现场诊断窗口."""
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
        """保存失能服务回执, 失败即锁存; 成功回执仍需新帧状态和静止证据."""
        self.receipt = bool(success)
        if not success:
            self.failure = 'disable service failed'

    def observe(self, positions, velocities, statuses, sequences, observed_at, now):
        """以七路新帧检查连续性及失能状态；速度峰累计诊断并延后确认。"""
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
        """服务成功, 七路失能且连续静止窗口达标才确认; 已有失败不能被恢复帧清除."""
        return not self.failure and self.receipt and self.disabled and self.evidence.confirmed(now)
