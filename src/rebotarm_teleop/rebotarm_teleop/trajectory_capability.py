"""网页和键盘入口共用的轨迹执行能力核对。

文件职责
    对照前端配置与后端发布的 legacy/prepared_v1，防止生成规则与执行规则错配。
对外接口
    TrajectoryCapability(expected) 保存期望模式；update 接收 String 消息；
    require 在未观察到能力或模式不一致时抛出 ValueError，由入口报告拒绝原因。
安全约束
    仅核对协议能力，不证明后端健康、使能或资源空闲；这些条件仍由正式准入检查。
    不依据执行失败切换模式，不自动回退，不自行排队重试操作员输入。
文件结构
    单个能力状态对象：初始化 → 接收观察 → 请求前核验。
"""


import time
from rebotarm_trajectory.capability import decode_capability


class TrajectoryCapability:
    """由入口节点持有的能力快照；初始 observed=None 表示尚无后端声明。"""
    def __init__(self, expected):
        if expected not in ('legacy', 'prepared_v1'):
            raise ValueError('trajectory_execution_mode must be legacy or prepared_v1')
        self.expected = expected
        self.observed = None
        self.snapshot = None
        self.observed_at = 0.

    def update_snapshot(self, message):
        """版本化能力的接收时刻独立于旧模式话题；非法声明清掉旧许可。"""
        self.snapshot = None
        self.observed_at = 0.
        try:
            self.snapshot = decode_capability(message.data)
            self.observed_at = time.monotonic()
        except (ValueError,TypeError,KeyError):
            pass

    def require_snapshot(self):
        """返回当前生效能力；过期为未知，不用最后一次快照冒充当前配置。"""
        self.require()
        if self.snapshot is None or time.monotonic()-self.observed_at > 1.:
            raise ValueError('fresh versioned trajectory capability unavailable')
        if self.snapshot['mode'] != self.expected:
            raise ValueError('versioned trajectory capability mode mismatch')
        return dict(self.snapshot)

    def update(self, message):
        """保留实际收到的值，未知字符串也留待 require 明确拒绝。"""
        self.observed = message.data

    def require(self):
        """只允许已观察值与配置精确相等；不会把 legacy 当成通用降级选项。"""
        if self.observed != self.expected:
            raise ValueError(f'trajectory mode mismatch: expected {self.expected}, observed {self.observed}')
