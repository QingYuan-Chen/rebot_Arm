"""操作入口必须观察到与配置一致的执行能力，不因失败自动回退。"""


class TrajectoryCapability:
    def __init__(self, expected):
        if expected not in ('legacy', 'prepared_v1'):
            raise ValueError('trajectory_execution_mode must be legacy or prepared_v1')
        self.expected = expected
        self.observed = None

    def update(self, message):
        self.observed = message.data

    def require(self):
        if self.observed != self.expected:
            raise ValueError(f'trajectory mode mismatch: expected {self.expected}, observed {self.observed}')
