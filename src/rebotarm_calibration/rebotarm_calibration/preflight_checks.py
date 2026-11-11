"""仅保存本次试采各阶段的诊断，正式采样重新执行原质量门，会话落盘归 SessionStore。"""

class PreflightChecks:
    def __init__(self, tcp=False):
        self.tcp = tcp
        self.reset()

    def reset(self):
        self.items = {key: {'state': 'skipped' if self.tcp and key in ('image', 'camera_info', 'aruco') else 'pending',
                            'detail': 'TCP 模式不需要此项' if self.tcp and key in ('image', 'camera_info', 'aruco') else '尚未检查'}
                      for key in ('image', 'camera_info', 'tf', 'aruco', 'stability')}
        self.current = None

    def start(self, key):
        self.current = key
        self.items[key] = {'state': 'waiting', 'detail': '等待本次请求后的有效数据'}

    def passed(self, detail):
        self.items[self.current] = {'state': 'passed', 'detail': detail}

    def failed(self, reason):
        if self.current:
            self.items[self.current] = {'state': 'failed', 'detail': str(reason)}


class PreflightFailure(TimeoutError):
    def __init__(self, reason, checks):
        super().__init__(reason)
        self.checks = checks.items
