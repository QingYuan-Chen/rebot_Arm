"""由机械臂目标生成六轴位置命令。

``ArmPositionControl`` 接收配置的逐轴速度上限，``command`` 把
``ArmTarget`` 转成携带同一代次的 ``PositionCommand``。任务速度覆盖存在时，
逐轴取配置上限与覆盖值的较小者；没有覆盖时沿用配置上限。

这里不读取反馈、不插值轨迹，也不访问设备。目标合法性由执行器检查，
最终位置和速度边界由设备驱动在发送前再次检查。
"""
from .controller_contracts import PositionCommand

class ArmPositionControl:
    def __init__(self, velocity_limits):
        self.velocity_limits = tuple(float(v) for v in velocity_limits)

    def command(self, target):
        """生成命令快照；任务覆盖不能提高配置中的逐轴速度上限。"""
        limits = self.velocity_limits if target.velocity_override is None else tuple(
            min(configured, requested) for configured, requested in zip(self.velocity_limits,target.velocity_override))
        return PositionCommand(target.positions,limits,target.generation)
