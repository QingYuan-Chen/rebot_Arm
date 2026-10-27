"""生成位置命令；设备读写由驱动负责。"""
from .controller_contracts import PositionCommand

class ArmPositionControl:
    def __init__(self, velocity_limits):
        self.velocity_limits = tuple(float(v) for v in velocity_limits)

    def command(self, target):
        limits = self.velocity_limits if target.velocity_override is None else tuple(
            min(configured, requested) for configured, requested in zip(self.velocity_limits,target.velocity_override))
        return PositionCommand(target.positions,limits,target.generation)
