"""由机械臂目标生成六轴位置命令.

职责与位置
    纯输出计算层, 把执行器快照映射为六轴位置和速度上限.

对外接口
    ArmPositionControl.command 返回 PositionCommand, 保留原目标代次.

安全与并发约束
    逐轴速度取配置与任务覆盖的较小值; 不发送, 不读反馈, 驱动边界仍需再次校验.

文件结构(按定义顺序)
    ArmPositionControl: 把六轴目标快照转换为位置命令的无状态计算适配.
"""

from ..contracts.controller_contracts import PositionCommand


class ArmPositionControl:
    """把六轴目标快照转换为位置命令的无状态计算适配."""

    def __init__(self, velocity_limits):
        """复制配置中的逐轴速度上限, 后续任务覆盖不得扩大这些上限."""
        self.velocity_limits = tuple(float(v) for v in velocity_limits)

    def command(self, target):
        """生成命令快照; 任务覆盖不能提高配置中的逐轴速度上限."""
        limits = self.velocity_limits if target.velocity_override is None else tuple(
            min(configured, requested) for configured, requested in zip(self.velocity_limits,target.velocity_override))
        return PositionCommand(target.positions,limits,target.generation)
