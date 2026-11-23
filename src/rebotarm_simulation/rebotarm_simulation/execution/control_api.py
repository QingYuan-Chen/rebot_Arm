"""仿真 ROS 执行接口:位置,保持,重力与持续态关节阻抗命令.

职责:只转发到仿真核心;调用方持有目标锁和仿真锁,核心原子校验数值.
对外接口:normalize_ros_control_mode, SimulationControlApi.
安全约束:写目标不得在已处于 position 模式时额外触发 set_mode,避免高频轨迹采样重入 100 Hz 控制拍.
文件结构:模式校验 → 仿真命令适配;raw torque 仍由本地诊断入口拥有.
"""
from typing import Any, Sequence
ROS_CONTROL_MODES = ("position", "hold", "gravity_comp")

def normalize_ros_control_mode(mode: Any) -> str:
    """仅接受 ROS 公共接口声明的位置,保持和重力补偿模式."""
    value = str(mode).strip().lower()
    if value not in ROS_CONTROL_MODES:
        raise ValueError(
            "ROS simulation mode must be position, hold, or gravity_comp; "
            "raw_torque is available only through the local diagnostic API"
        )
    return value


class SimulationControlApi:
    """将 ROS 执行意图交给仿真核心,返回实际接纳的数值."""

    def __init__(self, simulation: Any) -> None:
        self.simulation = simulation

    def reset_home_and_hold(self) -> None:
        """复位到 home 姿态并进入 hold;核心 reset 已完成重力前馈预置与 hold 模式."""
        self.simulation.reset()

    def set_mode(self, mode: str) -> str:
        public_mode = normalize_ros_control_mode(mode)
        return str(self.simulation.set_mode(public_mode))

    def command_joint_positions(self, values: Sequence[float]) -> tuple[float, ...]:
        """仅写入六轴目标;首次切入 position 由核心按需完成,避免每物理步重入控制拍."""
        return tuple(float(value) for value in self.simulation.command_joint_positions(values))

    def command_joint_impedance(self, positions, stiffness, damping):
        """完整命令同时设置平衡点与增益;允许由空闲 position 或 hold 切入或原地热更新."""
        return self.simulation.command_joint_impedance(
            positions, stiffness, damping, allow_from_position=True
        )

    def hold_current_position(self) -> None:
        self.simulation.set_mode("hold")

    def command_gripper_width(
        self, width: float, max_force_n: float | None = None
    ) -> float:
        return float(self.simulation.command_gripper_width(width, max_force_n=max_force_n))

    def get_control_status(self) -> Any:
        return self.simulation.get_control_status()
