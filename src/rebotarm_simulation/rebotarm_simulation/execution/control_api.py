"""仿真 ROS 执行接口：位置、保持、重力与具期限的关节阻抗命令。

职责：只转发到仿真核心；调用方持有目标锁和仿真锁，核心原子校验数值。
结构：模式校验 → 仿真命令适配；raw torque 仍由本地诊断入口拥有。
"""
from typing import Any, Sequence
ROS_CONTROL_MODES = ("position", "hold", "gravity_comp")

def normalize_ros_control_mode(mode: Any) -> str:
    """Validate the deliberately small, non-torque ROS control surface."""
    value = str(mode).strip().lower()
    if value not in ROS_CONTROL_MODES:
        raise ValueError(
            "ROS simulation mode must be position, hold, or gravity_comp; "
            "raw_torque is available only through the local diagnostic API"
        )
    return value


class SimulationControlApi:
    """ROS-facing operations on the simulation core."""

    def __init__(self, simulation: Any) -> None:
        self.simulation = simulation

    def reset_home_and_hold(self) -> None:
        self.simulation.reset_home()
        self.set_mode("hold")

    def set_mode(self, mode: str) -> str:
        public_mode = normalize_ros_control_mode(mode)
        return str(self.simulation.set_mode(public_mode))

    def command_joint_positions(self, values: Sequence[float]) -> tuple[float, ...]:
        reached = self.simulation.command_joint_positions(values)
        self.set_mode("position")
        return tuple(float(value) for value in reached)

    def command_joint_impedance(self, positions, stiffness, damping, *, timeout_s):
        """完整命令同时设置平衡点、增益和期限；裸模式切换保持原接口。"""
        return self.simulation.command_joint_impedance(
            positions, stiffness, damping, timeout_s=timeout_s)

    def hold_current_position(self) -> None:
        self.simulation.set_mode("hold")

    def command_gripper_width(
        self, width: float, max_force_n: float | None = None
    ) -> float:
        return float(self.simulation.command_gripper_width(width, max_force_n=max_force_n))

    def get_control_status(self) -> Any:
        return self.simulation.get_control_status()
