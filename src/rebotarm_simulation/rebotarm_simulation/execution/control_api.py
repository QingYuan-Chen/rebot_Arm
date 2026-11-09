"""Finite, non-torque simulation control surface shared by ROS execution."""
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

    def hold_current_position(self) -> None:
        self.simulation.set_mode("hold")

    def command_gripper_width(
        self, width: float, max_force_n: float | None = None
    ) -> float:
        return float(self.simulation.command_gripper_width(width, max_force_n=max_force_n))

    def get_control_status(self) -> Any:
        return self.simulation.get_control_status()
