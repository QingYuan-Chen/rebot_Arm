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
    """Compatibility boundary between ROS and the evolving simulation core."""

    def __init__(self, simulation: Any) -> None:
        self.simulation = simulation

    def reset_home_and_hold(self) -> None:
        self.simulation.reset_home()
        self.set_mode("hold")

    def set_mode(self, mode: str) -> str:
        public_mode = normalize_ros_control_mode(mode)
        if hasattr(self.simulation, "set_mode"):
            return str(self.simulation.set_mode(public_mode))
        legacy = "pos_vel" if public_mode == "position" else public_mode
        return str(self.simulation.set_control_mode(legacy))

    def command_joint_positions(self, values: Sequence[float]) -> tuple[float, ...]:
        if hasattr(self.simulation, "command_joint_positions"):
            reached = self.simulation.command_joint_positions(values)
        else:
            reached = self.simulation.set_joint_position_targets(values)
        self.set_mode("position")
        return tuple(float(value) for value in reached)

    def hold_current_position(self) -> None:
        if hasattr(self.simulation, "set_mode"):
            self.simulation.set_mode("hold")
            return
        current = tuple(self.simulation.get_state().joint_positions[:6])
        self.simulation.set_joint_position_targets(current)
        self.simulation.set_control_mode("hold")

    def command_gripper_width(
        self, width: float, max_force_n: float | None = None
    ) -> float:
        if hasattr(self.simulation, "command_gripper_width"):
            return float(
                self.simulation.command_gripper_width(
                    width, max_force_n=max_force_n
                )
            )
        return float(self.simulation.set_gripper_width(width))

    def get_control_status(self) -> Any:
        if hasattr(self.simulation, "get_control_status"):
            return self.simulation.get_control_status()
        return {
            "mode": "position" if self.simulation.control_mode == "pos_vel" else self.simulation.control_mode,
            "joint_targets": tuple(self.simulation.control_targets[:6]),
            "saturated": False,
            "watchdog_remaining_s": 0.0,
        }
