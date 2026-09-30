"""ROS-independent diagnostic evaluation for the MuJoCo ROS adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class DiagnosticValue:
    key: str
    value: str


@dataclass(frozen=True)
class ControlDiagnostic:
    """Message-neutral representation of one control diagnostic status."""

    warning: bool
    name: str
    hardware_id: str
    message: str
    values: tuple[DiagnosticValue, ...]


def _status_value(status: Any, name: str, default: Any) -> Any:
    if status is None:
        return default
    if isinstance(status, dict):
        return status.get(name, default)
    return getattr(status, name, default)


def _bool_count(value: Any) -> int:
    if isinstance(value, (tuple, list)):
        return sum(bool(item) for item in value)
    return int(bool(value))


def build_control_diagnostic(
    *,
    arm_namespace: str,
    configured_rate_hz: float,
    measured_rate_hz: float,
    state: Any,
    status: Any,
    contacts: Iterable[Any],
    max_contact_force_n: float,
    max_contact_penetration_m: float,
) -> ControlDiagnostic:
    """Evaluate control/contact health without importing ROS message types."""
    mode = str(_status_value(status, "mode", "unknown"))
    actual_positions = tuple(state.joint_positions[:6])
    targets = tuple(
        _status_value(status, "joint_targets", actual_positions)
    )
    if len(targets) != 6:
        targets = actual_positions
    max_error = max(
        abs(float(target) - float(actual))
        for target, actual in zip(targets, actual_positions)
    )
    saturated_count = _bool_count(
        _status_value(
            status,
            "saturated",
            _status_value(status, "saturation", False),
        )
    )
    watchdog_value = _status_value(status, "watchdog_remaining_s", 0.0)
    watchdog = 0.0 if watchdog_value is None else float(watchdog_value)
    contact_items = tuple(contacts)
    max_force = max(
        (float(contact.force) for contact in contact_items), default=0.0
    )
    max_penetration = max(
        (float(contact.penetration_depth) for contact in contact_items),
        default=0.0,
    )
    contact_anomaly = (
        max_force > float(max_contact_force_n)
        or max_penetration > float(max_contact_penetration_m)
    )
    warning = bool(saturated_count or contact_anomaly)
    message = (
        "contact anomaly"
        if contact_anomaly
        else (
            "actuator saturation"
            if saturated_count
            else "simulation control healthy"
        )
    )
    return ControlDiagnostic(
        warning=warning,
        name=f"{arm_namespace}/mujoco_control",
        hardware_id="mujoco",
        message=message,
        values=(
            DiagnosticValue(key="mode", value=mode),
            DiagnosticValue(
                key="configured_rate_hz",
                value=f"{float(configured_rate_hz):.3f}",
            ),
            DiagnosticValue(
                key="measured_rate_hz",
                value=f"{float(measured_rate_hz):.3f}",
            ),
            DiagnosticValue(
                key="saturated_actuators", value=str(saturated_count)
            ),
            DiagnosticValue(
                key="watchdog_remaining_s",
                value=f"{max(0.0, watchdog):.6f}",
            ),
            DiagnosticValue(
                key="max_tracking_error_rad", value=f"{max_error:.6f}"
            ),
            DiagnosticValue(
                key="contact_anomaly",
                value="true" if contact_anomaly else "false",
            ),
        ),
    )
