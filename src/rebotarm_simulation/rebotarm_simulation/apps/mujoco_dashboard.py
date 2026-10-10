"""Compact presentation model for the MuJoCo passive Viewer."""

from __future__ import annotations

from dataclasses import dataclass

from rebotarm_simulation.control.mujoco_jog import jog_speed


DASHBOARD_PAGES = ("overview", "joints")
PLOT_PAGES = ("off", "tracking", "effort")
_XYZ = ("X", "Y", "Z")
_RPY = ("ROLL", "PITCH", "YAW")


@dataclass(frozen=True)
class TextPanel:
    left: str
    right: str


@dataclass(frozen=True)
class DashboardPanels:
    top_left: TextPanel
    top_right: TextPanel | None
    bottom_left: TextPanel | None
    bottom_right: TextPanel | None


def _motion(state) -> str:
    if state.joint_jog_direction:
        sign = "+" if state.joint_jog_direction > 0 else "-"
        return f"{selected_item(state)} {sign}  [S: stop]"
    if state.gripper_jog_direction:
        direction = "OPEN" if state.gripper_jog_direction > 0 else "CLOSE"
        return f"GRIP {direction}  [S: stop]"
    return "STOPPED"


def selected_item(state) -> str:
    if state.interaction_mode == "joint":
        return f"J{state.selected_joint + 1}"
    if state.interaction_mode == "xyz":
        return _XYZ[state.selected_cartesian_axis]
    return _RPY[state.selected_cartesian_axis]


def _speed(state) -> str:
    if state.interaction_mode == "xyz":
        name, value = jog_speed(state.jog_speed_index, state.gripper_jog_rate)
        return f"{name}  {value:.3f} m/s"
    name, value = jog_speed(state.jog_speed_index, state.joint_jog_rate)
    return f"{name}  {value:.2f} rad/s"


def status_panel(state) -> TextPanel:
    target = "ON" if state.interaction_mode != "joint" else "OFF"
    collision = "ON" if state.collision_visible else "OFF"
    left = "reBotArm\nRUN\nCONTROL\nINPUT\nSELECT\nSPEED\nMOTION\nVIEW"
    right = (
        f"MuJoCo\n{'PAUSED' if state.paused else 'RUNNING'}\n"
        f"{state.active_mode.upper()}\n"
        f"{state.interaction_mode.upper()} / {state.cartesian_frame.upper()}\n"
        f"{selected_item(state)}\n{_speed(state)}\n{_motion(state)}\n"
        f"TARGET {target} / COLL {collision}"
    )
    return TextPanel(left, right)


def overview_panel(state) -> TextPanel:
    if state.interaction_mode == "joint":
        index = state.selected_joint
        actual = state.joint_positions[index]
        target = state.joint_targets[index]
        unit = "rad"
    elif state.interaction_mode == "xyz":
        index = state.selected_cartesian_axis
        actual = state.ee_position[index]
        target = state.target_position[index]
        unit = "m"
    else:
        index = state.selected_cartesian_axis
        actual = state.ee_rpy[index]
        target = state.target_rpy[index]
        unit = "rad"
    error = target - actual
    ik = "--" if state.ik_status == "idle" else state.ik_status.upper()
    return TextPanel(
        "OVERVIEW\nITEM\nACTUAL\nTARGET\nERROR\nGRIP A/T\nCONTACT\nIK",
        f"\n{selected_item(state)} [{unit}]\n{actual:+.3f}\n{target:+.3f}\n"
        f"{error:+.3f}\n{state.gripper_actual_width:.3f}/{state.gripper_width:.3f} m\n"
        f"{state.contact_count} / {state.max_contact_force:.2f} N\n{ik}",
    )


def joints_panel(state, *, compact: bool = False) -> TextPanel:
    digits = 2 if compact else 3
    rows = []
    values = []
    for index in range(6):
        marker = ">" if state.interaction_mode == "joint" and index == state.selected_joint else " "
        rows.append(f"{marker} J{index + 1}")
        actual = state.joint_positions[index]
        target = state.joint_targets[index]
        values.append(
            f"{actual:+.{digits}f}  {target:+.{digits}f}  "
            f"{target - actual:+.{digits}f}  {state.joint_velocities[index]:+.{digits}f}"
        )
    return TextPanel(
        "JOINTS\nJOINT\n" + "\n".join(rows),
        "[rad / rad/s]\n ACT     TGT     ERR      VEL\n" + "\n".join(values),
    )


def alert_panel(state) -> TextPanel | None:
    alerts = []
    if any(state.saturated):
        joints = ",".join(f"J{i + 1}" for i, value in enumerate(state.saturated) if value)
        alerts.append(f"TORQUE SATURATION: {joints}")
    if state.ik_status not in ("idle", "converged"):
        alerts.append(f"IK {state.ik_status.upper()}: {state.ik_error:.4f}")
    if state.active_mode == "raw_torque" and state.watchdog_remaining_s < 0.03:
        alerts.append(f"TORQUE WATCHDOG: {state.watchdog_remaining_s:.3f} s")
    if not alerts:
        return None
    return TextPanel("ALERT", "\n" + "\n".join(alerts))


def compact_help_panel() -> TextPanel:
    return TextPanel(
        "M / F8\nZ/X  J/K  C/O  S\n-/+  G/H/P  V\nF6/7/9  T/R  Q",
        "mode / frame\nselect  move  grip  stop\nspeed  control  collision\npage/help/plot  home/reset  quit",
    )


def full_help_panel() -> TextPanel:
    return TextPanel(
        "CONTROLS\nM\nF8\nZ / X\nJ / K\nC / O\nS\n- / +\nG / H / P\nV\nF9\nF6\nF7\nT / R\nQ",
        "\nJoint / XYZ / RPY\nWorld / Tool frame\nPrevious / next item\nStart negative / positive jog\nClose / open gripper\nStop and Hold\nSpeed down / up\nGravity / Hold / Position\nCollision layer\nPlot: off / tracking / effort\nOverview / Joints\nClose help\nHome / reset\nQuit",
    )


def compose_dashboard(state, *, compact: bool = False) -> DashboardPanels:
    top_right = None
    bottom_right = None
    if state.help_visible:
        top_right = full_help_panel()
    elif state.plot_page == "off":
        page = state.dashboard_page
        if page == "overview":
            top_right = overview_panel(state)
        elif page == "joints":
            top_right = joints_panel(state, compact=compact)
        else:
            raise ValueError(f"unsupported dashboard page: {page}")
        bottom_right = compact_help_panel()
    return DashboardPanels(
        top_left=status_panel(state),
        top_right=top_right,
        bottom_left=alert_panel(state),
        bottom_right=bottom_right,
    )
