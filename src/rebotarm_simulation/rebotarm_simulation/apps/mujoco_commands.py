from __future__ import annotations

from dataclasses import dataclass
import shlex

from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES


@dataclass(frozen=True)
class CommandResult:
    paused: bool
    value: object | None = None
    should_quit: bool = False
    mutated: bool = False


def _positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as exc:
        raise ValueError("must be a positive integer") from exc
    if number <= 0:
        raise ValueError("must be a positive integer")
    return number


def dispatch_sim_command(sim, line: str, *, paused: bool = False) -> CommandResult:
    parts = shlex.split(line)
    if not parts:
        return CommandResult(paused)
    command, arguments = parts[0].lower(), parts[1:]
    if command in ("quit", "q"):
        if arguments:
            raise ValueError("usage: quit")
        return CommandResult(paused, "bye", should_quit=True)
    if command == "state":
        if arguments:
            raise ValueError("usage: state")
        return CommandResult(paused, sim.get_state())
    if command == "control":
        if arguments:
            raise ValueError("usage: control")
        return CommandResult(paused, sim.get_control_status())
    if command == "joint":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: joint NAME VALUE")
        return CommandResult(
            paused,
            sim.command_joint_positions({arguments[0]: float(arguments[1])}),
            mutated=True,
        )
    if command == "joints":
        if len(arguments) != 6:
            raise ValueError("usage: joints J1 J2 J3 J4 J5 J6")
        values = [float(value) for value in arguments]
        return CommandResult(
            paused,
            sim.command_joint_positions(values),
            mutated=True,
        )
    if command == "jog":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: jog NAME DELTA")
        state = sim.get_state()
        index = ARM_JOINT_NAMES.index(arguments[0])
        target = float(state.joint_positions[index]) + float(arguments[1])
        return CommandResult(
            paused,
            sim.command_joint_positions({arguments[0]: target}),
            mutated=True,
        )
    if command == "gripper":
        if len(arguments) != 1:
            raise ValueError("usage: gripper WIDTH")
        return CommandResult(
            paused,
            sim.command_gripper_width(float(arguments[0])),
            mutated=True,
        )
    if command == "mode":
        if len(arguments) != 1 or arguments[0] not in {"gravity_comp", "hold", "position"}:
            raise ValueError("usage: mode gravity_comp|hold|position")
        return CommandResult(
            paused,
            sim.set_mode(arguments[0]),
            mutated=True,
        )
    if command == "home":
        if arguments or not hasattr(sim, "reset_home"):
            raise ValueError("usage: home")
        state = sim.reset_home()
        sim.set_mode("hold")
        return CommandResult(paused, state, mutated=True)
    if command == "step":
        if len(arguments) > 1:
            raise ValueError("usage: step [N]")
        count = 1 if not arguments else _positive_int(arguments[0])
        if paused:
            return CommandResult(paused, "paused; step ignored")
        return CommandResult(paused, sim.step(count), mutated=True)
    if command == "reset":
        if arguments:
            raise ValueError("usage: reset")
        return CommandResult(paused, sim.reset(), mutated=True)
    if command == "contacts":
        if arguments:
            raise ValueError("usage: contacts")
        return CommandResult(paused, sim.get_contacts())
    if command == "pause":
        if arguments:
            raise ValueError("usage: pause")
        return CommandResult(True, "paused")
    if command == "resume":
        if arguments:
            raise ValueError("usage: resume")
        return CommandResult(False, "running")
    raise ValueError(f"unknown command: {command}")
