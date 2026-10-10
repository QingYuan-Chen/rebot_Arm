from __future__ import annotations

import io
import json
import math
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
from types import MappingProxyType

import pytest

from rebotarm_simulation.apps import mujoco_cli
from rebotarm_simulation.diagnostics import mujoco_health
from rebotarm_simulation.core.mujoco_types import SimulationState


class FakeSim:
    joint_names = tuple(f"joint{i}" for i in range(1, 7)) + (
        "left_finger_joint", "right_finger_joint"
    )

    def __init__(self, model_path=None):
        self.model_path = str(model_path or "/fake/scene.xml")
        self.timestep = 0.01
        self.targets = [0.0] * 6
        self.width = 0.09
        self.time = 0.0
        self.calls = []
        self.closed = False
        self.control_mode = "hold"
        self.requested_torques = (0.0,) * 6

    def get_state(self):
        return SimpleNamespace(
            joint_names=self.joint_names,
            joint_positions=tuple(self.targets) + (self.width / 2, -self.width / 2),
            joint_velocities=(0.0,) * 8,
            actuator_forces=(0.0,) * 8,
            simulation_time=self.time,
        )

    def set_joint_position_targets(self, targets):
        self.calls.append(("joints", targets))
        if isinstance(targets, dict):
            for name, value in targets.items():
                self.targets[int(name.removeprefix("joint")) - 1] = float(value)
        else:
            self.targets[:] = list(map(float, targets))
        return tuple(self.targets)

    def set_gripper_width(self, width):
        self.calls.append(("gripper", float(width)))
        self.width = float(width)
        return self.width

    command_joint_positions = set_joint_position_targets
    command_gripper_width = set_gripper_width

    def set_mode(self, mode):
        self.calls.append(("mode", mode))
        self.control_mode = str(mode)
        return self.control_mode

    def command_joint_torques(self, values, timeout_s=0.1):
        self.calls.append(("torques", tuple(values), float(timeout_s)))
        self.requested_torques = tuple(float(value) for value in values)
        self.control_mode = "raw_torque"
        return self.requested_torques

    def get_control_status(self):
        return SimpleNamespace(
            mode=self.control_mode,
            requested_torques=self.requested_torques,
            applied_torques=self.requested_torques,
            saturated=(False,) * 6,
            watchdog_remaining_s=0.1 if self.control_mode == "raw_torque" else 0.0,
        )

    def step(self, n_steps=1):
        self.calls.append(("step", n_steps))
        self.time += n_steps * self.timestep
        return self.get_state()

    def reset(self):
        self.calls.append(("reset",))
        self.targets[:] = [0.0] * 6
        self.time = 0.0
        return self.get_state()

    reset_home = reset

    def get_contacts(self):
        self.calls.append(("contacts",))
        return (SimpleNamespace(body1="finger", body2="cube", force=1.25),)

    def close(self):
        self.closed = True


def test_parser_accepts_headless_duration_steps_and_model():
    args = mujoco_cli.build_parser().parse_args(
        ["run", "--duration", "1.5", "--steps", "3", "--model", "scene.xml"]
    )
    assert args.command == "run"
    assert args.duration == 1.5
    assert args.steps == 3
    assert args.model == "scene.xml"


def test_parser_rejects_unimplemented_real_time_option():
    with pytest.raises(SystemExit):
        mujoco_cli.build_parser().parse_args(["run", "--real-time"])


@pytest.mark.parametrize("value", ["-1", "nan", "inf"])
def test_parser_rejects_invalid_duration(value):
    with pytest.raises(SystemExit):
        mujoco_cli.build_parser().parse_args(["run", "--duration", value])


def test_headless_duration_uses_simulation_time_and_closes():
    created = []

    def factory(model_path=None):
        created.append(FakeSim(model_path))
        return created[-1]

    output = io.StringIO()
    code = mujoco_cli.main(
        ["run", "--duration", "0.025"], sim_factory=factory, stdout=output
    )
    assert code == 0
    assert created[0].time == pytest.approx(0.03)
    assert created[0].closed is True
    payload = json.loads(output.getvalue())
    assert payload["simulation_time"] == pytest.approx(0.03)
    assert payload["requested_duration"] == pytest.approx(0.025)
    assert payload["achieved_duration"] == pytest.approx(0.03)


def test_headless_steps_advance_exact_count():
    sim = FakeSim()
    assert mujoco_cli.main(
        ["run", "--steps", "4"], sim_factory=lambda _: sim, stdout=io.StringIO()
    ) == 0
    assert ("step", 4) in sim.calls


def test_interactive_commands_delegate_to_simulation():
    sim = FakeSim()
    commands = io.StringIO(
        "state\njoint joint2 0.5\njoints 1 2 3 4 5 6\njog joint3 -0.25\n"
        "gripper 0.04\nstep 2\ncontacts\nreset\npause\nresume\nquit\n"
    )
    output = io.StringIO()
    code = mujoco_cli.main(["shell"], sim_factory=lambda _: sim, stdin=commands, stdout=output)
    assert code == 0
    assert ("joints", {"joint2": 0.5}) in sim.calls
    assert ("joints", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) in sim.calls
    assert ("joints", {"joint3": 2.75}) in sim.calls
    assert ("gripper", 0.04) in sim.calls
    assert ("step", 2) in sim.calls
    assert ("contacts",) in sim.calls
    assert ("reset",) in sim.calls
    assert "paused" in output.getvalue()


def test_interactive_bad_command_is_reported_and_loop_continues():
    output = io.StringIO()
    code = mujoco_cli.main(
        ["shell"], sim_factory=lambda _: FakeSim(), stdin=io.StringIO("step 0\nnope\nquit\n"), stdout=output
    )
    assert code == 0
    assert output.getvalue().count("error:") == 2


def test_legacy_mujoco_teach_commands_are_rejected_in_favor_of_ros_teach_workflow():
    output = io.StringIO()
    code = mujoco_cli.main(
        ["shell"],
        sim_factory=lambda _: FakeSim(),
        stdin=io.StringIO("record start\nreplay start\ntrajectory state\nquit\n"),
        stdout=output,
    )

    assert code == 0
    assert output.getvalue().count("error: unknown command:") == 3


def test_pause_blocks_steps_until_resume_and_reset_preserves_pause():
    sim = FakeSim()
    paused, _, _ = mujoco_cli.dispatch_command(sim, "pause")
    paused, result, _ = mujoco_cli.dispatch_command(sim, "step 3", paused=paused)
    assert paused is True
    assert result == "paused; step ignored"
    assert sim.time == 0.0
    paused, _, _ = mujoco_cli.dispatch_command(sim, "reset", paused=paused)
    assert paused is True
    paused, _, _ = mujoco_cli.dispatch_command(sim, "resume", paused=paused)
    paused, _, _ = mujoco_cli.dispatch_command(sim, "step 3", paused=paused)
    assert paused is False
    assert sim.time == pytest.approx(0.03)


def test_plain_serializes_real_state_mappingproxy_and_path_without_deepcopy():
    state = SimulationState(
        joint_names=("joint1",),
        joint_positions=(0.0,),
        joint_velocities=(0.0,),
        actuator_forces=(0.0,),
        end_effector_position=(0.0, 0.0, 0.0),
        end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
        gripper_width=0.09,
        object_poses=MappingProxyType({"cube": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)}),
        simulation_time=0.0,
    )
    payload = mujoco_cli._plain({"state": state, "model": Path("scene.xml")})
    assert payload["state"]["object_poses"]["cube"][-1] == 1.0
    assert payload["model"] == "scene.xml"
    json.dumps(payload)


def test_cli_outputs_real_simulation_state_with_mappingproxy():
    class RealStateSim(FakeSim):
        def get_state(self):
            return SimulationState(
                joint_names=self.joint_names,
                joint_positions=tuple(self.targets) + (0.045, -0.045),
                joint_velocities=(0.0,) * 8,
                actuator_forces=(0.0,) * 8,
                end_effector_position=(0.0, 0.0, 0.0),
                end_effector_orientation=(0.0, 0.0, 0.0, 1.0),
                gripper_width=0.09,
                object_poses=MappingProxyType({"cube": (0.0,) * 6 + (1.0,)}),
                simulation_time=self.time,
            )

    output = io.StringIO()
    assert mujoco_cli.main(
        ["run", "--steps", "1"], sim_factory=lambda _: RealStateSim(), stdout=output
    ) == 0
    assert json.loads(output.getvalue())["object_poses"]["cube"][-1] == 1.0


def test_cli_dependency_or_model_error_returns_nonzero():
    def broken(_=None):
        raise RuntimeError("MuJoCo is required")

    error = io.StringIO()
    assert mujoco_cli.main(["run"], sim_factory=broken, stderr=error) != 0
    assert "MuJoCo is required" in error.getvalue()


def test_torque_subcommand_uses_watchdog_limited_api_and_reports_control():
    sim = FakeSim()
    output = io.StringIO()
    assert mujoco_cli.main(
        [
            "torque", "--values", "1", "2", "3", "4", "5", "6",
            "--timeout", "0.1", "--observe", "0.02",
        ],
        sim_factory=lambda _: sim,
        stdout=output,
    ) == 0
    assert ("torques", (1.0, 2.0, 3.0, 4.0, 5.0, 6.0), 0.1) in sim.calls
    payload = json.loads(output.getvalue())
    assert payload["control"]["mode"] == "raw_torque"


def test_collect_health_is_json_friendly_and_checks_expected_counts():
    result = mujoco_health.collect_health(
        sim_factory=lambda _: FakeSim(),
        mujoco_version="3.3.2",
        renderer_check=lambda _: (False, "no EGL device"),
    )
    assert result["python_version"]
    assert result["mujoco_version"] == "3.3.2"
    assert result["model_loaded"] is True
    assert result["physics_step_finite"] is True
    assert result["simulation_time"] == pytest.approx(0.01)
    assert result["joint_count"] == result["expected_joint_count"] == 8
    assert result["actuator_count"] == result["expected_actuator_count"] == 8
    assert result["headless"] is True
    assert result["renderer_available"] is False
    assert result["renderer_error"] == "no EGL device"
    json.dumps(result)


@pytest.mark.parametrize(
    ("completed", "available", "error_fragment"),
    [
        (
            SimpleNamespace(
                returncode=0,
                stdout='{"ok":true,"shape":[64,64,3],"backend":"default"}\n',
                stderr="",
            ),
            True,
            None,
        ),
        (SimpleNamespace(returncode=2, stdout="", stderr="GL unavailable"), False, "return code 2"),
        (SimpleNamespace(returncode=-6, stdout="", stderr="abort"), False, "signal 6"),
    ],
)
def test_renderer_probe_isolated_process_results(completed, available, error_fragment):
    calls = []

    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return completed

    result = mujoco_health.probe_renderer("scene.xml", runner=runner, timeout=1.25)
    assert result["available"] is available
    assert result["returncode"] == completed.returncode
    assert result["stdout"] == completed.stdout.strip()
    assert result["stderr"] == completed.stderr.strip()
    assert calls[0][1]["timeout"] == 1.25
    if error_fragment:
        assert error_fragment in result["error"]


def test_renderer_probe_timeout_is_reported_without_escaping():
    def runner(command, **kwargs):
        raise subprocess.TimeoutExpired(command, kwargs["timeout"], output="partial", stderr="hung")

    result = mujoco_health.probe_renderer("scene.xml", runner=runner, timeout=0.01)
    assert result["available"] is False
    assert result["timed_out"] is True
    assert "timed out" in result["error"]


def test_renderer_probe_passes_untrusted_model_only_as_argv_and_backend_as_env():
    captured = {}
    model_path = "odd ' model; $HOME.xml"

    def runner(command, **kwargs):
        captured["command"] = command
        captured["kwargs"] = kwargs
        return SimpleNamespace(
            returncode=0,
            stdout='{"ok":true,"shape":[64,64,3],"backend":"egl"}\n',
            stderr="",
        )

    result = mujoco_health.probe_renderer(
        model_path, backend="egl", runner=runner, timeout=2.0
    )
    command = captured["command"]
    assert command[-1] == model_path
    assert model_path not in command[2]
    assert "from_xml_path(sys.argv[1])" in command[2]
    assert "MjData(model)" in command[2]
    assert "mj_forward(model, data)" in command[2]
    assert "Renderer(model, height=64, width=64)" in command[2]
    assert "update_scene(data, camera='overview')" in command[2]
    assert "np.isfinite(rgb).all()" in command[2]
    assert "renderer.close()" in command[2]
    assert captured["kwargs"]["env"]["MUJOCO_GL"] == "egl"
    assert captured["kwargs"]["env"] is not os.environ
    assert result["available"] is True
    assert result["details"]["shape"] == [64, 64, 3]
    assert result["details"]["backend"] == "egl"


def test_renderer_probe_success_requires_valid_child_json_contract():
    def runner(command, **kwargs):
        return SimpleNamespace(returncode=0, stdout="renderer-ok", stderr="")

    result = mujoco_health.probe_renderer("scene.xml", runner=runner)
    assert result["available"] is False
    assert "invalid success output" in result["error"]


def test_renderer_probe_executes_child_when_mujoco_runtime_is_available():
    pytest.importorskip("mujoco")
    scene = (
        Path(__file__).parents[1]
        / "src"
        / "rebotarm_simulation"
        / "models"
        / "rebotarm"
        / "scene.xml"
    )
    result = mujoco_health.probe_renderer(scene, timeout=30.0)
    assert result["available"], result
    assert result["details"]["shape"] == [64, 64, 3]


@pytest.mark.parametrize("error", ["renderer probe terminated by signal 6", "renderer probe timed out"])
def test_health_main_survives_renderer_subprocess_failure(error):
    output = io.StringIO()
    probe = {
        "available": False,
        "timed_out": "timed out" in error,
        "returncode": -6 if "signal" in error else None,
        "signal": 6 if "signal" in error else None,
        "stdout": "",
        "stderr": "native failure",
        "error": error,
    }
    code = mujoco_health.main(
        [],
        sim_factory=lambda _: FakeSim(),
        renderer_check=lambda _sim: probe,
        stdout=output,
    )
    assert code == 0
    payload = json.loads(output.getvalue())
    assert payload["ok"] is True
    assert payload["renderer_available"] is False
    assert payload["renderer_error"] == error


def test_health_main_emits_json_and_nonzero_on_failure():
    output = io.StringIO()
    assert mujoco_health.main([], sim_factory=lambda _: FakeSim(), stdout=output) == 0
    assert json.loads(output.getvalue())["ok"] is True

    def broken(_=None):
        raise FileNotFoundError("scene.xml missing")

    output = io.StringIO()
    assert mujoco_health.main([], sim_factory=broken, stdout=output) != 0
    failure = json.loads(output.getvalue())
    assert failure["ok"] is False
    assert "scene.xml missing" in failure["error"]


def test_health_parser_renderer_timeout_default_and_override():
    assert mujoco_health.build_parser().parse_args([]).renderer_timeout == 30.0
    assert (
        mujoco_health.build_parser().parse_args(["--renderer-timeout", "45.5"]).renderer_timeout
        == 45.5
    )


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_health_parser_rejects_invalid_renderer_timeout(value):
    with pytest.raises(SystemExit):
        mujoco_health.build_parser().parse_args(["--renderer-timeout", value])


def test_health_main_passes_renderer_timeout_to_isolated_probe(monkeypatch):
    captured = []

    def probe(model_path, *, timeout, **_kwargs):
        captured.append((model_path, timeout))
        return {
            "available": True,
            "timed_out": False,
            "returncode": 0,
            "signal": None,
            "stdout": '{"ok":true,"shape":[64,64,3],"backend":"egl"}',
            "stderr": "",
            "error": None,
            "details": {"ok": True, "shape": [64, 64, 3], "backend": "egl"},
        }

    monkeypatch.setattr(mujoco_health, "probe_renderer", probe)
    assert mujoco_health.main(
        ["--renderer-timeout", "42"], sim_factory=lambda _: FakeSim(), stdout=io.StringIO()
    ) == 0
    assert captured == [("/fake/scene.xml", 42.0)]
