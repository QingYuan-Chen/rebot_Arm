from __future__ import annotations

import argparse
import json
import math
import os
import platform
import subprocess
import sys
from typing import Callable

from .mujoco_sim import RebotArmMujoco
from .mujoco_legacy_health import check_model_health


EXPECTED_JOINT_COUNT = 8
EXPECTED_ACTUATOR_COUNT = 8
DEFAULT_RENDERER_TIMEOUT = 30.0


def _positive_finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def _version() -> str:
    try:
        import mujoco
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            "MuJoCo is required; install docs/setup/dependencies/mujoco.md in the active environment"
        ) from exc
    return str(mujoco.__version__)


_RENDERER_PROBE = """\
import json
import mujoco
import numpy as np
import os
import sys

model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=64, width=64)
try:
    try:
        renderer.update_scene(data, camera='overview')
    except ValueError:
        renderer.update_scene(data)
    rgb = renderer.render()
    if tuple(rgb.shape) != (64, 64, 3):
        raise RuntimeError(f'unexpected RGB shape: {rgb.shape}')
    if not bool(np.isfinite(rgb).all()):
        raise RuntimeError('RGB output contains non-finite values')
finally:
    renderer.close()
print(json.dumps({
    'ok': True,
    'shape': [64, 64, 3],
    'backend': os.environ.get('MUJOCO_GL', 'default'),
}, separators=(',', ':')))
"""


def _text(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode(errors="replace").strip()
    return str(value).strip()


def probe_renderer(
    model_path,
    *,
    backend: str | None = None,
    timeout: float = DEFAULT_RENDERER_TIMEOUT,
    runner=subprocess.run,
    command=None,
) -> dict[str, object]:
    """Probe GL in a child so a native driver abort cannot kill this process."""
    probe_command = list(command or (sys.executable, "-c", _RENDERER_PROBE))
    probe_command.append(str(model_path))
    child_environment = os.environ.copy()
    if backend is not None:
        child_environment["MUJOCO_GL"] = backend
    try:
        completed = runner(
            probe_command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
            env=child_environment,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "available": False,
            "timed_out": True,
            "returncode": None,
            "signal": None,
            "stdout": _text(exc.output),
            "stderr": _text(exc.stderr),
            "error": f"renderer probe timed out after {timeout:g}s",
        }
    returncode = int(completed.returncode)
    signal = -returncode if returncode < 0 else None
    details = None
    if returncode == 0:
        try:
            details = json.loads(_text(completed.stdout))
            valid = (
                isinstance(details, dict)
                and details.get("ok") is True
                and details.get("shape") == [64, 64, 3]
                and isinstance(details.get("backend"), str)
            )
        except (json.JSONDecodeError, TypeError):
            valid = False
        error = None if valid else "renderer probe returned invalid success output"
    elif signal is not None:
        error = f"renderer probe terminated by signal {signal}"
    else:
        error = f"renderer probe failed with return code {returncode}"
    return {
        "available": returncode == 0 and error is None,
        "timed_out": False,
        "returncode": returncode,
        "signal": signal,
        "stdout": _text(completed.stdout),
        "stderr": _text(completed.stderr),
        "error": error,
        "details": details,
    }


def collect_health(
    model_path=None,
    *,
    renderer_check: Callable[[object], tuple[bool, str | None]] | None = None,
    renderer_timeout: float = DEFAULT_RENDERER_TIMEOUT,
    sim_factory=RebotArmMujoco,
    mujoco_version: str | None = None,
) -> dict[str, object]:
    sim = sim_factory(model_path)
    try:
        before = sim.get_state()
        after = sim.step()
        numeric_state = (
            *after.joint_positions,
            *after.joint_velocities,
            *after.actuator_forces,
            after.simulation_time,
        )
        joints = len(after.joint_names)
        actuators = len(after.actuator_forces)
        if renderer_check is None:
            renderer_probe = probe_renderer(sim.model_path, timeout=renderer_timeout)
        else:
            checked = renderer_check(sim)
            if isinstance(checked, dict):
                renderer_probe = checked
            else:
                renderer_available, renderer_error = checked
                renderer_probe = {
                    "available": bool(renderer_available),
                    "timed_out": False,
                    "returncode": None,
                    "signal": None,
                    "stdout": "",
                    "stderr": "",
                    "error": renderer_error,
                }
        result = {
            "ok": bool(
                all(math.isfinite(float(value)) for value in numeric_state)
                and after.simulation_time > before.simulation_time
                and joints == EXPECTED_JOINT_COUNT
                and actuators == EXPECTED_ACTUATOR_COUNT
            ),
            "python_version": platform.python_version(),
            "mujoco_version": (
                mujoco_version
                if mujoco_version is not None
                else str(getattr(getattr(sim, "_mj", None), "__version__", "injected-simulation"))
            ),
            "model_path": str(sim.model_path),
            "model_loaded": True,
            "physics_step_finite": all(math.isfinite(float(value)) for value in numeric_state),
            "simulation_time": float(after.simulation_time),
            "joint_count": joints,
            "expected_joint_count": EXPECTED_JOINT_COUNT,
            "actuator_count": actuators,
            "expected_actuator_count": EXPECTED_ACTUATOR_COUNT,
            "headless": True,
            "renderer_available": bool(renderer_probe["available"]),
            "renderer_error": renderer_probe["error"],
            "renderer_probe": renderer_probe,
        }
        return result
    finally:
        sim.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Check the reBotArm MuJoCo environment")
    parser.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    parser.add_argument("--skip-renderer", action="store_true", help="skip the offscreen renderer probe")
    parser.add_argument(
        "--renderer-timeout",
        type=_positive_finite,
        default=DEFAULT_RENDERER_TIMEOUT,
        help=f"renderer child-process timeout in seconds (default: {DEFAULT_RENDERER_TIMEOUT:g})",
    )
    return parser


def main(
    argv=None,
    *,
    sim_factory=RebotArmMujoco,
    renderer_check=None,
    stdout=None,
    stderr=None,
) -> int:
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    args = build_parser().parse_args(argv)
    check = (
        (lambda _sim: (False, "renderer check skipped"))
        if args.skip_renderer
        else renderer_check
    )
    try:
        result = collect_health(
            args.model,
            renderer_check=check,
            renderer_timeout=args.renderer_timeout,
            sim_factory=sim_factory,
        )
    except Exception as exc:
        result = {
            "ok": False,
            "python_version": platform.python_version(),
            "model_path": args.model,
            "model_loaded": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(result, ensure_ascii=False), file=stdout)
        print(f"MuJoCo health check failed: {exc}", file=stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
