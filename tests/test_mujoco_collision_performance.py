from __future__ import annotations

import ast
import copy
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import platform
import re
import socket
import statistics
from time import perf_counter

import pytest


ROOT = Path(__file__).resolve().parents[1]
BASELINE_PATH = ROOT / "tests/fixtures/mujoco_collision_baseline.json"
SCENE_PATH = ROOT / "src/rebotarm_simulation/models/rebotarm/scene.xml"
ROBOT_PATH = SCENE_PATH.with_name("robot.xml")


def newline_normalized_sha256(path: Path) -> str:
    content = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(content).hexdigest()


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant: {value}")


def load_baseline(path: Path) -> dict[str, object]:
    return json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=_reject_json_constant,
    )


def _cpu_model() -> str:
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.is_file():
        for line in cpuinfo.read_text(encoding="utf-8").splitlines():
            if line.lower().startswith("model name"):
                return line.split(":", 1)[1].strip()
    return platform.processor()


def host_metadata() -> dict[str, str | int]:
    return {
        "hostname": socket.gethostname(),
        "machine": platform.machine(),
        "cpu_model": _cpu_model(),
        "cpu_count": os.cpu_count() or 0,
        "platform": platform.platform(),
    }


def assert_compatible_host(
    baseline: dict[str, object], current: dict[str, object]
) -> None:
    for field in ("hostname", "machine", "cpu_model", "cpu_count", "mujoco_version"):
        assert current[field] == baseline[field], (
            f"incompatible benchmark host {field}: "
            f"expected {baseline[field]!r}, got {current[field]!r}"
        )


def _positive_number(value: object, field: str) -> float:
    assert type(value) in (int, float), f"{field} must be a JSON number, not {type(value).__name__}"
    assert math.isfinite(value) and value > 0, f"{field} must be finite and positive"
    return float(value)


def validate_baseline(baseline: dict[str, object]) -> None:
    assert set(baseline) == {
        "schema_version", "model_kind", "steps", "samples", "mujoco_version",
        "python_version", "hostname", "machine", "cpu_model", "platform",
        "cpu_count", "hash_mode", "scene_sha256", "robot_sha256",
        "measurements", "median_elapsed_seconds",
    }
    assert type(baseline["schema_version"]) is int and baseline["schema_version"] == 1
    assert baseline["model_kind"] == "primitive_collision"
    assert type(baseline["steps"]) is int and baseline["steps"] == 10_000
    assert type(baseline["samples"]) is int and baseline["samples"] == 3
    assert baseline["mujoco_version"] == "3.10.0"
    for field in ("python_version", "hostname", "machine", "cpu_model", "platform"):
        assert isinstance(baseline[field], str) and baseline[field]
    assert type(baseline["cpu_count"]) is int and baseline["cpu_count"] > 0
    assert baseline["hash_mode"] == "newline_normalized_sha256"
    for field in ("scene_sha256", "robot_sha256"):
        assert re.fullmatch(r"[0-9a-f]{64}", baseline[field]), f"invalid {field}"

    measurements = baseline["measurements"]
    assert isinstance(measurements, list) and len(measurements) == 3
    elapsed = []
    for item in measurements:
        assert isinstance(item, dict)
        assert set(item) == {"steps", "elapsed_seconds", "realtime_factor", "peak_contacts"}
        assert type(item["steps"]) is int and item["steps"] == 10_000
        elapsed.append(_positive_number(item["elapsed_seconds"], "elapsed_seconds"))
        _positive_number(item["realtime_factor"], "realtime_factor")
        assert type(item["peak_contacts"]) is int and item["peak_contacts"] >= 0
    median = _positive_number(baseline["median_elapsed_seconds"], "median_elapsed_seconds")
    assert median == statistics.median(elapsed)


def benchmark_scene(scene: Path, steps: int = 10_000) -> dict[str, int | float]:
    """Time only MuJoCo stepping/contact counting, then validate final state."""
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    peak_contacts = 0
    started = perf_counter()
    for step in range(steps):
        mujoco.mj_step(model, data)
        peak_contacts = max(peak_contacts, int(data.ncon))
    elapsed_seconds = perf_counter() - started
    for name in ("qpos", "qvel", "actuator_force"):
        values = getattr(data, name)
        if not all(math.isfinite(float(value)) for value in values):
            raise AssertionError(f"non-finite final {name} after {steps} MuJoCo steps")
    return {
        "steps": steps,
        "elapsed_seconds": elapsed_seconds,
        "realtime_factor": steps * float(model.opt.timestep) / elapsed_seconds,
        "peak_contacts": peak_contacts,
    }


def test_primitive_collision_baseline_schema() -> None:
    assert BASELINE_PATH.is_file(), f"missing collision baseline: {BASELINE_PATH}"
    validate_baseline(load_baseline(BASELINE_PATH))


def test_benchmark_scene_reports_real_mujoco_steps_and_finite_state() -> None:
    measurement = benchmark_scene(SCENE_PATH, steps=25)

    assert set(measurement) == {"steps", "elapsed_seconds", "realtime_factor", "peak_contacts"}
    assert measurement["steps"] == 25
    assert measurement["elapsed_seconds"] > 0
    assert measurement["realtime_factor"] > 0
    assert isinstance(measurement["peak_contacts"], int)
    assert measurement["peak_contacts"] >= 0


def test_arm_collision_contract_avoids_high_triangle_mesh_geoms() -> None:
    import xml.etree.ElementTree as ET

    robot = ET.parse(ROBOT_PATH).getroot()
    collisions = [geom for geom in robot.findall('.//geom[@class="collision"]')
                  if not geom.attrib.get("name", "").startswith("gemini2_")]
    mesh_collisions = [geom for geom in collisions if geom.attrib.get("type") == "mesh"]

    assert len(collisions) == 10
    assert {geom.attrib["mesh"] for geom in mesh_collisions} == {
        "left_finger", "right_finger",
    }
    assert all(geom.attrib.get("group") == "3" for geom in collisions)


@pytest.mark.skipif(
    os.environ.get("REBOTARM_RUN_COLLISION_BENCHMARK") != "1",
    reason="set REBOTARM_RUN_COLLISION_BENCHMARK=1 to run the 10k collision benchmark",
)
def test_primitive_collision_10k_benchmark() -> None:
    import mujoco

    baseline = load_baseline(BASELINE_PATH)
    validate_baseline(baseline)
    current = host_metadata() | {"mujoco_version": mujoco.__version__}
    assert_compatible_host(baseline, current)
    measurements = [benchmark_scene(SCENE_PATH, steps=baseline["steps"]) for _ in range(3)]
    result = {
        "measurements": measurements,
        "median_elapsed_seconds": statistics.median(
            item["elapsed_seconds"] for item in measurements
        ),
    }

    print(json.dumps(result, indent=2, sort_keys=True))


def test_baseline_validation_rejects_tampered_model_hash() -> None:
    baseline = load_baseline(BASELINE_PATH)
    tampered = copy.deepcopy(baseline)
    tampered["robot_sha256"] = "not-a-sha256"

    with pytest.raises(AssertionError, match="robot_sha256"):
        validate_baseline(tampered)


def test_strict_json_loader_rejects_infinity(tmp_path: Path) -> None:
    path = tmp_path / "invalid.json"
    path.write_text('{"value": Infinity}', encoding="utf-8")

    with pytest.raises(ValueError, match="non-standard JSON constant"):
        load_baseline(path)


def test_baseline_validation_rejects_bool_as_numeric() -> None:
    baseline = load_baseline(BASELINE_PATH)
    tampered = copy.deepcopy(baseline)
    tampered["measurements"][0]["elapsed_seconds"] = True

    with pytest.raises(AssertionError, match="elapsed_seconds"):
        validate_baseline(tampered)


def test_host_compatibility_rejects_mismatch() -> None:
    baseline = load_baseline(BASELINE_PATH)
    current = host_metadata()
    current["hostname"] = "not-the-baseline-host"

    with pytest.raises(AssertionError, match="hostname"):
        assert_compatible_host(baseline, current)


def test_benchmark_timed_loop_does_not_scan_state_arrays() -> None:
    tree = ast.parse(inspect.getsource(benchmark_scene))
    loop = next(node for node in ast.walk(tree) if isinstance(node, ast.For))
    calls = {
        node.func.id
        for node in ast.walk(ast.Module(body=loop.body, type_ignores=[]))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    attributes = {
        node.attr
        for node in ast.walk(ast.Module(body=loop.body, type_ignores=[]))
        if isinstance(node, ast.Attribute)
    }

    assert "getattr" not in calls
    assert {"qpos", "qvel", "actuator_force"}.isdisjoint(attributes)
