from __future__ import annotations

import io
import json

from rebotarm_simulation.diagnostics import mujoco_acceptance


def test_acceptance_suite_aggregates_core_steps(monkeypatch):
    monkeypatch.setattr(mujoco_acceptance, "collect_health", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_pick_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_contact_check", lambda *a, **k: {"ok": True})

    payload = mujoco_acceptance.run_acceptance_suite(skip_renderer=True)

    assert payload["ok"] is True
    assert [step["name"] for step in payload["steps"]] == [
        "health",
        "headless_reach_batch",
        "headless_pick_environment",
        "cube_contact",
    ]


def test_acceptance_suite_marks_failed_step_without_crashing(monkeypatch):
    def fail_contact(*_args, **_kwargs):
        raise RuntimeError("contact failed")

    monkeypatch.setattr(mujoco_acceptance, "collect_health", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_pick_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_contact_check", fail_contact)

    payload = mujoco_acceptance.run_acceptance_suite(skip_renderer=True)

    assert payload["ok"] is False
    failed = payload["steps"][-1]
    assert failed["name"] == "cube_contact"
    assert "contact failed" in failed["error"]


def test_acceptance_main_outputs_json_and_exit_code(monkeypatch):
    monkeypatch.setattr(mujoco_acceptance, "collect_health", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_pick_batch", lambda *a, **k: {"ok": True})
    monkeypatch.setattr(mujoco_acceptance, "run_contact_check", lambda *a, **k: {"ok": True})
    output = io.StringIO()

    code = mujoco_acceptance.main(["--skip-renderer"], stdout=output)

    assert code == 0
    assert json.loads(output.getvalue())["ok"] is True


def test_acceptance_source_documents_optional_ros_and_moveit_probes():
    source = mujoco_acceptance.__loader__.get_source(mujoco_acceptance.__name__)

    for value in (
        "mujoco_ros_acceptance",
        "mujoco_moveit_acceptance",
        "include_ros",
        "include_moveit",
        "headless_reach_batch",
        "headless_pick_environment",
        "cube_contact",
    ):
        assert value in source
    assert "use_hardware" not in source
