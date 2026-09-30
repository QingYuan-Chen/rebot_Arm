from __future__ import annotations

import io
from pathlib import Path
from types import SimpleNamespace

import pytest

from rebotarm_simulation import mujoco_moveit_acceptance


def test_moveit_acceptance_entrypoint_reports_runtime_errors_without_hardware(monkeypatch):
    output = io.StringIO()
    errors = io.StringIO()

    def fail_acceptance(**_kwargs):
        raise RuntimeError("moveit acceptance setup failed")

    monkeypatch.setattr(mujoco_moveit_acceptance, "run_acceptance", fail_acceptance)
    code = mujoco_moveit_acceptance.main(["--timeout", "0.1"], stdout=output, stderr=errors)

    assert code == 1
    assert output.getvalue() == ""
    assert "moveit acceptance setup failed" in errors.getvalue()
    assert "use_hardware" not in errors.getvalue()


def test_moveit_acceptance_source_plans_with_moveit_then_executes_mujoco_action():
    source = Path(
        "src/rebotarm_motion/rebotarm_motion/mujoco_moveit_acceptance.py"
    ).read_text(encoding="utf-8")

    required = (
        "/plan_kinematic_path",
        "/rebotarm/follow_joint_trajectory",
        "/rebotarm/joint_states",
        "/clock",
        "GetMotionPlan",
        "ActionClient",
        "FollowJointTrajectory",
        "MOVEIT_ACCEPTANCE_TARGET",
        "moveit_plan_success",
        "trajectory_action_success",
        "final_max_joint_error_rad",
    )
    for value in required:
        assert value in source
    assert "rebotarmcontroller" not in source.lower()
    assert "use_hardware" not in source


@pytest.mark.parametrize("code,ok", [(0, True), (2, False)])
def test_compatibility_probe_preserves_motion_verdict(monkeypatch, code, ok):
    def run(command, **kwargs):
        assert command[1:3] == ["-m", "rebotarm_motion.mujoco_moveit_acceptance"]
        assert kwargs["timeout"] > 0
        return SimpleNamespace(returncode=code, stdout='{"ok": ' + str(ok).lower() + '}', stderr="")

    monkeypatch.setattr(mujoco_moveit_acceptance.subprocess, "run", run)
    assert mujoco_moveit_acceptance.run_acceptance(timeout=0.1)["ok"] is ok


@pytest.mark.parametrize("code,payload", [(1, '{"ok": true}'), (0, '{"ok": false}'), (0, '{"ok": "true"}')])
def test_compatibility_probe_rejects_failed_or_inconsistent_results(monkeypatch, code, payload):
    monkeypatch.setattr(mujoco_moveit_acceptance.subprocess, "run", lambda *args, **kwargs:
                        SimpleNamespace(returncode=code, stdout=payload, stderr="probe failed"))
    with pytest.raises(RuntimeError):
        mujoco_moveit_acceptance.run_acceptance(timeout=0.1)
