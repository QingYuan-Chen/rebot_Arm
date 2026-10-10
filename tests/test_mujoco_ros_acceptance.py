from __future__ import annotations

import io
from pathlib import Path

from rebotarm_simulation.diagnostics import mujoco_ros_acceptance


def test_ros_acceptance_entrypoint_reports_runtime_errors_as_json_free_failure(monkeypatch):
    output = io.StringIO()
    errors = io.StringIO()

    def fail_acceptance(**_kwargs):
        raise RuntimeError("acceptance setup failed")

    monkeypatch.setattr(mujoco_ros_acceptance, "run_acceptance", fail_acceptance)
    code = mujoco_ros_acceptance.main(["--timeout", "0.1"], stdout=output, stderr=errors)

    assert code == 1
    assert output.getvalue() == ""
    assert "acceptance setup failed" in errors.getvalue()
    assert "use_hardware" not in errors.getvalue()


def test_ros_acceptance_source_checks_required_runtime_interfaces_without_hardware():
    source = Path(
        "src/rebotarm_simulation/rebotarm_simulation/diagnostics/mujoco_ros_acceptance.py"
    ).read_text(encoding="utf-8")

    required = (
        "/rebotarm/joint_states",
        "/clock",
        "/rebotarm/gripper/set",
        "/rebotarm/follow_joint_trajectory",
        "ActionClient",
        "SetGripper",
        "FollowJointTrajectory",
        "JointTrajectoryPoint",
        "create_node_class",
        "MultiThreadedExecutor",
        "joint_schema_ok",
        "clock_progress",
        "trajectory_action_success",
    )
    for value in required:
        assert value in source
    assert "rebotarmcontroller" not in source.lower()
    assert "use_hardware" not in source
