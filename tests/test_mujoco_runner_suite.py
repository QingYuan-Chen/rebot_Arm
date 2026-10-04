from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_step_response_suite_runs_each_requested_joint(monkeypatch, tmp_path):
    import rebotarm_simulation.diagnostics.mujoco_runner as runner
    from rebotarm_simulation.diagnostics.mujoco_runner import StepResponseResult, run_step_response_suite

    calls = []

    def fake_run_step_response(xml_path, *, joint, target, seconds):
        calls.append((xml_path, joint, target, seconds))
        return StepResponseResult(
            joint=joint,
            target=target,
            final_position=target,
            final_abs_error=0.0,
            max_abs_error=0.01,
            rms_error=0.005,
            max_abs_velocity=0.2,
            max_abs_actuator_force=1.0,
            sim_time=seconds,
        )

    monkeypatch.setattr(runner, "run_step_response", fake_run_step_response)

    result = run_step_response_suite(
        tmp_path / "robot.xml",
        targets={"joint1": 1.0, "joint2": -0.5},
        seconds=2.0,
    )

    assert [item.joint for item in result.results] == ["joint1", "joint2"]
    assert result.max_final_abs_error == 0.0
    assert result.max_rms_error == 0.005
    assert result.max_abs_error == 0.01
    assert calls == [
        (tmp_path / "robot.xml", "joint1", 1.0, 2.0),
        (tmp_path / "robot.xml", "joint2", -0.5, 2.0),
    ]


def test_grasp_benchmark_result_can_report_quality_status():
    from rebotarm_simulation.diagnostics.mujoco_runner import GraspBenchmarkResult

    result = GraspBenchmarkResult(
        xml_path=Path("scene.xml"),
        target_body="bottle",
        finite=True,
        initial_object_height_m=0.05,
        object_height_m=0.12,
        max_contacts=3,
        final_contacts=2,
        contact_detected=True,
        lift_detected=True,
        grasp_success=True,
        grasp_status="grasp_lift_success",
        sim_time=1.0,
    )

    assert result.grasp_success is True
    assert result.grasp_status == "grasp_lift_success"
