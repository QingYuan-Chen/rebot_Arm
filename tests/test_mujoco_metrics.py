from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "src" / "rebotarm_simulation"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))


def test_metrics_recorder_writes_csv_and_summary(tmp_path):
    from rebotarm_simulation.diagnostics.mujoco_metrics import TrajectoryMetricsRecorder

    recorder = TrajectoryMetricsRecorder(tmp_path, joint_names=["joint1", "joint2"])
    recorder.record(
        elapsed=0.0,
        targets=[0.0, -0.5],
        actual=[0.1, -0.6],
        velocities=[0.2, 0.3],
        actuator_forces=[1.0, 2.0],
    )
    recorder.record(
        elapsed=1.0,
        targets=[0.2, -0.7],
        actual=[0.25, -0.72],
        velocities=[0.1, 0.4],
        actuator_forces=[3.0, 4.0],
    )

    summary = recorder.finish(success=True, stop_reason="finished")

    assert (tmp_path / "trajectory_metrics.csv").exists()
    assert (tmp_path / "summary.json").exists()
    assert summary["success"] is True
    assert summary["stop_reason"] == "finished"
    assert summary["joint_count"] == 2
    assert summary["max_abs_error"] == 0.1
    assert summary["max_abs_velocity"] == 0.4
    assert summary["max_abs_actuator_force"] == 4.0


def test_metrics_summary_includes_violation_context(tmp_path):
    from rebotarm_simulation.diagnostics.mujoco_metrics import TrajectoryMetricsRecorder

    recorder = TrajectoryMetricsRecorder(tmp_path, joint_names=["joint1"])
    recorder.record(
        elapsed=0.0,
        targets=[1.0],
        actual=[0.8],
        velocities=[0.1],
        actuator_forces=[2.0],
    )

    summary = recorder.finish(
        success=False,
        stop_reason="goal_tolerance_violated",
        violated_joint="joint1",
        tolerance=0.05,
    )

    assert summary["success"] is False
    assert summary["stop_reason"] == "goal_tolerance_violated"
    assert summary["violated_joint"] == "joint1"
    assert summary["tolerance"] == 0.05


def test_metrics_recorder_can_downsample_written_rows(tmp_path):
    from rebotarm_simulation.diagnostics.mujoco_metrics import TrajectoryMetricsRecorder

    recorder = TrajectoryMetricsRecorder(tmp_path, joint_names=["joint1"], sample_stride=2)
    for index in range(5):
        recorder.record(
            elapsed=float(index),
            targets=[float(index)],
            actual=[float(index)],
            velocities=[0.0],
            actuator_forces=[0.0],
        )

    summary = recorder.finish(success=True, stop_reason="finished")
    csv_lines = (tmp_path / "trajectory_metrics.csv").read_text(encoding="utf-8").splitlines()

    assert summary["sample_count"] == 5
    assert summary["written_sample_count"] == 3
    assert len(csv_lines) == 4
