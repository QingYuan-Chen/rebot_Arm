from __future__ import annotations

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "src/rebotarm_vision/rebotarm_vision/single_bottle_grasp.py"


def _run_runner_script(script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            "-lc",
            'source install/setup.bash && exec "$@"',
            "bash",
            sys.executable,
            "-c",
            script,
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


def test_p6_bottle_plan_accepts_point_four_confidence_boundary() -> None:
    # Some unrelated tests replace rclpy in this interpreter. Exercise the real
    # runner in a clean process so test order cannot change this safety gate.
    script = f"""
import importlib.util
from rebotarm_msgs.msg import GraspPlan

spec = importlib.util.spec_from_file_location('p6_confidence_test', {str(RUNNER_PATH)!r})
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

def bottle_plan(confidence):
    plan = GraspPlan()
    plan.valid = True
    plan.header.frame_id = 'base_link'
    plan.candidate.class_name = 'bottle'
    plan.candidate.confidence = confidence
    plan.jaw_width = 0.07
    plan.grasp_pose.position.z = 0.05
    return plan

assert not runner._valid_bottle_plan(bottle_plan(0.3999))
assert runner._valid_bottle_plan(bottle_plan(0.4))
"""
    result = _run_runner_script(script)

    assert result.returncode == 0, result.stderr


def test_p6_bottle_plan_accepts_85mm_jaw_boundary() -> None:
    script = f"""
import importlib.util
from rebotarm_msgs.msg import GraspPlan

spec = importlib.util.spec_from_file_location('p6_jaw_width_test', {str(RUNNER_PATH)!r})
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

def bottle_plan(jaw_width):
    plan = GraspPlan()
    plan.valid = True
    plan.header.frame_id = 'base_link'
    plan.candidate.class_name = 'bottle'
    plan.candidate.confidence = 0.4
    plan.jaw_width = jaw_width
    plan.grasp_pose.position.z = 0.05
    return plan

assert runner._valid_bottle_plan(bottle_plan(0.085))
assert not runner._valid_bottle_plan(bottle_plan(0.085001))
"""
    result = _run_runner_script(script)

    assert result.returncode == 0, result.stderr


def test_p6_runner_default_open_effort_matches_web_teleop() -> None:
    script = f"""
import importlib.util
import sys

spec = importlib.util.spec_from_file_location('p6_open_effort_test', {str(RUNNER_PATH)!r})
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
sys.argv = [
    'rebotarm_single_bottle_grasp',
    '--gripper-open-m', '0.08',
    '--output', '/tmp/p6-parser-test.json',
]
args = runner._parse_args()
assert args.gripper_open_max_effort == 1.5, args.gripper_open_max_effort
"""
    result = _run_runner_script(script)

    assert result.returncode == 0, result.stderr
