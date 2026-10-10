from __future__ import annotations

from pathlib import Path
import sys


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_motion_feasibility_policy_requires_pregrasp_before_grasp():
    from rebotarm_vision.policies.motion_feasibility_policy import evaluate_motion_feasibility
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    calls: list[str] = []

    def check_target(_target, label: str):
        calls.append(label)
        if label.endswith("/pregrasp"):
            return None
        return object()

    result = evaluate_motion_feasibility(
        pregrasp=PoseTarget((0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)),
        grasp=PoseTarget((0.2, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
        variant_label="candidate0",
        check_target=check_target,
        motion_penalty=lambda _solution: (0.0, "joint_distance=0.000, joint6_delta=0.000"),
    )

    assert not result.accepted
    assert result.reason == "pregrasp infeasible"
    assert calls == ["candidate0/pregrasp"]


def test_motion_feasibility_policy_returns_motion_penalty_after_grasp_solution():
    from rebotarm_vision.policies.motion_feasibility_policy import evaluate_motion_feasibility
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    solutions = {"candidate0/pregrasp": object(), "candidate0/grasp": object()}

    def check_target(_target, label: str):
        return solutions[label]

    result = evaluate_motion_feasibility(
        pregrasp=PoseTarget((0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)),
        grasp=PoseTarget((0.2, 0.0, 0.1), (0.0, 0.0, 0.0, 1.0)),
        variant_label="candidate0",
        check_target=check_target,
        motion_penalty=lambda solution: (0.35, "joint_distance=1.000, joint6_delta=0.500")
        if solution is solutions["candidate0/grasp"]
        else (9.0, "wrong"),
    )

    assert result.accepted
    assert result.motion_penalty == 0.35
    assert result.reason == "joint_distance=1.000, joint6_delta=0.500"
    assert result.grasp_solution is solutions["candidate0/grasp"]
