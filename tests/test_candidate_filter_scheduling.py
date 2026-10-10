from __future__ import annotations

from types import SimpleNamespace


def test_latest_only_work_queue_processes_current_then_newest_pending_frame() -> None:
    from rebotarm_vision.utils.latest_only_work_queue import LatestOnlyWorkQueue

    queue = LatestOnlyWorkQueue[str]()

    first = queue.submit("frame-1", received_at=1.0)
    assert first is not None
    assert first.item == "frame-1"
    assert first.pending_age_ms == 0.0

    assert queue.submit("frame-2", received_at=2.0) is None
    assert queue.submit("frame-3", received_at=3.0) is None

    newest = queue.complete(completed_at=4.0)
    assert newest is not None
    assert newest.item == "frame-3"
    assert newest.pending_age_ms == 1000.0
    assert queue.complete(completed_at=5.0) is None

    stats = queue.snapshot()
    assert stats.received == 3
    assert stats.started == 2
    assert stats.completed == 2
    assert stats.coalesced == 1
    assert stats.busy is False
    assert stats.pending is False


def test_latest_only_work_queue_starts_a_new_owner_after_becoming_idle() -> None:
    from rebotarm_vision.utils.latest_only_work_queue import LatestOnlyWorkQueue

    queue = LatestOnlyWorkQueue[str]()
    assert queue.submit("frame-1", received_at=1.0) is not None
    assert queue.complete(completed_at=2.0) is None

    second = queue.submit("frame-2", received_at=3.0)
    assert second is not None
    assert second.item == "frame-2"


def test_candidate_precheck_rejects_low_confidence_and_invalid_width() -> None:
    from rebotarm_vision.policies.candidate_precheck_policy import (
        CandidatePrecheckConfig,
        evaluate_candidate_precheck,
    )

    config = CandidatePrecheckConfig(
        min_confidence=0.4,
        min_jaw_width_m=0.006,
        max_jaw_width_m=0.085,
    )

    low_score = evaluate_candidate_precheck(
        confidence=0.3999,
        jaw_width_m=0.05,
        config=config,
    )
    invalid_width = evaluate_candidate_precheck(
        confidence=0.8,
        jaw_width_m=float("nan"),
        config=config,
    )
    accepted = evaluate_candidate_precheck(
        confidence=0.4,
        jaw_width_m=0.085,
        config=config,
    )

    assert not low_score.accepted
    assert low_score.reason == "confidence below minimum (0.3999 < 0.4000)"
    assert not invalid_width.accepted
    assert invalid_width.reason == "jaw_width is not finite"
    assert accepted.accepted


def test_candidate_filter_rejects_low_confidence_before_target_or_ik_work() -> None:
    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy
    from rebotarm_vision.candidate_ik_runtime import CandidateIkRuntime
    from types import SimpleNamespace

    msg = GraspCandidateArray()
    candidate = GraspCandidate()
    candidate.confidence = 0.3999
    candidate.jaw_width = 0.05
    msg.candidates.append(candidate)

    published = []
    warnings = []
    parameters = {
        "max_candidates_per_frame": 20,
        "candidate_min_confidence": 0.4,
        "candidate_min_jaw_width_m": 0.006,
        "candidate_max_jaw_width_m": 0.085,
    }
    node = object.__new__(CandidateIkPolicy)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    node._candidate_target_variants = lambda *_args: (_ for _ in ()).throw(
        AssertionError("target generation and IK must not run for a rejected candidate")
    )
    node._publish_ranked = lambda original, ranked: published.append((original, ranked))

    CandidateIkRuntime(
        policy=node, gateway=SimpleNamespace(
            check_target=getattr(node, "_check_ik_and_collision", None),
            publish_ranked=node._publish_ranked, publish_empty=lambda message: None,
        ), max_candidates=10, logger=node.get_logger(),
    ).filter_frame(msg)

    assert published == [(msg, [])]
    assert warnings == []


def test_candidate_filter_runs_geometry_gate_before_ik_service_calls() -> None:
    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy
    from rebotarm_vision.candidate_ik_runtime import CandidateIkRuntime
    from types import SimpleNamespace
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    msg = GraspCandidateArray()
    candidate = GraspCandidate()
    candidate.confidence = 0.8
    candidate.jaw_width = 0.05
    msg.candidates.append(candidate)
    target = PoseTarget(position=(0.0, 0.0, 0.1), orientation=(0.0, 0.0, 0.0, 1.0))

    published = []
    ik_calls = []
    parameters = {
        "max_candidates_per_frame": 20,
        "candidate_min_confidence": 0.4,
        "candidate_min_jaw_width_m": 0.006,
        "candidate_max_jaw_width_m": 0.085,
    }
    node = object.__new__(CandidateIkPolicy)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)
    node._candidate_target_variants = lambda *_args: [(target, target, "original")]
    node._candidate_gate_allows = lambda *_args, **_kwargs: False
    node._check_ik_and_collision = lambda *args: ik_calls.append(args)
    node._publish_ranked = lambda original, ranked: published.append((original, ranked))

    CandidateIkRuntime(
        policy=node, gateway=SimpleNamespace(
            check_target=getattr(node, "_check_ik_and_collision", None),
            publish_ranked=node._publish_ranked, publish_empty=lambda message: None,
        ), max_candidates=10, logger=node.get_logger(),
    ).filter_frame(msg)

    assert published == [(msg, [])]
    assert ik_calls == []
