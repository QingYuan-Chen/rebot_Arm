"""候选 IK 过滤的运行时编排，不负责 ROS 生命周期。"""

from __future__ import annotations

from tf2_ros import TransformException

from .policies.candidate_scoring_policy import CandidateScoringInput, score_candidate
from .policies.motion_feasibility_policy import evaluate_motion_feasibility


class CandidateIkRuntime:
    """把候选过滤顺序与 ROS 节点适配分开。

    Policy 提供候选决策，gateway 提供 TF/MoveIt 与发布接口；本类只协调一帧内
    的预检、变体、可行性和排序顺序，不持有 ROS Node。
    """

    def __init__(self, *, policy, gateway, max_candidates: int, logger) -> None:
        self.policy = policy
        self.gateway = gateway
        self.max_candidates = max_candidates
        self.logger = logger

    def filter_frame(self, msg) -> dict[str, int]:
        begin_frame = getattr(self.gateway, "begin_frame", None)
        if begin_frame is not None:
            begin_frame()
        if not msg.candidates:
            self.gateway.publish_empty(msg)
            return {"input_candidates": 0, "precheck_passed": 0, "geometry_variants_passed": 0, "ranked": 0}
        ranked = []
        max_candidates = self.max_candidates
        input_candidates = min(len(msg.candidates), max_candidates)
        precheck_passed = 0
        geometry_variants_passed = 0
        for original_index, candidate in enumerate(msg.candidates[:max_candidates]):
            try:
                if not self.policy._candidate_precheck_allows(candidate):
                    continue
                precheck_passed += 1
                best = None
                variants = self.policy._candidate_target_variants(msg, candidate.pose)
                for pregrasp, grasp, variant_label in variants:
                    if not self.policy._candidate_gate_allows(candidate, grasp=grasp):
                        continue
                    geometry_variants_passed += 1
                    feasibility = evaluate_motion_feasibility(
                        pregrasp=pregrasp,
                        grasp=grasp,
                        variant_label=variant_label,
                        check_target=self.gateway.check_target,
                        motion_penalty=self.policy._joint_motion_penalty,
                    )
                    if not feasibility.accepted or feasibility.motion_penalty is None:
                        continue
                    scoring = score_candidate(
                        CandidateScoringInput(
                            original_index=original_index,
                            variant_label=variant_label,
                            motion_penalty=feasibility.motion_penalty,
                        )
                    )
                    score_value = float(scoring.score)
                    if best is None or score_value > float(best[0]):
                        best = (score_value, (pregrasp, grasp), variant_label, feasibility.reason)
                if best is not None:
                    score_value, targets, label, motion_reason = best
                    ranked.append((score_value, original_index, candidate, targets, label, motion_reason))
                    self.logger.info(
                        f"candidate IK filter accepted candidate={original_index} {label}: "
                        f"score={score_value:.2f}, {motion_reason}"
                    )
            except (TransformException, RuntimeError, ValueError) as exc:
                self.logger.warn(f"candidate IK filter rejected candidate: {exc}")
        self.gateway.publish_ranked(msg, ranked)
        return {
            "input_candidates": input_candidates,
            "precheck_passed": precheck_passed,
            "geometry_variants_passed": geometry_variants_passed,
            "ranked": len(ranked),
        }
