"""候选配置快照,参数拒绝和独立筛选运行时."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from rebotarm_vision.candidate_ik_config import CandidateIkConfig
from rebotarm_vision.candidate_ik_runtime import CandidateIkRuntime


class _Node:
    def __init__(self, values):
        self.values = values

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])

    def list_parameters(self, _prefixes, depth):
        return SimpleNamespace(names=list(self.values))


def _candidate_values():
    return {
        "input_topic": "/in", "target_frame": "base_link", "service_timeout_sec": 0.2,
        "max_candidates_per_frame": 3, "candidate_min_confidence": 0.4,
        "candidate_min_jaw_width_m": 0.006, "candidate_max_jaw_width_m": 0.085,
        "candidate_workspace_gate_enabled": False, "candidate_workspace_min_xyz": [0, 0, 0],
        "candidate_workspace_max_xyz": [1, 1, 1], "candidate_max_grasp_to_object_center_m": 0.15,
        "candidate_min_grasp_z_m": 0.0, "pose_policy": "preserve_candidate_pose",
        "fixed_grasp_orientation_xyzw": [0, 0, 0, 1], "base_approach_axis_xyz": [1, 0, 0],
        "base_pregrasp_distance_m": 0.08, "tcp_offset_xyz": [0, 0, 0],
        "target_base_offset_xyz": [0, 0, 0], "candidate_pregrasp_min_z_m": 0.04,
        "grasp_base_z_offset_m": 0.0, "orientation_yaw_offsets_rad": [0.0],
        "candidate_grasp_z_offsets_m": [0.0], "candidate_joint6_symmetry_enabled": True,
        "candidate_joint6_symmetry_angle_rad": 3.14, "candidate_score_joint_distance_weight": 0.15,
        "candidate_score_joint6_weight": 0.35, "candidate_max_joint6_delta_rad": 1.57,
        "moveit_group_name": "arm", "ee_frame_id": "end_link",
        "collision_check_enabled": True, "collision_group_name": "arm",
    }


def test_candidate_config_is_a_validated_snapshot():
    config = CandidateIkConfig.from_node(_Node(_candidate_values()))
    assert config.max_candidates_per_frame == 3
    assert config.tcp_offset_xyz == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(("overrides", "error"), [
    ({"tcp_offset_xyz": [0.0, 0.0]}, "tcp_offset_xyz"),
    ({"candidate_workspace_min_xyz": [1, 0, 0],
      "candidate_workspace_max_xyz": [0, 1, 1]}, "candidate_workspace_min_xyz"),
    ({"max_candidates_per_frame": 0}, "max_candidates_per_frame"),
    ({"candidate_min_jaw_width_m": 0.1,
      "candidate_max_jaw_width_m": 0.08}, "candidate_min_jaw_width_m"),
])
def test_candidate_config_rejects_invalid_parameters(overrides, error):
    values = _candidate_values()
    values.update(overrides)
    with pytest.raises(ValueError, match=error):
        CandidateIkConfig.from_node(_Node(values))


def test_candidate_runtime_runs_without_a_ros_node():
    candidate = SimpleNamespace(confidence=0.9, jaw_width=0.04, pose=object())
    message = SimpleNamespace(candidates=[candidate])

    class Policy:
        def _candidate_precheck_allows(self, _candidate):
            return True

        def _candidate_target_variants(self, _message, _pose):
            target = SimpleNamespace(position=(0.1, 0.0, 0.2), orientation=(0, 0, 0, 1))
            return [(target, target, "base")]

        def _candidate_gate_allows(self, _candidate, *, grasp):
            return True

        def _joint_motion_penalty(self, _state):
            return 0.1, "joint_delta=0.1"

    published = []

    class Gateway:
        def begin_frame(self):
            pass

        def check_target(self, _target, _label):
            return object()

        def publish_ranked(self, _message, ranked):
            published.append(ranked)

        def publish_empty(self, _message):
            published.append([])

    logger = SimpleNamespace(info=lambda _message: None, warn=lambda _message: None)
    counts = CandidateIkRuntime(
        policy=Policy(), gateway=Gateway(), max_candidates=1, logger=logger
    ).filter_frame(message)
    assert counts["ranked"] == 1
    assert len(published[0]) == 1


def test_candidate_filter_tf_failure_publishes_no_ranked_candidates():
    from tf2_ros import TransformException
    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy

    msg = GraspCandidateArray()
    msg.header.frame_id = "camera_depth_frame"
    candidate = GraspCandidate()
    candidate.confidence = 0.8
    candidate.jaw_width = 0.04
    msg.candidates.append(candidate)
    published = []
    warnings = []
    node = object.__new__(CandidateIkPolicy)
    parameters = {
        "max_candidates_per_frame": 10,
        "candidate_min_confidence": 0.4,
        "candidate_min_jaw_width_m": 0.006,
        "candidate_max_jaw_width_m": 0.085,
    }
    node._config = SimpleNamespace(values=parameters)
    node.get_parameter = lambda name: type(
        "Parameter", (), {"value": parameters[name]}
    )()
    node._candidate_target_variants = lambda _msg, _pose: (_ for _ in ()).throw(
        TransformException("TF lookup unavailable")
    )
    node._publish_ranked = lambda original, ranked: published.append((original, ranked))
    node.get_logger = lambda: type("Logger", (), {"warn": warnings.append})()

    CandidateIkRuntime(
        policy=node, gateway=SimpleNamespace(
            begin_frame=lambda: None,
            check_target=getattr(node, "_check_ik_and_collision", None),
            publish_ranked=node._publish_ranked, publish_empty=lambda message: None,
        ), max_candidates=10, logger=node.get_logger(),
    ).filter_frame(msg)

    assert published == [(msg, [])]
    assert warnings == ["candidate IK filter rejected candidate: TF lookup unavailable"]


def test_object_center_transform_failure_is_not_silently_ignored():
    from tf2_ros import TransformException
    from rebotarm_msgs.msg import GraspCandidate
    from rebotarm_vision.policies.candidate_ik_policy import CandidateIkPolicy
    policy = object.__new__(CandidateIkPolicy)
    def fail(*_args):
        raise TransformException("object frame unavailable")
    policy._transform_pose_to_target_frame = fail
    with pytest.raises(TransformException, match="object frame unavailable"):
        policy._candidate_object_center_in_target_frame(GraspCandidate())
