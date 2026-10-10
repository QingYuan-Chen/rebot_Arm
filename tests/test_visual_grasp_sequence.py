from __future__ import annotations
from rebotarm_vision.visual_grasp_workflow import VisualGraspWorkflow

from pathlib import Path
import sys

import pytest

ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def test_build_sequence_moves_pregrasp_grasp_then_closes():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.30, -0.10, 0.25),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.10, 0.17),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(close_position_m=0.025, close_max_effort=0.3)

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.kind for stage in stages] == [
        "move",
        "move",
        "gripper",
    ]
    assert [stage.name for stage in stages] == [
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
    ]
    assert stages[0].pose == pregrasp
    assert stages[1].pose == grasp
    assert stages[2].gripper_position_m == pytest.approx(0.025)
    assert stages[2].gripper_max_effort == pytest.approx(0.3)


def test_build_sequence_can_open_before_approach():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pose = PoseTarget(
        position=(0.1, 0.2, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(open_before_approach=True)

    stages = build_visual_grasp_sequence(pose, pose, config)

    assert [stage.name for stage in stages] == [
        "open_gripper",
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
    ]
    assert stages[0].kind == "gripper"
    assert stages[0].gripper_position_m == pytest.approx(config.open_position_m)


def test_build_sequence_rejects_too_low_grasp():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.2, 0.0, 0.2),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.2, 0.0, 0.08),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(min_grasp_z_m=0.12)

    with pytest.raises(ValueError, match="below minimum"):
        build_visual_grasp_sequence(pregrasp, grasp, config)


def test_build_sequence_allows_low_grasp_by_default_and_retreats_along_reverse_approach():
    from rebotarm_vision.policies.retreat_policy import RetreatPolicyConfig
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.22, 0.0, 0.035),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, 0.0, 0.035),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        retreat_policy=RetreatPolicyConfig(enabled=True, retreat_distance_m=0.06),
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert stages[1].pose == grasp
    assert stages[3].name == "safe_retreat"
    assert stages[3].pose.position == pytest.approx((0.24, 0.0, 0.035))


def test_build_sequence_uses_jaw_width_for_open_close():
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.30, -0.10, 0.25),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.10, 0.17),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        open_before_approach=True,
        auto_gripper_width=True,
        detected_jaw_width_m=0.04,
        open_clearance_m=0.0,
        close_margin_m=0.012,
        min_open_position_m=0.035,
        max_open_position_m=0.09,
        min_close_position_m=0.006,
        max_close_position_m=0.08,
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.name for stage in stages] == [
        "open_gripper",
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
    ]
    assert stages[0].gripper_position_m == pytest.approx(0.04)
    assert stages[3].gripper_position_m == pytest.approx(0.028)


def test_auto_gripper_width_clamps_to_safe_range():
    from rebotarm_vision.visual_grasp_sequence import VisualGraspSequenceConfig, resolve_gripper_widths

    config = VisualGraspSequenceConfig(
        auto_gripper_width=True,
        detected_jaw_width_m=0.20,
        open_clearance_m=0.02,
        close_margin_m=0.012,
        max_open_position_m=0.09,
        max_close_position_m=0.08,
    )

    open_width, close_width = resolve_gripper_widths(config)

    assert open_width == pytest.approx(0.09)
    assert close_width == pytest.approx(0.08)


def test_gripper_policy_adapts_width_but_uses_fixed_default_effort():
    from rebotarm_vision.policies.gripper_policy import GripperPolicyConfig, resolve_gripper_command

    narrow = resolve_gripper_command(
        jaw_width_m=0.028,
        object_length_m=0.08,
        class_name="bottle",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )
    wide = resolve_gripper_command(
        jaw_width_m=0.060,
        object_length_m=0.20,
        class_name="bottle",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )

    assert narrow.allowed
    assert wide.allowed
    assert narrow.open_width_m < wide.open_width_m
    assert narrow.close_width_m < wide.close_width_m
    assert narrow.max_effort == pytest.approx(0.4)
    assert wide.max_effort == pytest.approx(0.4)


def test_gripper_policy_does_not_add_class_based_effort_bonus():
    from rebotarm_vision.policies.gripper_policy import GripperPolicyConfig, resolve_gripper_command

    bottle = resolve_gripper_command(
        jaw_width_m=0.060,
        object_length_m=0.20,
        class_name="bottle",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )
    box = resolve_gripper_command(
        jaw_width_m=0.060,
        object_length_m=0.20,
        class_name="box",
        config=GripperPolicyConfig(auto_width=True, auto_effort=True),
    )

    assert bottle.max_effort == pytest.approx(0.4)
    assert box.max_effort == pytest.approx(0.4)


def test_gripper_policy_rejects_objects_outside_gripper_range():
    from rebotarm_vision.policies.gripper_policy import GripperPolicyConfig, resolve_gripper_command

    command = resolve_gripper_command(
        jaw_width_m=0.20,
        object_length_m=0.25,
        class_name="box",
        config=GripperPolicyConfig(max_open_width_m=0.09),
    )

    assert not command.allowed
    assert "too wide" in command.reason


def test_gripper_policy_default_accepts_85mm_and_rejects_anything_wider():
    from rebotarm_vision.policies.gripper_policy import resolve_gripper_command

    accepted = resolve_gripper_command(jaw_width_m=0.085)
    rejected = resolve_gripper_command(jaw_width_m=0.085001)

    assert accepted.allowed
    assert accepted.open_width_m == pytest.approx(0.085)
    assert not rejected.allowed
    assert "0.085m > 0.085m" in rejected.reason


def test_sequence_adds_reverse_approach_retreat_before_safe_home():
    from rebotarm_vision.policies.gripper_policy import GripperCommand
    from rebotarm_vision.policies.retreat_policy import RetreatPolicyConfig
    from rebotarm_vision.visual_grasp_sequence import (
        PoseTarget,
        VisualGraspSequenceConfig,
        build_visual_grasp_sequence,
    )

    pregrasp = PoseTarget(
        position=(0.22, -0.05, 0.13),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    grasp = PoseTarget(
        position=(0.30, -0.05, 0.13),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    config = VisualGraspSequenceConfig(
        gripper_command=GripperCommand(
            open_width_m=0.065,
            close_width_m=0.035,
            max_effort=0.42,
            allowed=True,
            reason="ok",
        ),
        retreat_policy=RetreatPolicyConfig(
            enabled=True,
            retreat_distance_m=0.06,
        ),
        include_safe_home=True,
    )

    stages = build_visual_grasp_sequence(pregrasp, grasp, config)

    assert [stage.name for stage in stages] == [
        "move_to_pregrasp",
        "approach_grasp",
        "close_gripper",
        "safe_retreat",
        "safe_home",
    ]
    assert stages[2].gripper_position_m == pytest.approx(0.035)
    assert stages[2].gripper_max_effort == pytest.approx(0.42)
    assert stages[3].pose.position == pytest.approx((0.24, -0.05, 0.13))
    assert stages[4].kind == "safe_home"


def test_safe_retreat_uses_grasp_to_pregrasp_direction():
    from rebotarm_vision.policies.retreat_policy import RetreatPolicyConfig, build_retreat_pose
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    grasp = PoseTarget(
        position=(0.30, -0.05, 0.13),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    pregrasp = PoseTarget(
        position=(0.22, -0.05, 0.13),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    retreat = build_retreat_pose(grasp, pregrasp, RetreatPolicyConfig(enabled=True))

    assert retreat.position == pytest.approx((0.24, -0.05, 0.13))


def test_safe_retreat_rejects_coincident_pregrasp_and_grasp():
    from rebotarm_vision.policies.retreat_policy import RetreatPolicyConfig, build_retreat_pose
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    pose = PoseTarget(position=(0.30, 0.0, 0.13), orientation=(0.0, 0.0, 0.0, 1.0))
    with pytest.raises(ValueError, match="must differ"):
        build_retreat_pose(pose, pose, RetreatPolicyConfig(enabled=True))


def test_close_gripper_contact_can_count_as_success_after_partial_closure():
    from rebotarm_vision.gripper_quality import close_contact_success

    assert close_contact_success(
        command_success=False,
        target_position_m=0.025,
        reached_position_m=0.041,
        previous_open_position_m=0.070,
        contact_margin_m=0.004,
        min_closure_delta_m=0.015,
    )


def test_close_gripper_contact_rejects_no_motion_from_open_position():
    from rebotarm_vision.gripper_quality import close_contact_success

    assert not close_contact_success(
        command_success=False,
        target_position_m=0.025,
        reached_position_m=0.068,
        previous_open_position_m=0.070,
        contact_margin_m=0.004,
        min_closure_delta_m=0.015,
    )


def test_base_axis_grasp_policy_builds_tcp_aligned_targets():
    from rebotarm_vision.policies.visual_grasp_pose_policy import (
        BaseAxisGraspPolicyConfig,
        build_base_axis_grasp_targets,
    )

    pregrasp, grasp = build_base_axis_grasp_targets(
        grasp_position_xyz=(0.40, 0.10, 0.16),
        config=BaseAxisGraspPolicyConfig(
            fixed_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            approach_axis_xyz=(1.0, 0.0, 0.0),
            pregrasp_distance_m=0.08,
            tcp_offset_xyz=(-0.04, 0.0, 0.0),
            target_base_offset_xyz=(0.0, 0.01, 0.0),
            grasp_z_offset_m=0.0,
        ),
    )

    assert grasp.position == pytest.approx((0.44, 0.11, 0.16))
    assert pregrasp.position == pytest.approx((0.36, 0.11, 0.16))
    assert grasp.orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert pregrasp.orientation == pytest.approx(grasp.orientation)


def test_visual_servo_policy_builds_bounded_step_toward_refreshed_grasp():
    from rebotarm_vision.visual_grasp_sequence import PoseTarget
    from rebotarm_vision.policies.visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step

    current = PoseTarget(position=(0.20, 0.00, 0.20), orientation=(0.0, 0.0, 0.0, 1.0))
    desired = PoseTarget(position=(0.30, 0.00, 0.20), orientation=(0.0, 0.0, 0.0, 1.0))

    step = build_visual_servo_step(current, desired, VisualServoApproachConfig(max_step_m=0.025))

    assert step.target.position == pytest.approx((0.225, 0.0, 0.20))
    assert step.target.orientation == desired.orientation
    assert step.error_m == pytest.approx(0.10)
    assert not step.reached


def test_visual_servo_policy_reports_reached_inside_threshold():
    from rebotarm_vision.visual_grasp_sequence import PoseTarget
    from rebotarm_vision.policies.visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step

    current = PoseTarget(position=(0.295, 0.00, 0.20), orientation=(0.0, 0.0, 0.0, 1.0))
    desired = PoseTarget(position=(0.300, 0.00, 0.20), orientation=(0.0, 0.0, 0.0, 1.0))

    step = build_visual_servo_step(current, desired, VisualServoApproachConfig(position_tolerance_m=0.010))

    assert step.target == desired
    assert step.reached


def test_retry_policy_orders_best_candidate_first_then_others():
    from rebotarm_vision.policies.grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices

    assert ordered_candidate_indices(
        candidate_count=4,
        best_index=2,
        failed_indices={2},
        config=RetryPolicyConfig(enabled=True, max_attempts=3),
    ) == [0, 1]


def test_retry_policy_returns_only_best_when_disabled():
    from rebotarm_vision.policies.grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices

    assert ordered_candidate_indices(
        candidate_count=4,
        best_index=2,
        failed_indices=set(),
        config=RetryPolicyConfig(enabled=False, max_attempts=3),
    ) == [2]


def test_grasp_verification_requires_contact_and_sufficient_closure():
    from rebotarm_vision.policies.grasp_verification_policy import (
        GraspVerificationConfig,
        GraspVerificationInput,
        verify_grasp_after_close,
    )

    ok = verify_grasp_after_close(
        GraspVerificationInput(
            gripper_contact_detected=True,
            closure_distance_m=0.020,
        ),
        GraspVerificationConfig(),
    )
    bad = verify_grasp_after_close(
        GraspVerificationInput(
            gripper_contact_detected=True,
            closure_distance_m=0.001,
        ),
        GraspVerificationConfig(),
    )

    assert ok.success
    assert not bad.success
    assert "closure distance too small" in bad.reason


def test_place_policy_builds_place_open_and_retreat_stages():
    from rebotarm_vision.policies.place_task_policy import PlaceTaskConfig, build_place_stages
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    stages = build_place_stages(
        PlaceTaskConfig(
            enabled=True,
            place_position_xyz=(0.20, -0.20, 0.25),
            place_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            open_position_m=0.08,
            open_max_effort=0.25,
            retreat_z_m=0.06,
        )
    )

    assert [stage.name for stage in stages] == ["move_to_place", "open_gripper_at_place", "place_retreat"]
    assert stages[0].pose == PoseTarget(position=(0.20, -0.20, 0.25), orientation=(0.0, 0.0, 0.0, 1.0))
    assert stages[1].gripper_position_m == pytest.approx(0.08)
    assert stages[2].pose.position == pytest.approx((0.20, -0.20, 0.31))


def test_recovery_policy_allows_retry_for_motion_failures_only():
    from rebotarm_vision.policies.trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage

    config = RecoveryConfig(auto_retry_enabled=True, safe_retreat_before_retry=True)

    pregrasp = recovery_decision_for_stage("move_to_pregrasp", attempt_index=0, remaining_attempts=1, config=config)
    approach = recovery_decision_for_stage("approach_grasp", attempt_index=0, remaining_attempts=1, config=config)
    close = recovery_decision_for_stage("close_gripper", attempt_index=0, remaining_attempts=1, config=config)

    assert pregrasp.retry
    assert not pregrasp.request_safe_retreat
    assert approach.retry
    assert approach.request_safe_retreat
    assert not close.retry
    assert close.abort


def test_filtered_plan_targets_are_used_without_reapplying_base_axis_policy():
    import importlib.util

    try:
        rclpy_spec = importlib.util.find_spec("rclpy")
    except ValueError:
        rclpy_spec = None
    if rclpy_spec is None:
        pytest.skip("rclpy is not installed in this Python environment")

    from geometry_msgs.msg import Pose
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode

    plan = GraspPlan()
    plan.header.frame_id = "base_link"
    plan.source = "candidate_ik_filter"
    plan.valid = True
    plan.pregrasp_pose = Pose()
    plan.pregrasp_pose.position.x = 0.30
    plan.pregrasp_pose.position.y = 0.10
    plan.pregrasp_pose.position.z = 0.25
    plan.pregrasp_pose.orientation.w = 1.0
    plan.grasp_pose = Pose()
    plan.grasp_pose.position.x = 0.38
    plan.grasp_pose.position.y = 0.10
    plan.grasp_pose.position.z = 0.18
    plan.grasp_pose.orientation.w = 1.0

    node = object.__new__(VisualGraspWorkflow)
    node._target_frame = "base_link"

    pregrasp, grasp = VisualGraspWorkflow._build_motion_targets_from_filtered_plan(node, plan)

    assert pregrasp.position == pytest.approx((0.30, 0.10, 0.25))
    assert grasp.position == pytest.approx((0.38, 0.10, 0.18))
