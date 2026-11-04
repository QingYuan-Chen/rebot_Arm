from __future__ import annotations

import json
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
from rebotarm_dashboard.status_panel_state import (
    StatusSnapshot,
    TeleopStatusStore,
    clamp_preview_value,
    encode_sse_event,
    format_angle_readout,
)
from rebotarm_dashboard.status_panel_api import (
    StatusPanelApiError,
    dispatch_post_request,
    is_allowed_post_path,
    post_paths,
)
from rebotarm_dashboard.arm_command_api import (
    arm_command_is_replay_locked,
    normalize_arm_command,
    should_stop_trajectory_before_arm_command,
    status_state,
)
from rebotarm_motion.trajectory_safety_monitor import (
    evaluate_replay_tracking,
)
from rebotarm_teach.teach_recording import (
    ReplayStartBand,
    TeachSample,
    analyze_teach_trajectory,
    build_replay_start_soft_points,
    classify_replay_start,
    compute_auto_align_duration,
    decode_teach_sample,
    encode_teach_sample,
    estimate_teach_replay,
    lowpass_filter_teach_samples,
    inspect_teach_record,
    is_quit_key,
    list_teach_record_files,
    load_teach_samples,
    normalize_teach_replay_settings,
    prepare_teach_replay_samples,
    prepared_record_path,
    prepared_teach_replay_to_dict,
    retime_teach_samples,
    smooth_teach_samples,
    teach_record_info_to_dict,
    teach_trajectory_preview_to_dict,
    validate_teach_dry_run_request,
    validate_teach_replay_execute_request,
    validate_teach_replay_stop_request,
    write_prepared_teach_record,
)
from rebotarm_teleop.teleop_core import (
    KeyboardCommandMapper,
    TeleopTargetPlanner,
    validate_web_keyboard_command,
)
from rebotarm_dashboard.web_robot_assets import (
    DEFAULT_GRIPPER_LIMITS_M,
    clamp_gripper_opening,
    gripper_opening_to_finger_joint_positions,
    load_gripper_limits,
    load_moveit_velocity_limits,
    load_urdf_joint_limits,
    merge_velocity_limits,
    merge_joint_limits,
    rewrite_package_mesh_uris,
    safe_mesh_path,
)
from rebotarm_teleop.web_execute import (
    interpolate_joint_points,
    validate_web_gripper_request,
    validate_web_execute_request,
)


class KeyboardTeleopCoreTests(unittest.TestCase):
    def test_keyboard_mapping_uses_configured_joint_pairs(self) -> None:
        mapper = KeyboardCommandMapper(
            joint_names=("joint1", "joint2"),
            key_bindings={"1": ("joint1", 1.0), "q": ("joint1", -1.0)},
        )

        self.assertEqual(mapper.command_for_key("1").joint_name, "joint1")
        self.assertEqual(mapper.command_for_key("1").direction, 1.0)
        self.assertEqual(mapper.command_for_key("q").direction, -1.0)
        self.assertIsNone(mapper.command_for_key("x"))

    def test_target_planner_clamps_joint_targets_to_limits(self) -> None:
        planner = TeleopTargetPlanner(
            joint_names=("joint1", "joint2"),
            joint_limits={"joint1": (-1.0, 1.0), "joint2": (-2.0, 2.0)},
            joint_step_rad=0.25,
        )

        target = planner.apply_delta(
            current_positions={"joint1": 0.9, "joint2": 0.0},
            joint_name="joint1",
            direction=1.0,
        )

        self.assertTrue(target.accepted)
        self.assertEqual(target.positions, (1.0, 0.0))
        self.assertIn("clamped", target.message)

    def test_web_keyboard_command_uses_key_mapping_and_duration(self) -> None:
        decision = validate_web_keyboard_command(
            {
                "confirm": "KEYBOARD_TELEOP",
                "key": "1",
                "step_rad": 0.02,
                "duration": 0.4,
                "max_joint_speed_rad_s": 1.0,
            },
            enabled=True,
            joint_names=("joint1", "joint2"),
            current_positions={"joint1": 0.1, "joint2": -0.1},
            joint_limits={"joint1": (-1.0, 1.0), "joint2": (-1.0, 1.0)},
            default_step_rad=0.02,
            min_step_rad=0.005,
            max_step_rad=0.1,
            default_duration=0.4,
            min_duration=0.1,
            max_duration=2.0,
            joint_velocity_limits={"joint1": 1.5, "joint2": 1.5},
            max_joint_speed_rad_s=1.5,
        )

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.key, "1")
        self.assertEqual(decision.joint_name, "joint1")
        self.assertAlmostEqual(decision.positions[0], 0.12)
        self.assertAlmostEqual(decision.positions[1], -0.1)
        self.assertEqual(decision.duration, 0.4)

    def test_web_keyboard_command_rejects_when_disabled_or_too_fast(self) -> None:
        disabled = validate_web_keyboard_command(
            {"confirm": "KEYBOARD_TELEOP", "key": "1"},
            enabled=False,
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            default_step_rad=0.02,
            min_step_rad=0.005,
            max_step_rad=0.1,
            default_duration=0.2,
            min_duration=0.1,
            max_duration=2.0,
            joint_velocity_limits={"joint1": 1.0},
            max_joint_speed_rad_s=1.0,
        )
        self.assertFalse(disabled.accepted)
        self.assertIn("disabled", disabled.message)

        too_fast = validate_web_keyboard_command(
            {"confirm": "KEYBOARD_TELEOP", "key": "1", "step_rad": 0.1, "duration": 0.05},
            enabled=True,
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            default_step_rad=0.02,
            min_step_rad=0.005,
            max_step_rad=0.1,
            default_duration=0.2,
            min_duration=0.05,
            max_duration=2.0,
            joint_velocity_limits={"joint1": 1.0},
            max_joint_speed_rad_s=1.0,
        )
        self.assertFalse(too_fast.accepted)
        self.assertIn("speed too high", too_fast.message)


class ArmCommandApiTests(unittest.TestCase):
    def test_normalize_arm_command_accepts_only_known_commands(self) -> None:
        self.assertEqual(normalize_arm_command("safe_home"), "safe_home")
        self.assertEqual(normalize_arm_command(" enable "), "enable")
        self.assertIsNone(normalize_arm_command("trajectory_stop"))
        self.assertIsNone(normalize_arm_command(""))

    def test_safe_home_and_disable_stop_active_trajectory_first(self) -> None:
        self.assertTrue(should_stop_trajectory_before_arm_command("safe_home"))
        self.assertTrue(should_stop_trajectory_before_arm_command("disable"))
        self.assertFalse(should_stop_trajectory_before_arm_command("enable"))

    def test_replay_lock_blocks_only_active_stop_sensitive_replay_states(self) -> None:
        for state in ("replaying", "cancel_requested", "stop_requested"):
            self.assertTrue(arm_command_is_replay_locked(state))
        for state in ("", "ready", "done", "failed", "safety_stop", "canceled"):
            self.assertFalse(arm_command_is_replay_locked(state))

    def test_status_state_accepts_dict_or_legacy_string_status(self) -> None:
        self.assertEqual(status_state({"state": "REPLAYING"}), "replaying")
        self.assertEqual(status_state("safety_stop"), "safety_stop")
        self.assertEqual(status_state(None), "")


class StatusPanelApiRouterTests(unittest.TestCase):
    class FakePanelNode:
        def __init__(self) -> None:
            self.calls: list[tuple[str, object]] = []

        def _handle_arm_service_command(self, command: str) -> dict:
            self.calls.append(("arm", command))
            return {"accepted": True, "command": command}

        def _handle_execute_preview(self, payload: dict) -> dict:
            self.calls.append(("execute", payload))
            return {"accepted": True}

    def test_routes_arm_safe_home_without_json_payload(self) -> None:
        node = self.FakePanelNode()

        result = dispatch_post_request(node, "/api/arm_safe_home", lambda: self.fail("payload should not be read"))

        self.assertEqual(result["command"], "safe_home")
        self.assertEqual(node.calls, [("arm", "safe_home")])

    def test_routes_execute_preview_with_json_payload(self) -> None:
        node = self.FakePanelNode()

        result = dispatch_post_request(node, "/api/execute_preview", lambda: {"confirm": "EXECUTE"})

        self.assertTrue(result["accepted"])
        self.assertEqual(node.calls, [("execute", {"confirm": "EXECUTE"})])

    def test_rejects_unknown_post_path(self) -> None:
        self.assertFalse(is_allowed_post_path("/api/does_not_exist"))
        with self.assertRaises(StatusPanelApiError):
            dispatch_post_request(self.FakePanelNode(), "/api/does_not_exist", lambda: {})

    def test_post_paths_include_current_web_actions(self) -> None:
        paths = post_paths()
        self.assertIn("/api/arm_safe_home", paths)
        self.assertIn("/api/teach_replay_execute", paths)
        self.assertIn("/api/keyboard_key", paths)


class TeachRecordingCoreTests(unittest.TestCase):
    def test_is_quit_key_matches_configured_key_case_insensitively(self) -> None:
        self.assertTrue(is_quit_key("q", quit_key="q"))
        self.assertTrue(is_quit_key("Q", quit_key="q"))
        self.assertFalse(is_quit_key("w", quit_key="q"))

    def test_teach_sample_jsonl_roundtrip_preserves_motor_state(self) -> None:
        sample = TeachSample(
            stamp=1.25,
            joint_names=("joint1", "joint2"),
            positions=(0.1, -0.2),
            velocities=(0.01, -0.02),
            efforts=(0.3, 0.4),
            motor_status={"joint1": 0, "joint2": 2},
            arm_state="GRAVITY_COMP",
        )

        encoded = encode_teach_sample(sample)
        decoded = decode_teach_sample(encoded)

        self.assertEqual(decoded, sample)
        self.assertEqual(json.loads(encoded)["arm_state"], "GRAVITY_COMP")

    def test_replay_start_classification_rejects_large_error(self) -> None:
        decision = classify_replay_start(
            current_positions=(0.0, 0.0, 0.0),
            start_positions=(0.1, 0.2, 0.3),
            direct_threshold=0.05,
            align_threshold=0.25,
        )

        self.assertEqual(decision.band, ReplayStartBand.REJECT)
        self.assertAlmostEqual(decision.max_error, 0.3)
        self.assertFalse(decision.allow_replay)
        self.assertFalse(decision.allow_auto_align)

    def test_inspect_teach_record_summarizes_file_and_start_error(self) -> None:
        first = TeachSample(
            stamp=10.0,
            joint_names=("joint1", "joint2"),
            positions=(0.1, -0.2),
            velocities=(0.0, 0.0),
            efforts=(0.0, 0.0),
            motor_status={"joint1": 0},
            arm_state="GRAVITY_COMP",
        )
        last = TeachSample(
            stamp=12.5,
            joint_names=("joint1", "joint2"),
            positions=(0.2, -0.1),
            velocities=(0.0, 0.0),
            efforts=(0.0, 0.0),
            motor_status={"joint1": 0},
            arm_state="GRAVITY_COMP",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "teach.jsonl"
            path.write_text(
                encode_teach_sample(first) + "\n" + encode_teach_sample(last) + "\n",
                encoding="utf-8",
            )

            info = inspect_teach_record(
                path,
                current_positions={"joint1": 0.12, "joint2": -0.23},
                direct_threshold=0.05,
                align_threshold=0.25,
            )
            payload = teach_record_info_to_dict(info)

        self.assertEqual(payload["samples"], 2)
        self.assertAlmostEqual(payload["duration_sec"], 2.5)
        self.assertEqual(payload["start_band"], "direct")
        self.assertEqual(payload["worst_joint"], "joint2")
        self.assertAlmostEqual(payload["start_positions"]["joint1"], 0.1)
        self.assertAlmostEqual(payload["end_positions"]["joint2"], -0.1)
        self.assertAlmostEqual(payload["per_joint_error"]["joint2"], 0.03)

    def test_inspect_teach_record_reports_invalid_jsonl(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "broken.jsonl"
            path.write_text("not-json\n", encoding="utf-8")

            payload = teach_record_info_to_dict(inspect_teach_record(path))

        self.assertTrue(payload["exists"])
        self.assertEqual(payload["start_band"], "invalid")
        self.assertIn("failed to read record", payload["message"])

    def test_validate_teach_dry_run_accepts_direct_or_align_only(self) -> None:
        self.assertTrue(validate_teach_dry_run_request("direct").accepted)
        self.assertTrue(validate_teach_dry_run_request("align").accepted)
        self.assertTrue(validate_teach_dry_run_request("moveit_align").accepted)

        rejected = validate_teach_dry_run_request("reject")

        self.assertFalse(rejected.accepted)
        self.assertIn("blocked", rejected.message)

    def test_validate_teach_replay_execute_requires_recent_dry_run(self) -> None:
        self.assertTrue(validate_teach_replay_execute_request("direct", dry_run_passed=True).accepted)
        self.assertTrue(validate_teach_replay_execute_request("moveit_align", dry_run_passed=True).accepted)

        missing_dry_run = validate_teach_replay_execute_request("direct", dry_run_passed=False)
        rejected_band = validate_teach_replay_execute_request("reject", dry_run_passed=True)

        self.assertFalse(missing_dry_run.accepted)
        self.assertIn("dry-run", missing_dry_run.message)
        self.assertFalse(rejected_band.accepted)
        self.assertIn("blocked", rejected_band.message)

    def test_validate_teach_replay_execute_blocks_red_quality_and_fast_yellow(self) -> None:
        red = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="red",
        )
        fast_yellow = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="yellow",
            replay_speed=1.0,
            yellow_max_speed=0.6,
        )

        self.assertFalse(red.accepted)
        self.assertIn("quality is red", red.message)
        self.assertFalse(fast_yellow.accepted)
        self.assertIn("yellow replay speed", fast_yellow.message)

    def test_validate_teach_replay_execute_uses_prepared_quality_when_available(self) -> None:
        recovered = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="red",
            prepared_risk_level="yellow",
            replay_speed=0.5,
            yellow_max_speed=0.6,
        )
        unsafe = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="yellow",
            prepared_risk_level="red",
        )

        self.assertTrue(recovered.accepted)
        self.assertFalse(unsafe.accepted)
        self.assertIn("prepared trajectory quality is red", unsafe.message)

    def test_validate_teach_replay_execute_blocks_large_prepared_jump(self) -> None:
        decision = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="red",
            prepared_risk_level="yellow",
            prepared_max_jump_rad=0.021,
            max_prepared_jump_rad=0.02,
            replay_speed=0.2,
            yellow_max_speed=0.6,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("prepared max jump", decision.message)

    def test_validate_teach_replay_execute_blocks_large_retimed_acceleration(self) -> None:
        decision = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="green",
            prepared_risk_level="green",
            retimed_max_acceleration_rad_s2=3.2,
            max_replay_acceleration_rad_s2=3.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("retimed max acceleration", decision.message)

    def test_validate_teach_replay_execute_blocks_large_retimed_jerk(self) -> None:
        decision = validate_teach_replay_execute_request(
            "direct",
            dry_run_passed=True,
            risk_level="green",
            prepared_risk_level="green",
            retimed_max_jerk_rad_s3=8.2,
            max_replay_jerk_rad_s3=8.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("retimed max jerk", decision.message)

    def test_validate_teach_replay_stop_requires_active_goal(self) -> None:
        self.assertTrue(validate_teach_replay_stop_request(True).accepted)

        rejected = validate_teach_replay_stop_request(False)

        self.assertFalse(rejected.accepted)
        self.assertIn("no active", rejected.message)

    def test_teach_replay_settings_are_clamped_to_safe_bounds(self) -> None:
        settings = normalize_teach_replay_settings(
            replay_speed=99.0,
            align_duration=0.1,
            align_steps=1,
            final_hold_sec=-1.0,
        )

        self.assertEqual(settings["replay_speed"], 1.0)
        self.assertEqual(settings["align_duration"], 1.0)
        self.assertEqual(settings["align_steps"], 2)
        self.assertEqual(settings["final_hold_sec"], 1.0)

    def test_auto_align_duration_scales_with_start_error(self) -> None:
        self.assertAlmostEqual(compute_auto_align_duration(0.02), 3.0)
        self.assertAlmostEqual(compute_auto_align_duration(0.9), 6.0)
        self.assertAlmostEqual(compute_auto_align_duration(3.0), 10.0)

    def test_estimate_teach_replay_includes_alignment_when_needed(self) -> None:
        estimate = estimate_teach_replay(
            samples=5,
            record_duration_sec=4.0,
            start_band="align",
            replay_speed=2.0,
            align_duration=3.0,
            align_steps=30,
        )

        self.assertEqual(estimate["trajectory_points"], 36)
        self.assertAlmostEqual(estimate["estimated_duration_sec"], 3.0 + 4.0 / 1.0 + 1.0)

    def test_estimate_teach_replay_includes_final_hold_when_requested(self) -> None:
        estimate = estimate_teach_replay(
            samples=5,
            record_duration_sec=4.0,
            start_band="direct",
            replay_speed=2.0,
            align_duration=3.0,
            align_steps=30,
            final_hold_sec=1.0,
        )

        self.assertEqual(estimate["trajectory_points"], 6)
        self.assertAlmostEqual(estimate["estimated_duration_sec"], 4.0 / 1.0 + 1.0)
        self.assertAlmostEqual(estimate["final_hold_sec"], 1.0)

    def test_build_replay_start_soft_points_holds_aligns_and_holds_first_pose(self) -> None:
        points = build_replay_start_soft_points(
            current_positions=(0.0,),
            first_positions=(0.12,),
            start_band="direct",
            start_hold_sec=0.8,
            soft_start_duration=1.2,
            soft_start_steps=4,
            align_duration=3.0,
            align_steps=30,
            first_hold_sec=0.3,
        )

        self.assertEqual(points[0].positions, (0.0,))
        self.assertAlmostEqual(points[0].time_from_start, 0.8)
        self.assertEqual(points[-1].positions, (0.12,))
        self.assertAlmostEqual(points[-1].time_from_start, 2.3)
        self.assertGreaterEqual(len(points), 5)
        for previous, current in zip(points, points[1:]):
            self.assertGreater(current.time_from_start, previous.time_from_start)

    def test_build_replay_start_soft_points_uses_full_align_duration_for_align_band(self) -> None:
        points = build_replay_start_soft_points(
            current_positions=(0.0,),
            first_positions=(0.2,),
            start_band="align",
            start_hold_sec=0.5,
            soft_start_duration=1.0,
            soft_start_steps=10,
            align_duration=4.0,
            align_steps=20,
            first_hold_sec=0.2,
        )

        self.assertAlmostEqual(points[-1].time_from_start, 4.7)
        self.assertEqual(points[-1].positions, (0.2,))

    def test_inspect_teach_record_reports_timestamp_and_jump_anomalies(self) -> None:
        first = TeachSample(
            stamp=10.0,
            joint_names=("joint1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(0.0,),
            motor_status={},
            arm_state="GRAVITY_COMP",
        )
        second = TeachSample(
            stamp=9.0,
            joint_names=("joint1",),
            positions=(2.0,),
            velocities=(0.0,),
            efforts=(0.0,),
            motor_status={},
            arm_state="GRAVITY_COMP",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "bad.jsonl"
            path.write_text(
                encode_teach_sample(first) + "\n" + encode_teach_sample(second) + "\n",
                encoding="utf-8",
            )

            payload = teach_record_info_to_dict(inspect_teach_record(path))

        self.assertTrue(any("timestamp" in item for item in payload["anomalies"]))
        self.assertTrue(any("jump" in item for item in payload["anomalies"]))

    def test_list_teach_record_files_summarizes_jsonl_records(self) -> None:
        sample = TeachSample(
            stamp=1.0,
            joint_names=("joint1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(0.0,),
            motor_status={},
            arm_state="GRAVITY_COMP",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "one.jsonl"
            path.write_text(encode_teach_sample(sample) + "\n", encoding="utf-8")

            records = list_teach_record_files(tmpdir)

        self.assertEqual(records[0]["path"], str(path))
        self.assertEqual(records[0]["samples"], 1)

    def test_list_teach_record_files_hides_prepared_outputs(self) -> None:
        sample = TeachSample(
            stamp=1.0,
            joint_names=("joint1",),
            positions=(0.0,),
            velocities=(0.0,),
            efforts=(0.0,),
            motor_status={},
            arm_state="GRAVITY_COMP",
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            raw_path = Path(tmpdir) / "teach1.jsonl"
            prepared_path = Path(tmpdir) / "teach1.prepared.v3.jsonl"
            raw_path.write_text(encode_teach_sample(sample) + "\n", encoding="utf-8")
            prepared_path.write_text(encode_teach_sample(sample) + "\n", encoding="utf-8")

            records = list_teach_record_files(tmpdir)

        self.assertEqual([record["name"] for record in records], ["teach1.jsonl"])

    def test_teach_trajectory_quality_classifies_green_yellow_and_red_jumps(self) -> None:
        green = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.02,), (), (), {}, "GRAVITY_COMP"),
        ]
        yellow = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.04,), (), (), {}, "GRAVITY_COMP"),
        ]
        red = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.07,), (), (), {}, "GRAVITY_COMP"),
        ]

        self.assertEqual(analyze_teach_trajectory(green).risk_level, "green")
        self.assertEqual(analyze_teach_trajectory(yellow).risk_level, "yellow")
        red_quality = analyze_teach_trajectory(red)

        self.assertEqual(red_quality.risk_level, "red")
        self.assertEqual(red_quality.worst_joint, "joint1")
        self.assertEqual(red_quality.worst_sample, 1)
        self.assertFalse(red_quality.allow_real_replay)

    def test_teach_trajectory_quality_marks_nonmonotonic_time_red(self) -> None:
        samples = [
            TeachSample(1.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.9, ("joint1",), (0.01,), (), (), {}, "GRAVITY_COMP"),
        ]

        quality = analyze_teach_trajectory(samples)

        self.assertEqual(quality.risk_level, "red")
        self.assertTrue(any("timestamp" in item for item in quality.anomalies))

    def test_teach_trajectory_quality_uses_signed_velocity_for_acceleration(self) -> None:
        samples = [
            TeachSample(0.00, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.1,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.20, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
        ]

        quality = analyze_teach_trajectory(
            samples,
            green_jump_rad=1.0,
            yellow_jump_rad=2.0,
            max_velocity_rad_s=10.0,
            max_acceleration_rad_s2=15.0,
        )

        self.assertGreaterEqual(quality.max_acceleration_rad_s2, 19.999)
        self.assertEqual(quality.risk_level, "yellow")
        self.assertTrue(any("acceleration" in item for item in quality.anomalies))

    def test_teach_trajectory_quality_reports_jerk_limit_violations(self) -> None:
        samples = [
            TeachSample(0.00, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.20, ("joint1",), (0.01,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.40, ("joint1",), (0.08,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.60, ("joint1",), (0.09,), (), (), {}, "GRAVITY_COMP"),
        ]

        quality = analyze_teach_trajectory(
            samples,
            green_jump_rad=1.0,
            yellow_jump_rad=2.0,
            max_velocity_rad_s=10.0,
            max_acceleration_rad_s2=10.0,
            max_jerk_rad_s3=2.0,
        )
        jerk_events = [event for event in quality.events if event.jerk_rad_s3 is not None]

        self.assertGreater(quality.max_jerk_rad_s3, 2.0)
        self.assertEqual(quality.jerk_limit_rad_s3, 2.0)
        self.assertEqual(quality.risk_level, "yellow")
        self.assertTrue(any("jerk" in item for item in quality.anomalies))
        self.assertTrue(jerk_events)

    def test_retime_teach_samples_keeps_positions_and_caps_velocity(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.04,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.08,), (), (), {}, "GRAVITY_COMP"),
        ]

        retimed = retime_teach_samples(
            samples,
            replay_speed=1.0,
            max_velocity_rad_s=0.5,
            initial_delay_sec=0.2,
        )

        self.assertEqual([point.positions for point in retimed], [sample.positions for sample in samples])
        self.assertGreaterEqual(retimed[0].time_from_start, 0.2)
        for previous, current in zip(retimed, retimed[1:]):
            dt = current.time_from_start - previous.time_from_start
            self.assertGreater(dt, 0.0)
            velocity = abs(current.positions[0] - previous.positions[0]) / dt
            self.assertLessEqual(velocity, 0.500001)

    def test_retime_teach_samples_rejects_position_length_mismatch(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1", "joint2"), (0.0, 0.0), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1", "joint2"), (0.1,), (), (), {}, "GRAVITY_COMP"),
        ]

        with self.assertRaises(ValueError):
            retime_teach_samples(
                samples,
                replay_speed=1.0,
                max_velocity_rad_s=1.0,
                max_acceleration_rad_s2=1.0,
            )

    def test_retime_teach_samples_caps_acceleration_and_sets_boundary_zero_velocity(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.2,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.21,), (), (), {}, "GRAVITY_COMP"),
        ]

        retimed = retime_teach_samples(
            samples,
            replay_speed=1.0,
            max_velocity_rad_s=1.0,
            max_acceleration_rad_s2=0.5,
            initial_delay_sec=0.2,
            boundary_zero_velocity=True,
        )

        self.assertEqual(retimed[0].velocities, (0.0,))
        self.assertEqual(retimed[-1].velocities, (0.0,))
        for previous, current in zip(retimed, retimed[1:]):
            dt = current.time_from_start - previous.time_from_start
            for last_velocity, velocity in zip(previous.velocities, current.velocities):
                acceleration = abs(float(velocity) - float(last_velocity)) / dt
                self.assertLessEqual(acceleration, 0.500001)

    def test_retime_teach_samples_caps_jerk(self) -> None:
        samples = [
            TeachSample(0.00, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.2,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.21,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.15, ("joint1",), (0.4,), (), (), {}, "GRAVITY_COMP"),
        ]

        retimed = retime_teach_samples(
            samples,
            replay_speed=1.0,
            max_velocity_rad_s=1.0,
            max_acceleration_rad_s2=0.8,
            max_jerk_rad_s3=1.2,
            initial_delay_sec=0.2,
            boundary_zero_velocity=True,
        )

        last_velocity = retimed[0].velocities[0]
        last_acceleration = 0.0
        for previous, current in zip(retimed, retimed[1:]):
            dt = current.time_from_start - previous.time_from_start
            velocity = current.velocities[0]
            acceleration = (velocity - last_velocity) / dt
            jerk = abs(acceleration - last_acceleration) / dt
            self.assertLessEqual(abs(acceleration), 0.800001)
            self.assertLessEqual(jerk, 1.200001)
            last_velocity = velocity
            last_acceleration = acceleration

    def test_retime_teach_samples_uses_per_joint_velocity_limits(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1", "joint4"), (0.0, 0.0), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1", "joint4"), (0.3, 0.3), (), (), {}, "GRAVITY_COMP"),
        ]

        retimed = retime_teach_samples(
            samples,
            replay_speed=1.0,
            max_velocity_rad_s=(3.0, 1.8),
            max_acceleration_rad_s2=999.0,
            max_jerk_rad_s3=999.0,
            initial_delay_sec=0.0,
            boundary_zero_velocity=False,
        )

        dt = retimed[1].time_from_start - retimed[0].time_from_start
        self.assertGreaterEqual(dt, 0.3 / 1.8)
        self.assertLessEqual(abs(retimed[1].velocities[0]), 3.000001)
        self.assertLessEqual(abs(retimed[1].velocities[1]), 1.800001)

    def test_analyze_teach_trajectory_uses_per_joint_velocity_limits(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1", "joint4"), (0.0, 0.0), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1", "joint4"), (0.25, 0.25), (), (), {}, "GRAVITY_COMP"),
        ]

        quality = analyze_teach_trajectory(
            samples,
            green_jump_rad=1.0,
            yellow_jump_rad=2.0,
            max_velocity_rad_s=(3.0, 1.8),
            max_acceleration_rad_s2=999.0,
            max_jerk_rad_s3=999.0,
        )

        self.assertEqual(quality.risk_level, "yellow")
        self.assertTrue(any("joint4 velocity" in item for item in quality.anomalies))
        self.assertFalse(any("joint1 velocity" in item for item in quality.anomalies))

    def test_teach_trajectory_preview_downsamples_and_marks_risk_points(self) -> None:
        samples = [
            TeachSample(float(index) * 0.05, ("joint1",), (float(index) * 0.02,), (), (), {}, "GRAVITY_COMP")
            for index in range(8)
        ]
        samples[4] = TeachSample(0.20, ("joint1",), (0.10,), (), (), {}, "GRAVITY_COMP")

        payload = teach_trajectory_preview_to_dict(samples, max_points=4)

        self.assertEqual(payload["quality"]["risk_level"], "yellow")
        self.assertLessEqual(len(payload["points"]), 4)
        self.assertTrue(any(event["sample"] == 4 for event in payload["events"]))

    def test_smooth_teach_samples_reduces_middle_spike_and_preserves_endpoints(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.09,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.15, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.20, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
        ]

        smoothed = smooth_teach_samples(samples, window=3)

        self.assertEqual(smoothed[0].positions, samples[0].positions)
        self.assertEqual(smoothed[-1].positions, samples[-1].positions)
        self.assertLess(smoothed[2].positions[0], samples[2].positions[0])
        self.assertEqual([sample.stamp for sample in smoothed], [sample.stamp for sample in samples])

    def test_lowpass_filter_teach_samples_reduces_spike_and_preserves_endpoints(self) -> None:
        samples = [
            TeachSample(0.00, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.02, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.04, ("joint1",), (0.12,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.06, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.08, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
        ]

        filtered = lowpass_filter_teach_samples(
            samples,
            sample_rate_hz=50.0,
            cutoff_hz=5.0,
        )

        self.assertEqual(filtered[0].positions, samples[0].positions)
        self.assertEqual(filtered[-1].positions, samples[-1].positions)
        self.assertLess(filtered[2].positions[0], samples[2].positions[0])

    def test_prepare_teach_replay_samples_auto_smooths_yellow_and_resamples(self) -> None:
        samples = [
            TeachSample(0.00, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.04,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.08,), (), (), {}, "GRAVITY_COMP"),
        ]

        prepared = prepare_teach_replay_samples(
            samples,
            smoothing_enabled=True,
            smoothing_window=3,
            resample_enabled=True,
            resample_rate_hz=50.0,
        )

        self.assertEqual(prepared.before_quality.risk_level, "yellow")
        self.assertEqual(prepared.samples[0].positions, samples[0].positions)
        self.assertEqual(prepared.samples[-1].positions, samples[-1].positions)
        self.assertGreater(len(prepared.samples), len(samples))
        self.assertTrue(prepared.smoothing_applied)
        self.assertTrue(prepared.resample_applied)
        self.assertLessEqual(prepared.after_quality.max_jump_rad, prepared.before_quality.max_jump_rad)

    def test_prepare_teach_replay_samples_can_recover_large_continuous_motion(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1",), (0.08,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.2, ("joint1",), (0.16,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.3, ("joint1",), (0.24,), (), (), {}, "GRAVITY_COMP"),
        ]

        prepared = prepare_teach_replay_samples(
            samples,
            smoothing_enabled=True,
            smoothing_window=3,
            resample_enabled=True,
            resample_rate_hz=50.0,
        )

        self.assertEqual(prepared.before_quality.risk_level, "red")
        self.assertIn(prepared.after_quality.risk_level, ("green", "yellow"))
        self.assertTrue(prepared.after_quality.allow_real_replay)
        self.assertTrue(prepared.smoothing_applied)
        self.assertGreater(len(prepared.samples), len(samples))

    def test_prepare_teach_replay_samples_reports_raw_filtered_and_retimed_quality(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.05, ("joint1",), (0.12,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.10, ("joint1",), (0.13,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.15, ("joint1",), (0.14,), (), (), {}, "GRAVITY_COMP"),
        ]

        prepared = prepare_teach_replay_samples(
            samples,
            filter_enabled=True,
            filter_cutoff_hz=5.0,
            filter_sample_rate_hz=50.0,
            resample_enabled=True,
            resample_rate_hz=100.0,
            retime_enabled=True,
            replay_speed=0.2,
            max_velocity_rad_s=1.0,
            max_acceleration_rad_s2=0.5,
        )

        self.assertEqual(prepared.raw_quality.risk_level, "red")
        self.assertIn(prepared.filtered_quality.risk_level, ("green", "yellow", "red"))
        self.assertIn(prepared.retimed_quality.risk_level, ("green", "yellow"))
        self.assertTrue(prepared.filter_applied)
        self.assertTrue(prepared.retime_applied)

    def test_prepare_teach_replay_samples_reports_time_parameterization_method(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1",), (0.1,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.2, ("joint1",), (0.2,), (), (), {}, "GRAVITY_COMP"),
        ]

        prepared = prepare_teach_replay_samples(
            samples,
            retime_enabled=True,
            time_parameterization_method="ruckig",
            replay_speed=0.2,
            max_velocity_rad_s=1.0,
            max_acceleration_rad_s2=0.5,
            max_jerk_rad_s3=1.2,
        )
        payload = prepared_teach_replay_to_dict(prepared)

        self.assertEqual(payload["time_parameterization"]["requested_method"], "ruckig")
        self.assertIn(payload["time_parameterization"]["used_method"], ("ruckig", "current_jerk_retime"))
        self.assertTrue(payload["time_parameterization"]["message"])
        self.assertTrue(prepared.retime_applied)

    def test_prepare_teach_replay_does_not_claim_python_ruckig_without_adapter(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.1, ("joint1",), (0.1,), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.2, ("joint1",), (0.2,), (), (), {}, "GRAVITY_COMP"),
        ]

        with patch("rebotarm_motion.trajectory_time_parameterization.ruckig_python_available", return_value=True):
            prepared = prepare_teach_replay_samples(
                samples,
                retime_enabled=True,
                time_parameterization_method="ruckig",
                replay_speed=0.2,
                max_velocity_rad_s=1.0,
                max_acceleration_rad_s2=0.5,
                max_jerk_rad_s3=1.2,
            )
        payload = prepared_teach_replay_to_dict(prepared)

        self.assertEqual(payload["time_parameterization"]["requested_method"], "ruckig")
        self.assertEqual(payload["time_parameterization"]["used_method"], "current_jerk_retime")
        self.assertIn("python ruckig waypoint adapter not implemented", payload["time_parameterization"]["message"])

    def test_prepare_teach_replay_samples_keeps_requested_speed_for_large_motion(self) -> None:
        samples = [
            TeachSample(0.0, ("joint1", "joint2"), (0.0, 0.0), (), (), {}, "GRAVITY_COMP"),
            TeachSample(0.5, ("joint1", "joint2"), (0.6, 0.4), (), (), {}, "GRAVITY_COMP"),
            TeachSample(1.0, ("joint1", "joint2"), (1.2, 0.8), (), (), {}, "GRAVITY_COMP"),
        ]

        prepared = prepare_teach_replay_samples(
            samples,
            retime_enabled=True,
            replay_speed=1.0,
            max_velocity_rad_s=2.0,
            max_acceleration_rad_s2=3.0,
            max_jerk_rad_s3=8.0,
            large_motion_span_rad=0.8,
            large_motion_total_rad=2.5,
            large_motion_max_speed=1.0,
        )
        payload = prepared_teach_replay_to_dict(prepared)

        self.assertTrue(prepared.large_motion)
        self.assertAlmostEqual(prepared.requested_replay_speed, 1.0)
        self.assertAlmostEqual(prepared.effective_replay_speed, 1.0)
        self.assertAlmostEqual(payload["large_motion"]["effective_speed"], 1.0)

    def test_write_prepared_teach_record_exports_retimed_jsonl(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            raw_path = Path(tmpdir) / "teach1.jsonl"
            samples = [
                TeachSample(0.0, ("joint1",), (0.0,), (), (), {}, "GRAVITY_COMP"),
                TeachSample(0.05, ("joint1",), (0.12,), (), (), {}, "GRAVITY_COMP"),
                TeachSample(0.10, ("joint1",), (0.13,), (), (), {}, "GRAVITY_COMP"),
                TeachSample(0.15, ("joint1",), (0.14,), (), (), {}, "GRAVITY_COMP"),
            ]
            raw_path.write_text(
                "\n".join(encode_teach_sample(sample) for sample in samples) + "\n",
                encoding="utf-8",
            )
            prepared = prepare_teach_replay_samples(
                samples,
                filter_enabled=True,
                filter_sample_rate_hz=100.0,
                resample_enabled=True,
                resample_rate_hz=100.0,
                retime_enabled=True,
                replay_speed=0.2,
                max_velocity_rad_s=1.0,
                max_acceleration_rad_s2=0.5,
                max_jerk_rad_s3=8.0,
            )

            output_path = write_prepared_teach_record(raw_path, prepared)
            exported = load_teach_samples(output_path)

            self.assertEqual(output_path, prepared_record_path(raw_path))
            self.assertEqual(output_path.name, "teach1.prepared.v5.jsonl")
            self.assertEqual([s.accelerations for s in exported],
                             [p.accelerations for p in prepared.retimed_points])
            self.assertGreater(len(exported), len(samples))
            self.assertTrue(all(sample.arm_state == "PREPARED_REPLAY" for sample in exported))
            self.assertEqual(exported[0].stamp, 0.0)
            self.assertTrue(all(a.stamp <= b.stamp for a, b in zip(exported, exported[1:])))
            self.assertEqual(exported[0].joint_names, samples[0].joint_names)
            self.assertTrue(output_path.exists())

    def test_interpolate_joint_positions_rejects_length_mismatch(self) -> None:
        with self.assertRaises(ValueError):
            from rebotarm_teach.teach_recording import interpolate_joint_positions

            interpolate_joint_positions(
                current_positions=(0.0, 1.0),
                target_positions=(0.1,),
                steps=3,
            )


class TrajectorySafetyMonitorTests(unittest.TestCase):
    def test_evaluate_replay_tracking_accepts_small_tracking_error(self) -> None:
        trajectory = {
            "joint_names": ("joint1", "joint2"),
            "points": [
                {"time_from_start": 0.0, "positions": (0.0, 0.0)},
                {"time_from_start": 1.0, "positions": (1.0, 0.5)},
            ],
        }

        result = evaluate_replay_tracking(
            trajectory,
            joint_names=("joint1", "joint2"),
            positions=(0.51, 0.24),
            velocities=(0.1, 0.1),
            elapsed_sec=0.5,
            max_tracking_error_rad=0.05,
            max_live_velocity_rad_s=2.0,
        )

        self.assertTrue(result.ok)
        self.assertEqual(result.reason, "ok")

    def test_evaluate_replay_tracking_blocks_large_tracking_error(self) -> None:
        trajectory = {
            "joint_names": ("joint1", "joint2"),
            "points": [
                {"time_from_start": 0.0, "positions": (0.0, 0.0)},
                {"time_from_start": 1.0, "positions": (1.0, 0.5)},
            ],
        }

        result = evaluate_replay_tracking(
            trajectory,
            joint_names=("joint1", "joint2"),
            positions=(0.85, 0.25),
            velocities=(0.1, 0.1),
            elapsed_sec=0.5,
            max_tracking_error_rad=0.2,
            max_live_velocity_rad_s=2.0,
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "tracking_error")
        self.assertEqual(result.worst_joint, "joint1")
        self.assertGreater(result.max_tracking_error_rad, 0.2)

    def test_evaluate_replay_tracking_blocks_live_velocity_spike(self) -> None:
        trajectory = {
            "joint_names": ("joint1",),
            "points": [
                {"time_from_start": 0.0, "positions": (0.0,)},
                {"time_from_start": 1.0, "positions": (0.1,)},
            ],
        }

        result = evaluate_replay_tracking(
            trajectory,
            joint_names=("joint1",),
            positions=(0.05,),
            velocities=(3.2,),
            elapsed_sec=0.5,
            max_tracking_error_rad=0.2,
            max_live_velocity_rad_s=3.0,
        )

        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "live_velocity")
        self.assertEqual(result.worst_joint, "joint1")


class StatusPanelStateTests(unittest.TestCase):
    def test_format_angle_readout_contains_radians_and_degrees(self) -> None:
        readout = format_angle_readout(1.57079632679)

        self.assertEqual(readout["rad"], "1.5708")
        self.assertEqual(readout["deg"], "90.0")

    def test_clamp_preview_value_uses_ordered_limits(self) -> None:
        self.assertEqual(clamp_preview_value(2.0, -1.0, 1.0), 1.0)
        self.assertEqual(clamp_preview_value(-2.0, 1.0, -1.0), -1.0)
        self.assertEqual(clamp_preview_value(0.25, -1.0, 1.0), 0.25)

    def test_status_store_snapshot_contains_latest_joint_and_motor_state(self) -> None:
        store = TeleopStatusStore()

        store.update_joint_state(
            names=("joint1", "joint2"),
            positions=(0.1, -0.2),
            velocities=(0.0, 0.01),
            efforts=(0.2, 0.3),
        )
        store.update_motor_state(
            joint_name="joint1",
            position=0.11,
            velocity=0.02,
            torque=0.25,
            status_code=3,
        )
        store.update_arm_status(
            mode="pos_vel",
            enabled=True,
            state_machine="GRAVITY_COMP",
            error_codes=("E1",),
        )

        snapshot = store.snapshot()

        self.assertIsInstance(snapshot, StatusSnapshot)
        self.assertEqual(snapshot.arm["state_machine"], "GRAVITY_COMP")
        self.assertEqual(snapshot.joints["joint1"]["position"], 0.11)
        self.assertEqual(snapshot.joints["joint1"]["status_code"], 3)

    def test_encode_sse_event_wraps_snapshot_json(self) -> None:
        event = encode_sse_event({"state": "active"}, event="status")

        self.assertTrue(event.startswith("event: status\n"))
        self.assertIn('data: {"state":"active"}', event)
        self.assertTrue(event.endswith("\n\n"))


class WebRobotAssetTests(unittest.TestCase):
    def test_rewrite_package_mesh_uris_points_to_local_route(self) -> None:
        urdf = 'filename="package://rebotarm_moveit_config/meshes/link1.STL"'

        rewritten = rewrite_package_mesh_uris(urdf)

        self.assertEqual(rewritten, 'filename="meshes/link1.STL"')

    def test_safe_mesh_path_rejects_nested_paths(self) -> None:
        mesh_dir = Path(__file__).resolve().parent

        self.assertIsNone(safe_mesh_path(mesh_dir, "../link1.STL"))

    def test_load_urdf_joint_limits_reads_revolute_limits(self) -> None:
        urdf_path = ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.urdf"

        limits = load_urdf_joint_limits(urdf_path, ("joint1", "joint2", "missing_joint"))

        self.assertEqual(limits["joint1"], (-2.8, 2.8))
        self.assertEqual(limits["joint2"], (-3.14, 0.02))
        self.assertNotIn("missing_joint", limits)

    def test_merge_joint_limits_prefers_urdf_and_falls_back_to_params(self) -> None:
        merged = merge_joint_limits(
            joint_names=("joint1", "joint2"),
            fallback_limits={"joint1": (-1.0, 1.0), "joint2": (-2.0, 2.0)},
            preferred_limits={"joint2": (-0.5, 0.5)},
        )

        self.assertEqual(merged["joint1"], (-1.0, 1.0))
        self.assertEqual(merged["joint2"], (-0.5, 0.5))

    def test_load_gripper_limits_defaults_to_controller_opening_range(self) -> None:
        self.assertEqual(load_gripper_limits({}), DEFAULT_GRIPPER_LIMITS_M)

    def test_load_gripper_limits_accepts_explicit_opening_range(self) -> None:
        limits = load_gripper_limits({"closed_position_m": 0.01, "open_position_m": 0.08})

        self.assertEqual(limits, (0.01, 0.08))

    def test_gripper_opening_maps_to_symmetric_visual_finger_joints(self) -> None:
        self.assertEqual(clamp_gripper_opening(0.12, (0.0, 0.09)), 0.09)
        self.assertEqual(gripper_opening_to_finger_joint_positions(0.08, (0.0, 0.09)), (0.04, -0.04))
        self.assertEqual(gripper_opening_to_finger_joint_positions(-1.0, (0.0, 0.09)), (0.0, -0.0))

    def test_gripper_finger_links_have_collision_meshes_for_moveit(self) -> None:
        for path in (
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.urdf",
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.urdf",
        ):
            root = ET.fromstring(path.read_text(encoding="utf-8"))
            links = {link.attrib["name"]: link for link in root.findall("link")}

            for link_name, mesh_name in (
                ("left_finger_link", "left_finger.stl"),
                ("right_finger_link", "right_finger.stl"),
            ):
                collision_meshes = [
                    mesh.attrib.get("filename", "")
                    for mesh in links[link_name].findall("./collision/geometry/mesh")
                ]
                self.assertTrue(
                    any(mesh_name in filename for filename in collision_meshes),
                    f"{path} missing {mesh_name} collision on {link_name}",
                )

    def test_end_link_collision_uses_new_gripper_base_mesh(self) -> None:
        for path in (
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.urdf",
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.urdf",
        ):
            root = ET.fromstring(path.read_text(encoding="utf-8"))
            links = {link.attrib["name"]: link for link in root.findall("link")}
            collision_meshes = [
                mesh.attrib.get("filename", "")
                for mesh in links["end_link"].findall("./collision/geometry/mesh")
            ]

            self.assertTrue(
                any("gripper_base.stl" in filename for filename in collision_meshes),
                f"{path} end_link collision should use gripper_base.stl",
            )
            self.assertFalse(
                any("end_link.STL" in filename for filename in collision_meshes),
                f"{path} end_link collision should not use old end_link.STL",
            )

    def test_status_panel_simulates_gripper_state_without_hardware(self) -> None:
        source = (
            ROOT
            / "src"
            / "rebotarm_dashboard"
            / "rebotarm_dashboard"
            / "teleop_status_panel_node.py"
        ).read_text(encoding="utf-8")
        client_source = (
            ROOT
            / "src"
            / "rebotarm_teleop"
            / "rebotarm_teleop"
            / "web_teleop_client.py"
        ).read_text(encoding="utf-8")

        self.assertIn('self._use_hardware = bool(self.get_parameter("use_hardware").value)', source)
        self.assertIn("use_hardware=self._use_hardware", source)
        self.assertIn('f"/{self._arm_namespace}/gripper/state"', source)
        self.assertIn("self._publish_simulated_gripper_state", source)
        self.assertIn('"simulated": True', client_source)

    def test_teach_recorder_retries_auto_gravity_comp_start(self) -> None:
        source = (
            ROOT
            / "src"
            / "rebotarm_teach"
            / "rebotarm_teach"
            / "teach_recorder_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('self.declare_parameter("auto_start_gravity_comp_retry_sec", 1.0)', source)
        self.assertIn('self.declare_parameter("auto_start_gravity_comp_max_attempts", 30)', source)
        self.assertIn("self._gravity_start_client = self.create_client", source)
        self.assertIn("self._try_auto_start_gravity_comp", source)
        self.assertIn("waiting for gravity compensation start service", source)

    def test_moveit_srdf_defines_gripper_group_and_end_effector(self) -> None:
        for path in (
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "rebotarm.srdf",
            ROOT / "src" / "rebotarm_moveit_config" / "config" / "reBot-DevArm_fixend.srdf",
        ):
            root = ET.fromstring(path.read_text(encoding="utf-8"))
            groups = {group.attrib["name"]: group for group in root.findall("group")}
            self.assertIn("gripper", groups)
            self.assertNotIn("arm_with_gripper", groups)
            gripper_joints = {joint.attrib["name"] for joint in groups["gripper"].findall("joint")}
            self.assertEqual(gripper_joints, {"left_finger_joint", "right_finger_joint"})

            end_effectors = {
                ee.attrib["name"]: ee.attrib
                for ee in root.findall("end_effector")
            }
            self.assertEqual(
                end_effectors["gripper"],
                {
                    "name": "gripper",
                    "parent_link": "end_link",
                    "group": "gripper",
                    "parent_group": "arm",
                },
            )

    def test_load_moveit_velocity_limits_reads_joint_limits_yaml(self) -> None:
        path = ROOT / "src" / "rebotarm_moveit_config" / "config" / "joint_limits.yaml"

        limits = load_moveit_velocity_limits(path, ("joint1", "joint6", "left_finger_joint"))

        self.assertEqual(limits, {"joint1": 3.0, "joint6": 1.8, "left_finger_joint": 0.2})

    def test_merge_velocity_limits_uses_default_when_missing(self) -> None:
        merged = merge_velocity_limits(
            joint_names=("joint1", "joint2"),
            default_limit=0.5,
            preferred_limits={"joint2": 1.0},
        )

        self.assertEqual(merged, {"joint1": 0.5, "joint2": 1.0})


class WebExecuteCoreTests(unittest.TestCase):
    def test_web_execute_requires_confirmation(self) -> None:
        decision = validate_web_execute_request(
            {"joint_positions": {"joint1": 0.1}},
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            max_delta_rad=0.25,
            min_duration=1.0,
            max_duration=6.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("confirmation", decision.message)

    def test_web_execute_rejects_large_delta(self) -> None:
        decision = validate_web_execute_request(
            {"confirm": "EXECUTE", "joint_positions": {"joint1": 0.5}, "max_delta_rad": 0.25},
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            max_delta_rad=1.0,
            min_duration=1.0,
            max_duration=6.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("too large", decision.message)

    def test_web_execute_caps_requested_max_delta_to_server_limit(self) -> None:
        decision = validate_web_execute_request(
            {"confirm": "EXECUTE", "joint_positions": {"joint1": 1.2}, "max_delta_rad": 2.0},
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-2.0, 2.0)},
            max_delta_rad=1.0,
            min_duration=1.0,
            max_duration=6.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("1.0000", decision.message)

    def test_web_execute_accepts_safe_joint_target_and_clamps_duration(self) -> None:
        decision = validate_web_execute_request(
            {
                "confirm": "EXECUTE",
                "joint_positions": {"joint1": 0.1},
                "max_delta_rad": 0.8,
                "duration": 99.0,
            },
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            max_delta_rad=1.0,
            min_duration=1.0,
            max_duration=6.0,
        )

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.joint_names, ("joint1",))
        self.assertEqual(decision.positions, (0.1,))
        self.assertEqual(decision.max_delta_limit, 0.8)
        self.assertEqual(decision.duration, 6.0)

    def test_interpolated_web_execute_starts_at_current_position(self) -> None:
        points = interpolate_joint_points(
            current=(0.0, 1.0),
            target=(0.2, 0.8),
            duration=1.0,
            step_period=0.5,
        )

        self.assertEqual(points[0], (0.0, (0.0, 1.0)))
        self.assertEqual(points[-1], (1.0, (0.2, 0.8)))
        self.assertGreater(len(points), 2)

    def test_web_execute_rejects_duration_that_exceeds_speed_limit(self) -> None:
        decision = validate_web_execute_request(
            {
                "confirm": "EXECUTE",
                "joint_positions": {"joint1": 0.8},
                "duration": 1.0,
                "max_joint_speed_rad_s": 0.4,
            },
            joint_names=("joint1",),
            current_positions={"joint1": 0.0},
            joint_limits={"joint1": (-1.0, 1.0)},
            max_delta_rad=1.0,
            min_duration=1.0,
            max_duration=8.0,
            joint_velocity_limits={"joint1": 1.0},
            max_joint_speed_rad_s=1.0,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("speed too high", decision.message)

    def test_web_gripper_rejects_targets_outside_limits(self) -> None:
        decision = validate_web_gripper_request(
            {"confirm": "SET_GRIPPER", "position": 0.2},
            gripper_limits=(0.0, 0.09),
            default_max_effort=0.3,
            max_effort_limit=1.5,
        )

        self.assertFalse(decision.accepted)
        self.assertIn("outside limit", decision.message)

    def test_web_gripper_accepts_target_and_caps_effort(self) -> None:
        decision = validate_web_gripper_request(
            {"confirm": "SET_GRIPPER", "position": 0.05, "max_effort": 9.0},
            gripper_limits=(0.0, 0.09),
            default_max_effort=0.3,
            max_effort_limit=1.5,
        )

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.position, 0.05)
        self.assertEqual(decision.max_effort, 1.5)


def test_unknown_motor_position_survives_json_and_sse_without_becoming_zero():
    store = TeleopStatusStore()
    store.update_motor_state(
        joint_name="gripper", position=float("nan"), velocity=0.0,
        torque=0.0, status_code=255,
    )
    payload = store.snapshot_dict()
    assert payload["joints"]["gripper"]["position"] is None
    assert payload["joints"]["gripper"]["status_code"] == 255
    # The exact payload used by HTTP and SSE must be strict JSON.
    assert json.loads(json.dumps(payload, allow_nan=False)) == payload
    event = encode_sse_event(payload)
    data = event.split("data: ", 1)[1].strip()
    assert json.loads(data)["joints"]["gripper"]["position"] is None
    assert "NaN" not in event
    store.update_motor_state(
        joint_name="gripper", position=0.01, velocity=0.0,
        torque=0.0, status_code=0,
    )
    assert store.snapshot_dict()["joints"]["gripper"]["position"] == 0.01


def test_dashboard_unknown_gripper_feedback_does_not_render_or_command_zero():
    import shutil
    import subprocess

    node = shutil.which("node")
    if node is None:
        raise unittest.SkipTest("Node.js required for dashboard behavior test")
    html = (ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html").read_text()

    def section(start, end):
        return html[html.index(start):html.index(end, html.index(start))]

    helpers = section("    const motorPosition =", "    const deg =")
    slider = section("    const updateSlider =", "    const syncPreviewFromLive =")
    # 网页已移除独立夹爪入口；继续验证共享实际反馈滑条不会用零冒充缺失值。
    script = r'''
const assert = require('node:assert/strict');
const elements = new Map();
const document = {getElementById(id) {
  if (!elements.has(id)) elements.set(id, {textContent: '', value: 0.04});
  return elements.get(id);
}};
const jointLimits = {gripper: [0, 0.085]};
const clamp = (x, low, high) => Math.max(low, Math.min(high, x));
const deg = (x) => String(x);
const previewState = {latestJoints: {gripper: {position: null}}, targets: {gripper: 0.02}};
let fetchCalls = 0;
const fetch = () => {fetchCalls++; throw Error('Unexpected request');};
'''+helpers+slider+r'''
(async () => {
  assert.ok(Number.isNaN(motorPosition({position: null})));
  assert.equal(motorPosition({position: 0}), 0);
  updateSlider('gripper', {position: null});
  assert.equal(document.getElementById('slider-rad-gripper').textContent, '-');
  assert.equal(document.getElementById('slider-gripper').value, 0.04);
  updateSlider('gripper', {position: 0.01});
  assert.equal(document.getElementById('slider-gripper').value, 0.01);
})().catch(error => {console.error(error); process.exitCode = 1;});
'''
    result = subprocess.run([node, "-e", script], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr


def test_dashboard_gripper_view_drives_whole_finger_joints_without_drift():
    import shutil
    import subprocess

    node = shutil.which("node")
    if node is None:
        raise unittest.SkipTest("Node.js required for dashboard behavior test")
    html = (ROOT / "src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html").read_text()

    def section(start, end):
        return html[html.index(start):html.index(end, html.index(start))]

    helpers = section("    const motorPosition =", "    const deg =")
    viewer = section("    const robotViewer =", "    const viewerStatus =")
    update = section("    const jointsFromTargets =", "    initRobotViewer().catch")
    script = r'''
const assert = require('node:assert/strict');
const document = {body: {dataset: {}}};
const window = {};
const jointLimits = {gripper: [0, 0.09]};
const armJointNames = ['joint1'];
const sliderNames = [...armJointNames, 'gripper'];
const clamp = (x, low, high) => Math.max(low, Math.min(high, x));
const scheduleRobotRender = () => {};
function joint(name) {
  return {name, jointValue: [0], calls: 0, setJointValue(value) {
    this.jointValue[0] = value;
    this.calls++;
  }};
}
''' + helpers + viewer + update + r'''
const left = joint('left_finger_joint');
const right = joint('right_finger_joint');
const arm = joint('joint1');
robotViewer.robot = {joints: {joint1: arm, left_finger_joint: left, right_finger_joint: right}};
robotViewer.gripperJoints = {left, right};
const feedback = position => updateRobotViewer({joint1: {position: 0.2}, gripper: {position}});
const check = half => {
  assert.ok(Math.abs(left.jointValue[0] - half) < 1e-12);
  assert.ok(Math.abs(right.jointValue[0] + half) < 1e-12);
};
feedback(0.06);
assert.equal(left.calls, 0); // 加载完成前不更新模型。
robotViewer.ready = true;
for (const [position, half] of [[0, 0], [0.06, 0.03], [0.09, 0.045], [-1, 0], [1, 0.045]]) {
  feedback(position);
  check(half);
}
assert.equal(arm.jointValue[0], 0.2);
jointLimits.gripper = [0.01, 0.08];
for (const [position, half] of [[0.01, 0], [0.05, 0.02], [0.08, 0.035]]) {
  feedback(position);
  check(half);
}
const calls = left.calls;
for (const position of [null, undefined, NaN, Infinity, -Infinity, '0.02']) {
  feedback(position);
  check(0.035);
}
assert.equal(left.calls, calls);
assert.equal(right.calls, calls);
jointLimits.gripper = [0, 0.09];
for (let i = 0; i < 100; i++) {
  updateRobotViewer(jointsFromTargets({joint1: 0.4, gripper: 0.06}));
  check(0.03);
  feedback(0);
  check(0);
}
assert.equal(window.__rebotGripperDebug.leftJoint, 'left_finger_joint');
assert.equal(window.__rebotGripperDebug.rightJoint, 'right_finger_joint');
assert.equal(window.__rebotGripperDebug.leftPosition, 0);
assert.equal(Math.abs(window.__rebotGripperDebug.rightPosition), 0);
'''
    result = subprocess.run([node, "-e", script], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr


def test_dashboard_robot_view_uses_horizontal_ground_grid_and_above_ground_camera():
    html = (
        ROOT
        / "src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html"
    ).read_text(encoding="utf-8")

    assert "camera.position.set(0.72, 0.56, 0.72);" in html
    assert "controls.target.set(0, 0.22, 0);" in html
    assert "grid.rotation.x" not in html


if __name__ == "__main__":
    unittest.main()
