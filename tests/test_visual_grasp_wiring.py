from __future__ import annotations

import ast
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _read(relative: str) -> str:
    vision = "src/rebotarm_vision/rebotarm_vision/"
    layers = {
        vision + "nodes/visual_grasp_executor_node.py": (
            "visual_grasp_workflow.py", "visual_grasp_runtime.py",
            "visual_grasp_service_gateway.py", "utils/visual_grasp_messages.py",
        ),
        vision + "nodes/candidate_ik_filter_node.py": (
            "policies/candidate_ik_policy.py", "candidate_ik_gateway.py", "candidate_ik_runtime.py",
        ),
    }
    if relative in layers:
        return "\n".join([_read_file(relative)] + [_read_file(vision + name) for name in layers[relative]])
    if relative == "src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py":
        return "\n".join(
            [
                _read_file("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py"),
                _read_file("src/rebotarm_teach/rebotarm_teach/teach_replay_workflow.py"),
                _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_page.py"),
                _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_http.py"),
                _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html"),
            ]
        )
    return _read_file(relative)


def _read_file(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def _console_scripts(setup_relative: str) -> set[str]:
    tree = ast.parse(_read(setup_relative))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", "") != "setup":
            continue
        for keyword in node.keywords:
            if keyword.arg != "entry_points":
                continue
            entry_points = ast.literal_eval(keyword.value)
            scripts = entry_points.get("console_scripts", [])
            return {script.split("=", 1)[0].strip() for script in scripts}
    raise AssertionError(f"setup() entry_points not found in {setup_relative}")


def test_status_panel_page_is_split_from_ros_node():
    panel_text = _read_file("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    page_text = _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_page.py")
    html_text = _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/index.html")
    setup_text = _read_file("src/rebotarm_dashboard/setup.py")

    assert "from .status_panel_page import HTML_PAGE" in panel_text
    assert "HTML_PAGE = r\"\"\"" not in panel_text
    assert "importlib.resources" in page_text
    assert "status_panel_assets" in page_text
    assert "HTML_PAGE = " in page_text
    assert "HTML_PAGE = r\"\"\"" not in page_text
    assert 'id="robot-view"' in html_text
    assert 'id="arm-safe-home"' in html_text
    assert '"rebotarm_dashboard.status_panel_assets": ["index.html", "calibration.html"]' in setup_text


def test_status_panel_http_server_is_split_from_ros_node():
    panel_text = _read_file("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    http_text = _read_file("src/rebotarm_dashboard/rebotarm_dashboard/status_panel_http.py")

    assert "from .status_panel_http import create_status_panel_server" in panel_text
    assert "BaseHTTPRequestHandler" not in panel_text
    assert "ThreadingHTTPServer" not in panel_text
    assert "def do_POST" not in panel_text
    assert "create_status_panel_server(" in panel_text
    assert "class StatusPanelRequestHandler" in http_text
    assert "dispatch_post_request" in http_text
    assert "encode_sse_event" in http_text


def test_rebotarm_vision_exposes_grasp_console_entrypoints():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")

    assert {
        "rebotarm_graspnet_baseline_node",
        "rebotarm_send_grasp_preview",
        "rebotarm_visual_grasp_executor",
        "rebotarm_visual_grasp_markers",
        "rebotarm_grasp_tcp_frame",
        "rebotarm_grasp_depth_probe",
        "rebotarm_visual_grasp_benchmark",
    }.issubset(scripts)
    assert "rebotarm_visual_ready" not in scripts
    assert "rebotarm_tcp_calibration" not in scripts


def test_visual_grasp_system_launch_defaults_to_safe_plan_only_mode():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("use_hardware", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("use_local_rviz", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("execution_mode", default_value="plan_only")' in launch_text
    assert 'DeclareLaunchArgument("execute_gripper", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("start_grasp_preview",' in launch_text
    assert (
        'DeclareLaunchArgument("start_visual_grasp_executor", default_value="false")'
        in launch_text
        or 'DeclareLaunchArgument("start_visual_grasp_executor", default_value="true")'
        in launch_text
    )
    assert 'executable="rebotarm_send_grasp_preview"' in launch_text
    assert 'executable="rebotarm_visual_grasp_executor"' in launch_text
    assert '"input_topic": executor_input_topic' in launch_text
    assert (
        '"/interactive_control/pose_target"' in launch_text
        or '"motion_execution"'
        in launch_text
    )
    assert '"min_grasp_z_m": min_target_z_m' in launch_text


def test_visual_grasp_system_uses_provisional_upper_motion_scaling_defaults():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    executor_text = _read(
        "src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py"
    )

    expected = {
        "move_velocity_scaling": "0.25",
        "approach_velocity_scaling": "0.08",
        "retreat_velocity_scaling": "0.15",
        "acceleration_scaling": "0.12",
    }
    for name, default in expected.items():
        assert f'{name} = LaunchConfiguration("{name}")' in launch_text
        assert f'DeclareLaunchArgument("{name}", default_value="{default}")' in launch_text
        assert f'"{name}": {name}' in launch_text
        assert f'self.declare_parameter("{name}", {float(default):.2f})' in executor_text

    assert '"default_velocity_scaling": move_velocity_scaling' in launch_text
    assert '"default_acceleration_scaling": acceleration_scaling' in launch_text


def test_visual_grasp_system_uses_measured_grasp_tcp_offset_by_default():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]")' in launch_text


def test_real_grasp_profiles_follow_operator_measured_tcp():
    expected = "[-0.04, 0.0, 0.0]"
    for relative in (
        "src/rebotarm_vision/config/camera_ubuntu.yaml",
    ):
        assert f"tcp_offset_xyz: {expected}" in _read(relative)

    local_model = _read("src/rebotarm_simulation/models/rebotarm/robot.xml")
    assert 'name="ee_site" pos="-0.04 0 0"' in local_model


def test_visual_grasp_strategy_defaults_are_split_into_yaml_profiles():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    expected_profiles = [
        "grasp_pose_policy.yaml",
        "gripper_policy.yaml",
        "retry_policy.yaml",
        "retreat_policy.yaml",
        "visual_servo.yaml",
        "table_safety.yaml",
    ]

    for filename in expected_profiles:
        profile = _read(f"src/rebotarm_vision/config/{filename}")
        assert profile.strip(), filename
        assert f'PathJoinSubstitution([vision_share, "config", "{filename}"])' in launch_text
        assert f'"config/{filename}"' in _read("src/rebotarm_vision/setup.py")

    assert "grasp_pose_policy_params" in launch_text
    assert "gripper_policy_params" in launch_text
    assert "retry_policy_params" in launch_text
    assert "retreat_policy_params" in launch_text
    assert "visual_servo_params" in launch_text
    assert "table_safety_params" in launch_text
    assert "visual_ready_params" in launch_text
    assert "parameters=[\n            visual_ready_params," in launch_text
    assert "graspnet_config,\n                {" in launch_text
    assert "policy_defaults.update(_read_policy" in launch_text
    assert "gripper_policy_params" in launch_text
    assert "retry_policy_params" in launch_text
    assert "retreat_policy_params" in launch_text
    assert "visual_servo_params" in launch_text
    assert "table_safety_params" in launch_text


def test_gripper_launch_compatibility_defaults_match_yaml_profile():
    """Keep legacy launch overrides numerically aligned with the canonical YAML defaults."""
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    profile = _read("src/rebotarm_vision/config/gripper_policy.yaml")
    defaults = {
        "close_position_m": "0.025",
        "close_max_effort": "0.4",
        "open_before_approach": "true",
        "auto_gripper_width": "true",
        "auto_gripper_effort": "true",
        "open_clearance_m": "0.0",
        "close_margin_m": "0.012",
        "min_gripper_effort": "0.22",
        "max_gripper_effort": "0.60",
        "max_allowed_grasp_width_m": "0.085",
        "gripper_grasp_enabled": "true",
        "gripper_grasp_close_force": "0.4",
        "gripper_grasp_timeout_sec": "8.0",
        "gripper_grasp_min_close_time_sec": "0.08",
        "gripper_grasp_velocity_threshold": "0.04",
        "gripper_grasp_min_closure_distance_m": "0.006",
    }
    for name, value in defaults.items():
        assert f"{name}: {value}" in profile
        assert (
            f'DeclareLaunchArgument("{name}", default_value="{value}")' in launch_text
            or f'"{name}"' in launch_text
        )


def test_visual_grasp_system_uses_graspnet_candidates_directly_before_ik():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("candidate_ik_input_topic", default_value="/grasp/graspnet_candidates")' in launch_text
    assert 'executable="rebotarm_regular_object_candidate_node"' not in launch_text
    assert 'executable="rebotarm_grasp_candidate_merge_node"' not in launch_text
    assert "/grasp/regular_candidates" not in launch_text
    assert "/grasp/merged_candidates" not in launch_text
    assert '"output_candidates_topic": graspnet_candidates_topic' in launch_text
    assert '"input_topic": candidate_ik_input_topic' in launch_text


def test_visual_grasp_system_can_move_to_visual_ready_on_start():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    node_text = _read("src/rebotarm_motion/rebotarm_motion/visual_ready_node.py")

    assert 'DeclareLaunchArgument("start_visual_ready", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("move_to_visual_ready_on_start", default_value="false")' in launch_text
    assert (
        'default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]"'
        in launch_text
    )
    assert 'DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0")' in launch_text
    assert 'DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5")' in launch_text
    assert 'executable="rebotarm_visual_ready"' in launch_text
    assert "OnProcessExit" in launch_text
    assert "target_action=visual_ready_startup" in launch_text
    assert "on_exit=post_visual_ready_actions" in launch_text
    assert 'name="rebotarm_visual_ready_startup"' in launch_text
    assert '"exit_after_startup_move": True' in launch_text
    assert "post_visual_ready_actions = [" in launch_text
    assert "run_visual_ready_startup = PythonExpression(" in launch_text
    assert "condition=IfCondition(run_visual_ready_startup)" in launch_text
    assert "condition=UnlessCondition(run_visual_ready_startup)" in launch_text
    assert '"auto_move_on_start": move_to_visual_ready_on_start' in launch_text
    assert '"startup_delay_sec": visual_ready_startup_delay_sec' in launch_text
    assert '"joint_positions": visual_ready_joint_positions' in launch_text
    assert "FollowJointTrajectory" in node_text
    assert "visual_ready startup move refused" in node_text
    assert 'self.declare_parameter("auto_move_on_start", True)' in node_text
    assert 'self.declare_parameter("exit_after_startup_move", False)' in node_text
    assert 'bool(node.get_parameter("exit_after_startup_move").value)' in node_text
    assert 'self.declare_parameter("startup_delay_sec", 0.0)' in node_text
    assert "[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]" in node_text
    assert 'f"/{namespace}/visual_ready/move"' in node_text
    assert "create_service(" in node_text
    assert "Trigger," in node_text
    assert "ARM_JOINT_NAMES" in node_text
    assert '"initial_joint_positions": visual_ready_joint_positions' in launch_text
    assert 'self.declare_parameter("initial_joint_positions", [0.0, -0.1, -0.2, 0.2, 0.0, 0.0])' in _read(
        "src/rebotarm_simulation/rebotarm_simulation/sim_trajectory_controller_node.py"
    )


def test_visual_grasp_system_starts_vision_chain_after_visual_ready():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    ready_index = launch_text.index("visual_ready_startup = Node(")
    post_ready_index = launch_text.index("post_visual_ready_actions = [")
    vision_index = launch_text.index('PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])')
    ik_index = launch_text.index('executable="rebotarm_grasp_candidate_ik_filter"')
    executor_index = launch_text.index("OpaqueFunction(function=_launch_executor)")
    handler_index = launch_text.index("OnProcessExit(")

    assert ready_index < post_ready_index < vision_index < ik_index < executor_index
    assert handler_index > executor_index
    assert "GroupAction(" in launch_text
    assert "condition=UnlessCondition(run_visual_ready_startup)" in launch_text


def test_visual_grasp_system_starts_sim_state_backend_before_moveit_and_ready_gate():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    sim_definition_index = launch_text.index("sim_trajectory_controller = Node(")
    post_ready_index = launch_text.index("post_visual_ready_actions = [")
    launch_sim_index = launch_text.index("            sim_trajectory_controller,")
    launch_moveit_index = launch_text.index("            interactive_system,", launch_sim_index)
    launch_ready_index = launch_text.index("            visual_ready_startup,", launch_moveit_index)

    assert sim_definition_index < post_ready_index
    assert launch_sim_index < launch_moveit_index < launch_ready_index
    assert launch_text.count('executable="rebotarm_sim_trajectory_controller"') == 1


def test_visual_grasp_system_uses_lightweight_rviz_config():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    rviz_text = _read("src/rebotarm_bringup/rviz/visual_grasp.rviz")

    assert '"rviz_config": PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"])' in launch_text
    assert "rviz_default_plugins/RobotModel" in rviz_text
    assert "rviz_default_plugins/TF" in rviz_text
    assert "rviz_default_plugins/MarkerArray" in rviz_text
    assert "/grasp/visual_markers" in rviz_text
    assert "moveit_rviz_plugin/MotionPlanning" not in rviz_text
    assert "moveit_rviz_plugin/Trajectory" in rviz_text
    assert "Trajectory Topic: /display_planned_path" in rviz_text
    assert "State Display Time: 0.03 s" in rviz_text
    assert '"publish_plan_only_preview": PythonExpression(' in launch_text
    assert 'DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="0.0")' in launch_text
    assert "rviz_default_plugins/MoveCamera" not in rviz_text
    assert "rviz_default_plugins/Select" not in rviz_text


def test_visual_grasp_system_owns_readonly_rviz_and_open3d_previews():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert not (
        ROOT / "src/rebotarm_bringup/launch/visual_grasp_perception_preview.launch.py"
    ).exists()
    assert 'executable="rebotarm_graspnet_baseline_node"' in launch_text
    assert 'executable="rebotarm_grasp_candidate_ik_filter"' in launch_text
    assert 'executable="rebotarm_visual_grasp_markers"' in launch_text
    assert 'executable="rebotarm_grasp_candidate_markers"' in launch_text
    assert 'DeclareLaunchArgument("start_open3d_viewer", default_value="true")' in launch_text
    assert 'executable="rebotarm_graspnet_open3d_viewer"' in launch_text
    assert 'condition=IfCondition(start_open3d_viewer)' in launch_text
    assert '"input_candidates_topic": graspnet_candidates_topic' in launch_text
    assert 'DeclareLaunchArgument("start_raw_candidate_markers", default_value="true")' in launch_text
    assert 'condition=IfCondition(start_raw_candidate_markers)' in launch_text
    assert '"output_topic": "/grasp/raw_candidate_markers"' in launch_text
    assert 'DeclareLaunchArgument("start_graspnet_baseline", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose")' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20")' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="1.5708")' in launch_text
    assert '"max_jaw_width_m": candidate_max_jaw_width_m' in launch_text
    assert '"input_topic": candidate_ik_input_topic' in launch_text
    assert "interactive_system.launch.py" in launch_text
    assert 'DeclareLaunchArgument("use_hardware", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("execution_mode", default_value="plan_only")' in launch_text
    assert 'condition=IfCondition(start_visual_ready)' in launch_text


def test_retired_real_perception_sim_execution_entrypoint_is_absent():
    assert not (
        ROOT
        / "src/rebotarm_bringup/launch/real_perception_sim_execution.launch.py"
    ).exists()



def test_visual_grasp_system_can_disable_rviz_only_controller_for_external_mujoco():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'start_sim_trajectory_controller = LaunchConfiguration("start_sim_trajectory_controller")' in launch_text
    assert 'DeclareLaunchArgument(\n                "start_sim_trajectory_controller",' in launch_text
    assert "start_sim_trajectory_controller," in launch_text
    assert '"max_input_skew_ms": graspnet_max_input_skew_ms' in launch_text
    assert "graspnet_config," in launch_text
    assert 'DeclareLaunchArgument("graspnet_config", default_value=graspnet_ubuntu_params)' in launch_text
    assert 'prefix=graspnet_python_executable' in launch_text


def test_sim_trajectory_controller_publishes_gripper_state_for_rviz_finger_links():
    sim_text = _read("src/rebotarm_simulation/rebotarm_simulation/sim_trajectory_controller_node.py")
    visual_text = _read("src/rebotarm_teleop/rebotarm_teleop/gripper_visual_joint_state_node.py")

    assert "from rebotarm_msgs.msg import JointMotorState" in sim_text
    assert 'self._gripper_state_pub = self.create_publisher(' in sim_text
    assert 'f"/{self._arm_namespace}/gripper/state"' in sim_text
    assert "self._publish_gripper_state(width)" in sim_text
    assert "def _publish_gripper_state" in sim_text
    assert "msg = JointMotorState()" in sim_text
    assert "msg.position = float(width_m)" in sim_text
    assert "self._gripper_state_pub.publish(msg)" in sim_text
    assert 'f"/{namespace}/gripper/state"' in visual_text
    assert "self._latest_gripper_position = float(msg.position)" in visual_text


def test_visual_grasp_system_forwards_hardware_channel_and_uses_safe_shutdown_default():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    interactive_text = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")

    assert 'DeclareLaunchArgument("channel", default_value="auto")' in launch_text
    assert 'DeclareLaunchArgument("shutdown_safe_home", default_value="false")' in launch_text
    assert '"channel": channel' in launch_text
    assert '"shutdown_safe_home": shutdown_safe_home' in launch_text
    assert 'DeclareLaunchArgument("shutdown_safe_home", default_value="false")' in interactive_text
    assert '"shutdown_safe_home": shutdown_safe_home' in interactive_text
    assert 'DeclareLaunchArgument("start_motion_execution", default_value="true")' in launch_text
    assert 'executable="PoseExecutionNode"' in launch_text
    assert 'DeclareLaunchArgument("moveit_planning_time", default_value="8.0")' in launch_text
    assert 'DeclareLaunchArgument("moveit_num_planning_attempts", default_value="5")' in launch_text
    assert '"moveit_planning_time": moveit_planning_time' in launch_text
    assert '"moveit_num_planning_attempts": moveit_num_planning_attempts' in launch_text
    assert 'f"/{self._arm_namespace}/motion_execution/execute_pose"' in _read(
        "src/rebotarm_motion/rebotarm_motion/pose_execution_node.py"
    )
    assert "ExecutePose" in _read("src/rebotarm_motion/rebotarm_motion/pose_execution_node.py")


def test_visual_grasp_markers_show_tcp_approach_and_open_axis():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    marker_text = _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_marker_node.py")

    assert 'DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]")' in launch_text
    assert 'DeclareLaunchArgument("show_tcp_markers", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("show_approach_arrow", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("show_gripper_open_axis", default_value="true")' in launch_text
    assert '"input_topic": executor_input_topic' in launch_text
    assert '"tcp_offset_xyz": tcp_offset_xyz' in launch_text
    assert '"gripper_open_axis_local_xyz": gripper_open_axis_local_xyz' in launch_text
    assert '"show_tcp_markers": show_tcp_markers' in launch_text
    assert '"show_approach_arrow": show_approach_arrow' in launch_text
    assert '"show_gripper_open_axis": show_gripper_open_axis' in launch_text
    assert '"show_object_marker": False' in launch_text
    assert '"show_object_center_marker": False' in launch_text
    assert '"show_object_label": False' in launch_text
    assert "visual_object_center" in marker_text
    assert "visual_pregrasp_tcp" in marker_text
    assert "visual_grasp_tcp" in marker_text
    assert "visual_approach_axis" in marker_text
    assert "visual_gripper_open_axis" in marker_text
    assert "Marker.ARROW" in marker_text
    assert "Marker.LINE_LIST" in marker_text


def test_visual_grasp_plan_age_default_is_mode_aware():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert '"max_plan_age_sec",\n                default_value=PythonExpression(' in launch_text
    assert "default: 10.0 for plan_only, 4.0 for execute" in launch_text
    assert "\"'10.0' if '\"," in launch_text
    assert "\"'.lower() == 'plan_only' else '4.0'\"," in launch_text
    assert '"max_plan_age_sec": max_plan_age_sec' in launch_text


def test_visual_grasp_benchmark_returns_ready_between_attempts():
    benchmark_text = _read(
        "src/rebotarm_vision/rebotarm_vision/benchmarks/visual_grasp_benchmark.py"
    )

    assert "--attempts" in benchmark_text
    assert "--wait-enter" in benchmark_text
    assert "--return-ready-before-each" in benchmark_text
    assert 'f"/{namespace}/visual_ready/move"' in benchmark_text
    assert 'f"/{namespace}/visual_grasp/execute"' in benchmark_text
    assert "class BenchmarkStats" in benchmark_text
    assert "classify_failure_stage" in benchmark_text
    assert "success_rate" in benchmark_text


def test_hybrid_grasp_sim_benchmark_waits_for_fresh_filtered_plan_before_execute():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")
    benchmark_text = _read(
        "src/rebotarm_vision/rebotarm_vision/benchmarks/hybrid_grasp_sim_benchmark.py"
    )
    doc_text = _read("docs/reference/commands/visual_grasp_commands.md")

    assert "rebotarm_hybrid_grasp_sim_benchmark" in scripts
    assert 'self.create_subscription(GraspPlan, self._plan_topic, self._on_plan, 10)' in benchmark_text
    assert 'f"/{namespace}/visual_grasp/execute"' in benchmark_text
    assert 'self._wait_for_fresh_valid_plan(' in benchmark_text
    assert "--min-success-rate" in benchmark_text
    assert "--plan-timeout-sec" in benchmark_text
    assert "--return-ready-after-each" in benchmark_text
    assert "return_ready_after_each" in benchmark_text
    assert "plan source=" in benchmark_text
    assert "benchmark" in doc_text
    assert "rebotarm_hybrid_grasp_sim_benchmark" in doc_text
    assert "--return-ready-after-each" in doc_text
    assert "candidate_max_joint6_delta_rad:=1.5708" in doc_text
    assert "candidate_joint6_symmetry_enabled:=true" in doc_text
    assert "gripper_grasp_enabled:=false" in doc_text


def test_visual_grasp_commands_document_strict_stability_test():
    doc_text = _read("docs/reference/commands/visual_grasp_commands.md")
    readme_text = _read("src/rebotarm_vision/README_zh.md")

    assert "rebotarm_visual_grasp_benchmark" in doc_text
    assert "--attempts 20" in doc_text
    assert "--return-ready-before-each" in doc_text
    assert "--wait-enter" in doc_text
    assert "/rebotarm/visual_ready/move" in doc_text
    assert "/rebotarm/visual_grasp/execute" in doc_text
    assert "failed_stage" in doc_text
    assert "V1.1" in readme_text
    assert "20" in readme_text
    assert "rebotarm_visual_grasp_benchmark" in readme_text
    assert "rebotarm_visual_grasp_benchmark" in readme_text


def test_graspnet_baseline_v13_is_wired_as_candidate_source_without_replacing_execution():
    setup_scripts = _console_scripts("src/rebotarm_vision/setup.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    node_text = _read("src/rebotarm_vision/rebotarm_vision/nodes/graspnet_baseline_node.py")

    assert "rebotarm_graspnet_baseline_node" in setup_scripts
    assert 'DeclareLaunchArgument("start_graspnet_baseline", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("graspnet_candidates_topic", default_value="/grasp/graspnet_candidates")' in launch_text
    assert '"GRASPNET_MODEL_ROOT", "third_party/graspnet-baseline"' in launch_text
    assert '"GRASPNET_CHECKPOINT_PATH"' in launch_text
    assert 'executable="rebotarm_graspnet_baseline_node"' in launch_text
    assert '"output_candidates_topic": graspnet_candidates_topic' in launch_text
    assert '"output_candidates_topic": graspnet_candidates_topic' in launch_text
    assert 'DeclareLaunchArgument("candidate_ik_input_topic", default_value="/grasp/graspnet_candidates")' in launch_text
    assert '"input_topic": candidate_ik_input_topic' in launch_text
    assert "candidate_scoring_mode" not in launch_text
    assert '"scoring_mode"' not in launch_text
    assert 'DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan")' in launch_text
    assert "InProcessGraspNetBackend" in node_text
    assert "GraspCandidateArray" in node_text
    assert "self.candidates_pub.publish(candidates)" in node_text
    assert "MoveIt" not in node_text
    assert "follow_joint_trajectory" not in node_text


def test_visual_grasp_executor_keeps_stop_paths_wired():
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )

    assert 'f"/{self._arm_namespace}/visual_grasp/execute"' in executor_text
    assert 'f"/{self._arm_namespace}/visual_grasp/stop"' in executor_text
    assert 'f"/{self._arm_namespace}/interactive_control/execute_preview"' not in executor_text
    assert 'f"/{self._arm_namespace}/interactive_control/stop"' not in executor_text
    assert 'f"/{self._arm_namespace}/motion_execution/execute_pose"' in executor_text
    assert 'f"/{self._arm_namespace}/motion_execution/stop"' in executor_text
    assert 'f"/{self._arm_namespace}/trajectory_stop"' in executor_text
    assert 'self._request_stop_service(self._trajectory_stop_client, "trajectory_stop")' in executor_text


def test_low_level_controller_exports_trajectory_stop_service():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_services.py")

    assert 'self._service("trajectory_stop")' in controller_text
    assert "def trajectory_stop(self, _request, response):" in controller_text


def test_rebotarm_msgs_exports_grasp_gripper_service():
    cmake_text = _read("src/rebotarm_msgs/CMakeLists.txt")
    srv_text = _read("src/rebotarm_msgs/srv/GraspGripper.srv")

    assert '"srv/GraspGripper.srv"' in cmake_text
    assert "float64 close_force" in srv_text
    assert "float64 hold_force" in srv_text
    assert "bool contact_detected" in srv_text
    assert "float64 contact_position" in srv_text


def test_retired_ordinary_grasp_is_absent_from_active_vision_path():
    package = ROOT / "src/rebotarm_vision"
    assert not (package / "rebotarm_vision/ordinary_grasp_node.py").exists()
    assert not (package / "rebotarm_vision/converters/ordinary_grasp_adapter.py").exists()
    assert "rebotarm_ordinary_grasp_node" not in _read("src/rebotarm_vision/setup.py")
    assert "ordinary_grasp_node" not in _read(
        "src/rebotarm_vision/rebotarm_vision/nodes/graspnet_baseline_node.py"
    )
    for relative in (
        "src/rebotarm_vision/launch/vision.launch.py",
        "src/rebotarm_vision/launch/vision_ubuntu.launch.py",
        "src/rebotarm_bringup/launch/visual_grasp_system.launch.py",
        "src/rebotarm_vision/config/camera_ubuntu.yaml",
    ):
        assert "ordinary_grasp" not in _read(relative)


def test_vision_tf_consumers_do_not_import_preview_node_helpers():
    for name in (
        "grasp_candidate_marker_node.py",
        "visual_grasp_marker_node.py",
    ):
        source = _read(f"src/rebotarm_vision/rebotarm_vision/nodes/{name}")
        assert "from ..utils.tf_message_adapter import" in source
        assert "_transform_from_msg" not in source
        assert "from ..grasp_preview_sender_node import" not in source
    source = _read("src/rebotarm_vision/rebotarm_vision/candidate_tf_adapter.py")
    assert "from .utils.tf_message_adapter import" in source
    assert "_transform_from_msg" not in source
    assert "from .grasp_preview_sender_node import" not in source


def test_rebotarm_vision_exposes_candidate_ik_filter_entrypoint():
    scripts = _console_scripts("src/rebotarm_vision/setup.py")

    assert "rebotarm_grasp_candidate_ik_filter" in scripts


def test_visual_grasp_system_launch_wires_candidate_ik_filter():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'DeclareLaunchArgument("start_candidate_ik_filter", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates")' in launch_text
    assert 'DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity")' in launch_text
    assert 'DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper")' in launch_text
    assert 'executable="rebotarm_grasp_candidate_ik_filter"' in launch_text
    assert 'DeclareLaunchArgument("grasp_candidates_topic", default_value="/grasp/candidates")' not in launch_text
    assert '"input_topic": candidate_ik_input_topic' in launch_text
    assert '"output_topic": filtered_candidates_topic' in launch_text
    assert '"output_plan_topic": filtered_plan_topic' in launch_text
    assert '"collision_check_enabled": candidate_collision_check_enabled' in launch_text
    assert '"collision_check_service": candidate_collision_check_service' in launch_text
    assert '"collision_group_name": candidate_collision_group_name' in launch_text


def test_visual_grasp_system_disables_extra_fake_joint_state_sources():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    interactive_text = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")

    assert 'DeclareLaunchArgument("start_passive_joint_state_publisher", default_value="true")' in interactive_text
    assert 'DeclareLaunchArgument("use_moveit_fake_joint_states", default_value="true")' in interactive_text
    assert 'start_passive_joint_state_publisher = LaunchConfiguration("start_passive_joint_state_publisher")' in interactive_text
    assert 'use_moveit_fake_joint_states = LaunchConfiguration("use_moveit_fake_joint_states")' in interactive_text
    assert '"use_moveit_fake_joint_states": "false"' in launch_text
    assert '"start_passive_joint_state_publisher": "false"' in launch_text
    assert "condition=IfCondition(" in interactive_text
    assert "PythonExpression(" in interactive_text
    assert 'start_passive_joint_state_publisher' in interactive_text


def test_visual_grasp_executor_consumes_grasp_plan_by_default():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )

    assert 'DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan")' in launch_text
    assert 'executor_input_topic = LaunchConfiguration("executor_input_topic")' in launch_text
    assert '"input_topic": executor_input_topic' in launch_text
    assert 'if str(getattr(plan, "source", "")).strip() == "candidate_ik_filter":' in executor_text
    assert "return self._build_motion_targets_from_filtered_plan(plan)" in executor_text
    assert "def _build_motion_targets_from_filtered_plan" in executor_text


def test_visual_grasp_executor_refreshes_plan_after_pregrasp():
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert 'self.declare_parameter("input_topic", "/grasp/filtered_plan")' in executor_text
    assert 'self.declare_parameter("refresh_plan_at_pregrasp_enabled", True)' in executor_text
    assert 'self.declare_parameter("refresh_plan_at_pregrasp_required", True)' in executor_text
    assert 'self.declare_parameter("refresh_plan_timeout_sec", 1.0)' in executor_text
    assert "state.plan_revision" in executor_text
    assert "def _wait_for_refreshed_plan" in executor_text
    assert 'stage.name == "move_to_pregrasp"' in executor_text
    assert "fresh grasp plan unavailable after pregrasp" in executor_text
    assert "stages = self._replace_remaining_after_pregrasp" in executor_text
    assert 'DeclareLaunchArgument("refresh_plan_at_pregrasp_enabled"' not in launch_text
    assert 'DeclareLaunchArgument("refresh_plan_at_pregrasp_required"' not in launch_text
    servo_profile = _read("src/rebotarm_vision/config/visual_servo.yaml")
    assert "refresh_plan_at_pregrasp_enabled: false" in servo_profile
    assert "refresh_plan_at_pregrasp_required: false" in servo_profile
    assert '"refresh_plan_at_pregrasp_enabled": False' not in launch_text
    assert '"refresh_plan_at_pregrasp_required": False' not in launch_text


def test_visual_grasp_executor_has_bounded_approach_visual_servo():
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "from .policies.visual_servo_policy import VisualServoApproachConfig, build_visual_servo_step" in executor_text
    assert 'self.declare_parameter("approach_visual_servo_enabled", False)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_max_iterations", 5)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_max_step_m", 0.02)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_position_tolerance_m", 0.008)' in executor_text
    assert 'self.declare_parameter("approach_visual_servo_require_fresh_plan", True)' in executor_text
    assert "def _run_visual_servo_approach" in executor_text
    assert "fresh visual servo plan unavailable" in executor_text
    assert "build_visual_servo_step" in executor_text
    assert 'VisualGraspStage(name="visual_servo_approach"' in executor_text
    assert 'DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true")' in launch_text
    assert '"approach_visual_servo_enabled": approach_visual_servo_enabled' in launch_text
    assert '"approach_visual_servo_require_fresh_plan": approach_visual_servo_require_fresh_plan' in launch_text


def test_visual_grasp_executor_wires_retry_verification_place_and_recovery():
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    recovery_text = _read("src/rebotarm_vision/rebotarm_vision/policies/trajectory_recovery_policy.py")

    assert "GraspCandidateArray, GraspPlan" in executor_text
    assert "from .policies.grasp_retry_policy import RetryPolicyConfig, ordered_candidate_indices" in executor_text
    assert "from .policies.grasp_verification_policy import" in executor_text
    assert "from .policies.place_task_policy import PlaceTaskConfig, build_place_stages" in executor_text
    assert "from .policies.trajectory_recovery_policy import RecoveryConfig, recovery_decision_for_stage" in executor_text
    assert 'self.declare_parameter("candidates_topic", "/grasp/filtered_candidates")' in executor_text
    assert 'self.declare_parameter("auto_retry_enabled", False)' in executor_text
    assert 'self.declare_parameter("grasp_verification_enabled", True)' in executor_text
    assert 'self.declare_parameter("place_after_grasp_enabled", False)' in executor_text
    assert 'self.declare_parameter("trajectory_precheck_enabled", True)' in executor_text
    assert "def _candidate_plans_for_attempts" in executor_text
    assert "attempts: list[tuple[int, GraspPlan]] = [(-1, plan)]" in executor_text
    assert "if index == int(candidates.best_index):" in executor_text
    assert "continue" in executor_text
    assert "def _verify_after_close" in executor_text
    assert 'return True, "gripper disabled: grasp verification skipped"' in executor_text
    assert "def _append_place_stages" in executor_text
    assert "def _precheck_execute_pose" in executor_text
    assert 'name="retry_safe_retreat"' in executor_text
    assert "retry_retreat_stage = stage" in executor_text
    assert "self._run_stage(retreat)" in executor_text
    assert "recovery_decision_for_stage" in executor_text
    assert '"close_gripper"' not in recovery_text
    assert '"lift"' not in recovery_text
    assert 'DeclareLaunchArgument("auto_retry_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("place_after_grasp_enabled", default_value="false")' in launch_text
    assert 'DeclareLaunchArgument("trajectory_precheck_enabled", default_value="true")' in launch_text
    assert '"candidates_topic": filtered_candidates_topic' in launch_text
    assert '"auto_retry_enabled": auto_retry_enabled' in launch_text
    assert '"grasp_verification_enabled": grasp_verification_enabled' in launch_text
    assert '"place_after_grasp_enabled": place_after_grasp_enabled' in launch_text
    assert '"trajectory_precheck_enabled": trajectory_precheck_enabled' in launch_text


def test_candidate_ik_filter_node_uses_moveit_ik_and_state_validity_without_execution():
    node_text = _read("src/rebotarm_vision/rebotarm_vision/nodes/candidate_ik_filter_node.py")
    runtime_text = _read("src/rebotarm_vision/rebotarm_vision/candidate_ik_runtime.py")
    node_text += "\n" + runtime_text
    node_text += "\n" + _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_ik_policy.py")
    node_text += "\n" + _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_motion_policy.py")
    node_text += "\n" + _read("src/rebotarm_vision/rebotarm_vision/candidate_ik_gateway.py")
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")
    motion_policy_text = _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_motion_policy.py")
    pose_variant_text = _read("src/rebotarm_vision/rebotarm_vision/policies/pose_variant_policy.py")
    target_policy_text = _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_target_policy.py")
    gate_policy_text = _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_gate_policy.py")
    scoring_policy_text = _read("src/rebotarm_vision/rebotarm_vision/policies/candidate_scoring_policy.py")
    tf_adapter_text = _read("src/rebotarm_vision/rebotarm_vision/candidate_tf_adapter.py")
    feasibility_policy_text = _read("src/rebotarm_vision/rebotarm_vision/policies/motion_feasibility_policy.py")

    assert "GetPositionIK" in node_text
    assert "GetStateValidity" in node_text
    assert "from sensor_msgs.msg import JointState" in node_text
    assert 'self.declare_parameter("moveit_ik_service", "/compute_ik")' in node_text
    assert 'self.declare_parameter("joint_state_topic", "/rebotarm/visual_joint_states")' in node_text
    assert 'self.declare_parameter("collision_check_service", "/check_state_validity")' in node_text
    assert 'self.declare_parameter("collision_group_name", "arm_with_gripper")' in node_text
    assert 'self.declare_parameter("service_timeout_sec", 5.0)' in node_text
    assert 'self.declare_parameter("pose_policy", "preserve_candidate_pose")' in node_text
    assert 'self.declare_parameter("orientation_yaw_offsets_rad",' in node_text
    assert 'self.declare_parameter("candidate_grasp_z_offsets_m",' in node_text
    assert 'self.declare_parameter("max_candidates_per_frame", 20)' in node_text
    assert 'self.declare_parameter("max_variants_per_candidate"' not in node_text
    assert 'self.declare_parameter("orientation_yaw_offsets_rad", [0.0])' in node_text
    assert 'self.declare_parameter("candidate_grasp_z_offsets_m", [0.0])' in node_text
    assert 'self.declare_parameter("candidate_min_jaw_width_m", 0.006)' in node_text
    assert 'self.declare_parameter("candidate_max_jaw_width_m", 0.085)' in node_text
    assert "def _symmetric_parallel_jaw_delta" not in node_text
    assert 'self.declare_parameter("candidate_max_joint6_delta_rad", 1.5708)' in node_text
    assert 'self.declare_parameter("candidate_joint6_symmetry_enabled", True)' in node_text
    assert 'self.declare_parameter("candidate_joint6_symmetry_angle_rad", math.pi)' in node_text
    assert "evaluate_joint_motion(" in node_text
    assert "JointMotionPolicyConfig(" in node_text
    assert 'deltas = {' in motion_policy_text
    assert 'abs(angle_delta' in motion_policy_text
    assert 'deltas["joint6"] = _symmetric_parallel_jaw_delta' not in node_text
    assert "build_candidate_target_variants(" in node_text
    assert "CandidateTargetPolicyConfig(" in node_text
    assert 'self.declare_parameter("candidate_pregrasp_min_z_m", 0.04)' in node_text
    assert 'pregrasp_min_z_m=float(_parameter(self, "candidate_pregrasp_min_z_m").value)' in node_text
    assert "build_parallel_jaw_pose_variants(" in target_policy_text
    assert "pregrasp_min_z_m: float = 0.0" in target_policy_text
    assert "build_parallel_jaw_symmetric_orientation" in pose_variant_text
    assert "parallel_jaw_symmetric" in pose_variant_text
    assert 'self.declare_parameter("candidate_min_grasp_z_m", 0.0)' in node_text
    assert "self._latest_joint_state" in node_text
    assert "def _on_joint_state" in node_text
    assert "def _valid_joint_state" in node_text
    assert "def _candidate_target_variants" in node_text
    assert "def _official_geometry_target_variants" not in node_text
    assert "def _hybrid_geometry_target_variants" not in node_text
    assert "def _preserve_candidate_target_variants" not in node_text
    assert "build_hybrid_geometry_grasp_targets" not in node_text
    assert "build_official_geometry_grasp_targets" not in node_text
    assert "build_preserve_candidate_grasp_targets" not in node_text
    assert "build_hybrid_geometry_grasp_targets" in target_policy_text
    assert "build_official_geometry_grasp_targets" in target_policy_text
    assert "build_preserve_candidate_grasp_targets" in target_policy_text
    assert "CandidateGateConfig" in node_text
    assert "evaluate_candidate_gate(" in node_text
    assert "CandidateWorkspaceGateConfig" not in node_text
    assert "candidate_workspace_gate(" not in node_text
    assert "from .candidate_workspace_gate" not in node_text
    assert "CandidateWorkspaceGateConfig" in gate_policy_text
    assert "candidate_workspace_gate(" in gate_policy_text
    assert 'self.declare_parameter("candidate_workspace_gate_enabled", False)' in node_text
    assert 'self.declare_parameter("candidate_workspace_min_xyz", [0.18, -0.35, 0.0])' in node_text
    assert 'self.declare_parameter("candidate_workspace_max_xyz", [0.64, 0.35, 0.45])' in node_text
    assert 'self.declare_parameter("candidate_max_grasp_to_object_center_m", 0.15)' in node_text
    assert "variants = self.policy._candidate_target_variants(msg, candidate.pose)" in node_text
    assert "for pregrasp, grasp, variant_label in variants:" in node_text
    assert "request.ik_request.robot_state.joint_state = deepcopy(seed)" in node_text
    assert "request.ik_request.avoid_collisions = False" in node_text
    assert "candidate IK filter IK failed" in node_text
    assert "orientation=(" in node_text
    assert "def _target_debug_text" in node_text
    assert "def check_state_validity" in node_text
    assert "candidate IK filter state validity failed" in node_text
    assert "candidate_scoring_policy" in node_text
    assert "CandidateScoringInput(" in node_text
    assert "score_candidate(" in node_text
    assert "def _preserve_input_score" not in node_text
    assert "candidate_tf_adapter" in node_text
    assert "transform_candidate_pose_to_target_frame(" in node_text
    assert "from ..grasp_preview_sender_node import _transform_from_msg, transform_pose_message" not in node_text
    assert "transform_pose_message(" not in node_text
    assert "_transform_from_msg(" not in node_text
    assert "lookup_transform(" in tf_adapter_text
    assert "transform_pose_message(" in tf_adapter_text
    assert "motion_feasibility_policy" in node_text
    assert "evaluate_motion_feasibility(" in node_text
    assert "pregrasp_solution = self._check_ik_and_collision" not in node_text
    assert "grasp_solution = self._check_ik_and_collision" not in node_text
    assert "motion_penalty, motion_reason = self._joint_motion_penalty" not in node_text
    assert "check_target(pregrasp" in feasibility_policy_text
    assert "check_target(grasp" in feasibility_policy_text
    assert "rank_score = -float" in scoring_policy_text
    assert "variant_penalty = z_variant_penalty" in scoring_policy_text
    assert "candidate IK filter best:" in node_text
    assert "ranked = sorted(ranked" in node_text
    assert "filter_candidate_array_by_reachability" in node_text
    assert "plan.pregrasp_pose = _pose_from_target(pregrasp)" in node_text
    assert "plan.grasp_pose = _pose_from_target(grasp)" in node_text
    assert "self._plan_pub.publish" in node_text
    assert "follow_joint_trajectory" not in node_text
    assert "execute_pose" not in node_text
    assert 'DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states")' in launch_text
    assert 'DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0")' in launch_text
    assert 'DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose")' in launch_text
    assert 'DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad",' in launch_text
    assert 'DeclareLaunchArgument("candidate_grasp_z_offsets_m",' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20")' in launch_text
    assert 'DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]")' in launch_text
    assert 'DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]")' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_variants_per_candidate"' not in launch_text
    assert 'DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006")' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.085")' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="1.5708")' in launch_text
    assert 'DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793")' in launch_text
    assert 'DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0")' in launch_text
    assert 'DeclareLaunchArgument("candidate_pregrasp_min_z_m", default_value="0.04")' in launch_text
    assert "candidate_safe_lift_min_z_m" not in launch_text
    assert 'DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[0.18, -0.35, 0.0]")' in launch_text
    assert 'DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.64, 0.35, 0.45]")' in launch_text
    assert 'DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]")' in launch_text
    assert 'default_value="[0.0, 0.0, 0.0, 1.0]"' in launch_text
    assert 'DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15")' in launch_text
    assert '"joint_state_topic": candidate_joint_state_topic' in launch_text
    assert '"service_timeout_sec": candidate_filter_service_timeout_sec' in launch_text
    assert '"pose_policy": candidate_pose_policy' in launch_text
    assert '"orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad' in launch_text
    assert '"candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m' in launch_text
    assert '"max_candidates_per_frame": candidate_max_candidates_per_frame' in launch_text
    assert '"max_variants_per_candidate": candidate_max_variants_per_candidate' not in launch_text
    assert '"candidate_min_jaw_width_m": candidate_min_jaw_width_m' in launch_text
    assert '"candidate_max_jaw_width_m": candidate_max_jaw_width_m' in launch_text
    assert '"max_jaw_width_m": candidate_max_jaw_width_m' in launch_text
    assert '"candidate_pregrasp_min_z_m": candidate_pregrasp_min_z_m' in launch_text
    assert '"candidate_workspace_gate_enabled": candidate_workspace_gate_enabled' in launch_text
    assert '"candidate_workspace_min_xyz": candidate_workspace_min_xyz' in launch_text
    assert '"candidate_workspace_max_xyz": candidate_workspace_max_xyz' in launch_text
    assert '"candidate_max_grasp_to_object_center_m": candidate_max_grasp_to_object_center_m' in launch_text


def test_gripper_visual_joint_state_node_rejects_empty_or_incomplete_arm_state():
    node_text = _read(
        "src/rebotarm_teleop/rebotarm_teleop/gripper_visual_joint_state_node.py"
    )

    assert '"required_arm_joint_names"' in node_text
    assert "self._warned_invalid_arm_state = False" in node_text
    assert "def _valid_arm_joint_state" in node_text
    assert "if not self._valid_arm_joint_state(msg):" in node_text
    assert "ignored empty or incomplete arm joint state" in node_text
    assert "return all(math.isfinite" in node_text


def test_candidate_ik_filter_builds_filtered_plan_from_reachable_targets_without_ros_init():
    import importlib.util

    try:
        moveit_msgs_spec = importlib.util.find_spec("moveit_msgs")
    except ValueError:
        moveit_msgs_spec = None
    if moveit_msgs_spec is None:
        pytest.skip("moveit_msgs is not installed in this Python environment")

    from rebotarm_msgs.msg import GraspCandidate, GraspCandidateArray
    from rebotarm_vision.nodes.candidate_ik_filter_node import CandidateIkFilterNode
    from rebotarm_vision.visual_grasp_sequence import PoseTarget

    filtered = GraspCandidateArray()
    filtered.header.frame_id = "camera_depth_frame"
    filtered.best_index = 0
    candidate = GraspCandidate()
    candidate.class_name = "bottle"
    candidate.jaw_width = 0.04
    filtered.candidates.append(candidate)

    node = object.__new__(CandidateIkFilterNode)
    node._target_frame = "base_link"
    pregrasp = PoseTarget(position=(0.30, 0.00, 0.24), orientation=(0.0, 0.0, 0.0, 1.0))
    grasp = PoseTarget(position=(0.38, 0.00, 0.18), orientation=(0.0, 0.0, 0.0, 1.0))

    plan = CandidateIkFilterNode._plan_from_filtered(node, filtered, [(pregrasp, grasp)])

    assert plan.valid is True
    assert plan.source == "candidate_ik_filter"
    assert plan.header.frame_id == "base_link"
    assert plan.pregrasp_pose.position.z == pytest.approx(0.24)
    assert plan.grasp_pose.position.x == pytest.approx(0.38)


def test_low_level_controller_exports_grasp_gripper_service():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_services.py")
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")

    assert "GraspGripper" in controller_text
    assert 'self._service("gripper/grasp")' in controller_text
    assert "def grasp_gripper(self, request, response):" in controller_text
    assert "def grasp_gripper(" in hardware_text
    assert "_gripper_mode = \"grasp_closing\"" in hardware_text
    assert "_gripper_mode = \"grasp_holding\"" in hardware_text
    assert "_G_GRASP_CLOSE_KP = 0.0" in hardware_text
    assert "_G_GRASP_CLOSE_KD = 0.5" in hardware_text
    assert "_G_GRASP_HOLD_KP = 5.0" in hardware_text
    assert "_G_GRASP_HOLD_KD = 1.0" in hardware_text


def test_visual_grasp_executor_uses_grasp_service_for_close_stage():
    executor_text = (
        _read("src/rebotarm_vision/rebotarm_vision/nodes/visual_grasp_executor_node.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_workflow.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_runtime.py")
        + "\n" + _read("src/rebotarm_vision/rebotarm_vision/visual_grasp_service_gateway.py")
    )

    assert "GraspGripper" in executor_text
    assert 'f"/{self._arm_namespace}/gripper/grasp"' in executor_text
    assert "gripper_grasp_enabled" in executor_text
    assert "def _call_grasp_gripper" in executor_text
    assert 'stage.name == "close_gripper"' in executor_text
    assert 'request.close_force = max(float(_parameter(self, "gripper_grasp_close_force").value), 0.0)' in executor_text
    assert "request.hold_force = max(float(stage.gripper_max_effort), 0.0)" in executor_text


def test_visual_grasp_launch_does_not_expose_hard_object_safety_params():
    launch_text = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert "hard_object_safety_enabled" not in launch_text
    assert "hard_object_max_close_force" not in launch_text
    assert "hard_object_max_hold_force" not in launch_text
    assert "hard_object_require_detected_width" not in launch_text


def test_low_level_trajectory_stop_holds_current_position_immediately():
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")
    stop_body = hardware_text.split("def stop_active_motion(self) -> None:", 1)[1].split("\n    def ", 1)[0]

    assert "self._endpos_ctrl._stop_send.set()" in stop_body
    assert "self.hold_current_position()" in stop_body
    assert "if self._enabled:" in stop_body
    assert 'self.set_state_machine("IDLE")' in stop_body


def test_follow_joint_trajectory_rechecks_stop_before_writing_next_target():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    execute_body = actions_text.split("def execute_follow_joint_trajectory(self, goal_handle):", 1)[1].split("\n    def _set_endpos_target", 1)[0]

    assert "def _trajectory_stopped" in actions_text
    assert "if self._trajectory_stopped(goal_handle, result):" in execute_body
    assert execute_body.index("if self._trajectory_stopped(goal_handle, result):") < execute_body.index("self._set_endpos_target")


def test_follow_joint_trajectory_keeps_running_until_goal_settles():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    execute_body = actions_text.split("def execute_follow_joint_trajectory(self, goal_handle):", 1)[1].split("\n    def _set_endpos_target", 1)[0]

    settle_index = execute_body.index("ok, max_error = self._wait_until_goal_reached")
    success_index = execute_body.index("goal_handle.succeed()")
    idle_index = execute_body.index('self._hardware.set_state_machine("IDLE")')

    assert settle_index < success_index < idle_index


def test_status_panel_stop_replay_falls_back_to_controller_stop():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    client_text = _read("src/rebotarm_teach/rebotarm_teach/teach_replay_client.py")

    assert "self._trajectory_stop_client = self.create_client(" in panel_text
    assert 'f"/{self._arm_namespace}/trajectory_stop"' in panel_text
    assert "self._teach_replay_client.stop(" in panel_text
    assert "trajectory_stop_client=self._trajectory_stop_client" in panel_text
    assert "controller trajectory_stop requested" in client_text


def test_status_panel_web_stop_always_requests_controller_stop():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    client_text = _read("src/rebotarm_teleop/rebotarm_teleop/web_teleop_client.py")
    stop_body = panel_text.split("def _handle_stop_execute(self) -> dict:", 1)[1].split("\n    def ", 1)[0]

    assert "self._web_teleop_client.stop(" in stop_body
    assert "trajectory_stop_client=self._trajectory_stop_client" in stop_body
    assert "call_trigger_service(" in client_text
    assert "no active web execute goal; controller trajectory_stop requested" in client_text
    assert "self._execute_goal_handle = None" in stop_body


def test_status_panel_web_execute_settings_are_number_inputs_only():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert 'id="execute-max-delta" type="number"' in panel_text
    assert 'id="execute-duration" type="number"' in panel_text
    assert 'id="execute-speed" type="number"' in panel_text
    assert 'id="execute-max-delta" type="range"' not in panel_text
    assert 'id="execute-duration" type="range"' not in panel_text
    assert 'id="execute-speed" type="range"' not in panel_text
    assert "bindExecuteSetting('execute-max-delta', 'maxDelta'" in panel_text


def test_status_panel_exposes_arm_service_buttons():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert 'id="arm-safe-home"' in panel_text
    assert 'id="arm-enable"' in panel_text
    assert 'id="arm-disable"' in panel_text
    assert 'id="arm-command-state"' in panel_text
    assert 'f"/{self._arm_namespace}/safe_home"' in panel_text
    assert 'f"/{self._arm_namespace}/enable"' in panel_text
    assert 'f"/{self._arm_namespace}/disable"' in panel_text
    assert "def _handle_arm_service_command" in panel_text
    assert panel_text.count("const runArmCommand = async") == 1
    assert "statusObj(data.teleop.arm_command).state" in panel_text
    assert "self._store.update_arm_status(" in panel_text
    assert "if command == \"disable\":" in panel_text
    assert "enabled = False" in panel_text


def test_status_panel_surfaces_gripper_motor_state():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    motor_body = panel_text.split("const updateMotorRows = (joints) => {", 1)[1].split(
        "const previewArmTargets = () => {", 1
    )[0]
    status_body = panel_text.split("source.addEventListener('status', (event) => {", 1)[1].split(
        "const replayStatus =", 1
    )[0]

    assert 'id="gripper-command-state"' in panel_text
    assert "joint7" in motor_body
    assert "閻㈠灚婧€7 / endjoint / gripper" not in motor_body
    assert "source.gripper = { missing: true }" in motor_body
    assert "motorOrder" in motor_body
    assert "statusObj(data.teleop.web_gripper).state" in status_body


def test_web_execute_returns_to_live_feedback_after_sending_gripper():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    execute_body = panel_text.split("const executePreviewAndGripper = async () => {", 1)[1].split(
        "const runTeachDryRun = async () => {", 1
    )[0]

    assert "const gripperResult = await setGripper({ confirm: false })" in execute_body
    assert "exitPreviewToLive" in execute_body
    assert "const exitPreviewToLive =" in panel_text
    assert "previewState.active = false" in panel_text
    assert "exitPreviewToLive" in execute_body
    assert "updateRobotViewer(previewState.latestJoints)" in panel_text


def test_gripper_action_aborts_when_target_is_not_reached():
    actions_text = _read("src/rebotarmcontroller/rebotarmcontroller/ros_actions.py")
    gripper_body = actions_text.split("def execute_gripper_command(self, goal_handle):", 1)[1].split(
        "\n    def ", 1
    )[0]

    assert "failure_reason is None and self._hardware.gripper_reached_target()" in gripper_body
    assert "self._hardware.cancel_gripper_position_command(" in gripper_body
    assert "if result.reached_goal:" in gripper_body
    assert "goal_handle.succeed()" in gripper_body
    assert "goal_handle.abort()" in gripper_body


def test_arm_service_buttons_are_interlocked_during_replay():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    run_body = panel_text.split("const runArmCommand = async (command, label) => {", 1)[1].split(
        "const bindTeachReplaySetting", 1
    )[0]
    status_body = panel_text.split("source.addEventListener('status', (event) => {", 1)[1].split(
        "attachControlCardToggles();", 1
    )[0]
    backend_body = panel_text.split("def _handle_arm_service_command(self, command: str) -> dict:", 1)[1].split(
        "\n    def _handle_teach_record_start", 1
    )[0]

    assert "isReplayArmCommandLocked()" in run_body
    assert "teach replay is still stopping; arm commands are locked" in run_body
    assert "isReplayArmCommandLocked(replayStatus)" in status_body
    assert "arm_command_is_replay_locked(replay_state)" in backend_body
    assert "arm command blocked during teach replay" in backend_body
    assert "restoreArmButtons" in run_body
    assert "const result = await response.json()" in run_body
    assert "should_stop_trajectory_before_arm_command(command)" in backend_body
    assert "trajectory_stop_requested" in backend_body
    assert "exitPreviewToLive(`${label}" in run_body


def test_safety_stop_allows_operator_recovery_controls():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    lock_body = panel_text.split("const isReplayArmCommandLocked =", 1)[1].split("const addReplayEvent", 1)[0]
    dry_run_button_body = panel_text.split("const updateTeachDryRunButton = (info) => {", 1)[1].split(
        "const refreshTeachFileInfo", 1
    )[0]
    backend_body = panel_text.split("def _handle_arm_service_command(self, command: str) -> dict:", 1)[1].split(
        "\n    def _handle_teach_record_start", 1
    )[0]

    assert "['replaying', 'cancel_requested', 'stop_requested'].includes(state)" in lock_body
    assert "safety_stop" not in lock_body
    assert "['replaying', 'cancel_requested', 'safety_stop'].includes(replayState)" in dry_run_button_body
    assert "arm_command_is_replay_locked(replay_state)" in backend_body
    assert '"safety_stop"' not in backend_body.split("arm_command_is_replay_locked(replay_state)", 1)[0]


def test_status_panel_defaults_cards_collapsed_and_removes_keyboard_sliders():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert 'class="panel collapsible-card collapsed" id="arm-status-card"' in panel_text
    assert 'class="panel slider-panel collapsible-card collapsed" id="web-teleop-card"' in panel_text
    assert 'class="panel collapsible-card collapsed" id="keyboard-teleop-card"' in panel_text
    assert 'class="panel teach-panel collapsible-card collapsed" id="teach-trajectory-card"' in panel_text
    assert 'class="panel collapsible-card collapsed" id="motor-state-card"' in panel_text
    assert 'id="keyboard-step" type="range"' not in panel_text
    assert 'id="keyboard-speed" type="range"' not in panel_text
    assert "arm-command-status" not in panel_text


def test_status_panel_right_card_order_and_simplified_teach_card():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert panel_text.index('id="arm-status-card"') < panel_text.index('id="motor-state-card"')
    assert panel_text.index('id="motor-state-card"') < panel_text.index('id="web-teleop-card"')
    assert panel_text.index('id="web-teleop-card"') < panel_text.index('id="teach-trajectory-card"')
    assert panel_text.index('id="teach-trajectory-card"') < panel_text.index('id="keyboard-teleop-card"')
    assert panel_text.index('id="robot-view"') < panel_text.index('id="arm-safe-home"')
    assert 'id="teach-record-name"' in panel_text
    assert "/api/teach_record_start" in panel_text
    assert "runTeachDryRun" in panel_text
    assert 'id="teach-trajectory-card"' in panel_text
    assert panel_text.index('id="teach-record-step"') < panel_text.index('id="teach-record-name"')
    assert panel_text.index('id="teach-check-step"') < panel_text.index('id="teach-record-select"')
    assert 'id="teach-record-select"' in panel_text
    assert '<details class="teach-step" id="teach-record-step">' in panel_text
    assert '<details class="teach-step" id="teach-check-step">' in panel_text
    assert '<details class="teach-step" id="teach-replay-step">' in panel_text
    assert '<details class="teach-step" id="teach-check-step" open>' not in panel_text
    assert '<details class="teach-step" id="teach-replay-step" open>' not in panel_text
    assert 'id="teach-record-select"' in panel_text
    assert "#teach-record-select:invalid" in panel_text
    assert 'Default file' not in panel_text
    assert 'id="teach-replay-speed" type="range" min="0.1" max="1.0"' in panel_text
    assert 'id="teach-align-duration"' not in panel_text
    assert 'id="teach-final-hold"' not in panel_text
    assert "Final Hold', '1.0 s'" not in panel_text
    assert "teach-replay-speed" in panel_text
    replay_params_body = panel_text.split("const renderReplayParams = (info) => {", 1)[1].split("const addReplayEvent", 1)[0]
    assert "estimate.speed" in replay_params_body
    assert "estimate.estimatedDuration" in replay_params_body
    assert "Hardware Mode" not in replay_params_body
    assert "Panel Mode" not in replay_params_body
    assert "Direct Threshold" not in replay_params_body
    assert "Align Threshold" not in replay_params_body
    assert "Execution Gate" not in replay_params_body
    assert "Final Hold" not in replay_params_body
    assert "replay-precheck-summary" in panel_text
    assert "Replay Checklist Details" not in panel_text
    assert "safe_home" in panel_text
    assert "Teach JSONL Schema" not in panel_text
    assert "Replay Checklist Details" not in panel_text
    assert "Recording / Gravity" not in panel_text
    assert "Default record_path" not in panel_text
    assert 'id="expand-teach-trajectory"' not in panel_text
    assert "toggleTeachTrajectoryChart" not in panel_text
    assert 'id="teach-trajectory-frame"' not in panel_text
    assert "previewTeachTrajectoryFrame" not in panel_text
    assert "Trajectory Limits" not in panel_text
    file_info_body = panel_text.split("const renderTeachFileInfo = (info) => {", 1)[1].split(
        "const renderTeachRecordSummary", 1
    )[0]
    assert "Playback Quality" not in file_info_body
    assert "Trajectory Risk" not in file_info_body
    assert "Direct Threshold" not in file_info_body
    assert "Align Threshold" not in file_info_body
    assert "Max Jump" not in file_info_body
    assert "Max Velocity" not in file_info_body
    precheck_body = panel_text.split("const renderReplayPrecheckSummary = (info) => {", 1)[1].split("const replayEstimate", 1)[0]
    assert "Raw Risk" not in precheck_body
    assert "effective_risk_level" in precheck_body
    assert "prepared_risk_level" in precheck_body
    assert "prepared_replay?.after_quality" in precheck_body
    assert "large_motion?.effective_speed" in precheck_body
    assert "large_motion?.enabled" in precheck_body
    assert "lastTeachDryRun?.prepared_replay?.after_quality" in precheck_body
    assert "lastTeachDryRun?.effective_risk_level" in precheck_body
    trajectory_body = panel_text.split("const renderTeachTrajectoryDetails = (payload) => {", 1)[1].split("const drawTeachTrajectoryChart", 1)[0]
    assert "Prepared Risk" not in trajectory_body
    assert "Max Jump" not in trajectory_body
    assert "Max Velocity" not in trajectory_body
    assert "effective_risk_level" in precheck_body
    assert "prepared_risk_level" in precheck_body
    assert "prepared_replay?.after_quality" in precheck_body
    assert "teachMetric('Risk'" not in precheck_body
    assert "prepared_record_path" in panel_text
    assert "preparedPoints.length" not in panel_text
    assert "await refreshTeachFileInfo({ force: true })" in panel_text
    assert "await refreshTeachFileInfo({ force: true })" in panel_text
    assert "setHtml('replay-precheck-summary', renderReplayPrecheckSummary(latestTeachFileInfo));" in panel_text


def test_teach_replay_dry_run_token_survives_live_start_error_drift():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    coordinator_text = _read("src/rebotarm_teach/rebotarm_teach/teach_replay_coordinator.py")

    button_body = panel_text.split("const updateTeachDryRunButton = (info) => {", 1)[1].split(
        "const refreshTeachFileInfo", 1
    )[0]

    assert "lastTeachDryRun?.record_path === info?.path" in button_body
    assert "lastTeachDryRun?.accepted === true" in button_body
    assert "dryRunErrorMatches" not in button_body
    assert "lastTeachDryRun?.max_error" not in button_body

    assert 'str(dry_run_token.get("record_path", "")) == str(info_payload.get("path", ""))' in coordinator_text
    assert 'dry_run_token.get("settings") == settings' in coordinator_text
    assert "error_matches" not in coordinator_text
    assert 'dry_run_token.get("worst_joint"' not in coordinator_text
    assert 'dry_run_token.get("risk_level"' not in coordinator_text


def test_status_panel_throttles_heavy_browser_rendering():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "FAST_RENDER_INTERVAL_MS" in panel_text
    assert "const shouldRenderFastPanels = nowMs - lastFastRenderMs > FAST_RENDER_INTERVAL_MS;" in panel_text
    assert "if (!shouldRenderFastPanels) return;" in panel_text
    assert "scheduleRobotRender" in panel_text
    assert "requestAnimationFrame(renderRobotFrame)" in panel_text
    assert "setInterval(refreshTeachFileInfo, 5000)" in panel_text


def test_status_panel_unloads_collapsed_details_and_reuses_motor_rows():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "isDetailsOpen(" in panel_text
    assert "attachDetailsUnloaders()" in panel_text
    assert "renderOptionalDetails(latestStatusData)" in panel_text
    assert "clearOptionalDetails" in panel_text
    assert "updateMotorRows(data.joints || {})" in panel_text
    assert 'document.getElementById("joints").innerHTML = rows' not in panel_text
    assert "motorRowsByName" in panel_text


def test_status_panel_control_cards_can_collapse_to_headers():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "collapsible-card" in panel_text
    assert "card-body" in panel_text
    assert "toggleControlCard" in panel_text
    assert "attachControlCardToggles()" in panel_text
    assert 'data-card-toggle="web-teleop-card"' in panel_text
    assert 'data-card-toggle="teach-trajectory-card"' in panel_text
    assert ".collapsible-card.collapsed .card-body" in panel_text


def test_status_panel_teach_info_accepts_record_path_alias_and_skips_collapsed_polling():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    teach_info_body = panel_text.split('if route == "/api/teach_record_info":', 1)[1].split('if route == "/api/teach_records":', 1)[0]
    refresh_body = panel_text.split("const refreshTeachFileInfo = async", 1)[1].split("const refreshTeachRecords", 1)[0]

    assert 'query.get("path", query.get("record_path", [""]))[0]' in teach_info_body
    assert "isControlCardOpen('teach-trajectory-card')" in refresh_body
    assert "refreshTeachFileInfo({ force: true })" in panel_text


def test_status_panel_compacts_large_teach_replay_payloads_for_sse():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "def _compact_quality_payload" in panel_text
    assert "events_total" in panel_text
    assert "events_truncated" in panel_text
    assert "anomalies_total" in panel_text
    assert "result = self._compact_replay_payload(result)" in panel_text
    assert "self._store.update_teleop_status(\"replay\", result)" in panel_text


def test_status_panel_check_mode_is_read_only_for_teach_actions():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert 'self.declare_parameter("panel_mode", "control")' in panel_text
    assert '"panel_mode": str(self.get_parameter("panel_mode").value)' in panel_text
    assert "const isCheckMode = panelMode === 'check';" in panel_text
    assert "isCheckMode" in panel_text
    assert "panel_mode" in panel_text


def test_status_panel_uses_workbench_cards_for_teleop_ui():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "teleop-workbench" in panel_text
    assert "robot-workspace" in panel_text
    assert "control-cards" in panel_text
    assert "arm-status-card" in panel_text
    assert "web-teleop-card" in panel_text
    assert "keyboard-teleop-card" in panel_text
    assert "teach-trajectory-card" in panel_text
    assert "motor-state-card" in panel_text
    assert "KEYBOARD_TELEOP" in panel_text
    assert "teach-record-select" in panel_text
    assert "gripper-command-state" in panel_text
    assert "/api/keyboard_enable" in panel_text
    assert "/api/keyboard_disable" in panel_text
    assert "/api/keyboard_key" in panel_text
    assert "KEYBOARD_TELEOP" in panel_text
    assert "/api/teach_record_start" in panel_text
    assert "/api/teach_record_stop" in panel_text
    assert 'id="show-live-sliders"' not in panel_text
    assert 'id="live-slider-pane"' not in panel_text
    assert ">Set Gripper<" not in panel_text


def test_teach_recorder_exposes_service_controlled_start_stop():
    recorder_text = _read("src/rebotarm_teach/rebotarm_teach/teach_recorder_node.py")
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    teleop_launch_text = _read("src/rebotarm_bringup/launch/teleop_system.launch.py")
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    cmake_text = _read("src/rebotarm_msgs/CMakeLists.txt")

    assert 'self.declare_parameter("start_on_launch", True)' in recorder_text
    assert "self.create_service(" in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/start"' in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/stop"' in recorder_text
    assert 'f"/{self._arm_namespace}/teleop/teach_record/set_path"' in recorder_text
    assert "SetTeachRecordPath" in recorder_text
    assert "SetTeachRecordPath" in panel_text
    assert 'body: JSON.stringify({ record_path: recordName })' in panel_text
    assert 'return Path("teleop_records") / name' in recorder_text
    assert '"srv/SetTeachRecordPath.srv"' in cmake_text
    assert "def _handle_start_recording" in recorder_text
    assert "def _handle_stop_recording" in recorder_text
    assert "InternalTeachRecorder" not in controller_text
    assert not (ROOT / "src/rebotarmcontroller/rebotarmcontroller/teach_recorder.py").exists()
    app_launch = _read("src/rebotarm_bringup/launch/rebotarm_app.launch.py")
    assert 'package="rebotarm_teach"' in app_launch
    assert 'executable="TeachRecorderNode"' in app_launch
    assert '"start_on_launch": False' in teleop_launch_text
    assert '"require_motor_status": ParameterValue(use_hardware, value_type=bool)' in teleop_launch_text


def test_teach_replay_prepared_pipeline_defaults_to_150hz():
    workflow_text = _read("src/rebotarm_teach/rebotarm_teach/teach_replay_workflow.py")
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    teach_config_text = _read("src/rebotarm_bringup/config/teach_control.yaml")

    assert '"filter_sample_rate_hz"' in workflow_text
    assert '"resample_rate_hz"' in workflow_text
    assert 'self.declare_parameter("filter_sample_rate_hz", 150.0)' in panel_text
    assert 'self.declare_parameter("resample_rate_hz", 150.0)' in panel_text
    assert "filter_sample_rate_hz: 150.0" in teach_config_text
    assert "resample_rate_hz: 150.0" in teach_config_text
    assert "time_parameterization_method: auto" in teach_config_text
    assert 'self.declare_parameter("time_parameterization_method", "auto")' in panel_text


def test_moveit_ompl_uses_ruckig_response_adapter_with_jerk_limits():
    ompl_text = _read("src/rebotarm_moveit_config/config/ompl_planning.yaml")
    joint_limits_text = _read("src/rebotarm_moveit_config/config/joint_limits.yaml")
    assert "default_planning_response_adapters/AddTimeOptimalParameterization" in ompl_text
    assert "default_planning_response_adapters/AddRuckigTrajectorySmoothing" in ompl_text
    import yaml
    adapters = yaml.safe_load(ompl_text)["response_adapters"]
    assert adapters == [
        "default_planning_response_adapters/AddTimeOptimalParameterization",
        "default_planning_response_adapters/AddRuckigTrajectorySmoothing",
        "default_planning_response_adapters/ValidateSolution",
        "default_planning_response_adapters/DisplayMotionPath",
    ]
    assert "has_jerk_limits: true" in joint_limits_text
    assert "max_jerk: 20.0" in joint_limits_text


def test_teach_replay_executes_prepared_retimed_points_directly():
    builder_text = _read("src/rebotarm_teach/rebotarm_teach/teach_replay_trajectory_builder.py")

    assert "retimed_points" in builder_text
    assert "trajectory.points.append(point)" in builder_text


def test_teach_replay_has_runtime_tracking_guard_for_web():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    monitor_text = _read("src/rebotarm_motion/rebotarm_motion/replay_runtime_monitor.py")
    config_text = _read("src/rebotarm_bringup/config/teach_control.yaml")

    assert "evaluate_replay_tracking" in monitor_text
    assert "ReplayRuntimeMonitor" in panel_text
    assert "_replay_runtime_monitor.check(" in panel_text
    assert 'self.declare_parameter("replay_monitor_enabled", True)' in panel_text
    assert 'self.declare_parameter("max_tracking_error_rad", 0.25)' in panel_text
    assert 'self.declare_parameter("max_live_velocity_rad_s", 3.0)' in panel_text
    assert "def check_tracking" in panel_text
    assert "self._request_controller_trajectory_stop" in panel_text
    for text in (panel_text, monitor_text):
        assert "tracking_error" in text
        assert "live_velocity" in text

    assert "replay_monitor_enabled: true" in config_text
    assert "max_tracking_error_rad: 0.25" in config_text
    assert "max_live_velocity_rad_s: 3.0" in config_text


def test_status_panel_preserves_runtime_safety_stop_result_reason():
    lifecycle_text = _read("src/rebotarm_teach/rebotarm_teach/replay_action_lifecycle.py")
    result_body = lifecycle_text.split("def _on_teach_replay_result", 1)[1]

    assert "previous_replay = self._snapshot().teleop.get(\"replay\", {})" in result_body
    assert "self._replay_runtime_monitor.stop_requested" in result_body
    assert "state = \"safety_stop\"" in result_body
    assert "action canceled after runtime monitor stop" in result_body


def test_status_panel_surfaces_time_parameterization_summary():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")

    assert "time_parameterization" in panel_text
    assert "time_parameterization?.used_method" in panel_text
    assert "time_parameterization?.used_method" in panel_text


def test_teach_trajectory_curve_card_shows_prepared_curve_without_duplicate_check_metrics():
    panel_text = _read("src/rebotarm_dashboard/rebotarm_dashboard/teleop_status_panel_node.py")
    details_body = panel_text.split("const renderTeachTrajectoryDetails = (payload) => {", 1)[1].split(
        "const drawTeachTrajectoryChart = (payload) => {", 1
    )[0]
    backend_body = panel_text.split("def trajectory_preview(", 1)[1].split("\n    def records", 1)[0]

    assert "curve_source" in details_body
    assert "preview_samples = load_teach_samples(prepared_path)" in backend_body
    assert "raw_record_path" in details_body
    assert "Prepared Risk" not in details_body
    assert "Max Jump" not in details_body
    assert "Max Velocity" not in details_body
    assert 'payload["curve_source"] = "prepared"' in backend_body
    assert "preview_samples = load_teach_samples(prepared_path)" in backend_body


def test_moveit_demo_standalone_publishes_fake_visual_joint_state_source():
    demo_text = _read("src/rebotarm_bringup/launch/moveit_demo.launch.py")
    hardware_text = _read("src/rebotarm_bringup/launch/moveit_hardware.launch.py")
    interactive_text = _read("src/rebotarm_bringup/launch/interactive_system.launch.py")

    assert 'DeclareLaunchArgument("use_fake_joint_states", default_value="true")' in demo_text
    assert 'executable="joint_state_publisher"' in demo_text
    assert 'condition=IfCondition(use_fake_joint_states)' in demo_text
    assert '"/joint_states", ["/", arm_namespace, "/joint_states"]' in demo_text
    assert 'executable="GripperVisualJointStateNode"' in demo_text
    assert '"/joint_states", ["/", arm_namespace, "/visual_joint_states"]' in demo_text
    assert '"use_moveit_fake_joint_states": "false"' in hardware_text
    assert '"use_hardware": "true"' in hardware_text
    assert '"use_moveit_preview": "true"' in hardware_text
    assert '"use_fake_joint_states": PythonExpression' in interactive_text
    assert 'use_moveit_fake_joint_states' in interactive_text


def test_controller_shutdown_safe_home_is_explicit_opt_in_before_disable():
    controller_text = _read("src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py")
    hardware_text = _read("src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py")

    assert 'self.declare_parameter("shutdown_safe_home", False)' in controller_text
    assert "and self.hardware.enabled" in controller_text
    assert "self._conditional_safe_home_before_shutdown()" in controller_text
    assert "self.hardware.stop_active_motion()" in controller_text
    assert 'initial_state in ("LOWLEVEL_STREAMING", "GRAVITY_COMP")' in controller_text
    assert "self.hardware.gripper_active" in controller_text
    assert "self.hardware.gripper_mode" in controller_text
    assert "joint state size mismatch" in controller_text
    assert "joint positions are not finite" in controller_text
    assert "shutdown safe_home skipped" in controller_text
    assert "shutdown conditional safe_home complete" in controller_text
    assert "self.hardware.safe_home(self.safe_home_joint_positions())" in controller_text
    assert "self.hardware.shutdown()" in controller_text
    assert "def connected(self) -> bool:" in hardware_text
    assert "def gripper_active(self) -> bool:" in hardware_text
    assert "def gripper_mode(self) -> str:" in hardware_text
    assert "except Exception:" in hardware_text
    assert "self._disconnect_after_failed_connect()" in hardware_text
    assert 'self.get_logger().error(f"hardware connect failed; disabled before exit: {exc}")' in controller_text
    assert "node = None" in controller_text
    assert "if node is not None:" in controller_text
