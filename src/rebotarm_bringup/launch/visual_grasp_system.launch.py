"""视觉抓取兼容总入口：声明旧参数，只组合后端与就绪生命周期。

内部节点阶段位于 includes/。保留旧参数兼容调用，不新增节点实现。
"""

import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def _workspace_path(environment_name: str, relative_path: str) -> str:
    configured = os.environ.get(environment_name, "").strip()
    if configured:
        return configured
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return str(candidate)
    return ""

def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    graspnet_ubuntu_params = PathJoinSubstitution([vision_share, "config", "graspnet_ubuntu.yaml"])
    execution_mode = LaunchConfiguration("execution_mode")
    return LaunchDescription([
        DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
        DeclareLaunchArgument("channel", default_value="auto"),
        DeclareLaunchArgument("use_hardware", default_value="false"),
        DeclareLaunchArgument(
            "gripper_position_torque_cap_nm", default_value="1.0"
        ),
        DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
        DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
        DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
        DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
        DeclareLaunchArgument("use_local_rviz", default_value="true"),
        DeclareLaunchArgument("execution_mode", default_value="plan_only"),
        DeclareLaunchArgument("start_vision", default_value="true"),
        DeclareLaunchArgument(
            "vision_profile",
            default_value="ubuntu_native",
            choices=["ubuntu_native"],
            description="Ubuntu native Gemini 2 vision input",
        ),
        DeclareLaunchArgument(
            "vision_camera_config",
            default_value=PathJoinSubstitution([vision_share, "config", "camera_ubuntu.yaml"]),
        ),
        DeclareLaunchArgument(
            "vision_handeye_config",
            default_value=PathJoinSubstitution([vision_share, "config", "handeye.yaml"]),
        ),
        DeclareLaunchArgument(
            "vision_yolo_model_path",
            default_value=PathJoinSubstitution(
                [
                    vision_share,
                    "models",
                    "yolo26s-seg.pt",
                ]
            ),
        ),
        DeclareLaunchArgument("start_graspnet_baseline", default_value="true"),
        DeclareLaunchArgument("graspnet_candidates_topic", default_value="/grasp/graspnet_candidates"),
        DeclareLaunchArgument("graspnet_output_frame_id", default_value="camera_depth_frame"),
        DeclareLaunchArgument("graspnet_config", default_value=graspnet_ubuntu_params),
        DeclareLaunchArgument(
            "visual_interfaces_config",
            default_value=PathJoinSubstitution(
                [bringup_share, "config", "visual_grasp_interfaces.yaml"]
            ),
        ),
        DeclareLaunchArgument("graspnet_max_input_skew_ms", default_value="100"),
        DeclareLaunchArgument(
            "graspnet_python_executable",
            default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
        ),
        DeclareLaunchArgument(
            "vision_python_executable",
            default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
        ),
        DeclareLaunchArgument(
            "graspnet_model_root",
            default_value=_workspace_path(
                "GRASPNET_MODEL_ROOT", "third_party/graspnet-baseline"
            ),
        ),
        DeclareLaunchArgument(
            "graspnet_checkpoint_path",
            default_value=_workspace_path(
                "GRASPNET_CHECKPOINT_PATH",
                ".local-models/checkpoints/checkpoint-rs.tar",
            ),
        ),
        DeclareLaunchArgument("graspnet_device", default_value="cuda:0"),
        DeclareLaunchArgument("graspnet_backend_module", default_value="graspnet_baseline_inference"),
        DeclareLaunchArgument("graspnet_max_grasps", default_value="10"),
        DeclareLaunchArgument("graspnet_max_points", default_value="20000"),
        DeclareLaunchArgument("start_grasp_preview", default_value="false"),
        DeclareLaunchArgument("start_candidate_ik_filter", default_value="true"),
        DeclareLaunchArgument("candidate_ik_input_topic", default_value="/grasp/graspnet_candidates"),
        DeclareLaunchArgument("start_visual_ready", default_value="true"),
        DeclareLaunchArgument("move_to_visual_ready_on_start", default_value="false"),
        DeclareLaunchArgument(
            "visual_ready_joint_positions",
            default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]",
        ),
        DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),
        DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),
        DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5"),
        DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0"),
        DeclareLaunchArgument("start_visual_grasp_executor", default_value="true"),
        DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
        DeclareLaunchArgument("start_raw_candidate_markers", default_value="true"),
        DeclareLaunchArgument("start_open3d_viewer", default_value="true"),
        DeclareLaunchArgument("start_motion_execution", default_value="true"),
        DeclareLaunchArgument("execute_gripper", default_value="true"),
        DeclareLaunchArgument(
            "start_sim_trajectory_controller",
            default_value="true",
            description="Start the RViz-only kinematic action server when no external simulation backend owns the namespace",
        ),
        DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]"),
        DeclareLaunchArgument("show_tcp_markers", default_value="true"),
        DeclareLaunchArgument("show_approach_arrow", default_value="true"),
        DeclareLaunchArgument("show_gripper_open_axis", default_value="true"),
        DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),
        DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),
        DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan"),
        DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),
        DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("candidate_collision_check_enabled", default_value="true"),
        DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),
        DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),
        DeclareLaunchArgument("pose_mode", default_value="pregrasp"),
        DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),
        DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("min_target_z_m", default_value="0.0"),
        DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),
        DeclareLaunchArgument("pose_policy", default_value="base_axis"),
        DeclareLaunchArgument(
            "fixed_grasp_orientation_xyzw",
            default_value="[0.0, 0.0, 0.0, 1.0]",
        ),
        DeclareLaunchArgument("base_approach_axis_xyz", default_value="[1.0, 0.0, 0.0]"),
        DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.06"),
        DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose"),
        DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]"),
        DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]"),
        DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20"),
        DeclareLaunchArgument("candidate_min_confidence", default_value="0.4"),
        DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006"),
        DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.085"),
        DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0"),
        DeclareLaunchArgument("candidate_pregrasp_min_z_m", default_value="0.04"),
        DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true"),
        DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[0.18, -0.35, 0.0]"),
        DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.64, 0.35, 0.45]"),
        DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15"),
        DeclareLaunchArgument("candidate_score_joint_distance_weight", default_value="0.15"),
        DeclareLaunchArgument("candidate_score_joint6_weight", default_value="0.35"),
        DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="1.5708"),
        DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true"),
        DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793"),
        DeclareLaunchArgument("close_position_m", default_value="0.025"),
        DeclareLaunchArgument("close_max_effort", default_value="0.4"),
        DeclareLaunchArgument("open_before_approach", default_value="true"),
        DeclareLaunchArgument("auto_gripper_width", default_value="true"),
        DeclareLaunchArgument("auto_gripper_effort", default_value="true"),
        DeclareLaunchArgument("open_clearance_m", default_value="0.0"),
        DeclareLaunchArgument("close_margin_m", default_value="0.012"),
        DeclareLaunchArgument("min_gripper_effort", default_value="0.22"),
        DeclareLaunchArgument("max_gripper_effort", default_value="0.60"),
        DeclareLaunchArgument("max_allowed_grasp_width_m", default_value="0.085"),
        DeclareLaunchArgument("gripper_grasp_enabled", default_value="true"),
        DeclareLaunchArgument("gripper_grasp_close_force", default_value="0.4"),
        DeclareLaunchArgument("gripper_grasp_timeout_sec", default_value="8.0"),
        DeclareLaunchArgument("gripper_grasp_min_close_time_sec", default_value="0.08"),
        DeclareLaunchArgument("gripper_grasp_velocity_threshold", default_value="0.04"),
        DeclareLaunchArgument("gripper_grasp_min_closure_distance_m", default_value="0.006"),
        DeclareLaunchArgument("safe_retreat_enabled", default_value="true"),
        DeclareLaunchArgument("safe_retreat_distance_m", default_value="0.06"),
        DeclareLaunchArgument("safe_home_after_grasp", default_value="false"),
        DeclareLaunchArgument("moveit_planning_time", default_value="8.0"),
        DeclareLaunchArgument("moveit_num_planning_attempts", default_value="5"),
        DeclareLaunchArgument("move_velocity_scaling", default_value="0.25"),
        DeclareLaunchArgument("approach_velocity_scaling", default_value="0.08"),
        DeclareLaunchArgument("retreat_velocity_scaling", default_value="0.15"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.12"),
        DeclareLaunchArgument("plan_only_stage_pause_sec", default_value="0.0"),
        DeclareLaunchArgument("approach_visual_servo_enabled", default_value="false"),
        DeclareLaunchArgument("approach_visual_servo_max_iterations", default_value="5"),
        DeclareLaunchArgument("approach_visual_servo_max_step_m", default_value="0.02"),
        DeclareLaunchArgument("approach_visual_servo_position_tolerance_m", default_value="0.008"),
        DeclareLaunchArgument("approach_visual_servo_require_fresh_plan", default_value="true"),
        DeclareLaunchArgument("auto_retry_enabled", default_value="false"),
        DeclareLaunchArgument("auto_retry_max_attempts", default_value="3"),
        DeclareLaunchArgument("safe_retreat_before_retry", default_value="true"),
        DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),
        DeclareLaunchArgument("grasp_verification_min_closure_distance_m", default_value="0.006"),
        DeclareLaunchArgument("grasp_verification_require_contact", default_value="true"),
        DeclareLaunchArgument("place_after_grasp_enabled", default_value="false"),
        DeclareLaunchArgument("place_position_xyz", default_value="[-0.20, -0.20, 0.25]"),
        DeclareLaunchArgument(
            "place_orientation_xyzw",
            default_value="[0.0, 0.0, 0.0, 1.0]",
        ),
        DeclareLaunchArgument("place_open_position_m", default_value="0.08"),
        DeclareLaunchArgument("place_open_max_effort", default_value="0.25"),
        DeclareLaunchArgument("place_retreat_z_m", default_value="0.06"),
        DeclareLaunchArgument("trajectory_precheck_enabled", default_value="true"),
        DeclareLaunchArgument(
            "max_plan_age_sec",
            default_value=PythonExpression(
                [
                    "'10.0' if '",
                    execution_mode,
                    "'.lower() == 'plan_only' else '4.0'",
                ]
            ),
            # 摄像头采帧→GraspNet→Top-10 IK/碰撞过滤实测可耗 3~5 s；
            # plan_only 再留出人工观察 RViz/触发服务的时间。只限不下发运动的预览。
            # execute 使用 4 s 默认门限，覆盖当前 RGB-D→GraspNet→IK/碰撞过滤实测延迟；
            # 更短或更长的窗口仍可由调用方显式覆盖。
            description="Maximum grasp-plan age in seconds (default: 10.0 for plan_only, 4.0 for execute)",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "includes", "visual_backend.launch.py"]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "includes", "visual_lifecycle.launch.py"]))),
    ])
