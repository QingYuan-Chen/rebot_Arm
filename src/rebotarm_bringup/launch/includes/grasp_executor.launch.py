from rebotarm_bringup.visual_profiles import merged_parameters
"""内部阶段 grasp_executor：参数由总入口声明，不单独启动。"""

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

def _read_policy(context, profile, node_name):
    """Read one node's defaults; fail with the resource and node in the error."""
    path = profile.perform(context)
    try:
        with open(path, encoding="utf-8") as stream:
            values = yaml.safe_load(stream)[node_name]["ros__parameters"]
        if not isinstance(values, dict):
            raise TypeError("ros__parameters must be a mapping")
        return values
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(f"Cannot load policy {path} for {node_name}: {exc}") from exc

def _build(context):
    vision_share = FindPackageShare("rebotarm_vision")
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    gripper_policy_params = PathJoinSubstitution([vision_share, "config", "gripper_policy.yaml"])
    retry_policy_params = PathJoinSubstitution([vision_share, "config", "retry_policy.yaml"])
    retreat_policy_params = PathJoinSubstitution([vision_share, "config", "retreat_policy.yaml"])
    visual_servo_params = PathJoinSubstitution([vision_share, "config", "visual_servo.yaml"])
    table_safety_params = PathJoinSubstitution([vision_share, "config", "table_safety.yaml"])
    arm_namespace = LaunchConfiguration("arm_namespace")
    execution_mode = LaunchConfiguration("execution_mode")
    use_hardware = LaunchConfiguration("use_hardware")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    execute_gripper = LaunchConfiguration("execute_gripper")
    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    pose_policy = LaunchConfiguration("pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    safe_retreat_enabled = LaunchConfiguration("safe_retreat_enabled")
    safe_retreat_distance_m = LaunchConfiguration("safe_retreat_distance_m")
    safe_home_after_grasp = LaunchConfiguration("safe_home_after_grasp")
    move_velocity_scaling = LaunchConfiguration("move_velocity_scaling")
    approach_velocity_scaling = LaunchConfiguration("approach_velocity_scaling")
    retreat_velocity_scaling = LaunchConfiguration("retreat_velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    plan_only_stage_pause_sec = LaunchConfiguration("plan_only_stage_pause_sec")
    approach_visual_servo_enabled = LaunchConfiguration("approach_visual_servo_enabled")
    approach_visual_servo_max_iterations = LaunchConfiguration("approach_visual_servo_max_iterations")
    approach_visual_servo_max_step_m = LaunchConfiguration("approach_visual_servo_max_step_m")
    approach_visual_servo_position_tolerance_m = LaunchConfiguration("approach_visual_servo_position_tolerance_m")
    approach_visual_servo_require_fresh_plan = LaunchConfiguration("approach_visual_servo_require_fresh_plan")
    auto_retry_enabled = LaunchConfiguration("auto_retry_enabled")
    auto_retry_max_attempts = LaunchConfiguration("auto_retry_max_attempts")
    safe_retreat_before_retry = LaunchConfiguration("safe_retreat_before_retry")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_min_closure_distance_m = LaunchConfiguration("grasp_verification_min_closure_distance_m")
    grasp_verification_require_contact = LaunchConfiguration("grasp_verification_require_contact")
    place_after_grasp_enabled = LaunchConfiguration("place_after_grasp_enabled")
    place_position_xyz = LaunchConfiguration("place_position_xyz")
    place_orientation_xyzw = LaunchConfiguration("place_orientation_xyzw")
    place_open_position_m = LaunchConfiguration("place_open_position_m")
    place_open_max_effort = LaunchConfiguration("place_open_max_effort")
    place_retreat_z_m = LaunchConfiguration("place_retreat_z_m")
    trajectory_precheck_enabled = LaunchConfiguration("trajectory_precheck_enabled")
    max_plan_age_sec = LaunchConfiguration("max_plan_age_sec")
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    def _launch_executor(context):
        # Flatten named YAML entries before Node normalization so explicit overrides
        # have predictable precedence, including values equal to former defaults.
        policy_defaults = {}
        for profile in (
            grasp_pose_policy_params,
            gripper_policy_params,
            retry_policy_params,
            retreat_policy_params,
            visual_servo_params,
            table_safety_params,
        ):
            policy_defaults.update(_read_policy(context, profile, "rebotarm_visual_grasp_executor"))
        # Keep launch compatibility: only values different from YAML defaults
        # are added as overrides. Equal defaults remain owned by the profile.
        override_names = ('close_position_m', 'close_max_effort', 'open_before_approach', 'auto_gripper_width', 'auto_gripper_effort', 'open_clearance_m', 'close_margin_m', 'min_gripper_effort', 'max_gripper_effort', 'max_allowed_grasp_width_m', 'gripper_grasp_enabled', 'gripper_grasp_close_force', 'gripper_grasp_timeout_sec', 'gripper_grasp_min_close_time_sec', 'gripper_grasp_velocity_threshold', 'gripper_grasp_min_closure_distance_m',)
        for name in override_names:
            value = LaunchConfiguration(name).perform(context).strip().lower()
            default = str(policy_defaults[name]).strip().lower()
            if value != default:
                policy_defaults[name] = LaunchConfiguration(name)
        return [Node(
    package="rebotarm_vision",
    executable="rebotarm_visual_grasp_executor",
    name="rebotarm_visual_grasp_executor",
    output="screen",
    condition=IfCondition(start_visual_grasp_executor),
    parameters=[merged_parameters(context, 'rebotarm_visual_grasp_executor', [visual_interfaces_config], {**{**policy_defaults, **{}, 'arm_namespace': arm_namespace, 'use_hardware': use_hardware, 'input_topic': executor_input_topic, 'candidates_topic': filtered_candidates_topic, 'tcp_offset_xyz': tcp_offset_xyz, 'target_base_offset_xyz': target_base_offset_xyz, 'grasp_base_z_offset_m': grasp_base_z_offset_m, 'pose_policy': pose_policy, 'fixed_grasp_orientation_xyzw': fixed_grasp_orientation_xyzw, 'base_approach_axis_xyz': base_approach_axis_xyz, 'base_pregrasp_distance_m': base_pregrasp_distance_m, 'min_grasp_z_m': min_target_z_m, 'safe_retreat_enabled': safe_retreat_enabled, 'safe_retreat_distance_m': safe_retreat_distance_m, 'safe_home_after_grasp': safe_home_after_grasp, 'move_velocity_scaling': move_velocity_scaling, 'approach_velocity_scaling': approach_velocity_scaling, 'retreat_velocity_scaling': retreat_velocity_scaling, 'acceleration_scaling': acceleration_scaling, 'execute_gripper': execute_gripper, 'execution_mode': execution_mode, 'max_plan_age_sec': max_plan_age_sec, 'plan_only_stage_pause_sec': plan_only_stage_pause_sec, 'approach_visual_servo_enabled': approach_visual_servo_enabled, 'approach_visual_servo_max_iterations': approach_visual_servo_max_iterations, 'approach_visual_servo_max_step_m': approach_visual_servo_max_step_m, 'approach_visual_servo_position_tolerance_m': approach_visual_servo_position_tolerance_m, 'approach_visual_servo_require_fresh_plan': approach_visual_servo_require_fresh_plan, 'auto_retry_enabled': auto_retry_enabled, 'auto_retry_max_attempts': auto_retry_max_attempts, 'safe_retreat_before_retry': safe_retreat_before_retry, 'grasp_verification_enabled': grasp_verification_enabled, 'grasp_verification_min_closure_distance_m': grasp_verification_min_closure_distance_m, 'grasp_verification_require_contact': grasp_verification_require_contact, 'place_after_grasp_enabled': place_after_grasp_enabled, 'place_position_xyz': place_position_xyz, 'place_orientation_xyzw': place_orientation_xyzw, 'place_open_position_m': place_open_position_m, 'place_open_max_effort': place_open_max_effort, 'place_retreat_z_m': place_retreat_z_m, 'trajectory_precheck_enabled': trajectory_precheck_enabled}})],
    )]
    return [OpaqueFunction(function=_launch_executor)]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build)])
