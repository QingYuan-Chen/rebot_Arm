"""视觉感知、候选筛选与显示组合入口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def _prepare_stages(context):
    """分域 YAML 是正式入口的唯一默认值来源。"""
    import os
    import json
    from rebotarm_bringup.visual_profiles import read_section

    retired = {
        "execution_mode", "use_hardware", "channel", "trajectory_execution_mode",
        "execute_gripper", "backend_profile", "motion_profile", "executor_input_topic",
        "start_motion_execution", "start_visual_grasp_executor", "start_sim_trajectory_controller",
        "start_grasp_preview", "pose_mode", "min_target_z_m", "pose_policy", "max_plan_age_sec",
        "moveit_planning_time", "moveit_num_planning_attempts", "move_velocity_scaling",
        "approach_velocity_scaling", "retreat_velocity_scaling", "acceleration_scaling",
        "close_position_m", "close_max_effort", "open_before_approach", "auto_gripper_width",
        "auto_gripper_effort", "open_clearance_m", "close_margin_m", "min_gripper_effort",
        "max_gripper_effort", "max_allowed_grasp_width_m", "gripper_grasp_enabled",
        "gripper_grasp_close_force", "gripper_grasp_timeout_sec", "gripper_grasp_min_close_time_sec",
        "gripper_grasp_velocity_threshold", "gripper_grasp_min_closure_distance_m",
        "safe_retreat_enabled", "safe_retreat_distance_m", "safe_home_after_grasp",
        "plan_only_stage_pause_sec", "approach_visual_servo_enabled", "approach_visual_servo_max_iterations",
        "approach_visual_servo_max_step_m", "approach_visual_servo_position_tolerance_m",
        "approach_visual_servo_require_fresh_plan", "auto_retry_enabled", "auto_retry_max_attempts",
        "safe_retreat_before_retry", "grasp_verification_enabled", "grasp_verification_min_closure_distance_m",
        "grasp_verification_require_contact", "place_after_grasp_enabled", "place_position_xyz",
        "place_orientation_xyzw", "place_open_position_m", "place_open_max_effort", "place_retreat_z_m",
        "trajectory_precheck_enabled", "grasp_hold_timeout_sec", "sim_initial_joint_positions",
        "shutdown_safe_home", "hardware_feedback_rate_hz", "gripper_position_torque_cap_nm",
        "gripper_position_max_speed_rad_s", "gripper_position_timeout_margin_sec",
        "gripper_feedback_stale_timeout_sec",
    }
    supplied = sorted(retired.intersection(context.launch_configurations))
    if supplied:
        raise RuntimeError(f"Retired visual execution arguments: {', '.join(supplied)}; "
                           "use robot_context:=offline or robot_context:=external")

    for name in ("strategy_profile", "visual_interfaces_config"):
        values = read_section(LaunchConfiguration(name).perform(context), "launch", None)
        obsolete = sorted(retired.intersection(values))
        if obsolete:
            raise RuntimeError(f"Retired visual execution parameters in {name}: "
                               f"{', '.join(obsolete)}; use robot_context:=offline "
                               "or robot_context:=external")
        for key, value in values.items():
            if isinstance(value, str):
                value = value.replace("{arm_namespace}", LaunchConfiguration("arm_namespace").perform(context).strip("/"))
            context.launch_configurations.setdefault(key, value if isinstance(value, str) else json.dumps(value))

    defaults = {
        "vision_python_executable": os.environ.get("REBOTARM_VISION_PYTHON", "python3"),
        "graspnet_python_executable": os.environ.get("GRASPNET_PYTHON", "python3"),
        "vision_yolo_model_path": PathJoinSubstitution([FindPackageShare("rebotarm_vision"), "models", "yolo26s-seg.pt"]).perform(context),
    }
    # GraspNet 资源只接受部署环境显式提供, 不沿工作区父目录猜测第三方位置;
    # 未设置时由启用后的推理节点在加载阶段明确失败.
    for name, env in (("graspnet_model_root", "GRASPNET_MODEL_ROOT"),
                      ("graspnet_checkpoint_path", "GRASPNET_CHECKPOINT_PATH")):
        defaults[name] = os.environ.get(env, "").strip()
    for key, value in defaults.items():
        context.launch_configurations.setdefault(key, value)
    context.launch_configurations["start_candidate_ik_filter"] = "true"
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "launch", "includes", name + ".launch.py"]
    ))) for name in ("visual_backend", "visual_lifecycle")]


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    names = {
        "arm_namespace": "rebotarm",
        "robot_context": "offline",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "use_sim_time": "false",
        "strategy_profile": PathJoinSubstitution([vision_share, "config", "visual_strategy_profile.yaml"]),
        "visual_interfaces_config": PathJoinSubstitution(
            [bringup_share, "config", "visual_grasp_interfaces.yaml"]),
        "vision_camera_config": PathJoinSubstitution(
            [vision_share, "config", "camera_ubuntu.yaml"]),
        "vision_handeye_config": "",
        "graspnet_config": PathJoinSubstitution(
            [vision_share, "config", "graspnet_ubuntu.yaml"]),
    }
    declarations = []
    for name, default in names.items():
        kwargs = {"default_value": default}
        if name == "robot_context":
            kwargs["choices"] = ["offline", "external"]
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_prepare_stages),
        ])
    )
    return LaunchDescription(declarations)
