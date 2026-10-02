"""内部阶段 candidate_filter：参数由总入口声明，不单独启动。"""

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

def generate_launch_description():
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    candidate_ik_input_topic = LaunchConfiguration("candidate_ik_input_topic")
    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    candidate_max_candidates_per_frame = LaunchConfiguration("candidate_max_candidates_per_frame")
    candidate_min_confidence = LaunchConfiguration("candidate_min_confidence")
    candidate_min_jaw_width_m = LaunchConfiguration("candidate_min_jaw_width_m")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    candidate_min_grasp_z_m = LaunchConfiguration("candidate_min_grasp_z_m")
    candidate_pregrasp_min_z_m = LaunchConfiguration("candidate_pregrasp_min_z_m")
    candidate_workspace_gate_enabled = LaunchConfiguration("candidate_workspace_gate_enabled")
    candidate_workspace_min_xyz = LaunchConfiguration("candidate_workspace_min_xyz")
    candidate_workspace_max_xyz = LaunchConfiguration("candidate_workspace_max_xyz")
    candidate_max_grasp_to_object_center_m = LaunchConfiguration("candidate_max_grasp_to_object_center_m")
    candidate_score_joint_distance_weight = LaunchConfiguration("candidate_score_joint_distance_weight")
    candidate_score_joint6_weight = LaunchConfiguration("candidate_score_joint6_weight")
    candidate_max_joint6_delta_rad = LaunchConfiguration("candidate_max_joint6_delta_rad")
    candidate_joint6_symmetry_enabled = LaunchConfiguration("candidate_joint6_symmetry_enabled")
    candidate_joint6_symmetry_angle_rad = LaunchConfiguration("candidate_joint6_symmetry_angle_rad")
    return LaunchDescription([
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_candidate_ik_filter",
            name="rebotarm_grasp_candidate_ik_filter",
            output="screen",
            condition=IfCondition(start_candidate_ik_filter),
            parameters=[
                visual_interfaces_config,
                # 该节点的完整候选过滤参数必须集中在同一个 launch 字典里。
                # 在 ROS 2 Jazzy 上，按节点名分组的 YAML 条目会覆盖 launch_ros 生成的
                # 通配字典（即使本字典写在后面），把参数拆到 YAML 里会让这些启动参数失效。
                # 保持单一完整档即可绕开该覆盖问题；这样候选过滤相关的闸门与评分才有唯一来源。
                {
                    "input_topic": candidate_ik_input_topic,
                    "output_topic": filtered_candidates_topic,
                    "output_plan_topic": filtered_plan_topic,
                    "joint_state_topic": candidate_joint_state_topic,
                    "service_timeout_sec": candidate_filter_service_timeout_sec,
                    "collision_check_enabled": candidate_collision_check_enabled,
                    "collision_check_service": candidate_collision_check_service,
                    "collision_group_name": candidate_collision_group_name,
                    "pose_policy": candidate_pose_policy,
                    "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                    "base_approach_axis_xyz": base_approach_axis_xyz,
                    "base_pregrasp_distance_m": base_pregrasp_distance_m,
                    "orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad,
                    "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                    "max_candidates_per_frame": candidate_max_candidates_per_frame,
                    "candidate_min_confidence": candidate_min_confidence,
                    "candidate_min_jaw_width_m": candidate_min_jaw_width_m,
                    "candidate_max_jaw_width_m": candidate_max_jaw_width_m,
                    "candidate_min_grasp_z_m": candidate_min_grasp_z_m,
                    "candidate_workspace_gate_enabled": candidate_workspace_gate_enabled,
                    "candidate_workspace_min_xyz": candidate_workspace_min_xyz,
                    "candidate_workspace_max_xyz": candidate_workspace_max_xyz,
                    "candidate_max_grasp_to_object_center_m": candidate_max_grasp_to_object_center_m,
                    "candidate_score_joint_distance_weight": candidate_score_joint_distance_weight,
                    "candidate_score_joint6_weight": candidate_score_joint6_weight,
                    "candidate_max_joint6_delta_rad": candidate_max_joint6_delta_rad,
                    "candidate_joint6_symmetry_enabled": candidate_joint6_symmetry_enabled,
                    "candidate_joint6_symmetry_angle_rad": candidate_joint6_symmetry_angle_rad,
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "candidate_pregrasp_min_z_m": candidate_pregrasp_min_z_m,
                    "grasp_base_z_offset_m": grasp_base_z_offset_m,
                }
            ],
        ),
    ])
