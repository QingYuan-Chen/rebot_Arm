from rebotarm_bringup.visual_profiles import merged_parameters
"""内部阶段 grasp_preview：参数由总入口声明，不单独启动。"""

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

def _build(context):
    vision_share = FindPackageShare("rebotarm_vision")
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    pose_mode = LaunchConfiguration("pose_mode")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    return [
        Node(
            package="rebotarm_vision",
            executable="rebotarm_send_grasp_preview",
            name="rebotarm_grasp_preview_sender",
            output="screen",
            condition=IfCondition(start_grasp_preview),
            parameters=[merged_parameters(context, 'rebotarm_grasp_preview_sender', [grasp_pose_policy_params, visual_interfaces_config], {**{'input_topic': executor_input_topic, 'output_topic': ['/', arm_namespace, '/interactive_control/pose_target'], 'pose_mode': pose_mode, 'tcp_offset_xyz': tcp_offset_xyz, 'target_base_offset_xyz': target_base_offset_xyz, 'min_target_z_m': min_target_z_m, 'publish_count': 5, 'exit_after_publish': False}})],
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_visual_grasp_markers",
            name="rebotarm_visual_grasp_markers",
            output="screen",
            condition=IfCondition(start_visual_grasp_markers),
            parameters=[merged_parameters(context, 'rebotarm_visual_grasp_markers', [grasp_pose_policy_params, visual_interfaces_config], {**{'input_topic': executor_input_topic, 'object_min_diameter_m': 0.06, 'object_min_height_m': 0.12, 'upright_object_marker': True, 'show_object_marker': False, 'show_object_center_marker': False, 'show_object_label': False, 'tcp_offset_xyz': tcp_offset_xyz, 'gripper_open_axis_local_xyz': gripper_open_axis_local_xyz, 'show_tcp_markers': show_tcp_markers, 'show_approach_arrow': show_approach_arrow, 'show_gripper_open_axis': show_gripper_open_axis}})],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_build)])
