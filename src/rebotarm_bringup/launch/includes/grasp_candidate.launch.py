"""内部阶段 grasp_candidate：参数由总入口声明，不单独启动。"""

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
    start_graspnet_baseline = LaunchConfiguration("start_graspnet_baseline")
    graspnet_candidates_topic = LaunchConfiguration("graspnet_candidates_topic")
    graspnet_output_frame_id = LaunchConfiguration("graspnet_output_frame_id")
    graspnet_config = LaunchConfiguration("graspnet_config")
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    graspnet_max_input_skew_ms = LaunchConfiguration("graspnet_max_input_skew_ms")
    graspnet_python_executable = LaunchConfiguration("graspnet_python_executable")
    graspnet_model_root = LaunchConfiguration("graspnet_model_root")
    graspnet_checkpoint_path = LaunchConfiguration("graspnet_checkpoint_path")
    graspnet_device = LaunchConfiguration("graspnet_device")
    graspnet_backend_module = LaunchConfiguration("graspnet_backend_module")
    graspnet_max_grasps = LaunchConfiguration("graspnet_max_grasps")
    graspnet_max_points = LaunchConfiguration("graspnet_max_points")
    start_raw_candidate_markers = LaunchConfiguration("start_raw_candidate_markers")
    start_open3d_viewer = LaunchConfiguration("start_open3d_viewer")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    return LaunchDescription([
        Node(
            package="rebotarm_vision",
            executable="rebotarm_graspnet_baseline_node",
            name="rebotarm_graspnet_baseline_node",
            output="screen",
            prefix=graspnet_python_executable,
            condition=IfCondition(start_graspnet_baseline),
            parameters=[
                graspnet_config,
                visual_interfaces_config,
                {
                    "output_candidates_topic": graspnet_candidates_topic,
                    "output_frame_id": graspnet_output_frame_id,
                    "max_input_skew_ms": graspnet_max_input_skew_ms,
                    "model_root": graspnet_model_root,
                    "checkpoint_path": graspnet_checkpoint_path,
                    "device": graspnet_device,
                    "backend_module": graspnet_backend_module,
                    "max_grasps": graspnet_max_grasps,
                    "max_jaw_width_m": candidate_max_jaw_width_m,
                    "max_points": graspnet_max_points,
                }
            ],
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_candidate_markers",
            name="rebotarm_grasp_candidate_markers",
            output="screen",
            condition=IfCondition(start_raw_candidate_markers),
            parameters=[
                {
                    "input_topic": graspnet_candidates_topic,
                    "max_candidates": 5,
                },
                visual_interfaces_config,
            ],
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_graspnet_open3d_viewer",
            name="rebotarm_graspnet_open3d_viewer",
            output="screen",
            prefix=graspnet_python_executable,
            condition=IfCondition(start_open3d_viewer),
            parameters=[
                {
                    "input_candidates_topic": graspnet_candidates_topic,
                },
                visual_interfaces_config,
            ],
        ),
    ])
