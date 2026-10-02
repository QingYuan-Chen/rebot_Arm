"""内部阶段 visual_input：参数由总入口声明，不单独启动。"""

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
    vision_share = FindPackageShare("rebotarm_vision")
    start_vision = LaunchConfiguration("start_vision")
    vision_python_executable = LaunchConfiguration("vision_python_executable")
    vision_profile = LaunchConfiguration("vision_profile")
    vision_camera_config = LaunchConfiguration("vision_camera_config")
    vision_handeye_config = LaunchConfiguration("vision_handeye_config")
    vision_yolo_model_path = LaunchConfiguration("vision_yolo_model_path")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        start_vision,
                        "' == 'true' and '",
                        vision_profile,
                        "' == 'ubuntu_native'",
                    ]
                )
            ),
            launch_arguments={
                "camera_config": vision_camera_config,
                "handeye_config": vision_handeye_config,
                "yolo_model_path": vision_yolo_model_path,
                "vision_python_executable": vision_python_executable,
                "yolo_device": "0",
            }.items(),
        ),
    ])
