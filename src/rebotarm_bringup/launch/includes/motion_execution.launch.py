"""内部阶段 motion_execution：参数由总入口声明，不单独启动。"""

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
    arm_namespace = LaunchConfiguration("arm_namespace")
    execution_mode = LaunchConfiguration("execution_mode")
    start_motion_execution = LaunchConfiguration("start_motion_execution")
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    moveit_planning_time = LaunchConfiguration("moveit_planning_time")
    moveit_num_planning_attempts = LaunchConfiguration("moveit_num_planning_attempts")
    move_velocity_scaling = LaunchConfiguration("move_velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    return LaunchDescription([
        Node(
            package="rebotarm_motion",
            executable="PoseExecutionNode",
            name="motion_execution",
            output="screen",
            condition=IfCondition(start_motion_execution),
            parameters=[
                visual_interfaces_config,
                {
                    "arm_namespace": arm_namespace,
                    "moveit_planning_time": moveit_planning_time,
                    "moveit_num_planning_attempts": moveit_num_planning_attempts,
                    # 只在纯规划模式发布 RViz 幻影；真机执行/预检不发布虚拟轨迹。
                    "publish_plan_only_preview": PythonExpression(
                        ["'", execution_mode, "'.lower() == 'plan_only'"]
                    ),
                    "default_velocity_scaling": move_velocity_scaling,
                    "default_acceleration_scaling": acceleration_scaling,
                }
            ],
        ),
    ])
