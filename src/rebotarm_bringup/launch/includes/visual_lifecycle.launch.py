"""视觉处理阶段组合；观察位服务由独立操作员入口启动。"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("rebotarm_bringup"), "launch", "includes", name + ".launch.py"
        ])))
        for name in ("visual_input", "grasp_candidate", "candidate_filter",
                     "motion_execution", "grasp_executor", "grasp_preview")
    ])
