"""只读入口：复用感知阶段，不加载 backend、MoveIt、就绪移动或执行节点。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rebotarm_bringup"), "launch", "visual_grasp_system.launch.py"])),
        launch_arguments={'execution_mode': 'readonly', 'use_hardware': 'false', 'start_visual_ready': 'false', 'move_to_visual_ready_on_start': 'false', 'start_motion_execution': 'false', 'start_visual_grasp_executor': 'false', 'execute_gripper': 'false'}.items(),
    )])
