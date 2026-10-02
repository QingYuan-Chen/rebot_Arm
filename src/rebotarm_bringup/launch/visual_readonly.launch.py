"""只读入口：复用统一 25 参数公共接口，但不加载 backend、MoveIt、就绪移动或执行节点。

参数面保持统一，便于脚本在 system/readonly/plan_only 间切换；只读语义由固定的
``execution_mode`` 和阶段开关保证，而不是复制一套隐藏默认值。
"""

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
