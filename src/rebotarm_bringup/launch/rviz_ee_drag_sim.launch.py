"""RViz 末端拖动入口:复用唯一 MuJoCo continuous 后端与完整 MoveIt 场景."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rebotarm_bringup"), "launch", "mujoco_moveit_sim.launch.py"])),
            launch_arguments={"use_rviz": "true", "use_mujoco_viewer": "false"}.items()),
    ])
