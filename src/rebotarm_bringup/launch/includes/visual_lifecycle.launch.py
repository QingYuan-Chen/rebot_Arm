"""依次组合视觉输入,抓取候选生成,筛选和标记显示."""
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
        for name in ("visual_input", "grasp_candidate", "candidate_filter", "candidate_markers")
    ])
