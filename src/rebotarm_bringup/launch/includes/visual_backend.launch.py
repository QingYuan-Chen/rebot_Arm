"""视觉筛选使用的离线模型环境及可选 RViz 显示。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("rebotarm_bringup")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([bringup, "launch", "moveit_demo.launch.py"])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("robot_context"), "' == 'offline'",
            ])),
            launch_arguments={
                "arm_namespace": LaunchConfiguration("arm_namespace"),
                "use_rviz": "false",
                "use_fake_joint_states": "true",
                "allow_trajectory_execution": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", name="visual_grasp_rviz", output="screen",
            condition=IfCondition(LaunchConfiguration("use_local_rviz")),
            arguments=["-d", PathJoinSubstitution([bringup, "rviz", "visual_grasp.rviz"])],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
