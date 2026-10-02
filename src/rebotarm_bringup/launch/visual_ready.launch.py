"""视觉就绪位姿的独立入口。

本入口只负责启动 ``rebotarm_visual_ready``，不包含相机、MoveIt、GraspNet 或抓取
执行器。需要把就绪位姿作为视觉链路的前置门控时，由上层组合入口显式编排；失败不得
被普通进程退出事件当成成功。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_share = FindPackageShare("rebotarm_motion")
    params = PathJoinSubstitution([motion_share, "config", "visual_ready.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("auto_move_on_start", default_value="false"),
            DeclareLaunchArgument("exit_after_startup_move", default_value="false"),
            DeclareLaunchArgument(
                "joint_positions", default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]"
            ),
            DeclareLaunchArgument("duration_sec", default_value="4.0"),
            DeclareLaunchArgument("wait_timeout_sec", default_value="12.0"),
            DeclareLaunchArgument("max_start_delta_rad", default_value="2.5"),
            DeclareLaunchArgument("startup_delay_sec", default_value="0.0"),
            Node(
                package="rebotarm_motion",
                executable="rebotarm_visual_ready",
                name="rebotarm_visual_ready",
                output="screen",
                parameters=[
                    params,
                    {
                        "arm_namespace": LaunchConfiguration("arm_namespace"),
                        "auto_move_on_start": LaunchConfiguration("auto_move_on_start"),
                        "exit_after_startup_move": LaunchConfiguration(
                            "exit_after_startup_move"
                        ),
                        "joint_positions": LaunchConfiguration("joint_positions"),
                        "duration_sec": LaunchConfiguration("duration_sec"),
                        "wait_timeout_sec": LaunchConfiguration("wait_timeout_sec"),
                        "max_start_delta_rad": LaunchConfiguration("max_start_delta_rad"),
                        "startup_delay_sec": LaunchConfiguration("startup_delay_sec"),
                    },
                ],
            ),
        ]
    )
