"""受控视觉执行入口。

执行仍要求调用方显式设置 ``use_hardware:=true`` 并通过控制器自身的 Enable/安全门；
默认值保持无硬件。此入口只暴露执行相关的少量组合参数，视觉和策略参数继续由各包
的 YAML/兼容入口提供。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    names = (
        "use_hardware",
        "use_local_rviz",
        "start_vision",
        "start_graspnet_baseline",
        "start_candidate_ik_filter",
        "start_motion_execution",
        "start_visual_grasp_executor",
        "start_visual_grasp_markers",
        "start_raw_candidate_markers",
        "start_open3d_viewer",
        "start_visual_ready",
        "move_to_visual_ready_on_start",
        "start_sim_trajectory_controller",
        "execute_gripper",
    )
    defaults = {
        "use_hardware": "false",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_candidate_ik_filter": "true",
        "start_motion_execution": "true",
        "start_visual_grasp_executor": "true",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "start_visual_ready": "true",
        "move_to_visual_ready_on_start": "false",
        "start_sim_trajectory_controller": "true",
        "execute_gripper": "true",
    }
    args = [DeclareLaunchArgument(name, default_value=defaults[name]) for name in names]
    args += [
        DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
        DeclareLaunchArgument("channel", default_value="auto"),
    ]
    args.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "visual_grasp_compact.launch.py"])
            ),
            launch_arguments=(
                {name: LaunchConfiguration(name) for name in names}
                | {
                    "arm_namespace": LaunchConfiguration("arm_namespace"),
                    "channel": LaunchConfiguration("channel"),
                    "execution_mode": "execute",
                }
            ).items(),
        )
    )
    return LaunchDescription(args)
