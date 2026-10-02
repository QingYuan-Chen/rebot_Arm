"""日常使用的精简视觉抓取入口。

这里只暴露组合层参数；视觉、候选、运动和夹爪策略继续从各自 profile 读取。旧的
``visual_grasp_system.launch.py`` 保留完整参数，仅用于兼容历史脚本。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    names = {
        "arm_namespace": "rebotarm",
        "channel": "auto",
        "use_hardware": "false",
        "execution_mode": "plan_only",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_candidate_ik_filter": "true",
        "start_motion_execution": "true",
        "start_visual_grasp_executor": "false",
        "execute_gripper": "false",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "start_visual_ready": "true",
        "move_to_visual_ready_on_start": "false",
        "start_sim_trajectory_controller": "true",
    }
    declarations = []
    for name, default in names.items():
        kwargs = {"default_value": default}
        if name == "execution_mode":
            kwargs["choices"] = ["plan_only", "execute"]
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "visual_grasp_system.launch.py"])
            ),
            launch_arguments={
                name: LaunchConfiguration(name) for name in names
            }.items(),
        )
    )
    return LaunchDescription(declarations)
