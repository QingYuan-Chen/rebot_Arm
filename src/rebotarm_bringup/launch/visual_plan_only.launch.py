"""视觉规划入口：感知 → GraspNet → IK/碰撞过滤 → MoveIt 规划预览。

该入口固定 ``execution_mode=plan_only``、``execute_gripper=false``，并默认不连接真机。
它保留兼容入口中的参数档和节点实现，但把规划阶段的生命周期从只读感知和执行阶段中
分离出来，便于单独验证。
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
        "start_visual_grasp_markers",
        "start_raw_candidate_markers",
        "start_open3d_viewer",
        "start_visual_ready",
        "move_to_visual_ready_on_start",
        "start_sim_trajectory_controller",
    )
    defaults = {
        "use_hardware": "false",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_candidate_ik_filter": "true",
        "start_motion_execution": "true",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "start_visual_ready": "true",
        "move_to_visual_ready_on_start": "false",
        "start_sim_trajectory_controller": "true",
    }
    args = [DeclareLaunchArgument(name, default_value=defaults[name]) for name in names]
    args += [DeclareLaunchArgument("arm_namespace", default_value="rebotarm")]
    args += [DeclareLaunchArgument("channel", default_value="auto")]
    args.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "visual_grasp_compact.launch.py"])
            ),
            launch_arguments=(
                {
                    name: LaunchConfiguration(name) for name in names
                }
                | {
                    "arm_namespace": LaunchConfiguration("arm_namespace"),
                    "channel": LaunchConfiguration("channel"),
                    "execution_mode": "plan_only",
                    "execute_gripper": "false",
                    "start_visual_grasp_executor": "false",
                    "place_after_grasp_enabled": "false",
                    "auto_retry_enabled": "false",
                }
            ).items(),
        )
    )
    return LaunchDescription(args)
