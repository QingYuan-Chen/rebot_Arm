# 真机 MoveIt 启动入口：interactive_system.launch.py 的真机专用包装。
#
# 本文件不复制硬件控制器、状态发布器或 MoveIt 的节点定义。
# interactive_system.launch.py 负责共享实现，本文件只固定选择真机、MoveIt、
# 真实关节状态和唯一状态源。启动后仍保持失能，执行前须显式调用
# /rebotarm/enable。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """固定使用真机后端的 MoveIt 包装入口。"""
    bringup_share = FindPackageShare("rebotarm_bringup")
    interactive_launch = PathJoinSubstitution(
        [bringup_share, "launch", "interactive_system.launch.py"]
    )
    interactive_rviz = PathJoinSubstitution(
        [bringup_share, "rviz", "interactive_system.rviz"]
    )

    forwarded = {
        "arm_config": LaunchConfiguration("arm_config"),
        "gripper_config": LaunchConfiguration("gripper_config"),
        "arm_namespace": LaunchConfiguration("arm_namespace"),
        "channel": LaunchConfiguration("channel"),
        "shutdown_safe_home": LaunchConfiguration("shutdown_safe_home"),
        "joint_state_rate": LaunchConfiguration("joint_state_rate"),
        "hardware_feedback_rate_hz": LaunchConfiguration("hardware_feedback_rate_hz"),
        "gripper_position_torque_cap_nm": LaunchConfiguration(
            "gripper_position_torque_cap_nm"
        ),
        "gripper_position_max_speed_rad_s": LaunchConfiguration(
            "gripper_position_max_speed_rad_s"
        ),
        "gripper_position_timeout_margin_sec": LaunchConfiguration(
            "gripper_position_timeout_margin_sec"
        ),
        "gripper_feedback_stale_timeout_sec": LaunchConfiguration(
            "gripper_feedback_stale_timeout_sec"
        ),
        "grasp_hold_timeout_sec": LaunchConfiguration("grasp_hold_timeout_sec"),
        "cmd_arbitration": LaunchConfiguration("cmd_arbitration"),
        "frame_id": LaunchConfiguration("frame_id"),
        "ee_frame_id": LaunchConfiguration("ee_frame_id"),
        "use_hardware": "true",
        "use_moveit_preview": "true",
        "use_local_rviz": LaunchConfiguration("use_rviz"),
        "start_passive_joint_state_publisher": "false",
        "use_moveit_fake_joint_states": "false",
        "rviz_config": interactive_rviz,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "arm.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "gripper.yaml"]
                ),
            ),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="10.0"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(interactive_launch),
                launch_arguments=forwarded.items(),
            ),
        ]
    )
