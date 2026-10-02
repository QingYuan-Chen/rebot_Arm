"""内部阶段 visual_backend：参数由总入口声明，不单独启动。"""

import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    use_hardware = LaunchConfiguration("use_hardware")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    grasp_hold_timeout_sec = LaunchConfiguration("grasp_hold_timeout_sec")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    start_sim_trajectory_controller = LaunchConfiguration("start_sim_trajectory_controller")
    return LaunchDescription([
                                    Node(
            package="rebotarm_simulation",
            executable="rebotarm_sim_trajectory_controller",
            name="rebotarm_sim_trajectory_controller",
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        use_hardware,
                        "'.lower() != 'true' and '",
                        start_sim_trajectory_controller,
                        "'.lower() == 'true'",
                    ]
                )
            ),
            parameters=[
                {
                    "arm_namespace": arm_namespace,
                    "initial_joint_positions": visual_ready_joint_positions,
                }
            ],
        ),
                             IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
            ),
            launch_arguments={
                "arm_namespace": arm_namespace,
                "use_moveit_preview": "true",
                "use_hardware": use_hardware,
                "channel": channel,
                "shutdown_safe_home": shutdown_safe_home,
                "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
                "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                "grasp_hold_timeout_sec": grasp_hold_timeout_sec,
                "use_local_rviz": use_local_rviz,
                "start_passive_joint_state_publisher": "false",
                "use_moveit_fake_joint_states": "false",
                "rviz_config": PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"]),
            }.items(),
        ),
    ])
