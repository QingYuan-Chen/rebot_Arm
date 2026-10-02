"""就绪生命周期：成功才启动视觉链；失败保留底层状态，不自动失能。"""

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

from launch.actions import LogInfo

def _after_ready(event, context, actions):
    # Legacy contract: on_exit=post_visual_ready_actions is now guarded by returncode.
    if event.returncode == 0:
        return actions
    return [LogInfo(msg="visual_ready failed; visual pipeline remains closed. Backend stays alive; no automatic disable.")]

def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    visual_ready_params = PathJoinSubstitution([FindPackageShare("rebotarm_motion"), "config", "visual_ready.yaml"])
    arm_namespace = LaunchConfiguration("arm_namespace")
    start_visual_ready = LaunchConfiguration("start_visual_ready")
    move_to_visual_ready_on_start = LaunchConfiguration("move_to_visual_ready_on_start")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    visual_ready_startup_delay_sec = LaunchConfiguration("visual_ready_startup_delay_sec")
    run_visual_ready_startup = PythonExpression(
        [
            "'",
            start_visual_ready,
            "'.lower() == 'true' and '",
            move_to_visual_ready_on_start,
            "'.lower() == 'true'",
        ]
    )
    visual_ready_startup = Node(
        package="rebotarm_motion",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready_startup",
        output="screen",
        condition=IfCondition(run_visual_ready_startup),
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": move_to_visual_ready_on_start,
                "exit_after_startup_move": True,  # 启动实例摆位结束后立即退出进程，由进程退出事件驱动后续节点（成败只体现在日志）
                "startup_delay_sec": visual_ready_startup_delay_sec,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )
    def stage(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_share, "launch", "includes", name + ".launch.py"])))
    post_visual_ready_actions = [
        Node(
            package="rebotarm_motion",
            executable="rebotarm_visual_ready",
            name="rebotarm_visual_ready",
            output="screen",
            condition=IfCondition(start_visual_ready),
            parameters=[
                visual_ready_params,
                {
                    "arm_namespace": arm_namespace,
                    "auto_move_on_start": False,  # 常驻实例不自动运动，只响应 visual_ready/move 服务调用
                    "joint_positions": visual_ready_joint_positions,
                    "duration_sec": visual_ready_duration_sec,
                    "wait_timeout_sec": visual_ready_wait_timeout_sec,
                    "max_start_delta_rad": visual_ready_max_start_delta_rad,
                }
            ],
        ),
        stage("visual_input"),
        stage("grasp_candidate"),
        stage("candidate_filter"),
        stage("motion_execution"),
        stage("grasp_executor"),
        stage("grasp_preview"),
    ]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=visual_ready_startup, on_exit=lambda event, context: _after_ready(event, context, post_visual_ready_actions))),
        visual_ready_startup,
        GroupAction(condition=UnlessCondition(run_visual_ready_startup), actions=post_visual_ready_actions),
    ])
