"""内部阶段 grasp_preview：参数由总入口声明，不单独启动。"""

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
    vision_share = FindPackageShare("rebotarm_vision")
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    visual_interfaces_config = LaunchConfiguration("visual_interfaces_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    start_grasp_preview = LaunchConfiguration("start_grasp_preview")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    pose_mode = LaunchConfiguration("pose_mode")
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    min_target_z_m = LaunchConfiguration("min_target_z_m")
    return LaunchDescription([
        Node(
            package="rebotarm_vision",
            executable="rebotarm_send_grasp_preview",
            name="rebotarm_grasp_preview_sender",
            output="screen",
            condition=IfCondition(start_grasp_preview),
            parameters=[
                grasp_pose_policy_params,
                visual_interfaces_config,
                {
                    "input_topic": executor_input_topic,
                    "output_topic": ["/", arm_namespace, "/interactive_control/pose_target"],
                    "pose_mode": pose_mode,
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "target_base_offset_xyz": target_base_offset_xyz,
                    "min_target_z_m": min_target_z_m,
                    "publish_count": 5,  # 重复发布 5 次，抵消话题发现延迟
                    "exit_after_publish": False,  # 发布后不退出，保持常驻以便反复预览
                }
            ],
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_visual_grasp_markers",
            name="rebotarm_visual_grasp_markers",
            output="screen",
            condition=IfCondition(start_visual_grasp_markers),
            parameters=[
                grasp_pose_policy_params,
                visual_interfaces_config,
                {
                    "input_topic": executor_input_topic,
                    "object_min_diameter_m": 0.06,  # 物体标记最小直径 6 cm，仅影响 RViz 可见性
                    "object_min_height_m": 0.12,  # 物体标记最小高度 12 cm，仅影响 RViz 可见性
                    "upright_object_marker": True,  # 物体框画成竖直方向，便于观察物体位置
                    # 候选消息只有类别、置信度和抓取尺度，并不包含真实物体轮廓。
                    # 关闭示意圆柱、中心绿点和文字，避免把它们误认为视觉测得的瓶子外形；
                    # TCP、接近方向和夹爪开合轴仍保留，用于核对实际抓取计划。
                    "show_object_marker": False,
                    "show_object_center_marker": False,
                    "show_object_label": False,
                    "tcp_offset_xyz": tcp_offset_xyz,
                    "gripper_open_axis_local_xyz": gripper_open_axis_local_xyz,
                    "show_tcp_markers": show_tcp_markers,
                    "show_approach_arrow": show_approach_arrow,
                    "show_gripper_open_axis": show_gripper_open_axis,
                }
            ],
        ),
    ])
