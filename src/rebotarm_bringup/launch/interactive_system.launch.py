# 真机交互式 MoveIt 组合:唯一硬件控制器 + MoveIt 场景/准备服务 + 可选 RViz.
#
# 职责与边界
#   本文件只组合真机后端和共享 MoveIt 实现, 不启动仿真或假状态源. 硬件控制器由
#   hardware_controller.launch.py 唯一拥有, MoveIt/状态发布/夹爪可视化桥由
#   moveit_demo.launch.py 统一提供, 本文件不复制任何节点定义.
#   无硬件规划统一使用 mujoco_moveit_sim.launch.py, 不在本文件保留无硬件分支.
#
# 对外接口
#   硬件与抓爪安全参数原样转发给硬件片段; use_local_rviz 控制本文件是否启动
#   interactive_system.rviz. 启动后仍保持失能, 运动前须显式调用 /rebotarm/enable.
#
# 安全约束
#   默认不启动 RViz 以外的额外状态源; 同一命名空间只能存在一个真机或仿真执行后端.
#   cmd_arbitration 默认 reject, shutdown_safe_home 默认 false, 不自动使能.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    """从已安装的某个包共享目录里读取 YAML 配置并解析为 Python 对象.

    参数: package_name 为包名; relative_path 为相对该包 share 目录的路径.
    路径不存在或 YAML 非法时直接抛异常, 不做静默兜底.
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    """构建真机共享组合:硬件片段 + MoveIt 组合 + 可选交互式 RViz."""
    arm_namespace = LaunchConfiguration("arm_namespace")
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    rviz_config = LaunchConfiguration("rviz_config")

    # MoveIt 规划所需配置与 moveit_demo 使用同一份模型, 保证 RViz 显示和规划一致.
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    ompl_planning_yaml = load_yaml("rebotarm_moveit_config", "config/ompl_planning.yaml")

    return LaunchDescription(
        [
            # 机械臂电机/串口配置(电机 ID,增益,限速等), 只用于真机后端.
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # 夹爪电机配置, 只用于真机后端.
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 空串或 auto 表示由控制器配置选择通道, 显式路径优先.
            DeclareLaunchArgument("channel", default_value=""),
            # 退出时是否先回安全位再失能: 默认 false, 避免退出流程带动机械臂.
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "rviz", "interactive_system.rviz"]),
            ),
            # MoveIt 与准备服务共享唯一场景; 真机必须使用真实关节状态.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "moveit_demo.launch.py"])
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "arm_namespace": arm_namespace,
                    "prepare_trajectories": "true",
                    "use_fake_joint_states": "false",
                }.items(),
            ),
            # 唯一真实硬件后端由底层片段拥有.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "hardware_controller.launch.py"]
                    )
                ),
                launch_arguments={
                    "arm_config": arm_config,
                    "gripper_config": gripper_config,
                    "channel": channel,
                    "shutdown_safe_home": shutdown_safe_home,
                    "joint_state_rate": joint_state_rate,
                    "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                    "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                    "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                    "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                    "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
                    "cmd_arbitration": cmd_arbitration,
                    "arm_namespace": arm_namespace,
                    "frame_id": frame_id,
                    "ee_frame_id": ee_frame_id,
                }.items(),
            ),
            # 交互式 RViz: 注入 MoveIt 规划参数, 供 MotionPlanning 面板使用.
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    ompl_planning_yaml,
                ],
                condition=IfCondition(use_local_rviz),
            ),
        ]
    )
