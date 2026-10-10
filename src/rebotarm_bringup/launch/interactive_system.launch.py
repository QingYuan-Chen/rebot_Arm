#RViz 交互与遥操作的共享底层启动文件（真机 / 仿真两种后端二选一）。是rviz_ee_drag_sim、visual_grasp_system 的共享子文件（已测）
#
#用途：统一实现状态源、真机控制器、MoveIt 和 RViz 的互斥组合，供 moveit_hardware、仿真末端拖动和视觉抓取入口复用。真实拖动控制使用 MoveIt 原生 MotionPlanning 工作流。
#
#后端选择逻辑（互斥，同一条启动里只会生效一个）：
#
#- use_moveit_preview=true：包含 MoveIt 配置包自带的 demo 启动文件，由它提供"move_group"、假的关节状态与规划能力（纯预览，不接硬件）；
#- use_moveit_preview=false：本文件自己拉起机器人状态发布、被动关节状态发布与可选的真实控制器节点，由 "use_hardware" 决定是否打开硬件通道。
#
#关键话题约定：机器人模型订阅 "/<arm_namespace>/visual_joint_states"（可视化用的关节状态，可能来自控制器反馈，也可能由假状态源填充）；"/<arm_namespace>/joint_states"则是控制器/仿真后端发布的执行侧关节状态。
#
#安全默认值："use_hardware=false"、"shutdown_safe_home=false"、"cmd_arbitration=reject"，即默认不碰真机、退出时不自动回安全位、多路命令冲突时直接拒绝。
#抓爪的力矩上限、速度上限与反馈新鲜度阈值都在本文件显式声明并下发，不依赖节点内默认值。

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    """从已安装的某个包共享目录里读取 YAML 配置并解析为 Python 对象。

    参数： package_name 为包名; relative_path 为相对该包 share 目录的路径。
    路径不存在或 YAML 非法时直接抛异常，不做静默兜底。
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    """构建本启动文件的 LaunchDescription（参数声明 + 互斥节点组合）。

    参数按用途分三类：硬件/串口与抓爪安全阈值（仅在 "use_hardware=true" 时下发给控制器）、机器人模型与可视化开关、以及 MoveIt 规划所需的配置。
    """
    arm_namespace = LaunchConfiguration("arm_namespace")
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    use_moveit_preview = LaunchConfiguration("use_moveit_preview")
    use_hardware = LaunchConfiguration("use_hardware")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    grasp_hold_timeout_sec = LaunchConfiguration("grasp_hold_timeout_sec")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    start_passive_joint_state_publisher = LaunchConfiguration("start_passive_joint_state_publisher")
    use_moveit_fake_joint_states = LaunchConfiguration("use_moveit_fake_joint_states")
    rviz_config = LaunchConfiguration("rviz_config")

    # 直接用 cat 输出 URDF 文本作为 robot_description；参数值与 MoveIt 配置使用同一份 URDF，保证 RViz 显示、TF 与规划模型三者一致
    urdf_file = PathJoinSubstitution([FindPackageShare("rebotarm_moveit_config"), "config", "rebotarm.urdf"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)
    # MoveIt 规划所需的全部配置：模型、语义、运动学、限位、控制器与规划管线
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
    # 读取 OMPL 规划管线参数：作为独立参数集传给 RViz 内的 MotionPlanning 插件
    ompl_planning_yaml = load_yaml("rebotarm_moveit_config", "config/ompl_planning.yaml")

    return LaunchDescription(
        [
            # 机械臂电机/串口配置（电机 ID、增益、限速等），仅硬件模式使用
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # 夹爪电机配置，仅硬件模式使用
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # 机械臂命名空间：决定话题前缀（/<ns>/joint_states 等）
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 串口设备路径；留空表示由上层自行解析（例如 auto 探测 ttyACM0/1）
            DeclareLaunchArgument("channel", default_value=""),
            # 退出时是否先回到安全位再失能：默认 false，避免退出流程带动机械臂
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            # 控制器关节状态发布频率（Hz），也是真机反馈进入可视化链路的节拍
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # 硬件反馈读取频率（Hz）：调高更实时，但占用串口带宽更多
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            # 抓爪位置模式的力矩上限（N·m）：限制夹持力，防止压坏物体或堵转。
            # 抓爪安全限值必须在这里显式给出、不能依赖节点内默认值：本文件才是真实抓取实际走的启动路径。
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            # 抓爪位置模式的最大速度（rad/s）：限制闭合/张开的冲击
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            # 抓爪位置命令的超时余量（s）：在预估运动时长基础上额外允许的时间
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            # 抓爪反馈过期阈值（s）：超过该时长没有新反馈即判定为不新鲜
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            # 保持夹持的最长时间（s）：超时后松开，避免长时间堵转发热
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
            # 多路命令仲裁策略：reject 表示发现冲突命令直接拒绝（不做静默覆盖）
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # 是否在本文件内启动 RViz
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            # 是否改用 MoveIt demo 预览栈（关掉硬件通道与被动状态源）
            DeclareLaunchArgument("use_moveit_preview", default_value="false"),
            # 是否打开真实硬件通道；默认 false，必须显式开启
            DeclareLaunchArgument("use_hardware", default_value="false"),
            # 规划与执行使用的基座坐标系
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            # 末端（法兰）坐标系名
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            # 是否启动被动关节状态发布（无硬件时给 RViz 一个可拖动/可显示的假状态源）
            DeclareLaunchArgument("start_passive_joint_state_publisher", default_value="true"),
            # 是否让 MoveIt 侧使用假关节状态（与真机反馈互斥）
            DeclareLaunchArgument("use_moveit_fake_joint_states", default_value="true"),
            # 本启动文件配套的 RViz 布局（含 MoveIt MotionPlanning 面板）
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([bringup_share, "rviz", "interactive_system.rviz"]),
            ),
            # 预览模式：包含 MoveIt demo 启动文件（提供 move_group 与假状态），不接真实控制器
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "moveit_demo.launch.py"])
                ),
                condition=IfCondition(use_moveit_preview),
                launch_arguments={
                    "use_rviz": "false",
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": PythonExpression(
                        [
                            "'false' if '",
                            use_hardware,
                            "'.lower() == 'true' or '",
                            use_moveit_fake_joint_states,
                            "'.lower() != 'true' else 'true'",
                        ]
                    ),
                }.items(),
            ),
            # 真实硬件后端统一由底层片段拥有；预览/仿真时整个 include 被跳过。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "hardware_controller.launch.py"]
                    )
                ),
                condition=IfCondition(use_hardware),
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
                    "grasp_hold_timeout_sec": grasp_hold_timeout_sec,
                    "cmd_arbitration": cmd_arbitration,
                    "arm_namespace": arm_namespace,
                    "frame_id": frame_id,
                    "ee_frame_id": ee_frame_id,
                }.items(),
            ),
            # 夹爪可视化状态桥：把夹爪反馈映射成 RViz 手指关节角，随 /visual_joint_states 发布；
            # 预览模式下由 MoveIt demo 负责该职责，故用 UnlessCondition 互斥
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
                condition=UnlessCondition(use_moveit_preview),
            ),
            # 机器人状态发布：用 URDF 把 /visual_joint_states 转成 TF；
            # 订阅被重映射到带命名空间的可视化关节状态话题，避免与执行侧 /joint_states 混用
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                condition=UnlessCondition(use_moveit_preview),
            ),
            # 被动关节状态源：只在无硬件、非 MoveIt 预览且显式开启时发布假关节状态。
            # MoveIt 预览由 moveit_demo.launch.py 自己提供同一话题，必须互斥，避免两个发布器竞争。
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="interactive_joint_state_publisher",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_hardware,
                            "'.lower() != 'true' and '",
                            use_moveit_preview,
                            "'.lower() != 'true' and '",
                            start_passive_joint_state_publisher,
                            "'.lower() == 'true'",
                        ]
                    )
                ),
                parameters=[
                    {"robot_description": robot_description},
                    # 假状态发布频率（Hz）：30 足以驱动可视化，不追求控制精度
                    {"rate": 30.0},
                ],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            # RViz：加载本场景布局，并把 MoveIt 规划所需参数一并注入（MotionPlanning 插件在进程内读取）
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
