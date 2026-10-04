# 键盘遥操作启动文件：真机键盘点动，或无硬件时的仿真键盘点动。
#
#用途
#    真机模式用键盘按固定增量向控制器发送关节目标，并把夹爪状态桥接成可视化关节状态；
#    无硬件模式启动轻量 FollowJointTrajectory 仿真服务端，按键目标由它插值执行并反馈到
#    RViz；这不是 MuJoCo 物理仿真，只用于键盘、动作接口、关节状态和可视化联调。
#    本文件只做启动组合，遥操作逻辑在操作交互包里。
#
#节点组合与真实/仿真后端选择
#    - "use_hardware=true"：启动硬件控制器（唯一硬件入口）；robot_state_publisher订阅控制器发布的视觉关节状态；
#    - "use_hardware=false"（默认）：不启动硬件节点，由仿真轨迹控制器发布关节状态并提供
#      FollowJointTrajectory 动作服务；按键会反馈到 RViz 姿态，也绝不会驱动真实电机；
#    - 键盘节点与夹爪可视化关节节点始终启动：前者读取并尝试发送目标，后者补齐夹爪关节；
#    - RViz 仅当 "use_local_rviz=true" 时启动。
#
#参数来源
#    "common_config" 提供关节名称/限位，"keyboard_config" 只提供终端键盘参数。
#    示教录制不在本启动文件中启动；由组合入口的 Dashboard 提供录制和回放。
#
#安全默认值
#    "use_hardware" 默认 "false"：不会因为顺手敲一条启动命令就驱动真机，真机必须显式传 "use_hardware:=true"。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造键盘入口：在真机控制器与轻量仿真轨迹控制器之间二选一。"""
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    common_config = LaunchConfiguration("common_config")
    keyboard_config = LaunchConfiguration("keyboard_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    # bringup_share 用于定位 RViz 布局；config_share 用于定位 YAML 参数文件，两者都是本启动组合包的 share 目录。
    bringup_share = FindPackageShare("rebotarm_bringup")
    config_share = FindPackageShare("rebotarm_bringup")
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("rebotarm_moveit_config"), "config", "rebotarm.urdf"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    # 用 cat 把 URDF 读成字符串，并强制 value_type=str，避免长文本被当作 YAML 解析。
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "\n"
                    "========== reBotArm 键盘遥操作 =========="
                    "\n"
                    "1 / q : joint1 正转 / 反转    2 / w : joint2 正转 / 反转"
                    "\n"
                    "3 / e : joint3 正转 / 反转    4 / r : joint4 正转 / 反转"
                    "\n"
                    "5 / t : joint5 正转 / 反转    6 / y : joint6 正转 / 反转"
                    "\n"
                    "每次按键约 0.02 rad；空格发送软件停止请求；Ctrl+C 退出。"
                    "\n"
                    "当前默认 use_hardware=false：只驱动 RViz 轻量仿真，不连接真机。"
                    "\n"
                    "真机使用前必须确认现场安全，并显式 use_hardware:=true 后再 Enable。"
                    "\n"
                    "RViz 相机：左键旋转，中键平移，滚轮缩放；工具栏可选择 MoveCamera。"
                    "\n"
                    "=========================================="
                )
            ),
            # arm_namespace：话题/服务/动作的命名空间前缀，必须与控制器一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # use_hardware：true=驱动真机；false=启动轻量仿真轨迹控制器。默认 false，防止误启动真机。
            DeclareLaunchArgument("use_hardware", default_value="false"),
            # use_local_rviz：是否启动 RViz 可视化。
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            # channel：电机总线串口设备；空串表示交由 arm.yaml/gripper.yaml 决定。
            DeclareLaunchArgument("channel", default_value=""),
            # joint_state_rate：控制器关节状态发布频率（Hz）；越高越实时，总线负载越大。
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # keyboard_prefix：在键盘节点命令前插入的包装命令。默认用 bash -lc 把节点进程的
            #   stdin 重定向到 /dev/tty，否则通过 ros2 launch 启动时键盘读不到终端输入。
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            # arm_config：机械臂电机配置（电机 ID、型号、PID/增益、限速）。
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # gripper_config：夹爪电机配置。
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # common_config：公共命名空间、关节名称和关节限位。
            DeclareLaunchArgument(
                "common_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "operator_common.yaml"]
                ),
            ),
            # keyboard_config：仅终端键盘参数，deadman 只在这个节点中生效。
            DeclareLaunchArgument(
                "keyboard_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "keyboard_control.yaml"]
                ),
            ),
            # 真机控制器由统一底层片段提供；无硬件模式不会解析或启动该片段。
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
                    "joint_state_rate": joint_state_rate,
                    "arm_namespace": arm_namespace,
                }.items(),
            ),
            # 无硬件分支的轨迹服务端：提供键盘所需的 FollowJointTrajectory，并发布可视化用的
            # /<arm_namespace>/joint_states；它不加载物理引擎、不接触真实电机。
            Node(
                package="rebotarm_preview",
                executable="rebotarm_sim_trajectory_controller",
                name="rebotarm_sim_trajectory_controller",
                output="screen",
                condition=UnlessCondition(use_hardware),
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            # 夹爪可视化关节状态：把夹爪宽度映射成 URDF 里的左右指关节角，供 RViz 显示。
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            # TF 发布：订阅控制器的视觉关节状态（夹爪关节已补齐），据此计算并发布 TF。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
            ),
            # 键盘输入节点：读取键盘增量并通过 prefix 保证能拿到终端 stdin。
            Node(
                package="rebotarm_teleop",
                executable="TeleopKeyboardNode",
                name="teleop_keyboard_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[common_config, keyboard_config, {"arm_namespace": arm_namespace}],
            ),
            # 可选 RViz：加载本包的标准布局 rebotarm.rviz。
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_local_rviz),
            ),
        ]
    )
