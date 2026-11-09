#遥操作组合启动文件：键盘遥操作 + 示教录制 + 网页状态面板。
#
#启动用途：一次性拉起“人操作机械臂”这条最小链路，节点组合为——
#  1. 包含 teleop_keyboard.launch.py：按需包含键盘节点、夹爪可视化状态桥接节点、关节状态发布节点与 RViz（该文件内部再决定真实/仿真后端）；
#  2. 示教录制节点（示教包）：真机和无硬件分支都启动，但默认不自动开始录制；
#  3. 网页状态面板节点（网页面板包）：提供本机 HTTP/SSE 界面与命令入口。
#
#真实/仿真后端选择逻辑：
#  - use_hardware=false（默认）：不启动硬件控制器，由 include 的启动文件拉起轻量仿真轨迹控制器，
#    键盘可以驱动 RViz 姿态，录制时不要求真实电机状态；
#  - use_hardware=true：由 include 的启动文件启动真机控制器，录制时要求有效电机状态。真机上电后仍处于失能态，必须显式调用 enable 服务才会运动。
#
#参数来源：键盘、网页和示教分别加载自己的配置，并额外加载 operator_common.yaml。launch 参数只覆盖命名空间、记录路径与面板开关等运行期选择项。
#
#安全默认值：web_execute_enabled=false（网页不开放执行，只能看状态）、channel 留空表示由机械臂配置文件或自动探测决定、use_local_rviz=true 便于操作者直接看到机械臂状态。
#网页执行模式默认为 execute，但 web_execute_enabled 默认关闭；只有显式打开网页执行开关后，
#Dashboard 才会把命令发送给当前后端（无硬件时为轻量仿真控制器，真机时为真实控制器）。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 先取出全部 LaunchConfiguration 句柄：它们是“延迟求值”的替换表达式，在声明DeclareLaunchArgument 之前就可以安全引用，真正取值发生在启动阶段。
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_hardware = LaunchConfiguration("use_hardware")
    trajectory_mode = LaunchConfiguration("trajectory_execution_mode")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    channel = LaunchConfiguration("channel")
    panel = LaunchConfiguration("panel")
    web_execute_enabled = LaunchConfiguration("web_execute_enabled")
    execution_mode = LaunchConfiguration("execution_mode")
    record_path = LaunchConfiguration("record_path")
    common_config = LaunchConfiguration("common_config")
    keyboard_config = LaunchConfiguration("keyboard_config")
    web_config = LaunchConfiguration("web_config")
    teach_config = LaunchConfiguration("teach_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    bringup_share = FindPackageShare("rebotarm_bringup")
    config_share = FindPackageShare("rebotarm_bringup")

    return LaunchDescription(
        [
            # arm_namespace：所有话题/服务/动作的命名空间段，必须与被包含的启动文件和控制器保持一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # use_hardware=false（默认）：无真机/只演练流程；true 时才由被包含的启动
            # 文件拉起硬件控制器。真机分支不会自动使能电机。
            DeclareLaunchArgument("use_hardware", default_value="false"),
            DeclareLaunchArgument("trajectory_execution_mode", default_value=PythonExpression(
                ["'continuous' if '", use_hardware, "'.lower() == 'true' else 'legacy'"]),
                choices=["legacy", "continuous"]),
            # use_local_rviz：是否随键盘遥操作一起打开本机 RViz，便于直接观察机械臂。
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            # channel：电机总线通道覆盖值；空串表示交给机械臂配置文件或自动探测。
            DeclareLaunchArgument("channel", default_value=""),
            # panel：是否启动网页状态面板；关掉后没有任何 HTTP 界面与命令入口。
            DeclareLaunchArgument("panel", default_value="true"),
            # web_execute_enabled：网页是否允许执行运动命令。默认 false = 只读面板，网页只能看状态；放开运动必须由操作者显式打开。
            DeclareLaunchArgument("web_execute_enabled", default_value="false"),
            # execution_mode：网页命令网关模式。web_execute_enabled=false 时仍然不会执行；
            # 显式打开网页执行后，默认 execute 让网页键盘与终端键盘共用同一个后端。
            DeclareLaunchArgument("execution_mode", default_value="execute"),
            # record_path：示教记录 JSONL 输出路径；相对路径按各节点自己的工作目录解析。
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            # keyboard_prefix：键盘节点必须拿到真实 TTY 才能逐字符读键，这里用 bash -lc
            # 把标准输入重定向到 /dev/tty（launch 自身的 stdin 通常不是终端）。
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'",
            ),
            # 各功能配置彼此独立；operator_common.yaml 只放公共关节定义。
            DeclareLaunchArgument(
                "common_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "operator_common.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "keyboard_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "keyboard_control.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "web_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "web_teleop.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "teach_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "teach_control.yaml"]
                ),
            ),
            # 包含键盘遥操作组合（键盘节点 + 夹爪可视化状态桥接 + 关节状态发布 + RViz），
            # 真实/仿真后端的选择在被包含文件内部完成。这里只做参数转发，不实现任何控制。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "teleop_keyboard.launch.py"])
                ),
                launch_arguments={
                    "arm_namespace": arm_namespace,
                    "use_hardware": use_hardware,
                    "trajectory_execution_mode": trajectory_mode,
                    "use_local_rviz": use_local_rviz,
                    "channel": channel,
                    "common_config": common_config,
                    "keyboard_config": keyboard_config,
                    "keyboard_prefix": keyboard_prefix,
                }.items(),
            ),
            # 示教录制节点在两种分支中都启动；本组合只创建这一个录制器。
            # 覆盖项含义：
            #   start_on_launch=False   不随启动自动开录，必须由操作者/面板显式发 start；
            #   keyboard_quit_enabled=False 本组合由 launch 管理生命周期，不监听终端退出键；
            #   require_motor_status 真机为 true，无硬件演练为 false。
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                parameters=[
                    common_config,
                    teach_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "start_on_launch": False,
                        "keyboard_quit_enabled": False,
                        "require_motor_status": ParameterValue(use_hardware, value_type=bool),
                    },
                ],
            ),
            # 网页状态面板节点（网页面板包）：承载本机 HTTP/SSE 界面与“预览/执行/停止”
            # 命令入口。IfCondition(panel) 关闭时整个面板不启动，也就没有任何网页入口。
            # 覆盖项含义：
            #   web_execute_enabled 是否允许网页执行运动（默认 false，只读状态）；
            #   execution_mode    命令网关模式；默认 execute，但仍受 web_execute_enabled 总开关保护；
            #   record_path         与录制节点共用，面板据此展示/切换记录目标；
            #   use_hardware        面板据此判断“真实后端”语义（例如是否展示电机状态）。
            Node(
                package="rebotarm_dashboard",
                executable="TeleopStatusPanelNode",
                name="teleop_status_panel_node",
                output="screen",
                condition=IfCondition(panel),
                parameters=[
                    common_config,
                    web_config,
                    teach_config,
                    {
                        "arm_namespace": arm_namespace,
                        "web_execute_enabled": web_execute_enabled,
                        "execution_mode": execution_mode,
                        "record_path": record_path,
                        "use_hardware": use_hardware,
                        "trajectory_execution_mode": trajectory_mode,
                    },
                ],
            ),
        ]
    )
