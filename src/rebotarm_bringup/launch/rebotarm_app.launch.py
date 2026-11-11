#真实机械臂「一体化工作台」启动入口：MoveIt + 示教录制 + 网页遥操作面板 + 可选 RViz。
#
#用途：面向真机的单命令入口。它包含完整的硬件与 MoveIt 栈（由本包的硬件启动文件组装），再叠加示教录制节点和网页遥操作状态面板；示教录制/检查/回放由网页界面里的示教卡片驱动，不提供 mode/profile 之类的分支参数。
#
#启动组合与后端选择：
#
#- 先包含“moveit_hardware.launch.py”（真机控制器 + MoveIt“move_group”），统一传入串口通道与抓爪安全阈值；
#- 启动唯一的 TeachRecorderNode，录制由 Dashboard 卡片显式开始和停止；
#- 可选启动 RViz（“use_rviz”，使用网页遥操作专用的只读状态布局）；
#- “panel=true”时启动网页遥操作状态面板节点，参数示例 `panel_mode="control"` 表示面板处于可下发控制命令的模式（“web_execute_enabled”再叠加一层执行许可）。
#
#安全语义：这是“真机全量启动”，因此“use_hardware”默认“true”且在“_launch_setup”中强制校验——传“use_hardware:=false”会直接抛错拒绝启动，避免有人误把真机入口当仿真入口用；
#“execution_mode”默认“execute”，“web_execute_enabled”默认“true”。串口通道为空或“auto”时由控制器配置解析，不在启动入口探测设备名。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bool_text(value: str) -> str:
    """把常见写法（1/true/yes/on，大小写与空白无关）规整为字符串“true”/“false”。

    返回字符串而不是 bool：launch 参数与 PythonExpression 都需要文本形式的布尔值。
    """
    return "true" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "false"


def _as_bool(value: str) -> bool:
    """把启动参数字符串转成真正的 Python 布尔值，供 ROS 参数使用。"""
    return _bool_text(value) == "true"


def _record_path(name: str, explicit_path: str) -> str:
    """决定示教记录文件的落盘路径。

    显式指定 ``explicit_path`` 时原样使用；否则用 ``name`` 生成文件名：先取 basename
    去掉目录部分（防止 ``../`` 之类的路径穿越），再补 ``.jsonl`` 后缀，最后统一放到
    ``teleop_records/`` 目录下。``name`` 为空时退回默认名 ``teach_record``。
    """
    if explicit_path:
        return explicit_path
    safe_name = name.strip() or "teach_record"
    safe_name = os.path.basename(safe_name.replace("\\", "/")) or "teach_record"
    if not safe_name.endswith(".jsonl"):
        safe_name = f"{safe_name}.jsonl"
    return f"teleop_records/{safe_name}"


def _panel_node(
    *,
    common_config,
    web_config,
    teach_config,
    arm_namespace: str,
    record_path: str,
    use_hardware: str,
    web_execute_enabled: str,
    execution_mode: str,
    panel_mode: str,
):
    """构造网页遥操作状态面板节点。

    ``panel_mode="control"`` 表示面板可下发控制命令；``web_execute_enabled`` 是面板
    侧的执行总开关，``execution_mode`` 决定下发的是干跑还是真执行，``use_hardware``
    告诉面板当前是否接着真机（影响它展示的状态与允许的操作）。
    """
    return Node(
        package="rebotarm_dashboard",
        executable="TeleopStatusPanelNode",
        name="teleop_status_panel_node",
        output="screen",
        parameters=[
            common_config,
            web_config,
            teach_config,
            {
                "arm_namespace": arm_namespace,
                "web_execute_enabled": _as_bool(web_execute_enabled),
                "record_path": record_path,
                "use_hardware": _as_bool(use_hardware),
                "execution_mode": execution_mode,
                "panel_mode": panel_mode,
            },
        ],
    )


def _launch_setup(context, *args, **kwargs):
    """OpaqueFunction 回调：在启动时（而非描述构建时）求值参数并组装动作列表。

    需要 OpaqueFunction 的原因是这里要根据 ``use_hardware`` 做硬校验并组装条件节点，
    这些只能在 launch 上下文里完成；串口 auto 解析由控制器配置负责。
    """
    name = LaunchConfiguration("name").perform(context).strip()
    explicit_record_path = LaunchConfiguration("record_path").perform(context).strip()
    record_path = _record_path(name, explicit_record_path)
    arm_namespace = LaunchConfiguration("arm_namespace").perform(context)
    channel = LaunchConfiguration("channel").perform(context)
    use_hardware = _bool_text(LaunchConfiguration("use_hardware").perform(context))
    use_rviz = _bool_text(LaunchConfiguration("use_rviz").perform(context))
    panel = _bool_text(LaunchConfiguration("panel").perform(context))
    web_execute_enabled = _bool_text(LaunchConfiguration("web_execute_enabled").perform(context))
    execution_mode = LaunchConfiguration("execution_mode").perform(context).strip().lower()
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz").perform(context)
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm").perform(context)
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s").perform(context)
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec").perform(context)
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec").perform(context)
    common_config = LaunchConfiguration("common_config")
    web_config = LaunchConfiguration("web_config")
    teach_config = LaunchConfiguration("teach_config")
    bringup_share = FindPackageShare("rebotarm_bringup")
    moveit_launch = PathJoinSubstitution([bringup_share, "launch", "moveit_hardware.launch.py"])
    web_rviz_config = PathJoinSubstitution([bringup_share, "rviz", "web_teleop_status.rviz"])

    # 安全门：本入口只服务真机，显式传 use_hardware:=false 时直接失败，防止误当仿真入口
    if use_hardware != "true":
        raise RuntimeError(
            "rebotarm_app.launch.py is the full hardware app and requires use_hardware:=true"
        )

    actions = [
        LogInfo(msg="reBotArm app: starting full MoveIt + web teleop workbench"),
        # 硬件 + MoveIt 主体：空串或 auto 原样下传，由控制器配置选择通道；
        # RViz 由本文件统一决定是否启动，因此这里固定传“false”避免重复
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "arm_namespace": arm_namespace,
                "channel": channel,
                "use_rviz": "false",
                "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
            }.items(),
        ),
        # 示教录制属于完整工作台，不属于基础 MoveIt 入口；默认不自动开始录制。
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
                    "require_motor_status": _as_bool(use_hardware),
                },
            ],
        ),
        # 网页遥操作专用 RViz 布局：只显示机器人状态与 TF，不提供运动规划交互
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", web_rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ]

    if panel == "true":
        actions.append(
            _panel_node(
                common_config=common_config,
                web_config=web_config,
                teach_config=teach_config,
                arm_namespace=arm_namespace,
                record_path=record_path,
                use_hardware=use_hardware,
                web_execute_enabled=web_execute_enabled,
                execution_mode=execution_mode,
                # control：面板处于可下发控制命令的模式（区别于只读状态展示）
                panel_mode="control",
            )
        )

    actions.append(LogInfo(msg="teach recording/check/replay are controlled from the web Teach Trajectory card"))

    # 标定节点消费已有图像/TF；此开关只组合采集服务，相机启动和电机使能各由独立入口负责。
    actions.append(Node(
        package="rebotarm_calibration", executable="rebotarm_handeye_capture",
        output="screen", condition=IfCondition(LaunchConfiguration("calibration")),
        parameters=[{"arm_namespace": arm_namespace,
                     "image_topic": LaunchConfiguration("calibration_image_topic"),
                     "camera_info_topic": LaunchConfiguration("calibration_camera_info_topic"),
                     "session_directory": LaunchConfiguration("calibration_session_directory")}],
    ))
    return actions


def generate_launch_description():
    """声明启动参数并注册 ``_launch_setup``（参数求值与动作组装延迟到启动时）。"""
    config_share = FindPackageShare("rebotarm_bringup")
    common_config = PathJoinSubstitution([config_share, "config", "operator_common.yaml"])
    web_config = PathJoinSubstitution([config_share, "config", "web_teleop.yaml"])
    teach_config = PathJoinSubstitution([config_share, "config", "teach_control.yaml"])

    return LaunchDescription(
        [
            # 示教记录名：未显式给 record_path 时用它生成 teleop_records/<name>.jsonl
            DeclareLaunchArgument("name", default_value="teach_record"),
            # 显式记录路径；留空表示按 name 自动生成
            DeclareLaunchArgument("record_path", default_value=""),
            # 机械臂命名空间（话题前缀）
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 串口通道：空串或 auto 表示使用控制器配置中的通道，显式路径优先
            DeclareLaunchArgument("channel", default_value="auto"),
            # 真机入口：默认 true，且传 false 会被 _launch_setup 拒绝
            DeclareLaunchArgument("use_hardware", default_value="true"),
            # 是否启动网页遥操作专用 RViz
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # 是否启动网页遥操作状态面板
            DeclareLaunchArgument("panel", default_value="true"),
            # 面板执行总开关：true 才允许从网页下发执行命令
            DeclareLaunchArgument("web_execute_enabled", default_value="true"),
            # 执行模式：execute 为真正下发；plan_only 只做规划干跑
            DeclareLaunchArgument("execution_mode", default_value="execute"),
            # 硬件反馈读取频率（Hz）：调高更实时，占用串口带宽更多
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            # 抓爪位置模式力矩上限（N·m）：限制夹持力，防止压坏物体或堵转
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            # 抓爪位置模式最大速度（rad/s）：限制闭合/张开冲击
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            # 抓爪位置命令超时余量（s）：在预估运动时长之外额外允许的时间
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            # 抓爪反馈过期阈值（s）：超时未收到新反馈即判定反馈不新鲜
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            # 配置按消费者拆分：公共关节定义、网页参数和示教镜像参数。
            DeclareLaunchArgument("common_config", default_value=common_config),
            DeclareLaunchArgument("web_config", default_value=web_config),
            DeclareLaunchArgument("teach_config", default_value=teach_config),
            DeclareLaunchArgument("calibration", default_value="false"),
            DeclareLaunchArgument("calibration_image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("calibration_camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("calibration_session_directory", default_value="~/.ros/rebotarm_calibration"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
