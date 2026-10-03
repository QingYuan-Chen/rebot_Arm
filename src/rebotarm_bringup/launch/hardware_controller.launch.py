#真实机械臂控制器的唯一底层启动片段。
#
#本文件只声明硬件公共参数并启动 "reBotArmController"。它不启动 MoveIt、RViz、robot_state_publisher、示教、遥操作、视觉或网页节点。
#所有需要真实硬件的 bringup组合都应包含本文件，避免复制控制器节点和安全参数。
#
#安全语义保持不变：控制器启动后处于失能状态，必须获得新鲜反馈、完成现场检查并显式调用 "/<arm_namespace>/enable" 后才允许运动；默认退出不自动回位。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")

    controller_parameters = {
        "arm_config": LaunchConfiguration("arm_config"),
        "gravity_config": LaunchConfiguration("gravity_config"),
        "gripper_config": LaunchConfiguration("gripper_config"),
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
        "arm_namespace": LaunchConfiguration("arm_namespace"),
        "frame_id": LaunchConfiguration("frame_id"),
        "ee_frame_id": LaunchConfiguration("ee_frame_id"),
    }

    return LaunchDescription(
        [
            # arm_config：机械臂电机配置（电机 ID/型号/增益/限速）。
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            DeclareLaunchArgument("gravity_config", default_value=""),
            # gripper_config：夹爪电机配置。
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # channel：电机总线串口设备；空串表示由上面的配置文件决定。
            DeclareLaunchArgument("channel", default_value=""),
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            # joint_state_rate：关节状态发布频率（Hz）；越高越实时，总线负载越大。
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # hardware_feedback_rate_hz：硬件层反馈刷新频率上限（Hz），必须在 [20, 100]，
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            # gripper_position_torque_cap_nm：夹爪位置指令的力矩上限（N·m），允许范围[0.05, 1.5]；调大夹持更牢但堵转发热与夹伤风险更高。
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            # gripper_position_max_speed_rad_s：夹爪开合角速度上限（rad/s），硬件层允许范围 [0.05, 3.0]；调大更快但冲击更大。控制器内置默认是 0.5，本文件取 1.5。
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            # gripper_position_timeout_margin_sec：夹爪到位超时余量（s），实际超时 = 行程/速度 + 该余量；允许范围 [0.1, 10.0]。
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            # gripper_feedback_stale_timeout_sec：夹爪反馈新鲜度阈值（s），超过即判反馈过期并拒绝新的夹爪命令；允许范围 [0.05, 2.0]。
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            # 保持夹持的最长时间（s）：超时后松开，避免长时间堵转发热
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
            # cmd_arbitration：轨迹运行期间收到单关节透传指令时的仲裁策略。"reject"=直接拒绝（默认，安全优先）；"preempt"=先停下轨迹再执行透传指令。
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # arm_namespace：话题/服务/动作命名空间前缀，必须与 MoveIt 配置一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # frame_id / ee_frame_id：对外位姿的参考坐标系与末端坐标系名称，需与 URDF 一致。
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[controller_parameters],
            ),
        ]
    )
