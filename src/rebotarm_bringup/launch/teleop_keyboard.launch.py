"""键盘遥操作组合：真机或 MuJoCo 共用 continuous 规划和执行协议。

职责：只装配唯一后端、MoveIt 场景与键盘节点；无硬件默认使用 MuJoCo。
接口：use_hardware 选择后端，use_local_rviz 控制显示，keyboard_prefix 提供 TTY。
安全：真机默认失能；仿真和真机状态、时钟与执行服务保持互斥。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("rebotarm_bringup")
    hardware = LaunchConfiguration("use_hardware")
    namespace = LaunchConfiguration("arm_namespace")
    rviz = LaunchConfiguration("use_local_rviz")
    use_sim_time = ParameterValue(PythonExpression([
        "'", hardware, "'.lower() != 'true'"
    ]), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
        DeclareLaunchArgument("use_hardware", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_local_rviz", default_value="true"),
        DeclareLaunchArgument("channel", default_value=""),
        DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
        DeclareLaunchArgument("keyboard_prefix",
            default_value="bash -lc 'exec \"$0\" \"$@\" < /dev/tty'"),
        DeclareLaunchArgument("arm_config",
            default_value=PathJoinSubstitution([share, "config", "arm.yaml"])),
        DeclareLaunchArgument("gripper_config",
            default_value=PathJoinSubstitution([share, "config", "gripper.yaml"])),
        DeclareLaunchArgument("common_config",
            default_value=PathJoinSubstitution([share, "config", "operator_common.yaml"])),
        DeclareLaunchArgument("keyboard_config",
            default_value=PathJoinSubstitution([share, "config", "keyboard_control.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, "launch", "interactive_system.launch.py"])),
            condition=IfCondition(hardware),
            launch_arguments={
                "use_hardware": "true", "use_moveit_preview": "true",
                "use_moveit_fake_joint_states": "false",
                "start_passive_joint_state_publisher": "false",
                "arm_namespace": namespace, "use_local_rviz": rviz,
                "arm_config": LaunchConfiguration("arm_config"),
                "gripper_config": LaunchConfiguration("gripper_config"),
                "channel": LaunchConfiguration("channel"),
                "joint_state_rate": LaunchConfiguration("joint_state_rate"),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, "launch", "mujoco_moveit_sim.launch.py"])),
            condition=UnlessCondition(hardware),
            launch_arguments={"arm_namespace": namespace, "use_rviz": rviz,
                "use_mujoco_viewer": "false", "use_dashboard": "false"}.items()),
        Node(package="rebotarm_teleop", executable="TeleopKeyboardNode",
            name="teleop_keyboard_node", output="screen",
            prefix=LaunchConfiguration("keyboard_prefix"),
            parameters=[LaunchConfiguration("common_config"), LaunchConfiguration("keyboard_config"),
                {"arm_namespace": namespace, "use_sim_time": use_sim_time}]),
    ])
