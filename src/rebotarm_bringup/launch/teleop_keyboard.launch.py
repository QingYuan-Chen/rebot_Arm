"""键盘遥操作组合:真机或 MuJoCo 共用 continuous 规划,力控状态与执行协议.

职责:只装配唯一后端,MoveIt 场景与键盘节点;无硬件默认使用 MuJoCo.
接口:use_hardware 选择后端,use_mujoco_viewer 控制 MuJoCo 原生窗口(默认 true),
     use_rviz / use_local_rviz 控制 RViz(默认 false),keyboard_prefix 提供 TTY.
安全:真机默认失能;仿真与真机阻抗参数文件物理隔离(use_hardware=true 不加载仿真阻抗 YAML).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_keyboard_node(context):
    hardware = LaunchConfiguration("use_hardware").perform(context).strip().lower() == "true"
    namespace = LaunchConfiguration("arm_namespace").perform(context)
    common_config = LaunchConfiguration("common_config").perform(context)
    keyboard_config = LaunchConfiguration("keyboard_config").perform(context)
    sim_impedance_config = LaunchConfiguration("sim_impedance_config").perform(context).strip()
    hardware_impedance_config = LaunchConfiguration("hardware_impedance_config").perform(context).strip()
    keyboard_prefix = LaunchConfiguration("keyboard_prefix").perform(context)

    parameters = [common_config, keyboard_config]
    if hardware:
        if hardware_impedance_config:
            parameters.append(hardware_impedance_config)
    elif sim_impedance_config:
        parameters.append(sim_impedance_config)
    use_sim_time = not hardware
    parameters.append({
        "arm_namespace": namespace,
        "use_sim_time": use_sim_time,
        "use_hardware": hardware,
    })
    return [
        Node(
            package="rebotarm_teleop",
            executable="TeleopKeyboardNode",
            name="teleop_keyboard_node",
            output="screen",
            prefix=keyboard_prefix,
            parameters=parameters,
        )
    ]


def generate_launch_description():
    share = FindPackageShare("rebotarm_bringup")
    hardware = LaunchConfiguration("use_hardware")
    namespace = LaunchConfiguration("arm_namespace")
    rviz = LaunchConfiguration("use_local_rviz")
    mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    return LaunchDescription([
        DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
        DeclareLaunchArgument("use_hardware", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_local_rviz", default_value=LaunchConfiguration("use_rviz"), choices=["true", "false"]),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true", choices=["true", "false"]),
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
        DeclareLaunchArgument("sim_impedance_config",
            default_value=PathJoinSubstitution([share, "config", "keyboard_impedance_sim.yaml"])),
        DeclareLaunchArgument("hardware_impedance_config", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, "launch", "interactive_system.launch.py"])),
            condition=IfCondition(hardware),
            launch_arguments={
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
                "use_mujoco_viewer": mujoco_viewer, "use_dashboard": "false"}.items()),
        OpaqueFunction(function=_build_keyboard_node),
    ])
