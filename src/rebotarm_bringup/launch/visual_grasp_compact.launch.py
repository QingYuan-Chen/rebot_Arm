"""日常使用的精简视觉抓取入口。

这里只暴露组合层参数；视觉、候选、运动和夹爪策略继续从各自 profile 读取。旧的
``visual_grasp_system.launch.py`` 保留完整参数，仅用于兼容历史脚本。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def _prepare_stages(context):
    """初始化内部兼容默认值，不向公开参数列表递归展开旧入口。

    共享默认值仍只有一份；这不是新增策略 profile。已传入的公开配置与旧覆盖值
    按 DeclareLaunchArgument 原规则保留。这里只构造启动动作，不启动任何 ROS 进程。
    """
    source = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "launch", "visual_grasp_system.launch.py"]
    ))
    description = source.get_launch_description(context)
    stages = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        else:
            stages.append(action)
    return stages


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    names = {
        "arm_namespace": "rebotarm",
        "channel": "auto",
        "use_hardware": "false",
        "execution_mode": "plan_only",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_candidate_ik_filter": "true",
        "start_motion_execution": "true",
        "start_visual_grasp_executor": "false",
        "execute_gripper": "false",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "start_visual_ready": "true",
        "move_to_visual_ready_on_start": "false",
        "start_sim_trajectory_controller": "true",
        "use_sim_time": "false",
        "visual_interfaces_config": PathJoinSubstitution(
            [bringup_share, "config", "visual_grasp_interfaces.yaml"]),
        "vision_camera_config": PathJoinSubstitution(
            [vision_share, "config", "camera_ubuntu.yaml"]),
        "vision_handeye_config": PathJoinSubstitution(
            [vision_share, "config", "handeye.yaml"]),
        "graspnet_config": PathJoinSubstitution(
            [vision_share, "config", "graspnet_ubuntu.yaml"]),
    }
    declarations = []
    for name, default in names.items():
        kwargs = {"default_value": default}
        if name == "execution_mode":
            kwargs["choices"] = ["plan_only", "execute"]
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_prepare_stages),
        ])
    )
    return LaunchDescription(declarations)
