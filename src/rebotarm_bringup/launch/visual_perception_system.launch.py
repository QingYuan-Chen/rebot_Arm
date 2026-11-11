"""视觉感知、抓取候选生成、筛选与显示组合入口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def _prepare_stages(context):
    """分域 YAML 是正式入口的唯一默认值来源。"""
    import os
    import json
    from rebotarm_bringup.visual_profiles import read_section

    for name in ("strategy_profile", "visual_interfaces_config"):
        values = read_section(LaunchConfiguration(name).perform(context), "launch", None)
        for key, value in values.items():
            if isinstance(value, str):
                value = value.replace("{arm_namespace}", LaunchConfiguration("arm_namespace").perform(context).strip("/"))
            context.launch_configurations.setdefault(key, value if isinstance(value, str) else json.dumps(value))

    defaults = {
        "vision_python_executable": os.environ.get("REBOTARM_VISION_PYTHON", "python3"),
        "graspnet_python_executable": os.environ.get("GRASPNET_PYTHON", "python3"),
        "vision_yolo_model_path": PathJoinSubstitution([FindPackageShare("rebotarm_vision"), "models", "yolo26s-seg.pt"]).perform(context),
    }
    # GraspNet 资源只接受部署环境显式提供, 不沿工作区父目录猜测第三方位置;
    # 未设置时由启用后的推理节点在加载阶段明确失败.
    for name, env in (("graspnet_model_root", "GRASPNET_MODEL_ROOT"),
                      ("graspnet_checkpoint_path", "GRASPNET_CHECKPOINT_PATH")):
        defaults[name] = os.environ.get(env, "").strip()
    for key, value in defaults.items():
        context.launch_configurations.setdefault(key, value)
    context.launch_configurations["start_candidate_ik_filter"] = "true"
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "launch", "includes", name + ".launch.py"]
    ))) for name in ("visual_backend", "visual_lifecycle")]


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    names = {
        "arm_namespace": "rebotarm",
        "robot_context": "offline",
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
        "use_sim_time": "false",
        "strategy_profile": PathJoinSubstitution([vision_share, "config", "visual_strategy_profile.yaml"]),
        "visual_interfaces_config": PathJoinSubstitution(
            [bringup_share, "config", "visual_grasp_interfaces.yaml"]),
        "vision_camera_config": PathJoinSubstitution(
            [vision_share, "config", "camera_ubuntu.yaml"]),
        "vision_handeye_config": "",
        "graspnet_config": PathJoinSubstitution(
            [vision_share, "config", "graspnet_ubuntu.yaml"]),
    }
    declarations = []
    for name, default in names.items():
        kwargs = {"default_value": default}
        if name == "robot_context":
            kwargs["choices"] = ["offline", "external"]
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_prepare_stages),
        ])
    )
    return LaunchDescription(declarations)
