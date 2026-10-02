"""正式视觉抓取总组合入口：25 个公共参数，分域 YAML 提供策略默认值。

旧参数兼容入口是 visual_grasp_legacy.launch.py；本入口不加载旧入口。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def _prepare_stages(context):
    """分域 YAML 是正式入口的来源；旧入口不参与新链路。"""
    import os
    import json
    from pathlib import Path
    from rebotarm_bringup.visual_profiles import read_section

    for name in ("backend_profile", "strategy_profile", "motion_profile", "visual_interfaces_config"):
        values = read_section(LaunchConfiguration(name).perform(context), "launch", None)
        for key, value in values.items():
            if isinstance(value, str):
                value = value.replace("{arm_namespace}", LaunchConfiguration("arm_namespace").perform(context).strip("/"))
            context.launch_configurations.setdefault(key, value if isinstance(value, str) else json.dumps(value))

    defaults = {
        "vision_python_executable": os.environ.get("REBOTARM_VISION_PYTHON", "python3"),
        "graspnet_python_executable": os.environ.get("GRASPNET_PYTHON", "python3"),
        "vision_yolo_model_path": PathJoinSubstitution([FindPackageShare("rebotarm_vision"), "models", "yolo26s-seg.pt"]).perform(context),
        "max_plan_age_sec": "10.0" if LaunchConfiguration("execution_mode").perform(context) == "plan_only" else "4.0",
    }
    for name, env, relative in (
        ("graspnet_model_root", "GRASPNET_MODEL_ROOT", "third_party/graspnet-baseline"),
        ("graspnet_checkpoint_path", "GRASPNET_CHECKPOINT_PATH", ".local-models/checkpoints/checkpoint-rs.tar"),
    ):
        value = os.environ.get(env, "").strip()
        if not value:
            for parent in Path(__file__).resolve().parents:
                candidate = parent / relative
                if candidate.exists():
                    value = str(candidate)
                    break
        defaults[name] = value
    for key, value in defaults.items():
        context.launch_configurations.setdefault(key, value)
    hardware = LaunchConfiguration("use_hardware").perform(context).lower() == "true"
    if LaunchConfiguration("execution_mode").perform(context) == "readonly":
        if hardware:
            raise RuntimeError("readonly cannot start a real hardware backend")
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("rebotarm_bringup"), "launch", "includes", name + ".launch.py"]
        ))) for name in ("visual_input", "grasp_candidate")]
    if hardware and LaunchConfiguration("use_sim_time").perform(context).lower() == "true":
        raise RuntimeError("Real hardware requires use_sim_time=false")
    if LaunchConfiguration("execution_mode").perform(context) == "plan_only":
        context.launch_configurations["execute_gripper"] = "false"
        if hardware and LaunchConfiguration("move_to_visual_ready_on_start").perform(context).lower() == "true":
            raise RuntimeError("plan_only must not automatically move real hardware")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "launch", "includes", name + ".launch.py"]
    ))) for name in ("visual_backend", "visual_lifecycle")]


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    motion_share = FindPackageShare("rebotarm_motion")
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
        "backend_profile": PathJoinSubstitution([bringup_share, "config", "visual_backend_profile.yaml"]),
        "strategy_profile": PathJoinSubstitution([vision_share, "config", "visual_strategy_profile.yaml"]),
        "motion_profile": PathJoinSubstitution([motion_share, "config", "visual_motion_profile.yaml"]),
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
            kwargs["choices"] = ["readonly", "plan_only", "execute"]
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_prepare_stages),
        ])
    )
    return LaunchDescription(declarations)
