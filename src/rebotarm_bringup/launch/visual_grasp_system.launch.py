"""正式视觉抓取总组合入口：统一公共参数，分域 YAML 提供策略默认值。"""

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
    from pathlib import Path
    from rebotarm_bringup.visual_profiles import read_section

    # Reject old preview commands before loading any backend; never silently execute them.
    requested = context.launch_configurations.get("execution_mode", "execute")
    if requested != "execute":
        raise RuntimeError("execution_mode is retired: this entry executes on the selected backend. "
                           "Use use_hardware=false for simulation; scripts/run_ubuntu_vision.sh for camera diagnostics")
    context.launch_configurations["execution_mode"] = "execute"

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
        "max_plan_age_sec": "4.0",
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
    required = {
        "start_candidate_ik_filter": True,
        "start_motion_execution": True,
        "start_visual_grasp_executor": True,
        "start_sim_trajectory_controller": not hardware,
    }
    for key, enabled in required.items():
        context.launch_configurations[key] = str(enabled).lower()

    if hardware and LaunchConfiguration("use_sim_time").perform(context).lower() == "true":
        raise RuntimeError("Real hardware requires use_sim_time=false")
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
        "use_local_rviz": "true",
        "start_vision": "true",
        "start_graspnet_baseline": "true",
        "execute_gripper": "false",
        "start_visual_grasp_markers": "true",
        "start_raw_candidate_markers": "true",
        "start_open3d_viewer": "false",
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
        declarations.append(DeclareLaunchArgument(name, **kwargs))
    declarations.append(
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_prepare_stages),
        ])
    )
    return LaunchDescription(declarations)
