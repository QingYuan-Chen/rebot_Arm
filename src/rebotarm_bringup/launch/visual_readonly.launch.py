"""只读视觉入口：相机/YOLO/TCP TF、GraspNet 候选和可视化。

该入口不启动 MoveIt、IK、轨迹执行、夹爪或真实硬件。它适合检查 RGB-D、检测和候选
数据流；需要规划或执行时使用同目录的 ``visual_plan_only`` / ``visual_execute``。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vision_share = FindPackageShare("rebotarm_vision")
    bringup_share = FindPackageShare("rebotarm_bringup")
    camera = LaunchConfiguration("camera_config")
    handeye = LaunchConfiguration("handeye_config")
    python = LaunchConfiguration("vision_python_executable")
    graspnet_python = LaunchConfiguration("graspnet_python_executable")
    candidates = LaunchConfiguration("candidates_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=PathJoinSubstitution([vision_share, "config", "camera_ubuntu.yaml"]),
            ),
            DeclareLaunchArgument(
                "handeye_config",
                default_value=PathJoinSubstitution([vision_share, "config", "handeye.yaml"]),
            ),
            DeclareLaunchArgument("vision_python_executable", default_value="python3"),
            DeclareLaunchArgument("graspnet_python_executable", default_value="python3"),
            DeclareLaunchArgument("graspnet_model_root", default_value=""),
            DeclareLaunchArgument("graspnet_checkpoint_path", default_value=""),
            DeclareLaunchArgument("graspnet_device", default_value="cuda:0"),
            DeclareLaunchArgument("candidates_topic", default_value="/grasp/graspnet_candidates"),
            DeclareLaunchArgument("start_open3d_viewer", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
                ),
                launch_arguments={
                    "camera_config": camera,
                    "handeye_config": handeye,
                    "vision_python_executable": python,
                }.items(),
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_graspnet_baseline_node",
                name="rebotarm_graspnet_baseline_node",
                output="screen",
                prefix=graspnet_python,
                parameters=[
                    PathJoinSubstitution([vision_share, "config", "graspnet_ubuntu.yaml"]),
                    {
                        "output_candidates_topic": candidates,
                        "model_root": LaunchConfiguration("graspnet_model_root"),
                        "checkpoint_path": LaunchConfiguration("graspnet_checkpoint_path"),
                        "device": LaunchConfiguration("graspnet_device"),
                    },
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_grasp_candidate_markers",
                name="rebotarm_grasp_candidate_markers",
                output="screen",
                parameters=[
                    {"input_topic": candidates, "output_topic": "/grasp/raw_candidate_markers"}
                ],
            ),
            Node(
                package="rebotarm_vision",
                executable="rebotarm_graspnet_open3d_viewer",
                name="rebotarm_graspnet_open3d_viewer",
                output="screen",
                prefix=graspnet_python,
                condition=IfCondition(LaunchConfiguration("start_open3d_viewer")),
                parameters=[{"input_candidates_topic": candidates}],
            ),
        ]
    )
