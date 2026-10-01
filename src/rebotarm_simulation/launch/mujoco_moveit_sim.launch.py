"""仿真后端与运动规划的集成启动文件。

用途
    以 MuJoCo 物理仿真作为唯一执行后端，配合运动规划配置包提供的规划演示启动，
    让规划出的轨迹在没有真实硬件、但带物理仿真的机器人上执行与验证。
    真实硬件通道在本文件中完全没有被拉起。

节点组合
    1. 仿真节点 ``rebotarm_mujoco_node``（属本仿真包）：按物理模型推进关节状态、
       发布关节状态并提供轨迹执行动作；此处以 ``backend="mujoco"``、
       ``headless=True`` 启动，模型指向本包自带的桌面场景
       ``models/rebotarm/scene.xml``（机器人 + 桌面 + 被操作物代理）。
    2. 运动规划配置包的演示启动 ``demo.launch.py``：拉起规划节点与可选可视化，
       使用同一命名空间与仿真时钟。

后端选择逻辑
    本文件不做后端分支：仿真节点的 ``backend`` 被固定为 ``"mujoco"``，参数不是该值时
    节点会拒绝启动，因此不存在"悄悄连上真机"的回退路径。

参数来源与安全默认值
    - ``use_fake_joint_states=false``：关节状态必须来自真实物理步进，不能用假发布器，
      否则规划会基于与仿真不一致的状态；
    - ``use_sim_time=true``：规划与执行统一使用仿真时钟，避免墙钟漂移影响轨迹判定；
    - ``use_rviz`` 默认 true，只影响可视化，不影响控制；
    - ``use_mujoco_viewer`` 默认 true，仅打开本地查看器；无显示环境应改用同包的无头
      启动文件；
    - ``python_executable`` 默认取环境变量 ``REBOTARM_MUJOCO_PYTHON``，否则使用
      PATH 中的 python3；该解释器必须装有物理引擎和 ROS 2 依赖，
      不依赖工作目录或虚拟环境目录布局。

安全说明
    启动本文件不会使能任何真实电机：真机使能始终需要显式指令与现场安全检查。
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 两个 share 目录：前者提供规划演示启动，后者提供本包自带的场景模型。
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    simulation_share = FindPackageShare("rebotarm_simulation")
    arm_namespace = LaunchConfiguration("arm_namespace")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    python_executable = LaunchConfiguration("python_executable")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    # 桌面场景：机器人在原点，附带桌面与可被抓取的物体代理，便于验证抓取流程。
    upstream_model = PathJoinSubstitution(
        [simulation_share, "models", "rebotarm", "scene.xml"]
    )

    return LaunchDescription(
        [
            # 命名空间：关节状态、动作与服务名都带该前缀，必须与规划侧一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 可视化开关：仅影响是否打开 RViz，不改变控制链路。
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # 统一使用仿真时钟；仿真推进与墙钟解耦，必须为 true。
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # 是否打开物理引擎原生查看器窗口，仅用于肉眼观察仿真姿态。
            DeclareLaunchArgument(
                "use_mujoco_viewer",
                default_value="true",
                description="Open a native MuJoCo viewer for visual inspection",
            ),
            # 仿真节点发布关节状态与推进物理的节拍（Hz），过高只是空耗 CPU。
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            # 默认使用已部署依赖的 python3，允许显式覆盖为其他解释器。
            DeclareLaunchArgument(
                "python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_MUJOCO_PYTHON",
                    default_value="python3",
                ),
                description=(
                    "Python interpreter containing MuJoCo and ROS 2 dependencies; "
                    "defaults to python3 on PATH"
                ),
            ),
            # 仿真执行后端节点：维护关节状态并提供轨迹执行动作。
            # backend 与 headless 在此写死，保证本启动文件只跑仿真、不触碰硬件。
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                prefix=python_executable,
                parameters=[
                    {
                        "backend": "mujoco",
                        "headless": True,
                        "show_viewer": use_mujoco_viewer,
                        "model_path": upstream_model,
                        "arm_namespace": arm_namespace,
                        "publish_rate_hz": publish_rate_hz,
                    }
                ],
            ),
            # 规划侧演示启动：共用命名空间与仿真时钟，并明确关闭假关节状态，
            # 让规划读取仿真节点发布的真实状态。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])
                ),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": "false",
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
