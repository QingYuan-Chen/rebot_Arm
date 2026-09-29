"""带桌面窗口的仿真集成工作区启动文件：同时打开物理查看器与 RViz。

用途
    面向有显示器的开发机：在"仿真后端 + 运动规划"的集成启动之上，显式打开可视化
    窗口，便于观察规划出的轨迹在仿真中的执行过程。它只改变显示相关开关，不改变
    控制链路，也不会拉起任何真实硬件通道。

组合与参数覆盖
    本文件不直接声明节点，而是引用 bringup 包的集成启动文件，并把显示开关覆盖为
    ``use_rviz="true"`` 与 ``use_mujoco_viewer="true"``：前者是规划侧可视化，
    后者是物理引擎的原生查看器。两者都只是观测手段；无显示环境（服务器、CI）
    应改用 bringup 包的无头启动文件。

安全说明
    可视化开关与硬件使能完全无关：打开本文件不会使能任何真实电机。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    # 仅覆盖两个显示开关，其余参数沿用集成启动文件的安全默认值
    # （仿真时钟、真实关节状态、固定仿真后端）。
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "mujoco_moveit_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_rviz": "true",
                    "use_mujoco_viewer": "true",
                }.items(),
            )
        ]
    )
