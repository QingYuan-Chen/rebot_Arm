"""无桌面窗口的仿真集成启动文件：只跑物理仿真与运动规划，不开任何图形界面。

用途
    面向服务器、CI 与远程终端：复用 bringup 包的集成启动流程，但把显示开关覆盖为
    ``use_rviz="false"`` 与 ``use_mujoco_viewer="false"``，因此在没有显示环境时
    也能稳定运行，不会因为打不开窗口而失败。

组合与参数覆盖
    本文件不直接声明节点，只引用 bringup 包的集成启动文件；后端固定为物理仿真，
    不涉及任何真实硬件通道。

安全说明
    关闭窗口只是去掉观测手段，仿真执行与规划行为不受影响，也不会使能真实电机。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    # 仅关闭两个显示开关，其余参数沿用集成启动文件的安全默认值。
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "mujoco_moveit_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "use_mujoco_viewer": "false",
                }.items(),
            )
        ]
    )
