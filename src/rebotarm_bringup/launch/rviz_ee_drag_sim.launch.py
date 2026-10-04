#纯仿真下的 RViz 末端拖动入口（不接真实硬件）。（已测）
#
#用途：在 RViz 里用 MoveIt 原生的交互标记拖动机械臂末端，验证规划与执行链路，全程不打开任何硬件通道，也不依赖真实电机反馈。
#
#启动组合：
#    1. 本包内的交互系统共享启动文件（真机 / 仿真二选一的后端选择逻辑都在其中），这里显式选择「仿真预览」一侧；
#    2. 仿真轨迹控制器节点，顶替真机执行层担任 ”follow_joint_trajectory“ 动作服务端，并按固定频率发布关节状态，让 RViz 里的模型真正动起来。
#
#后端选择逻辑："use_hardware=false"关闭真机通道，"use_moveit_preview=true" 让共享启动文件包含规划包自带的预览栈（提供规划能力与假关节状态），因此本入口只能用于可视化与上层流程联调，不构成对真实控制器的验证。
#
#安全默认值："start_passive_joint_state_publisher=false" 且 "use_moveit_fake_joint_states=false"，即不再引入额外的假关节状态源——仿真轨迹控制器是本入口唯一的关节状态发布者，同时也是规划执行所依赖的动作服务端；两个状态源同时存在会让规划器看到互相矛盾的关节状态。


from __future__ import annotations

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构建本入口的 LaunchDescription：包含交互系统启动文件，并追加仿真轨迹控制器。"""
    bringup_share = FindPackageShare("rebotarm_bringup")
    return LaunchDescription(
        [
            # 共享交互系统启动文件：真机侧需要显式传串口通道，仿真侧不需要
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "interactive_system.launch.py"]
                    )
                ),
                # use_hardware=false：不启动真机控制器节点，串口通道保持关闭；
                # use_local_rviz=true：由共享启动文件拉起 RViz（末端拖动界面所在）；
                # use_moveit_preview=true：使用规划包自带的预览栈提供规划能力；
                # start_passive_joint_state_publisher=false：抑制额外的假关节状态源；
                # use_moveit_fake_joint_states=false：规划器关节状态改由仿真轨迹控制器提供。
                launch_arguments={
                    "use_hardware": "false",
                    "use_local_rviz": "true",
                    "use_moveit_preview": "true",
                    # The simulated trajectory controller is the only joint-state
                    # publisher for this entrypoint and provides MoveIt's action server.
                    "start_passive_joint_state_publisher": "false",
                    "use_moveit_fake_joint_states": "false",
                }.items(),
            ),
            # 仿真执行后端：发布关节状态并提供规划执行所需的动作服务端；
            # arm_namespace 与共享启动文件中的命名空间保持一致，否则话题/动作前缀对不上
            Node(
                package="rebotarm_preview",
                executable="rebotarm_sim_trajectory_controller",
                name="rebotarm_sim_trajectory_controller",
                output="screen",
                parameters=[{"arm_namespace": "rebotarm"}],
            ),
        ]
    )
