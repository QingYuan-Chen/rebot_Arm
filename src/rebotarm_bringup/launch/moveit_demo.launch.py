"""MoveIt 独立演示/调试启动文件——只做规划与可视化，不接真机执行。

职责：在没有任何硬件与控制器动作的前提下，拉起一套最小可用的 MoveIt 规划环境，
用来在 RViz 里检查 URDF/SRDF、规划组、命名姿态、运动学插件与 OMPL 参数是否配好。
上层启动组合包（真机、示教、交互、仿真）都通过 IncludeLaunchDescription 复用它，
并用 launch 参数切换「假关节状态」与「真实关节状态」两种数据源。

节点组合（按启动顺序）：
1. tf2_ros 的 static_transform_publisher：发布 world -> base_link 静态变换，让模型
   在固定坐标系 world 下有确定的落位；
2. rebotarm_teleop 的夹爪可视化关节桥：把六轴关节状态与夹爪开口合成
   /<arm_namespace>/visual_joint_states，用于驱动 RViz 中左右手指 link；
3. joint_state_publisher：仅在 use_fake_joint_states=true 时启动，向
   /<arm_namespace>/joint_states 发布滑条式的假关节角（离线调试用）；
4. robot_state_publisher：订阅 /<arm_namespace>/visual_joint_states，计算并发布 TF；
5. move_group：MoveIt 规划节点，模型与参数全部来自本包 config/ 目录；
6. rviz2：按 rviz/moveit.rviz 打开 MotionPlanning 面板。

launch 参数（由上层启动组合包传入，命名必须与控制器/驱动一致）：
- use_rviz：是否启动 rviz2，默认 true；
- arm_namespace：话题命名空间，默认 rebotarm；
- use_fake_joint_states：true 时用 joint_state_publisher 造假的六轴关节角；接真机或
  接仿真时上层传 false，改为消费真实反馈；
- use_sim_time：是否使用 /clock 仿真时间，接仿真时必须为 true。

安全约束：本文件不下发任何轨迹执行目标，也不启用硬件；执行权限由控制器与上层启动
组合包掌握。这里的轨迹执行容差只用于判断规划结果能否被执行器接受，不放宽硬件保护。
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, relative_path):
    """读取已安装包内的 YAML 文件并返回解析结果。

    参数：
    - package_name：包名，只用于向 ament 索引查询安装目录（不是路径）；
    - relative_path：相对该包 share 目录的路径，例如 "config/ompl_planning.yaml"。

    返回值：`yaml.safe_load` 的结果，通常是 dict；文件缺失或 YAML 语法错误会直接抛异常，
    属于启动期配置错误，应当让启动失败而不是静默降级。

    注意：这里显式指定 encoding="utf-8"，因为配置文件里含中文注释，依赖系统默认编码会
    在非 UTF-8 locale 下解析失败。
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    """构造 MoveIt 演示的启动描述。

    返回值：launch.LaunchDescription，包含 4 个 launch 参数声明、一条规划参数日志、
    以及 6 个节点（静态 TF、夹爪可视化关节桥、可选假关节状态源、机器人姿态发布器、
    move_group、可选 rviz2）。

    副作用：会读取本包 config/ 下的 YAML（OMPL 规划参数）并在启动时打印规划插件概要；
    所有配置均以只读方式加载，不写回文件。
    """
    use_rviz = LaunchConfiguration("use_rviz")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_fake_joint_states = LaunchConfiguration("use_fake_joint_states")
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_share = FindPackageShare("rebotarm_moveit_config")
    rviz_config = PathJoinSubstitution([moveit_share, "rviz", "moveit.rviz"])

    # MoveItConfigsBuilder 按「包 share 目录 + 相对路径」加载模型与配置，并把它们整理成
    # move_group / rviz2 需要的参数树；文件名必须与 config/ 下实际文件一致，改名会直接
    # 导致启动期找不到文件。这里的 robot_description 是 URDF 文本本身（不是路径），
    # robot_description_semantic 是 SRDF 文本，二者同时下发给节点，MoveIt 才能建立规划组。
    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        # planning_scene_monitor 的 5 个 publish_* 开关决定 move_group 往
        # /monitored_planning_scene 上发布哪些增量（机器人描述、几何、状态、TF）；全部打开
        # 时 RViz 的运动规划插件无需自己拉取模型即可显示规划场景。
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        # 只启用 OMPL 一条规划流水线；其它流水线（如 Pilz）即使配置文件存在也不会加载，
        # 避免 RViz 里出现不可用的规划器选项。
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # 单独再读一次 OMPL 参数：MoveItConfigsBuilder 会把它们放进参数树，而普通 dict 更适合
    # 直接作为节点参数下发，也便于下面打印概要。
    ompl_planning_yaml = load_yaml(
        "rebotarm_moveit_config", "config/ompl_planning.yaml"
    )

    # 3D 传感器占位配置：列表里只放一个占位名，且 sensor_plugin 写成 "~"（无插件），
    # 等价于「已声明 3D 传感器配置但实际不接深度相机」，避免 move_group 因缺少该参数报错。
    sensors_3d = {
        "sensors": ["no_depth_sensor"],
        "no_depth_sensor": {"sensor_plugin": "~"},
    }

    # 轨迹执行校验参数（move_group 侧，单位与 MoveIt 官方定义一致）：
    # - moveit_manage_controllers=False：不允许 MoveIt 自己加载/启停控制器，控制器生命周期
    #   由外部控制器管理器负责，防止规划节点越权切换控制权；
    # - allowed_execution_duration_scaling=1.2：实际执行时长最多允许为规划时长的 1.2 倍；
    # - allowed_goal_duration_margin=0.5：在 1.2 倍基础上再放宽 0.5 s 的绝对余量；
    # - allowed_start_tolerance=0.01：轨迹起点与当前关节状态的最大允许偏差（弧度），
    #   超过即拒绝执行，避免从错误姿态直接插补导致机械臂猛动。
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    # 把 OMPL 的关键参数整理成一段文本，启动时用 LogInfo 打印。历史上 "Planning plugin name
    # is empty or not defined in namespace 'ompl'" 这类故障只表现为规划失败，这里把实际生效的
    # 插件、请求/响应适配器与规划器清单直接打到终端，便于对照配置文件排查。
    planning_debug_summary = yaml.safe_dump(
        {
            "planning_plugins": ompl_planning_yaml.get("planning_plugins"),
            "request_adapters": ompl_planning_yaml.get("request_adapters"),
            "response_adapters": ompl_planning_yaml.get("response_adapters"),
            "planner_configs": list(
                (ompl_planning_yaml.get("planner_configs") or {}).keys()
            ),
            "arm": ompl_planning_yaml.get("arm"),
        },
        sort_keys=False,
        allow_unicode=True,
    )

    # 参数默认值面向「离线调试」：假关节状态默认打开，即使真实控制器没启动，RViz/MoveIt
    # 也能显示并拖动模型；真机与仿真场景会显式覆盖 use_fake_joint_states=false 并传入
    # use_sim_time=true。
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("use_fake_joint_states", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            # 启动时把实际生效的 OMPL 规划参数打到终端，便于排查「规划失败但看不出原因」。
            LogInfo(msg="move_group planning params:\n" + planning_debug_summary),
            # 固定基座变换：world -> base_link。SRDF 里的 world_joint 描述的是同一段关系，
            # 这里发布真正的 TF，使 RViz 的 Fixed Frame 可以选择 world。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                output="screen",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # 夹爪可视化桥：URDF 用两个手指关节表达夹爪，而真实夹爪只有一路开口反馈，
            # 该节点把二者合成 visual_joint_states，只影响显示，不参与控制。
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace, "use_sim_time": use_sim_time}],
            ),
            # 假关节状态源：由滑条驱动，只用于离线观察模型。它发到 /<ns>/joint_states，
            # 与真实反馈话题分离，因此即使真机在运行也不会污染规划场景。
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="moveit_demo_joint_state_publisher",
                output="screen",
                condition=IfCondition(use_fake_joint_states),
                parameters=[
                    moveit_config.robot_description,
                    {"rate": 30.0, "use_sim_time": use_sim_time},
                ],
                remappings=[("/joint_states", ["/", arm_namespace, "/joint_states"])],
            ),
            # 机器人姿态发布器：订阅合成后的 /<ns>/visual_joint_states（含手指关节），
            # 据此发布各 link 的 TF；它不产生任何控制输出。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/joint_states", ["/", arm_namespace, "/visual_joint_states"])
                ],
            ),
            # move_group：MoveIt 规划节点，本身不直接驱动硬件。参数树 = MoveItConfigsBuilder
            # （URDF/SRDF/运动学/关节限位/控制器映射）+ OMPL 规划参数 + 上面的轨迹执行容差
            # + 3D 传感器占位配置；joint_states 重映射到合成后的 visual_joint_states，
            # 使规划起点带上夹爪手指关节，规划场景与 RViz 显示保持一致。
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                parameters=[
                    moveit_config.to_dict(),
                    ompl_planning_yaml,
                    trajectory_execution,
                    sensors_3d,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            # rviz2：按本包 rviz/moveit.rviz 打开 MotionPlanning 面板；节点侧再注入一份
            # 模型与规划参数，使面板在 move_group 就绪前也能显示机器人。use_rviz=false 时不启动。
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    ompl_planning_yaml,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
