# 标准真机启动文件:机械臂控制器,MoveIt 场景/准备服务,状态与可选 RViz.
#
# 节点组合:
#   1. reBotArmController(硬件包)-真实电机通信与轨迹执行;
#   2. GripperVisualJointStateNode(操作交互包)-把夹爪状态桥接成可视化关节状态;
#   3. robot_state_publisher -依据 URDF 发布 TF;
#   4. move_group 与轨迹准备节点 -提供 continuous 规划,场景复核与准备服务;
#   5. rviz2 -仅当 use_rviz=true 时启动.
#
# 注意:本文件不含任何运动规划或控制算法,只做启动组合与参数传递.
#
# 真实/仿真后端选择逻辑:本文件是"真实硬件"入口,唯一包含的就是硬件控制器节点,没有仿真分支,也不会启动仿真轨迹控制器;
# 因此与仿真执行后端互斥,任何时候不要与仿真启动文件同时拉起同名的关节状态与轨迹接口.
# bringup.launch.py = hardware_controller.launch.py + moveit_demo.launch.py 的规划,准备与状态链路.
# 参数来源与安全默认值:
#   - arm_config / gripper_config 默认指向启动组合包 config/ 下的 YAML;
#   - channel 为空串表示沿用配置文件里的通道,不在这里硬编码设备名;
#   - cmd_arbitration 默认 "reject":轨迹运行期间收到单关节透传指令时直接拒绝,是安全优先的一侧;只有明确需要抢占时才改成 "preempt";
#   - use_rviz 默认 false,不带可视化也能跑;
#   - 真机上电后控制器处于失能态,必须显式调用 enable 服务才会运动,本文件不做任何自动使能.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")

    return LaunchDescription(
        [
            # arm_config:机械臂(六关节)硬件配置文件,含总线通道,电机 ID 与厂商整定增益;默认取启动组合包内的 config/arm.yaml.
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # gripper_config:夹爪硬件配置文件(电机 ID,行程映射与力矩上限);与arm_config 分文件维护,便于两条机构分别整定.
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # channel:电机总线通道覆盖值(如 /dev/ttyACM0 或 can0);空串表示用配置文件里的 channel,不在此硬编码设备名.
            DeclareLaunchArgument("channel", default_value=""),
            # joint_state_rate(Hz):状态发布频率,默认 100 Hz;设备采集周期由控制器配置独立管理.
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # cmd_arbitration:轨迹运行期间收到单关节透传指令时的仲裁策略.默认 "reject"直接拒绝(安全优先);"preempt" 会先停下轨迹再执行透传指令.
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # arm_namespace:话题/服务/动作命名空间段,去首尾斜杠后拼成 /{ns}/....
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # use_rviz:是否启动 RViz.默认 false,无可视化也能完成启动与执行.
            DeclareLaunchArgument("use_rviz", default_value="false"),
            # frame_id / ee_frame_id:对外位姿的参考坐标系与末端坐标系名,需与 URDF 中的 link 名一致,否则 TF 查询与可视化会失败.
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            # 唯一硬件底层片段;本文件叠加规划,准备,状态发布与可视化.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "hardware_controller.launch.py"]
                    )
                ),
                launch_arguments={
                    "arm_config": arm_config,
                    "gripper_config": gripper_config,
                    "channel": channel,
                    "joint_state_rate": joint_state_rate,
                    "cmd_arbitration": cmd_arbitration,
                    "arm_namespace": arm_namespace,
                    "frame_id": frame_id,
                    "ee_frame_id": ee_frame_id,
                }.items(),
            ),
            # 规划,准备,状态和可视化由同一 MoveIt 组合提供.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([bringup_share, "launch", "moveit_demo.launch.py"])),
                launch_arguments={"arm_namespace": arm_namespace, "use_rviz": use_rviz,
                    "use_fake_joint_states": "false", "use_sim_time": "false"}.items()),
        ]
    )
