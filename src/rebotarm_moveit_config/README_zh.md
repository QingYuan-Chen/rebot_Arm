# MoveIt 规划与执行说明

本包提供 reBotArm 当前使用的模型、规划配置和 RViz 资源。包结构及文件归属见
[包说明](README.md)，各项验证及当前待办见[项目状态](../../docs/reference/project_status.md)。

## 配置与执行关系

- `config/rebotarm.urdf` 是关节、几何和网格的权威模型；SRDF 定义 `arm` 和 `gripper` 两个独立规划组。
- `arm` 使用 `base_link -> end_link` 串联链和 KDL IK；命名姿态为 `home`、`safe_home`。
- `gripper` 包含左右手指，命名姿态为 `open`、`closed`；碰撞规则由 SRDF 管理。
- OMPL 默认使用 `RRTConnectkConfigDefault`，另提供 RRTstar 等配置。具体限位和规划参数读取本包 YAML。
- MoveIt 六轴执行映射到 `/rebotarm/follow_joint_trajectory`；夹爪映射到
  `/rebotarm/gripper/moveit_command`，按左指位移请求，后端转换为双指总开口。
- 真机及 MuJoCo 执行统一使用 `continuous`：motion 负责最终起点绑定、重定时及执行前准备，
  后端复核会话、代次、能力与场景后执行。接口见
  [P005 执行说明](../../docs/implemented/features/p005_prepared_trajectory.md)。

启动组合由 `rebotarm_bringup` 拥有。真机由唯一硬件控制器执行，MuJoCo 由唯一仿真后端执行；
同一机械臂命名空间只运行一个轨迹执行服务端。RViz 无硬件执行同样使用 MuJoCo 后端。

## 构建与离线查看

在仓库根目录、已安装 ROS 2 Jazzy 依赖的新终端执行：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --symlink-install --packages-up-to rebotarm_bringup
source install/setup.bash
ros2 launch rebotarm_bringup moveit_demo.launch.py use_rviz:=true
```

`moveit_demo.launch.py` 默认使用假关节状态，启动 TF、robot_state_publisher、MoveIt 和 RViz，
用于模型与规划查看。RViz 中检查 `arm`/`gripper` 组、命名姿态和规划轨迹显示。
准备服务和物理执行使用下列完整组合入口。

## 选择操作入口

| 用途 | 入口 |
| --- | --- |
| MuJoCo 物理执行、MoveIt 与 RViz | `mujoco_moveit_sim.launch.py` |
| RViz 末端拖动与 MuJoCo 执行 | `rviz_ee_drag_sim.launch.py` |
| 真机 MoveIt 规划与执行 | `moveit_hardware.launch.py` |
| 真机网页、示教与 MoveIt | `rebotarm_app.launch.py` |

所有入口属于 `rebotarm_bringup`。可复制命令、参数和操作顺序统一维护在
[功能命令参考](../../docs/reference/commands/rebotarm_feature_commands.md)及
[MuJoCo 使用说明](../rebotarm_simulation/README_mujoco.md)。

真机启动默认失能，执行前须确认现场安全和新鲜反馈，再显式使能。
规划场景、碰撞配置与实际装配和标定共同决定可执行范围；修改模型后同步检查 MuJoCo 生成资源。
