# rebotarm_moveit_config

MoveIt 2 模型与规划配置包。它只安装模型、规划参数、网格和 RViz 布局；启动组合由 `rebotarm_bringup` 负责。现有的中文操作说明见 [`README_zh.md`](README_zh.md)。

## 目录结构

```text
rebotarm_moveit_config/
├── config/
│   ├── rebotarm.urdf             # 规范机器人模型与 mesh URI
│   ├── rebotarm.srdf             # arm 规划组、命名姿态、碰撞语义
│   ├── reBot-DevArm_fixend.srdf  # 兼容/参考语义模型
│   ├── kinematics.yaml           # IK 求解器
│   ├── joint_limits.yaml         # 关节速度/加速度/jerk 限制
│   ├── ompl_planning.yaml        # OMPL 规划器
│   ├── pilz_cartesian_limits.yaml# Pilz 笛卡尔限制配置
│   ├── moveit_controllers.yaml   # FollowJointTrajectory 控制器映射
│   ├── moveit_cpp.yaml           # MoveIt 通用参数
│   └── sensors_3d.yaml           # 3D 传感器占位/配置
├── meshes/                       # URDF 引用的 STL 网格
├── rviz/moveit.rviz              # MotionPlanning RViz 配置
├── setup.py / package.xml / resource/*
```

## Bringup 演示入口

启动参数为 `use_rviz`、`arm_namespace`、`use_fake_joint_states` 和 `use_sim_time`。启动组合包括：

1. `world -> base_link` 静态 TF；
2. `rebotarm_teleop/GripperVisualJointStateNode`，把夹爪开口转换为左右手指显示关节；
3. 可选的 `joint_state_publisher` 假状态源；
4. `robot_state_publisher`；
5. MoveIt `move_group`；
6. 可选 `rviz2` MotionPlanning 面板。

```bash
ros2 launch rebotarm_bringup moveit_demo.launch.py use_rviz:=true
```

`moveit_demo.launch.py` 位于 `rebotarm_bringup`。这个入口用于规划/可视化检查，不会自动连接真机。真实硬件或 MuJoCo 的执行后端由启动组合选择，必须确保同名 `FollowJointTrajectory` 只有一个服务端。

## 配置职责

- `rebotarm.urdf` 是几何、关节和 mesh URI 的权威模型；修改后需要同步检查仿真生成的 MJCF。
- `rebotarm.srdf`、`kinematics.yaml`、`ompl_planning.yaml` 决定规划组、IK 和规划管线。
- `moveit_controllers.yaml` 只描述控制器映射，不负责启动控制器。
- 本包不应加入电机控制、视觉策略、示教文件管理或 Dashboard 逻辑。
