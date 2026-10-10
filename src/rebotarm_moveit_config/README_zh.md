# rebotarm_moveit_config

这是 `reBotArmController_ROS2-main` 中给 MoveIt2 预留的配置包。

当前阶段目标不是一次性把完整 MoveIt 系统全部跑通，而是先把结构搭对，让后续可以逐步接入：

- `SRDF` 机械臂语义模型
- `kinematics.yaml` IK 求解器配置
- `joint_limits.yaml` 关节速度/加速度限制
- `ompl_planning.yaml` 规划器参数
- `moveit_controllers.yaml` 控制器映射
- `moveit.rviz` 调试布局；启动入口 `moveit_demo.launch.py` 位于 `rebotarm_bringup`

## 当前状态

当前已经完成：

- 建立 `arm` 规划组
- 使用 `base_link -> end_link` 串联链
- 预置 `home` / `zero` 两个命名姿态
- 配置了 KDL IK 求解器
- 配置了基础 OMPL 规划器骨架
- 配置了 6 轴 FollowJointTrajectory 控制器占位映射
- 配置了第一版 `move_group` 启动入口
- 已切换为官方风格 `MoveItConfigsBuilder` 启动组织方式
- 已补齐更接近官方 MoveIt config 包的标准命名文件骨架
- 已补 joint_state_publisher，支持离线无实机状态发布

当前还未完成：

- 完整自碰撞禁用矩阵
- 和现有 `PreviewNode` 的 MoveIt 后端联调
- 和真实执行层的轨迹桥接

## 推荐推进顺序

1. 使用 `rebotarm_bringup/moveit_demo.launch.py` 启动 MoveIt `move_group`
2. 在 Ubuntu / ROS2 Jazzy 中验证：
   - 模型能加载
   - `arm` group 能识别
   - `home` / `zero` 姿态能显示
3. 再把 `PreviewNode` 改成支持双后端：
   - 现有 SDK 预览后端
   - 新的 MoveIt IK / 可达性后端
4. 最后再接轨迹规划和真实执行桥接

## 设计原则

这里遵循当前项目已经确定的路线：

- 先稳定 RViz 交互层
- 再逐步引入 MoveIt2
- 不直接推翻现有 Python SDK 控制链
- 保持“仿真/预览”和“真机执行”分层

## Ubuntu 验证步骤

下面这组步骤用于验证当前 MoveIt 基础层是否正常。

注意：

- 当前正在做的是“标准 MoveIt 配置包重整”
- 这一阶段的目标是先让 MoveIt 配置包结构正确
- 如果仍有报错，优先关注 `MoveItConfigsBuilder` 是否还在报推断/配置相关问题

### 第 1 步：编译

在 Ubuntu 工作区执行：

```bash
cd ~/robotarm_ros2
colcon build --symlink-install --packages-select rebotarm_moveit_config rebotarm_bringup
source install/setup.bash
```

验证方式：

- 终端没有 `error` 中断
- 能找到包：

```bash
ros2 pkg list | grep rebotarm_moveit_config
```

### 第 2 步：启动 MoveIt 基础演示

```bash
ros2 launch rebotarm_bringup moveit_demo.launch.py use_rviz:=true
```

验证方式：

- 终端里能看到：
  - `robot_state_publisher` 启动
  - `joint_state_publisher` 启动
  - `move_group` 启动
  - `rviz2` 启动
- 不应出现 `robot_description_semantic` 缺失、`arm` group 不存在、SRDF 解析失败等报错
- 不应再出现 `Planning plugin name is empty or not defined in namespace 'ompl'`

### 第 3 步：检查节点是否存在

新开一个终端，执行：

```bash
source ~/robotarm_ros2/install/setup.bash
ros2 node list | grep -E "move_group|robot_state_publisher|rviz"
```

验证方式：

- 至少应看到：
  - `/move_group`
  - `/joint_state_publisher`
  - `/robot_state_publisher`
  - `/rviz2`

### 第 4 步：检查 MoveIt 核心话题

```bash
ros2 topic list | grep -E "planning_scene|monitored_planning_scene|display_planned_path"
```

验证方式：

- 至少应该看到和规划场景相关的话题
- 如果完全没有，说明 `move_group` 没真正进入 MoveIt 工作状态

### 第 5 步：检查 RViz 中模型和规划插件

验证方式：

- RViz 中应能看到机械臂模型
- `Fixed Frame` 应能切到 `base_link` 或 `world`
- 如果没有 `MotionPlanning` 面板，可手动添加：
  - `Panels -> Add New Panel -> MotionPlanning`

### 第 6 步：检查规划组

在 RViz 的 `MotionPlanning` 面板里验证：

- Planning Group 能看到 `arm`
- Goal State 中能看到 `home` 和 `zero`

如果看不到，说明 `SRDF` 没被 MoveIt 正确加载
