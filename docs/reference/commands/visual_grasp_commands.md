# 视觉候选命令

> 状态：REFERENCE；类型：相机诊断、候选筛选和显示命令。

当前数据流为 D405 → YOLO → ROS RGB-D/CameraInfo/检测 → 本地 GraspNet → 候选筛选 → 筛选结果与 Marker。环境准备见 [Ubuntu 视觉配置](../../setup/ubuntu_vision_setup_zh.md)。

相机更换后需重标定；默认 `vision_handeye_config` 为空，机器人系筛选缺 TF 时拒绝候选。
下列完整筛选命令需传入经过核验的 D405 手眼文件。独立相机/YOLO 可直接启动。

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export GRASPNET_MODEL_ROOT=/absolute/path/to/graspnet-model-root
export GRASPNET_CHECKPOINT_PATH=/absolute/path/to/checkpoint-rs.tar
```

## 独立离线筛选

默认启动被动关节状态、模型、TF、MoveIt 查询、感知与候选显示。被动状态源使用默认姿态。

```bash
ros2 launch rebotarm_bringup visual_perception_system.launch.py \
  robot_context:=offline start_open3d_viewer:=true \
  vision_handeye_config:=/absolute/path/to/d405_handeye.yaml
ros2 topic echo /grasp/filtered_plan --once
ros2 topic echo /grasp/filtered_candidates --once
ros2 topic echo /grasp/visual_markers --once
```

## 接入已有机器人环境

先独立启动所需的 MoveIt、TF 和关节状态来源，再启动视觉节点。MuJoCo 环境使用仿真时钟：

```bash
ros2 launch rebotarm_bringup visual_perception_system.launch.py \
  robot_context:=external use_sim_time:=true \
  vision_handeye_config:=/absolute/path/to/d405_handeye.yaml
```

接入使用系统时钟的真机环境时设置 `use_sim_time:=false`。确保每个机器人环境的关节状态、TF 和时钟来源一致。候选检查依赖新鲜关节状态、传感器时间 TF、`/compute_ik` 和 `/check_state_validity`。

相机和模型独立诊断见 [Ubuntu 视觉只读测试](ubuntu_vision_readonly_test_zh.md)。视觉总入口使用 `robot_context` 选择离线或外部环境。独立运动、夹爪位置开合及仿真命令见[命令索引](README.md)。
