# 视觉抓取命令

> 状态：REFERENCE；类型：视觉运行命令；适用范围：只读视觉、plan-only、候选检查和仿真边界；不自动授权真机动作。

## 数据流

```text
Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/detections
-> local GraspNet -> candidate filters -> IK/collision gates
-> MoveIt plan -> MuJoCo 或明确授权的真机控制器
```

当前路线只支持 Ubuntu 原生视觉。Windows、HTTP、MJPEG、远程 JSON 和独立 GraspNet 服务不属于当前入口。

## 前置条件

```bash
cd /home/huangbin/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

只读视觉环境准备见 [Ubuntu 视觉环境配置](../../setup/ubuntu_vision_setup_zh.md)。需要 GraspNet 推理时，先设置已审核模型：

```bash
export GRASPNET_MODEL_ROOT=/absolute/path/to/graspnet-model-root
export GRASPNET_CHECKPOINT_PATH=/absolute/path/to/checkpoint-rs.tar
```

## 只读视觉

```bash
./tools/run_ubuntu_vision.sh
```

检查图像、检测和候选：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/color/camera_info --once
ros2 topic echo /grasp/detections --once
ros2 topic echo /grasp/graspnet_candidates --once
```

完整相机、YOLO 和 GraspNet 环境检查见 [Ubuntu 视觉只读测试](ubuntu_vision_readonly_test_zh.md)。

## plan-only 预览

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  execution_mode:=plan_only \
  execute_gripper:=false \
  move_to_visual_ready_on_start:=false
```

检查过滤后的候选和计划：

```bash
ros2 topic echo /grasp/filtered_candidates --once
ros2 topic echo /grasp/filtered_plan --once
```

触发一次 plan-only 预览：

```bash
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

服务会规划预抓取、接近、抓取和可选撤退阶段，并把完整轨迹发布到 RViz；`plan_only` 不向控制器 Action 或夹爪发送动作。

`plan_only` 只验证软件规划，不发送真机轨迹，也不能证明真实相机安装或真实抓取安全。

## 仿真执行

```bash
ros2 launch rebotarm_simulation mujoco_moveit_sim.launch.py use_rviz:=true
```

在 RViz 中先 `Plan` 再 `Execute`。仿真 Action 应保持唯一：

```bash
ros2 action list | grep follow_joint_trajectory
```

## 真机执行

启动真机视觉执行入口：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  channel:=/dev/ttyACM0 \
  execution_mode:=execute \
  vision_profile:=ubuntu_native \
  start_sim_trajectory_controller:=false \
  move_to_visual_ready_on_start:=false \
  start_visual_grasp_executor:=true \
  start_motion_execution:=true \
  execute_gripper:=true \
  shutdown_safe_home:=false
```

启动后先确认控制器和 MoveIt 状态：

```bash
ros2 topic echo --once /rebotarm/arm_status
ros2 topic echo --once /rebotarm/joint_states
ros2 action list | grep follow_joint_trajectory
ros2 topic echo /grasp/filtered_plan --once
```

完成现场净空、支撑、急停和轨迹预览检查后，显式使能：

```bash
ros2 service call /rebotarm/enable std_srvs/srv/Trigger "{}"
```

确认当前位置保持和最新抓取计划有效后，才触发一次完整视觉抓取：

```bash
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

需要停止当前视觉抓取时：

```bash
ros2 service call /rebotarm/visual_grasp/stop std_srvs/srv/Trigger "{}"
```

停止后确认机械臂静止，再按现场流程受控回位并失能：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

真机视觉执行前必须先完成一次 `plan_only` 预览。视觉候选、规划或仿真接触通过都不等于真实抓取成功；健康但可恢复的失败不得直接把机械臂失能在未知姿态。

## 相关文档

- 功能说明：[视觉抓取规划与执行编排](../../implemented/features/visual_grasp.md)；
- 只读功能：[Ubuntu 原生视觉只读链路](../../implemented/features/vision_readonly.md)；
- 命令索引：[commands/README.md](README.md)。
