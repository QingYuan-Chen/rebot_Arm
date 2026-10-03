# 视觉抓取命令

> 状态：REFERENCE；类型：视觉运行命令；适用范围：相机诊断、候选检查、仿真及真机执行边界；不自动授权真机动作。

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

## 仿真视觉执行

初始姿态为 safe_home（joint3 为 −1°，其余关节为 0°）。唯一视觉入口固定执行流程，
无需 execution_mode 或 ready 参数；启动后等待 execute 服务调用，不自动开始抓取。

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=false \
  start_open3d_viewer:=true
```

检查 `/grasp/filtered_plan` 为新鲜有效计划，再触发仿真抓取：

```bash
ros2 topic echo /grasp/filtered_plan --once
ros2 service call /rebotarm/visual_grasp/execute std_srvs/srv/Trigger "{}"
```

这会实际驱动仿真轨迹控制器，不是只显示幻影。使用真实相机输入，不验证物体接触和
真实夹持。仿真姿态须与相机对应的 TF 合理匹配；仿真通过不代表真机安全验收。
主入口不再接受 plan_only/readonly；旧命令会明确报错，避免静默切换为执行。

稳定性测试还应保留候选过滤参数，例如
`candidate_max_joint6_delta_rad:=1.5708` 和
`candidate_joint6_symmetry_enabled:=true`；涉及夹爪时明确设置
`gripper_grasp_enabled:=false`。涉及真机或多次尝试时，使用
`/rebotarm/visual_grasp/execute` 并记录每次的 `failed_stage`。
普通重复抓取直接调用 execute，不附带自动回位。

## 真机执行

先结束仿真 launch，再启动真机视觉执行入口；两套不要同时运行。
真机启动不自动使能或抓取，也不自动回 safe_home：

```bash
ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  use_hardware:=true \
  channel:=auto \
  execute_gripper:=true \
  start_open3d_viewer:=true
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

停止服务有界等待旧执行请求退出、动作终态、控制器停止响应和新鲜关节静止反馈。
返回 `success=True` 才表示停止已确认；机械臂保持当前位置，不自动回 safe_home。
普通流程不再提供 `/visual_grasp/reset`。下一次有有效计划时直接调用 execute。
如果停止返回失败，执行入口保持阻断；排查反馈、控制器和未结束的请求，解决后再次调用 stop。
不要用重启进程绕过停止确认。修改代码后须重建并重启 launch，已运行的 Python 节点不会自动加载修复。

停止后确认机械臂静止，再按现场流程受控回位并失能：

```bash
ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
```

先完成仿真执行检查；退出仿真后启动真机，核对真实起点、目标和现场净空。真机 execute 会重新规划并进行轨迹预检。视觉候选、规划或仿真接触通过都不等于真实抓取成功；健康但可恢复的失败不得直接把机械臂失能在未知姿态。

## 相关文档

- 功能说明：[视觉抓取规划与执行编排](../../implemented/features/visual_grasp.md)；
- 只读功能：[Ubuntu 原生视觉只读链路](../../implemented/features/vision_readonly.md)；
- 命令索引：[commands/README.md](README.md)。

## 再次抓取与计划过期

抓取不强制回观察位。
正常抓取完成后，可直接再次调用 execute；相机仍需看到目标，当前姿态必须通过
TF、IK、碰撞和实际轨迹规划检查。执行请求在缓存缺失/过期时最多等待
`service_timeout_sec`（节点默认 20 秒）的新有效计划，可用 stop 中断等待。

`/grasp/filtered_plan` 持续发布不等于可执行：检查 `valid`、`reason` 和采集时间戳。
execute 模式默认 `max_plan_age_sec=4.0`。新消息到达时已超期会撤销旧缓存，超时响应
会包含 `age_sec`、`max_plan_age_sec` 或最新无效原因；不要修改消息时间戳或关闭新鲜度门槛。
若仍超时，应根据该原因检查感知/候选过滤耗时、目标可见性和时钟。


## 停止确认与独立位姿操作

`safe_home` 是操作员独立命令，不属于抓取或停止恢复流程。
运动层停止确认默认超时 8 秒；关节反馈须覆盖至少 0.3 秒、至少三个不同时间戳，
最新反馈不超过 0.5 秒，六轴速度绝对值不超过 0.03 rad/s、各轴位置跨度不超过 0.01 rad。
真机还要求 2 秒内的无故障 IDLE 状态。上述软件阈值已经过仿真测试，尚不代表真机停稳验收。
视觉层还会等待旧服务响应排空，再取得一次运动层停止确认，避免迟到的旧请求重新启动运动。
停止中或确认失败后不会自动重试抓取或执行退避运动。
