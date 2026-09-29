# reBotArm ROS 2

面向 reBot B601 的 ROS 2 Jazzy 工作空间，包含机械臂与夹爪控制、MoveIt 规划、
网页遥操作、示教、RGB-D 视觉抓取和 MuJoCo 仿真。

## 从这里开始

| 内容 | 入口 |
| --- | --- |
| 环境、依赖与 MotorBridge 安装 | [本机环境说明](docs/setup/ubuntu_ros2_jazzy.md) |
| 网页遥操作、RViz 与实机操作 | [功能命令参考](docs/reference/commands/rebotarm_feature_commands.md) |
| 控制器接口 | [ROS SDK 说明](README_zh.md) |
| 仿真 | [MuJoCo 使用说明](src/rebotarm_simulation/README_mujoco.md) |
| Gymnasium / PPO 强化学习 | [训练与评估命令](docs/reference/commands/mujoco_rl.md) |
| 包职责与依赖边界 | [架构说明](docs/implemented/architecture.md) |
| 当前阶段及验收依据 | [Agent 状态](Agent/README.md) |
| 其他文档 | [文档索引](docs/README_zh.md) |

## 工程结构

- `src/`：13 个分层 ROS 2 包，硬件、规划、示教、交互、视觉和仿真分别归属。
- `tools/`：环境安装、验证和离线工具。
- `tests/`：软件回归与架构检查。
- `patches/`：固定版本的 MotorBridge 反馈与校零安全补丁。
- `Agent/`：实施状态、决策和验收记录；历史完成度不等于新的真机动作授权。
- `third_party/`：仅本机参考快照、SDK 和虚拟环境；不纳入版本管理。
- `star_arm_102_rebot_b601_follow/`：独立 Star Arm 跟随工具，按其自身说明使用。

标准本机构建目录为 `build/`、`install/`、`log/`，不提交到 Git。
模型下载、TensorRT engine、本地运行数据及额外实验原始证据也不自动纳入提交。
当前仍随仓库提供 `tools/yolo26s-seg.pt`；历史比较副本仅本机留存。
正式仿真资源及其来源见 [第三方说明](THIRD_PARTY_NOTICES.md)。

## 环境与构建

基线为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。先按
[安装说明](docs/setup/ubuntu_ros2_jazzy.md) 准备系统依赖和所需运行组件；
`requirements-*.txt` 按控制器、视觉、GraspNet、MuJoCo、TensorRT 拆分；
`requirements-rl.txt` 在 MuJoCo 环境中追加 GPU 强化学习依赖。其它用途的依赖
不要合并安装到同一个环境。

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --base-paths src --executor sequential --symlink-install
source tools/source_local_environment.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
```

在未激活其他虚拟环境的新终端构建。源码构建不要求相机、GPU、TensorRT engine
或 GraspNet 权重；视觉依赖安装脚本不再重建工作区。检测启动时才检查模型，
可用现有 `.pt` 或显式指定本机 engine，见 [视觉安装与模型准备](docs/setup/ubuntu_vision_setup_zh.md)。
运行时按节点设置 [解释器参数](docs/setup/launch_python_configuration.md)，不要全局注入视觉依赖。
完整回归还需要已构建的 ROS 消息及固定厂商 SDK；纯源码构建通过不代表真机验收。

视觉只支持 Ubuntu 原生 Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/detections -> 本机
GraspNet。Windows、HTTP、MJPEG、远端 JSON 和独立 GraspNet service 已删除。
MoveIt Execute 还需要 `ros-jazzy-moveit-simple-controller-manager`；Plan 成功不等于
Execute 成功。真机启动默认失能，必须在新鲜反馈和现场安全检查后显式 Enable。

## 真机边界

连接不等于使能；运动必须明确授权并显式使能。串口只允许一个控制器占用。
主控制器要求 `motorbridge 0.4.7+rebotarm.1`、逐电机反馈序号能力及上游
`dm-serial` 10 ms 读写超时预算，启动前运行：

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/setup_motorbridge_fresh_feedback.py --check-installed
```

可恢复的任务失败应停止运动并保持使能，或受控回到已验证基线后再失能；
不要用批量结束进程代替安全停机。软件测试通过不代表本轮完成了硬件验收。

## 许可

项目许可见 [LICENSE](LICENSE)，第三方资产分别遵守其来源与许可记录。
