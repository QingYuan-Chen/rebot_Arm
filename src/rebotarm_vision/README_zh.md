# rebotarm_vision

本包当前维护 Ubuntu 原生视觉链路：Gemini 2、YOLO、深度、GraspNet ROS
进程、候选过滤和视觉抓取执行接口。Windows HTTP/MJPEG/GraspNet 中转工具
以及对应的 PowerShell 启动脚本已从仓库删除。

## 当前文档

- 安装、模型和相机：`docs/setup/ubuntu_vision_setup_zh.md`
- 视觉启动、MuJoCo benchmark 和真机边界：`docs/reference/commands/visual_grasp_commands.md`
- 候选、IK、workspace、夹爪和执行参数：以 `src/rebotarm_bringup/launch/visual_grasp_system.launch.py` 和 `src/rebotarm_vision/config/` 当前源码为准
- 分进程 Python 解释器：`docs/setup/launch_python_configuration.md`

## V1.1 稳定性检查

V1.1 软件 benchmark 可执行 20 次候选/规划循环：

```bash
ros2 run rebotarm_vision rebotarm_visual_grasp_benchmark \
  --attempts 20 --return-ready-before-each --wait-enter
```

输出包含 `failed_stage`；它只用于软件和仿真稳定性分析，不代表真实抓取成功。

## Ubuntu 原生启动

```bash
cd /home/a/project/rebot_Arm
source scripts/source_local_environment.bash
./scripts/run_ubuntu_vision.sh \
  yolo_model_path:="$PWD/scripts/yolo26s-seg.pt" yolo_device:=0
```

该命令只启动相机/YOLO，不启动机械臂控制器。GraspNet 使用独立的
`.venv-graspnet` 进程并通过 ROS 话题接收同帧 RGB-D、CameraInfo 和检测结果。

## 配置边界

- `config/camera_ubuntu.yaml` 是本机 Gemini 2 配置。
- 视觉配置以 `config/camera_ubuntu.yaml` 为准；不再支持远端 HTTP 输入。
- `config/graspnet_ubuntu.yaml` 是 Ubuntu GraspNet 配置。
- 视觉候选必须经过 IK、workspace、碰撞和轨迹安全门，不能直接调用硬件 SDK。
- 真机执行前必须完成控制器反馈检查、显式 Enable、现场安全确认和单独授权。

缺少模型、相机或 GraspNet checkpoint 时，节点应 fail closed；不要通过复制旧
Windows 脚本、旧 HTTP JSON 或机器专属 engine 绕过当前 ROS 接口。
