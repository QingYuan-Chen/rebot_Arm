# 维护工具

本目录只放环境安装、启动和诊断脚本。命令从仓库根目录执行，完整步骤见
[环境安装说明](../docs/setup/ubuntu_ros2_jazzy.md)与
[视觉安装说明](../docs/setup/ubuntu_vision_setup_zh.md)。

| 工具 | 用途 |
| --- | --- |
| `setup_ubuntu_vision.sh` | 安装视觉依赖到 `.venv-vision` |
| `setup_ubuntu_graspnet.sh` | 安装 GraspNet 依赖到 `.venv-graspnet` |
| `patch_graspnet_deprecations.py` | 为本机第三方 GraspNet 源码应用兼容修补，由安装脚本调用 |
| `install_orbbec_udev_rules.sh` | 安装相机 USB 访问规则，需要 sudo |
| `setup_motorbridge_fresh_feedback.py` | 检查、构建或显式安装 MotorBridge 安全补丁 |
| `source_local_environment.bash` | 加载本机 ROS 与解释器环境，使用 source 调用 |
| `run_ubuntu_vision.sh` | 启动相机与 YOLO 感知链路 |
| `check_ubuntu_graspnet_env.py` | 检查 GraspNet 依赖、CUDA 和模型路径 |
| `check_rgbd_camera_info.py` | 订阅并检查图像与 CameraInfo，不自行启动相机 |

依赖清单位于 [依赖文档](../docs/setup/dependencies/README.md)，由 `install_python_dependencies.py` 读取安装。
GraspNet 推理实现属于 `src/rebotarm_vision/rebotarm_vision/backends/`；
YOLO 权重属于 `src/rebotarm_vision/models/`，不在维护脚本目录中保存。
