# 分环境依赖清单

保留标准 pip requirements 格式，版本以各 TXT 文件为准。
迁移仅改变路径，没有修改依赖版本或合并 Python 环境。

| 清单 | 用途 |
| --- | --- |
| `requirements-runtime.txt` | 控制器基础依赖，另需 MotorBridge 安全补丁 |
| `requirements-vision.txt` | 相机、YOLO、ROS 图像桥接依赖 |
| `requirements-graspnet.txt` | GraspNet 点云与推理依赖 |
| `requirements-tensorrt.txt` | TensorRT 运行库，由视觉安装脚本单独安装 |
| `requirements-mjlab.txt` | 统一仿真与 GPU 强化学习环境 |
| `requirements-mujoco.txt` | 统一环境中的 CPU MuJoCo 组件版本 |

安装步骤和目标环境见 [部署说明](../docs/setup/ubuntu_ros2_jazzy.md)。
`src/rebotarm_simulation/requirements-mujoco.txt` 仍是仿真包自己的依赖声明，
与工作空间部署清单职责不同，本次保留原位置。
