# GraspNet 与点云依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

安装到独立 `.venv-graspnet`，与视觉/ROS 环境隔离；模型源码和 checkpoint 由本机单独维护。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
# Ubuntu-local GraspNet environment.
# Keep this environment separate from ROS 2 / cv_bridge's .venv-vision.
numpy==1.26.4
scipy==1.11.4
opencv-python==4.11.0.86
PyYAML==6.0.1
Pillow==11.1.0
tqdm==4.67.1
transforms3d==0.4.1
cffi==1.17.1
open3d==0.19.0

# Proven CUDA wheel pair already validated on this host in .venv-vision.
torch==2.11.0+cu128
torchvision==0.26.0+cu128
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
./scripts/setup_ubuntu_graspnet.sh
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py graspnet --show`。

返回 [依赖索引](README.md)。
