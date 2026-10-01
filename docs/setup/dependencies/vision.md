# 相机与 YOLO 依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

安装到 `.venv-vision`；固定 NumPy 1.x 以兼容 ROS 2 Jazzy 的 cv_bridge。视觉安装脚本同时准备匹配的 PyTorch 和 TensorRT。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
# Keep NumPy 1.x for ROS 2 Jazzy's apt-installed cv_bridge ABI.
numpy==1.26.4
scipy==1.11.4
opencv-python==4.11.0.86
ultralytics==8.4.66

# 2.1.1 requires NumPy 2 on Python 3.12, which conflicts with cv_bridge.
pyorbbecsdk2==2.0.18
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
./scripts/setup_ubuntu_vision.sh
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py vision --show`。

返回 [依赖索引](README.md)。
