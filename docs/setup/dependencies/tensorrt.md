# TensorRT 推理依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

与视觉共用 `.venv-vision`；使用 `--no-deps`，避免重复安装 CUDA 构建工具链。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
# TensorRT inference runtime only. scripts/setup_ubuntu_vision.sh installs these
# with --no-deps so NVIDIA's multi-GB CUDA builder/toolkit meta-package is not
# duplicated; this project uses the already installed CUDA 12.8 driver/runtime.
tensorrt-cu12==10.13.3.9.post1
tensorrt-cu12-bindings==10.13.3.9.post1
tensorrt-cu12-libs==10.13.3.9.post1
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
python3 scripts/install_python_dependencies.py tensorrt --python .venv-vision/bin/python --no-deps
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py tensorrt --show`。

返回 [依赖索引](README.md)。
