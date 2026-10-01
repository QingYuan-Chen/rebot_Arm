# CPU Reach 与 PPO 依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

保留的 CPU 物理 Reach 基线和 CUDA SB3/PPO 工具，使用独立训练环境；不表示已接入 MJLab 或验收 GPU 训练。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
# Ubuntu 24.04 / Python 3.12 / NVIDIA driver 535+ / CUDA 12.x.
# PyTorch 2.5.1 CUDA 12.1 wheel is published by PyTorch; +cu121 avoids CPU wheel selection.
--extra-index-url https://download.pytorch.org/whl/cu121
-r mujoco.md
gymnasium==1.2.3
imageio==2.37.4
stable-baselines3==2.8.0
torch==2.5.1+cu121
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
python3 scripts/install_python_dependencies.py rl --python third_party/rebotarm_rl_venv/bin/python
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py rl --show`。

返回 [依赖索引](README.md)。
