# MuJoCo 仿真依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

本机 MuJoCo 仿真、模型工具和 CPU Reach/Pick 的固定版本；默认使用系统 Python 3.12。
本机通过用户级安装提供依赖，不再要求 MuJoCo 专属虚拟环境。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
mujoco==3.3.0
gymnasium==1.2.3
numpy==1.26.4
cffi==1.17.1
pyyaml==6.0.3
jinja2==3.1.6
typeguard==4.5.2
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
python3 scripts/install_python_dependencies.py mujoco --user --break-system-packages
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py mujoco --show`。

Ubuntu 24.04 的系统 Python 受 PEP 668 管理；这里的 `--user` 将包安装到
`~/.local/lib/python3.12/site-packages`，`--break-system-packages` 允许该用户级安装，
不使用 sudo 或覆盖 `/usr/lib` 内的文件。用户级包会优先于同名系统包被导入。
NumPy 保持 1.26.4，视觉、GraspNet 和 GPU 训练仍使用各自独立环境。
外部部署也可通过 `--python /绝对路径/python` 将依赖安装到指定解释器，
再用 launch 参数或 `REBOTARM_MUJOCO_PYTHON` 显式选择。

返回 [依赖索引](README.md)。
