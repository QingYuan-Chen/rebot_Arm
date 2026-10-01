# MuJoCo 仿真依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

本机 MuJoCo 仿真和模型工具的固定版本；使用 `third_party/rebotarm_mujoco_venv`。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
mujoco==3.3.0
numpy==1.26.4
cffi==1.17.1
pyyaml==6.0.3
jinja2==3.1.6
typeguard==4.5.2
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
python3 scripts/install_python_dependencies.py mujoco --python third_party/rebotarm_mujoco_venv/bin/python
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py mujoco --show`。

返回 [依赖索引](README.md)。
