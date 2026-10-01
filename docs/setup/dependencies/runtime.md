# 控制器基础依赖

> 状态：SETUP；类型：Python 依赖版本清单；适用范围：Ubuntu 24.04 / Python 3.12。

## 当前范围

用于系统 Python 的 MotorBridge bootstrap；已有补丁版本时不要重复覆盖。安装后仍需按部署说明安装并检查反馈序号补丁。

## 依赖版本

下列 `requirements` 代码块是安装脚本的唯一依赖来源，说明正文不参与安装。

```requirements
# Python dependencies that are not distributed as ROS packages.
# Bootstrap package only. The controller additionally requires the reviewed
# feedback-sequence patch installed by scripts/setup_motorbridge_fresh_feedback.py.
motorbridge==0.4.7
```

## 安装入口

从仓库根目录执行；先按 [环境部署说明](../ubuntu_ros2_jazzy.md) 准备对应环境。

```bash
python3 scripts/install_python_dependencies.py runtime --user --break-system-packages
```

只读预览安装命令：`python3 scripts/install_python_dependencies.py runtime --show`。

返回 [依赖索引](README.md)。
