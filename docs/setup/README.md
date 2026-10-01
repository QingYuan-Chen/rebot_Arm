# 环境配置与项目接手

> 状态：SETUP；类型：环境配置索引；适用范围：clone、依赖、构建和运行环境。

本目录面向需要 clone 项目、安装依赖、构建工作区和复现软件环境的开发者。

## 文档分类

- `ubuntu_ros2_jazzy.md`：Ubuntu 24.04、ROS 2 Jazzy、系统依赖、构建和基础安全顺序；
- `launch_python_configuration.md`：ROS launch 中 MuJoCo、视觉和 GraspNet 的逐进程 Python 配置；
- `ubuntu_vision_setup_zh.md`：Gemini 2、YOLO、GraspNet 和视觉运行环境；
- [`dependencies/`](dependencies/)：按功能拆分的 Markdown 依赖版本与安装说明，不要混装成一个环境。

## 复现原则

- 先确认 Ubuntu/ROS 版本，再安装系统依赖；
- ROS 工作区使用系统 Python 构建，MuJoCo、视觉和 GraspNet 按各自文档使用隔离环境；
- 安装模型、权重和本地 SDK 前先核对版本、来源、许可证和磁盘位置；
- 新环境完成后先运行软件测试和仿真健康检查，不自动启动真实机械臂；
- 真机相关依赖安装完成不等于硬件已经连接、使能或验收。
