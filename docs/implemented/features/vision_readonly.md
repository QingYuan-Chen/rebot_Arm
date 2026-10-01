# Ubuntu 原生视觉只读链路

> 状态：IMPLEMENTED；类型：视觉功能说明；适用范围：Ubuntu 原生视觉只读链路；不启动控制器、不使能、不发送机械臂轨迹。

## 功能概述

当前支持：`Gemini 2 → YOLO → ROS RGB-D/CameraInfo/detections → 本机 GraspNet candidates`。

## 当前实现

- Gemini 2 RGB-D 和 CameraInfo 发布；
- YOLO 本地检测与标注图像；
- 深度融合和候选生成；
- GraspNet 候选话题、Marker 和可选 Open3D 查看；
- 过期、无深度、TF 失败时的 fail-closed 行为。

## 验证结果

该链路用于验证相机消息、检测消息、时间戳、坐标系和候选输出；当前验证范围见 [项目状态](../../reference/project_status.md)，采样方法见视觉只读命令参考。

## 边界与未完成

本功能不证明真实手眼精度、抓取可达性、碰撞安全或真实抓取成功，也不会自动启动控制器或发送轨迹。

## 运行入口

安装见 [`../../setup/ubuntu_vision_setup_zh.md`](../../setup/ubuntu_vision_setup_zh.md)，启动和检查见 [视觉只读命令参考](../../reference/commands/ubuntu_vision_readonly_test_zh.md)。
