"""D405 RGB-D、YOLO、本地 GraspNet 候选、筛选与显示。

camera 管理相机会话，detector 封装检测，converters 负责消息转换，
policies 保留候选策略，nodes 与 diagnostics 提供 ROS 和诊断入口。
视觉包只查询 TF、IK 和碰撞，不下发运动目标或访问电机 SDK。
"""
