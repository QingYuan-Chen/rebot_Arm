"""ROS 协议与消息适配。

文件职责：声明 ros 子包，不创建节点、服务或 Action server。
对外接口：从 ros_actions/ros_services/ros_publishers/motor_passthrough/conversions 取对应适配能力。
安全约束：ROS 回调通过职责端口请求原所有者，不能直接写设备或以回调等待推进硬件周期。
文件结构：本文件仅声明用途，协议适配和位姿消息转换分别位于五个模块。
"""
