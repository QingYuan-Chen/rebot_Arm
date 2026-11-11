# 单关节（或夹爪）电机状态。发布在 /<namespace>/joints/<joint>/state 与
# /<namespace>/gripper/state，属于"只读反馈"，上层据此观测硬件而不是发命令。
# 数值来自已校验的反馈缓存：读取失败时宁可不发新帧，也不把陈旧位置伪装成当前样本。

# 时间戳与坐标系。frame_id 通常为空（关节量不在任何笛卡尔坐标系下）；
# stamp 沿用其所属反馈批次的接收时刻，同一批次重复发布时保持不变。
std_msgs/Header header
# 关节名，取值与硬件的 joint_names 一致（joint1..joint6，夹爪固定为 "gripper"）。
string joint_name
# 关节角，单位 rad。手臂关节为电机角度；夹爪这条话题给的是开口距离，单位 m（0 = 完全闭合）。
float64 position
# 关节角速度，单位 rad/s。
float64 velocity
# 关节力矩，单位 N·m（电机侧估计/测量值，未做重力补偿剔除）。
float64 torque
# 电机状态码，由驱动返回：0 = 失能，1 = 使能（可接受位置命令），255 = 取不到样本（不可用）。
uint8 status_code
