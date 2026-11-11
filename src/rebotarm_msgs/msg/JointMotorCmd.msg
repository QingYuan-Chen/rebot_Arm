# 单关节（或夹爪）低层电机命令。由外部直接下发到
# /<namespace>/joints/<joint>/cmd 与 /<namespace>/gripper/cmd，硬件层透传执行，
# 不经过规划、插值、限位裁剪与碰撞检查。只允许在硬件已使能且反馈新鲜时下发。
#
# 使用约定：先选 mode 决定电机侧走哪种控制律，再用 use_* 布尔位逐字段"选择是否生效"。
# 未置位的字段不会下发，而是回落到当前反馈值或关节默认增益（见硬件层
# send_joint_motor_cmd），这样可以只覆盖想改的那一项，避免用 0 覆盖正常指令。

# 控制模式枚举，取值即协议里的整数编号：
#   0 = MIT 模式（同时给 pos/vel/kp/kd/tau，可做重力补偿与阻抗控制）；
#   1 = 位置-速度模式（只给 pos 与 vlim，速度受限的位置控制，最常用）；
#   2 = 纯速度模式（只给 vel，需要电机固件支持 send_vel，否则下发会报错）；
#   其它取值一律拒绝。
uint8 MODE_MIT     = 0
uint8 MODE_POS_VEL = 1
uint8 MODE_VEL     = 2
# 本条命令使用的控制模式，取值见上方 MODE_* 常量。
uint8 mode

# 以下 6 个布尔位表示对应字段是否生效：true 用本消息里的值，false 回落到反馈值/默认增益。
bool use_pos
bool use_vel
bool use_kp
bool use_kd
bool use_tau
bool use_vlim

# 目标关节角，单位 rad；编码器零点已在硬件侧标定（set_zero）。
float64 pos
# 目标关节速度，单位 rad/s；MIT 模式的速度项、纯速度模式的目标速度。
float64 vel
# MIT 模式的刚度增益 kp，量纲 N·m/rad；越大越硬，过大易引起振荡。
float64 kp
# MIT 模式的阻尼增益 kd，量纲 N·m·s/rad；用于抑制超调与振荡。
float64 kd
# MIT 模式的前馈力矩，单位 N·m；重力补偿时由模型计算后填入（含正负号约定）。
float64 tau
# 位置-速度模式下的速度上限，单位 rad/s；限制本次运动的快慢，也是安全限速入口。
float64 vlim

# 命令时间戳（ROS 时间，秒 + 纳秒），用于上层判断指令新鲜度；不参与电机协议帧。
builtin_interfaces/Time stamp
