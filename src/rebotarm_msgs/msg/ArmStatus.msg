# 机械臂整体运行状态快照，锁存（latched）且可靠发布，晚加入的订阅者立刻能拿到最后一帧。
# 界面/示教/验收等模块用它判断"现在能不能发命令"，它同时也是反馈读取失败时唯一的故障通告通道。

# 时间戳取发布瞬间的 ROS 时间；frame_id 不使用。
std_msgs/Header header
# 当前电机控制模式，字符串（小写），取值："mit"、"pos_vel"、"vel"。
string mode
# 全部电机是否已使能：false 时任何运动命令都会被拒绝（真机上电默认失能）。
bool enabled
# 统一硬件控制环是否在运行：位置/抓取命令要求它为 true，否则硬件层直接报错。
bool control_loop_active
# 硬件状态机当前状态，字符串取值："IDLE"（空闲）、"TRAJ_RUNNING"（轨迹执行中）、
# "LOWLEVEL_STREAMING"（低层直通命令流中）、"GRAVITY_COMP"（手动拖拽/重力补偿中）；
# 上层据此判断是否允许抢占（例如低层直通命令是否要打断正在执行的轨迹）。
string state_machine
# 关节名列表，顺序与 per_joint_status_code 一一对应。
string[] joint_names
# 每个关节的电机状态码，与 joint_names 同序：0 = 失能，1 = 使能，255 = 样本不可用。
uint8[] per_joint_status_code
# 当前错误码列表，为空表示无错误。内容是"累计故障（如 ENABLE_ROLLBACK_FAILED、
# FEEDBACK_PROTECTIVE_DISABLE）+ 当前反馈健康状态派生的项（如 ARM_FEEDBACK: ...）"，
# 因此反馈恢复后派生项会自动消失，不需要额外清理历史。
string[] error_codes

# 对应topic通信，给订阅者和发布者使用
