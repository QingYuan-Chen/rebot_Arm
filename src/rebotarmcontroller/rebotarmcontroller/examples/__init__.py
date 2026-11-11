"""手动演示与现场调试客户端集合。

职责与位置
    手动演示脚本所在包，属于 ROS 服务与 Action 客户端示例。

对外接口
    具体入口在 move_to，move_to_pose，gripper_control 和 gravity_compensation。

安全与并发约束
    导入本包不运行示例；示例不是生产安全验收流程。

文件结构（按定义顺序）
    本文件仅声明包的用途，不在导入时创建节点，线程或设备。
"""
