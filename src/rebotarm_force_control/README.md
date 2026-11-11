# rebotarm_force_control

共享重力补偿与关节力控算法包。硬件控制器和仿真单向调用本包；输入为关节数值、模型、配置和时间步长，输出为待发送命令与算法状态。

`gravity_control.py` 拥有六轴模型加载、重力与雅可比计算、MIT 总力矩及变化率限制、抗漂移积分和拖动跟随状态。`config/gravity_compensation.yaml` 保存既有补偿参数。

`joint_impedance.py` 实现静态平衡点控制律 `tau = g(q) + K*(q_ref-q) - D*q_dot`。
`JointImpedanceCommand(target, stiffness, damping)` 保存不可变六轴配置；
`compute_joint_impedance(command, position, velocity, gravity, effort_limits, rate_limits, previous, dt)`
返回请求力矩、受约束力矩及饱和标志。刚度和阻尼单位为 N·m/rad 与 N·m·s/rad，允许零增益。
输入维度、有限值、力矩/变化率上限、上一输出及正时间步均受校验；调用方拥有目标位置限位、
反馈新鲜度、命令期限和执行互斥。上一输出使用已成功施加的实际力矩。

MuJoCo 接入和 Python、CLI、ROS 用法见 [仿真说明](../rebotarm_simulation/README_mujoco.md)。
新阻抗模式从静态关节平衡点开始，重力来自当前仿真质量模型的零速度动力学计算。

六轴顺序为 `joint1..joint6`，单位为 rad、rad/s、N·m 和 s。模型使用 `rebotarm_moveit_config` 的规范 URDF。`GravityModel` 内部计算缓存须串行调用；每个执行实例单独持有 `GravityExecutionState`。

`rebotarmcontroller/gravity/gravity_controller.py` 持有硬件门控、反馈验证、模式交接、唯一设备线程调度、发送确认及保护。发送全部成功后才更新上一周期总力矩。仿真后端持有物理时间、生命周期和执行接口。

现有重力补偿服务与参数数值保持。算法迁移的验证使用离线假设备与仿真，真机力控需单独验收。
