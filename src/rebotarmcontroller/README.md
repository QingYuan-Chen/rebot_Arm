# rebotarmcontroller

真实机械臂与夹爪的硬件控制包，是仓库中唯一允许访问电机 SDK、串口/总线和真实执行器的包。它发布已校验反馈、提供 ROS 服务/动作，并执行最后一道低层安全检查。

## 目录结构

```text
rebotarmcontroller/
├── rebotarmcontroller/
│   ├── rebotarm_controller.py       # reBotArmController ROS 节点装配
│   ├── hardware_manager.py           # 公共门面、准入和组件交接
│   ├── configuration.py              # 配置解析、范围和资源定位
│   ├── controller_contracts.py       # 不可变反馈与发送回执
│   ├── sdk_driver.py                 # MotorBridge 唯一总线与句柄
│   ├── device_requests.py            # 有界请求、优先级、有效期与资源代次
│   ├── controller_runtime.py         # 唯一设备线程、反馈与周期调度
│   ├── feedback_acquisition.py       # 序号采集、缓存和只读健康判断
│   ├── lifecycle.py                  # 生命周期状态与关闭策略
│   ├── arm_execution.py              # 六轴目标、速度覆盖和轨迹游标
│   ├── arm_trajectory.py             # 六轴任务、到位与取消结果
│   ├── arm_position_control.py       # 位置命令计算
│   ├── controller_kinematics.py       # 状态查询所需的规范模型 FK
│   ├── gravity_control.py            # 重力模型、补偿状态和积分计算
│   ├── gripper_execution.py          # 独立夹爪任务与周期推进
│   ├── gripper_control.py            # 夹爪斜坡和有界力矩
│   ├── ros_services.py               # enable/disable/safe_home/stop/zero/夹爪等服务
│   ├── ros_actions.py                # FJT、夹爪 Action 协议映射
│   ├── ros_publishers.py              # joint_states、JointMotorState、ArmStatus
│   ├── motor_passthrough.py           # 受门控的低层电机命令透传
│   ├── conversions.py                 # 位姿、RPY、旋转矩阵互转
│   ├── examples/
│   │   ├── gravity_compensation.py   # 重力补偿/手动拖拽演示
│   │   ├── gripper_control.py         # 夹爪开合交互演示
│   │   ├── move_to.py                 # 关节空间点到点演示
│   │   └── move_to_pose.py            # 末端位姿演示
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 模块职责

- `HardwareManager` 是兼容门面，所有设备请求由 `ControllerRuntime` 的唯一线程串行执行；驱动独占串口和电机句柄。控制器与 motion 不再导入厂商 Python SDK，参考目录保留。
- `rebotarm_controller.py` 组合参数、硬件管理器、服务、动作和发布器，形成唯一真实 controller 节点。
- `ros_publishers.py` 只发布已校验反馈；`ArmStatus` 用于通告 connected/enabled/control loop/error 状态。
- `ros_services.py` / `ros_actions.py` 只适配协议和任务结果；到位、超时、取消保持由本地执行器及运行时处理。
- `motor_passthrough.py` 提交带有效期的底层请求；设备线程原子处理 reject/preempt、配置范围检查和 0.25 s 断流租约，不做上层碰撞规划。
- 位姿 IK、笛卡尔路径、重定时及碰撞检查归 `rebotarm_motion`。其 `PoseCompatNode` 唯一提供旧 MoveToPose/MoveToPoseIK 接口并转 FJT；服务成功只表示接纳，Action 等待到位。新增模块按职责平铺，不新建职责子目录。

## 对外入口

```text
reBotArmController      -> 真机控制主节点
GravityCompensation     -> 重力补偿演示
GripperControl          -> 夹爪交互演示
MoveTo                  -> 关节点位演示
MoveToPose              -> 末端位姿演示
```

真实 controller 通常由 `rebotarm_bringup/launch/hardware_controller.launch.py` 组合，其他 launch 文件只 include 这个硬件片段，避免重复创建控制器。

## 真机启动原则

1. 上电默认失能；先确认新鲜反馈、通信和现场安全状态。
2. 必须通过显式 `/rebotarm/enable` 才允许运动。
3. `FollowJointTrajectory`、停止、safe home、夹爪和底层命令都必须经过本包。
4. 健康 enabled 状态下，任务级失败不能直接把机械臂带离已验证 baseline 后失能；应停止并保持，或受保护回 baseline、确认到位后再失能。

请求队列、停止确认、SDK 对照及验证范围见[硬件执行说明](../../docs/implemented/features/hardware_execution.md#请求资源与停止确认)。软件回归和内存假硬件执行均不替代实机验收。
