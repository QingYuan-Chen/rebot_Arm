# rebotarmcontroller

真实机械臂与夹爪的硬件控制包，是仓库中唯一允许访问电机 SDK、串口/总线和真实执行器的包。它发布已校验反馈、提供 ROS 服务/动作，并执行最后一道低层安全检查。

## 文件结构

```text
rebotarmcontroller/
├── rebotarmcontroller/
│   ├── rebotarm_controller.py        # ROS 节点组合、参数与退出协调
│   ├── hardware_manager.py           # 运行时、生命周期和六个职责组件的纯装配
│   ├── configuration.py              # 配置加载、校验与默认资源定位
│   ├── controller_parameters.py      # 控制器常量与参数边界
│   ├── contracts/                   # 跨组件数值和能力接口
│   │   ├── controller_contracts.py   # 不可变命令、反馈与发送回执
│   │   ├── controller_values.py      # 跨组件快照、准备接纳与结果
│   │   ├── component_ports.py        # 组件间冻结能力端口
│   │   └── ros_ports.py              # ROS 适配与关闭协调的接口声明
│   ├── hardware/                    # 设备通信与已校验反馈
│   │   ├── sdk_driver.py             # MotorBridge 总线、电机句柄与发送回执
│   │   ├── feedback_acquisition.py   # 逐路序号、验证缓存与健康判断
│   │   └── feedback_reader.py        # 反馈分组、一致快照与组件查询
│   ├── runtime/                     # 设备线程、生命周期与安全协调
│   │   ├── controller_runtime.py     # 唯一设备线程、周期调度与停止续体
│   │   ├── device_requests.py        # 有界队列、优先级、资源与代次
│   │   ├── lifecycle.py              # 连接、使能、失能、模式、置零与关闭
│   │   ├── controller_cycle.py       # 反馈与六轴、夹爪、重力周期协调
│   │   ├── controller_protection.py  # 故障协调、保护失能和恢复确认
│   │   ├── command_guard.py          # 透传准入、故障决策与保护锁存
│   │   └── stationary_evidence.py    # 静止窗口、位移与固定锚点证据
│   ├── arm/                         # 六轴执行与 FK
│   │   ├── arm_controller.py         # 六轴准入、保持、停靠与透传
│   │   ├── arm_execution.py          # 六轴目标、速度覆盖和轨迹游标
│   │   ├── arm_trajectory.py         # 任务、到位与取消结果
│   │   ├── arm_position_control.py   # 六轴位置命令计算
│   │   ├── arm_stop.py               # 停止保持与周期确认状态机
│   │   └── controller_kinematics.py  # 规范六轴模型 FK
│   ├── gripper/                     # 夹爪事务、任务与控制计算
│   │   ├── gripper_controller.py     # 坐标、零位验证、准入和发送
│   │   ├── gripper_execution.py      # 独立任务及周期推进
│   │   └── gripper_control.py        # 斜坡、限力和保持计算
│   ├── gravity/                     # 重力补偿与模式交接
│   │   ├── gravity_controller.py     # 启停、模式交接与诊断
│   │   └── gravity_control.py        # 动力学模型、力矩约束与补偿计算
│   ├── ros/                         # ROS 协议与消息适配
│   │   ├── ros_services.py           # 生命周期、停止、夹爪等 Service
│   │   ├── ros_actions.py            # FJT 和夹爪 Action
│   │   ├── ros_publishers.py         # JointState、JointMotorState、ArmStatus
│   │   ├── motor_passthrough.py      # 受准入及租约约束的命令订阅
│   │   └── conversions.py            # ROS 位姿与矩阵转换
│   ├── examples/
│   │   ├── gravity_compensation.py   # 重力补偿/手动拖拽演示
│   │   ├── gripper_control.py         # 夹爪开合交互演示
│   │   ├── move_to.py                 # 关节空间点到点演示
│   │   └── move_to_pose.py            # 末端位姿演示
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 模块职责

- `HardwareManager` 只创建唯一运行时、生命周期及六个职责组件并接线，仅有构造函数，不提供业务转发。所有设备请求由 `ControllerRuntime` 的唯一线程串行执行；驱动独占串口和电机句柄。控制器与 motion 不再导入厂商 Python SDK，参考目录保留。
- ROS 适配和关闭协调直接接收 `ros_ports.py` 声明的职责端口；接口由原所有者实现，查询实时读取原状态，发布器只拿只读能力。组件间以冻结端口和方法引用装配。`device_request` 在生命周期、六轴、夹爪和重力组件的命令入口调用，21 个入口的资源、优先级、TTL 和代次策略集中于 `device_requests.REQUEST_POLICIES`；生命周期显式接收同一运行时。含同步等待的停靠、夹爪位置和抓取流程由调用线程等待，设备子命令入队；关闭确认后由调用线程回收 worker。
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
