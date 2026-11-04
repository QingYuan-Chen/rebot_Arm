# rebotarmcontroller

真实机械臂与夹爪的硬件控制包. 在主工程 ROS 包中, 本包拥有 MotorBridge 通信、
唯一串口/总线会话、已校验反馈和最后一道执行安全检查. 支持 Ubuntu 24.04 / ROS 2 Jazzy.
当前实施及验收范围以[项目状态](../../docs/reference/project_status.md)为准.

P005 协同位置路径由 `runtime/coordinated_trajectory.py` 持有整体任务，唯一设备周期
取一次参考时间，复用六轴和夹爪组件的发送与反馈。七轴执行不建立第二条夹爪斜坡；
部分发送失败保留真实回执，停止确认前不释放两侧资源。夹爪整体结束中性释放，
机械臂保持命令终点；健康任务失败不自动失能。接口见
[P005 协同位置](../../docs/implemented/features/p005_prepared_trajectory.md#修订协同接口)。

## 职责与执行关系

```text
reBotArmController ROS 节点
  -> HardwareManager 装配运行时、生命周期与职责组件
  -> Service / Action / 透传适配调用对应组件端口
  -> device_requests 排队, ControllerRuntime 唯一设备线程执行
  -> 本地 SdkMotorDriver -> MotorBridge -> 六轴与夹爪共用总线

设备反馈 -> FeedbackReader / feedback_acquisition 验证缓存
         -> 周期执行与保护判断
         -> ROS 发布器读取一致快照, 不直接刷新设备
```

- `HardwareManager` 只有构造与接线, 暴露 runtime、lifecycle、arm、gripper、feedback、
  gravity、cycle、protection 组件引用. 业务操作由原组件负责, manager 不提供方法或属性转发.
- ROS 适配层通过 `ros_ports.py` 中的职责接口接收原组件; 组件之间通过
  `component_ports.py` 中的冻结能力端口和绑定方法交接, 不复制状态或访问对方私有成员.
- 生命周期、六轴、夹爪、重力组件的设备命令入口使用 `device_request`;
  `device_requests.REQUEST_POLICIES` 集中声明资源、优先级、TTL 和代次规则.
  含同步等待的停靠、夹爪位置和抓取流程在调用线程等待, 设备子命令仍由唯一设备线程执行.
- 六轴轨迹、夹爪任务和重力状态分别由对应组件拥有. 控制器执行已接纳的数值请求;
  IK、笛卡尔路径、轨迹生成、重定时、MoveIt 适配和碰撞检查归 `rebotarm_motion`.
  必要的周期与保护回调仍存在, 组件间仍有协调关系.

## 文件结构

根目录保留入口、装配与配置，其余模块按一层职责子目录组织；`examples/` 只提供 ROS 客户端示例。
模块文件名与组件接口保持原样，Python 导入使用新目录，例如
`rebotarmcontroller.arm.arm_execution`。旧平铺路径不提供转发，各子包不汇总导入。

```text
rebotarmcontroller/
├── config/
│   ├── arm.yaml                      # 六轴电机、共享通道和控制参数
│   ├── gripper.yaml                  # 夹爪电机和控制参数
│   └── gravity_compensation.yaml     # 重力模型与补偿参数
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
│   │   ├── coordinated_trajectory.py # 协同任务、同周期参考与整体终态
│   │   ├── controller_protection.py  # 故障协调、保护失能和恢复确认
│   │   ├── command_guard.py          # 透传准入、故障决策与保护锁存
│   │   └── stationary_evidence.py    # 新鲜独立反馈与速度窗口证据
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
│   │   ├── gravity_compensation.py   # 重力补偿/手动拖拽客户端
│   │   ├── gripper_control.py        # 夹爪交互客户端
│   │   ├── move_to.py                # 六轴 FJT 客户端
│   │   └── move_to_pose.py           # motion 位姿兼容接口客户端
│   └── __init__.py
├── resource/rebotarmcontroller       # ament 包索引标记
└── setup.py / setup.cfg / package.xml
```

## 依赖、配置与旧 SDK 退出

正常构建、完整测试和部署均不需要 `third_party/reBotArm_control_py`. 本地
`SdkMotorDriver` 直接调用 MotorBridge, 不构造旧 SDK 的 `RobotArm` / `ArmEndPos`,
也不读取旧 SDK 的配置、模型或使用其线程. 旧目录与固定版本清单仅作为可选历史参考保留.

- 通信依赖为带逐电机反馈序号补丁的 MotorBridge `0.4.7+rebotarm.1`.
  原始 PyPI `0.4.7` 不能替代该真机运行版本; 安装和回退见[部署说明](../../docs/setup/ubuntu_ros2_jazzy.md).
- Pinocchio 继续负责规范模型计算. URDF 及网格由 `rebotarm_moveit_config` 拥有;
  默认模型为该包的 `config/rebotarm.urdf`, 六轴按 `joint1..joint6` 排列.
- 直接运行控制器且配置路径为空时使用本包的三份 YAML.
  `hardware_controller.launch.py` 默认显式传入 bringup 的 arm/gripper YAML;
  通道参数只覆盖内存配置. 两种入口的参数以各自文件为准, 不能假定默认值相同.
- 默认 FK 回归读取 `tests/fixtures/controller_fk_reference_v1.json` 固定数值,
  记录来源提交、旧 URDF SHA-256、Pinocchio 版本、单位、坐标系及 `join3`/`joint3` 映射.
  既有 IK 样本保留; 默认测试不再读取旧参考树.

以下检查均不连接设备, 命令从工作区根目录执行:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/setup_motorbridge_fresh_feedback.py --check-installed
python3 scripts/check_controller_fk.py

# 可选人工对照, 只有显式提供路径才读取历史模型
python3 scripts/check_controller_fk.py --legacy-urdf /absolute/path/to/legacy.urdf
```

FK 检查的位置及旋转误差均须严格小于 `1e-12`; 超差退出 1, 输入错误退出 2.
样本缺失或损坏、模型缺失均失败, 不自动回退或重写样本.
无旧目录的源码验证、非软链接构建及隐藏源码的安装区验证见
[SDK 退出的离线验收](../../docs/implemented/features/hardware_execution.md#sdk-退出的离线验收).
来源和许可记录见[第三方说明](../../docs/reference/third_party_notices.md).

## ROS 入口与接口

`setup.py` 注册五个入口. 示例通过 ROS 与控制器或 motion 通信, 不另开电机连接.

| 可执行入口 | 用途 |
| --- | --- |
| `reBotArmController` | 真机控制主节点, 启动即连接设备并验证默认失能 |
| `GravityCompensation` | 重力补偿客户端 |
| `GripperControl` | 夹爪交互客户端 |
| `MoveTo` | 六轴 FJT 点位客户端 |
| `MoveToPose` | motion 位姿兼容 Action 客户端 |

真机组合通常包含[hardware_controller.launch.py](../rebotarm_bringup/launch/hardware_controller.launch.py).
该片段启动唯一硬件节点和 motion 的 `PoseCompatNode`, 其他组合复用该片段避免重复占用总线.

以下名称均相对 `/<arm_namespace>/`, 默认前缀为 `/rebotarm/`:

| 类型 | 名称与用途 |
| --- | --- |
| 状态话题 | `joint_states`, `joints/<joint>/state`, `gripper/state`, `arm_status` |
| 生命周期服务 | `enable`, `disable`, `set_mode`, `set_zero` |
| 执行服务 | `trajectory_stop`, `safe_home`, `gripper/set`, `gripper/grasp` |
| 重力补偿服务 | `gravity_compensation/start`, `gravity_compensation/stop` |
| Action | `follow_joint_trajectory` (`FollowJointTrajectory`), `gripper/command` (`GripperCommand`) |
| 低层命令话题 | `joints/<joint>/cmd`, `gripper/cmd`, 仍受模式、范围、仲裁和断流租约约束 |

`move_to_pose` Action 和 `move_to_pose_ik` Service 由 motion 的 `PoseCompatNode` 唯一提供,
通过规划/校验后转 FJT. IK 服务成功只表示 FJT 接纳; 位姿 Action 等待执行到位.
MoveIt 执行需要 `moveit_simple_controller_manager`; 规划成功不代表执行成功.
完整参数、消息与命令见[接口参考](../../docs/reference/ros_sdk.md).

## 并发与真机安全

1. `HardwareManager()` 只加载配置、模型并接线, 不连接设备或启动线程.
   节点随后调用 `lifecycle.connect()`, 该事务可能发送失能命令, 不能作为只读探测.
   连接前须检查串口占用、新鲜反馈获取条件、现场支撑、空载、净空和急停.
2. 启动默认失能. 获取新鲜合法反馈、完成本轮现场检查并显式调用
   `/<arm_namespace>/enable` 后才允许运动. 已连接不等于已使能或运动就绪.
3. 设备请求由唯一 `rebotarm-hardware` 线程执行. 失能期间仍采集反馈;
   逐电机新序号才构成新帧, 重复发布不能更新旧反馈的源时间或代替停止证据.
4. 停止和取消需要发送确认及新鲜实际反馈的速度窗口，不以位置极差或实际位置到保持命令的
   绝对偏差判定停止；两侧均确认前保留联合预约。旧任务终态与迟到清理不能影响后继代次。
   取消接纳或送完轨迹点不能作为停止/到位。
   健康的可恢复任务失败应停止并保持使能, 或在安全监控下返回本轮基线, 验证后再显式失能.
   电机/控制器错误、通信丢失、反馈过期或明确急停才允许基线外保护失能.
5. 保护锁存不能被恢复后的新帧自动清除; 需要显式失能及验证后再使能.
   正常置零只允许在失能且新反馈验证通过后执行, 不随连接或恢复自动置零.
6. 已使能时关闭应停止并保持控制、继续响应服务, 等待现场确认后显式失能再退出.
   失能未知或验证失败时不能宣称已安全释放. MotorBridge 的 shutdown 仍可能追加失能帧,
   释放连接也不是未知状态下的只读操作. ROS 上下文已失效时的现场处置见
   [硬件执行说明](../../docs/implemented/features/hardware_execution.md#位姿兼容接口与-sdk-退出).

停止窗口、反馈量化和导数诊断见[量化反馈判定](../../docs/implemented/features/quantized_feedback_safety.md).
结构及跨语言样本见[控制器对照样本](../../docs/implemented/features/controller_reference_samples.md).
软件、PTY 和内存假设备验证不替代实机验收; 各项验收及批次 7 的边界仅在项目状态中记录.
