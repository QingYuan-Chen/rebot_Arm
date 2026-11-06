# 控制器行为对照样本

`tests/fixtures/controller_reference_v1.json` 是固定输入与预期输出，不是项目状态文件。
`tests/test_controller_reference_samples.py` 直接驱动生产的执行器、命令生成、MotorBridge
适配驱动、生命周期和停止组件；设备句柄为内存替身，不打开真实串口。

`controller_fk_reference_v1.json` 另保存四组历史模型 FK 的位置和旋转矩阵,
含固定 SDK 提交、URDF SHA-256、Pinocchio 版本、关节映射、坐标系与单位。默认回归
不读取旧 SDK 目录, 不自动重写期望值; 位置及旋转误差均严格小于 `1e-12`。
既有 `controller_pose_vectors.json` 的 IK 对照继续保留, 提取方式与显式模型对照见
[SDK 退出验收](hardware_execution.md#sdk-退出的离线验收)。

量化反馈修复后，停止组件使用 `controller_stop_reference_v2.json`：窗口为
0.20 s，并检查 150 ms 连续性。按用户最新要求，三个位置编码的位置极差和锚点
门限均已删除；当前只以独立新帧、速度和完整窗口确认，不重新改写历史样本。
v1 的停止片段保留为历史数据，当前停止测试读取 v2，其他 v1 场景继续执行。
该策略及独立负例见[量化反馈判定与审计](quantized_feedback_safety.md)。

批次 6.5 另增 `tests/fixtures/controller_fault_reference_v1.json`，由
`tests/test_controller_fault_reference.py` 驱动生命周期回滚、失能验证失败、保护锁存、
透传仲裁和重力补偿交接。故障样本只比较跨语言可表达的设备操作、发送次数、状态迁移和
`SafetyDecision`；不把 Python 回调名或私有字段作为 C++ 接口。生命周期样本使用
`open_device`、`set_hold_target`、`set_mode_pos_vel`、`enable_arm`、`disable_arm`、
`release_device` 等语义操作名，失败路径覆盖回滚失能失败、失能反馈/写入失败以及关闭时
状态未确认仍保留句柄。保护样本覆盖一次性保护发送、发送失败后的 `DISABLING` 锁存、
显式失能复位和重新使能后的再次保护；透传样本覆盖忙碌拒绝、非法抢占和新鲜静止反馈后
接替；重力样本覆盖模式拒绝回到位置保持、健康模型故障保持使能、通信发送故障保护失能
及正常退出回到位置保持。

组件间装配使用 `component_ports.py` 中按职责冻结的具名能力端口。端口只传递方法能力，
不复制反馈、目标或设备状态；周期交接和保护回调仍保留必要的控制环，但调用方只能使用
端口声明的公开窄接口。结构门禁会检查端口类型为冻结数据类、调用字段属于声明集合，且
禁止 manager 穿透组件私有成员或为构造好的状态保留 `getattr` 夹具回退。该边界为控制器输入、输出与故障决策对照提供稳定入口。

批次 6.7 按用户追加决定将 manager 收敛为纯装配：仅有构造函数，不再保留业务方法或
属性转发。节点向 ROS 适配和关闭协调注入 `ros_ports.py` 声明的职责端口，由原组件实现，
不创建新门面或状态副本；发布器只获得查询能力。测试专用入口和转出名称已移除，
测试从实际所有者查询状态、从定义模块导入常量。组件间端口直接绑定方法；测试通过冻结端口替身
显式注入反馈刷新或生命周期操作，不再依赖装配后 monkeypatch 方法的动态查找。
生产组件仍只执行一次 `bind`，取夹爪设备绑定和补充保护类别的适配回调保留。

`device_requests.REQUEST_POLICIES` 是命令入口策略的唯一来源，生命周期、六轴、夹爪、
重力组件的装饰器直接引用对应冻结策略。生命周期构造必须显式提供同一请求运行时；
独立事务样本以明确的写线程替身运行，实际排队和锁交接另由生产运行时配合假设备验证。
`test_controller_structure.py` 检查纯装配、ROS 职责端口、导入、纯透传 lambda 和请求层级；
`test_device_request_arbitration.py` 检查生命周期命令只入队一次、锁外排队、可变输入
复制、队列满时失能保留入口及旧请求作废；既有停止并发和关闭后重连测试继续覆盖。
原门面路径的六个参数化重复案例随接口移除，六个生命周期组件入口的策略断言保留，
适配回调到组件的交接由新增集成测试覆盖。
同步编排不入队等待，已开始事务的回执、停止优先级和 worker 关闭规则沿用原契约。
`test_controller_ros_ports.py` 正式构造服务、Action、发布器和透传适配器，注册与消息出口
使用内存替身，回调运行在真实请求运行时和假电机上；检查默认失能拒绝、显式使能、停止
及夹爪取消入队策略、FJT 取消保持、状态实时读取和关闭延期。该测试不建立 DDS、
不打开真实串口，不证明 ROS 调度时延，也不替代现场验收。

版本为 `schema_version=1`。位置 rad、速度 rad/s、力矩 N·m、时间为单调秒；
数值绝对容差 `1e-9`、相对容差 0，状态、代次、调用顺序及回执电机名精确匹配。
各场景分别重建时钟和状态，并使用同一 JSON 中的预期结果。

| 输入 | 必须保持的输出与语义 |
| --- | --- |
| 六轴轨迹请求、周期时刻及反馈 | 同周期位置命令、目标代次、成功回执、游标、到位结果；末点同时间反馈不能确认成功 |
| 健康反馈下终点超时 | `tolerance`、实测保持目标、`STOP_HOLD`；不发送失能 |
| 第三电机发送失败 | 前两轴已发送、第三轴失败、后三轴未尝试；游标不推进；保护决策锁存且不重复触发 |
| 停止发送及新反馈 | 新帧累计静止窗口才能确认；重复缓存和速度超差最终超时并保持 |
| 连接、显式使能、失能、关闭 | 默认失能、反馈检查及设备调用顺序、生命周期和使能状态 |
| 夹爪已到目标的接纳请求 | 接纳代次、MIT 中性释放回调及成功结果；旧代次只能是 superseded |

夹爪样本的 `command` 是发送回调参数 `(pos, vel, kp, kd)`，省略的前馈为 0，
同时检查 `tau_limit`。设备边界再计算并限制实际 MIT 力矩。

实际值契约分别为 `ArmTarget`、`PositionCommand`、`CommandReceipt`、
`JointSnapshot`、`GripperTarget`、`GripperResult`、`ArmResult`、`StopResult`
及 `SafetyDecision`。反馈以元组复制；兼容 ROS 查询仍返回独立数组和既有元组。
设备、任务事件和 SDK 可变对象不进入这些值契约。

样本覆盖数值和状态对照；`test_stop_runtime_arbitration.py` 另外使用生产线程及假设备，
验证停止等待期间的失能响应和超时保持，`test_controller_structure.py` 防止门面重新吸收
控制事务。生命周期回滚、保护去重、夹爪释放失败及旧目标取消由既有故障测试继续覆盖。
这些结果不能证明真机时延、反馈连续性或运动验收。

离线执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_controller_reference_samples.py tests/test_controller_structure.py \
  tests/test_arm_stop.py tests/test_stop_runtime_arbitration.py -q
python3 scripts/check_motorbridge_abi.py
```

ABI 探测只加载动态库、核对符号和 C++/ctypes 结构布局，不创建设备句柄。
结构变更后的检查按仓库规范选择；真机验证另按现场授权安排。
