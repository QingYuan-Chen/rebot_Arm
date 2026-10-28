# Python / C++ 控制器对照样本

`tests/fixtures/controller_reference_v1.json` 是固定输入与预期输出，不是项目状态文件。
`tests/test_controller_reference_samples.py` 直接驱动生产的执行器、命令生成、MotorBridge
适配驱动、生命周期和停止组件；设备句柄为内存替身，不打开真实串口。

量化反馈修复后，停止组件使用 `controller_stop_reference_v2.json`：窗口改为
0.20 s，增加三个位置编码的全极差和固定锚点约束、150 ms 连续性检查。
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
禁止 manager 穿透组件私有成员或为构造好的状态保留 `getattr` 夹具回退。该边界为后续
C++ 输入/输出与故障决策对照提供稳定入口，但不宣称已经消除所有物理耦合或完成 C++ 实现。

版本为 `schema_version=1`。位置 rad、速度 rad/s、力矩 N·m、时间为单调秒；
数值绝对容差 `1e-9`、相对容差 0，状态、代次、调用顺序及回执电机名精确匹配。
场景之间时钟和状态重建；C++ 应读取同一 JSON，不生成另一套“期望结果”。

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
进入批次 7 前仍按仓库规定执行完整软件检查；结构变更后的真机验证另按现场授权安排。
