# Python / C++ 控制器对照样本

`tests/fixtures/controller_reference_v1.json` 是固定输入与预期输出，不是项目状态文件。
`tests/test_controller_reference_samples.py` 直接驱动生产的执行器、命令生成、MotorBridge
适配驱动、生命周期和停止组件；设备句柄为内存替身，不打开真实串口。

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
