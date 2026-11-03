# P005 共享轨迹与 prepared_v1 执行

> 范围：第二批软件和 MuJoCo；实机启用与运动验收属于第三批。

## 组件与契约

`rebotarm_trajectory` 提供不可变数据、结构校验、系数解码和时间采样。
`rebotarm_motion` 生成系数、规范化边界，检查段内位置、速度、加速度、jerk 与碰撞。
控制器拥有设备线程、发送确认、任务及资源；MuJoCo 拥有物理时间、控制模式和生命周期。
控制器、仿真都不导入 motion 实现，不把固件速度上限当作参考曲线的有符号速度。

| 输入 | 插值 | 连续性 |
| --- | --- | --- |
| q | 线性 | C0 |
| q/v | 三次 Hermite | C1 |
| q/v/a | 五次 Hermite | C2，jerk 不保证连续 |

同一轨迹混合缺失导数拒绝。C0/C1 段内导数检查不代表节点跳变满足高阶限值。
网页点对点和键盘由 motion 构造首末完整 q/v/a 约束；prepared 键盘默认为
`0.02 rad / 0.4 s`，独立起停、忙时拒绝、不排队。legacy 时长仍为 0.2 s。
PoseCompat 保留 q-only；示教保留完整可用字段和非零中间导数，混合字段拒绝。
MoveIt 对齐段和录制段的导数等级不同时明确拒绝，不能删除规划导数来迁就录制字段。

`/<ns>/follow_joint_trajectory` 始终由选定的唯一后端提供。
`/<ns>/prepare_arm_trajectory` 是内部服务，契约位于
[PrepareArmTrajectory.srv](../../../src/rebotarm_msgs/srv/PrepareArmTrajectory.srv)。
后端从 Goal 接收开始预留六轴和夹爪，准备期间继续保持；期限为
`min(2.0 s, 0.2T + 0.25 s)`，使用同机单调时钟。MoveIt 预算仍为 `1.2T + 0.5 s`。
准备失败、超时、超限、碰撞均拒绝，不拉长时间、裁剪或降低连续性。

起点只从新鲜静止反馈且满足既有 `POSITION_SPAN` 时规范化到最后成功发送的保持目标；
首末已有导数归零，中间导数、原时间及末点位置不变。返回的精确系数绑定会话、
执行代次、保持目标、固定夹爪状态/代次及场景版本。实际接纳再次原子复核；
取消、停止、失能、重连和已观察到的绑定变化使结果失效，迟到结果不能发送。
普通夹爪运动在准备、执行和停止确认期间拒绝，保护性释放保留可达并撤销绑定。

碰撞检查复制完整 RobotState（包括附着物），填入六轴和固定双指，在同一
MoveItPy `read_only()` 场景锁中完成所有样本。按段内速度极值安排 ≤0.02 rad 的样本跨度，
包括内部位置极值；超预算拒绝，不抽稀。场景摘要包含环境、附着物、碰撞规则和固定变换，
排除反馈关节与时间戳。执行中已观察到相关环境变化即停止保持，不承担动态避障。

实机周期按单调时间、MuJoCo 每个物理步按仿真时间直接采样，不补发过期点。
只有完整发送成功才推进确认；正常完成保留最终命令目标，成功仍要求末点发送后的新反馈。
物理收敛观察与 Action 成功判定独立，不修改既有容差。

## 模式与启动

`trajectory_execution_mode` 只接受 `legacy / prepared_v1`。
后端以锁存 `/<ns>/trajectory_execution_mode` 发布能力；网页和键盘必须观察到匹配模式，
不匹配或能力未知拒绝，无失败后回退。真实硬件默认 legacy，专用 safe_home 等路径保留原语义。
日常 `mujoco_moveit_sim.launch.py` 已默认 prepared_v1，终端键盘须显式选择相同模式。
轻量 RViz preview 发布 legacy 能力，保留旧入口兼容，不提供 prepared_v1。

先构建并 source 安装区：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rebotarm_bringup
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py \
  use_rviz:=false use_mujoco_viewer:=false trajectory_execution_mode:=prepared_v1 \
  use_dashboard:=true dashboard_port:=18088 \
  trajectory_trace_path:=/tmp/p005_physics.jsonl
```

此链仅加载 MuJoCo、MoveIt、准备服务和可选 Dashboard，不加载真实控制器。
Dashboard 的 `simulation_backend=mujoco` 消费物理后端的反馈，夹爪发往仿真服务，
不再发布模拟落点污染物理反馈。`use_dashboard` 默认 false，不改变既有界面启动偏好。
终端键盘可另启动 `TeleopKeyboardNode --ros-args -p trajectory_execution_mode:=prepared_v1`。

Jazzy 2.12.4 实测：MoveItPy 位于 `moveit.planning`；关闭它默认提供的规划场景服务，
只由 move_group 提供 `/get_planning_scene`。C++ topic 常量通过 remapping 设置；
MoveItCpp 使用 `planning_pipelines.pipeline_names`。提前声明 /clock QoS 避免
TEM 延迟启用仿真时间时拒绝新参数导致异常。准备节点不扩大 MoveIt 执行预算。

## 验收与记录

共享内核等价性、分级连续性、段内极值、系数往返、非法输入与超限拒绝由专项回归覆盖。
正式控制器加假驱动覆盖周期发送、延迟跳点、部分写失败、迟到结果、取消/失能/重连、
忙时拒绝、固定夹爪、保护释放和场景变化。它们均不连接真实设备。

冻结模型、增益与限值的物理对照及真实 ROS 操作链：

```bash
python3 scripts/validate_p005_simulation.py \
  --output log/p005_acceptance --ros --dashboard http://127.0.0.1:18088
```

工具记录资源摘要、参考 q/v/a/jerk、实际 q/v/力矩和模式。
离线对照以相同物理起点比较五次曲线与旧阶梯；正式链另以每物理步 JSONL 记录执行。
固定单步最大误差要求 ≤0.005 rad 且优于旧方案；结束后独立 5 s 观察内达到
q 误差 ≤0.001 rad、速度 ≤0.01 rad/s，持续 0.5 s；静止 position→hold
观察 1 s，漂移 ≤POSITION_SPAN 且支撑力矩不被清零。
旧越限目标现在在状态变更之前拒绝，position→hold 保留控制器记忆。
位置模式下更新参考目标不再额外推进 PI 积分或力矩滤波；只在配置的物理控制节拍
更新控制器，目标采样频率不能改变其实际更新频率，模式切入仍保留原路径。

第一批 jerk 超限 MoveIt 样本保留为拒绝回归；至少一条符合原有限制的真实 MoveIt
规划必须通过 ExecuteTrajectory、simple_controller_manager 和新后端实际完成。
当前验证结果与剩余边界统一记录于 [项目状态](../../reference/project_status.md)，
第一批提交为 `39028d1`。软件和仿真结果不构成实机验收。

### 2026-11-03 验证结果

最终全量回归 **1743 passed / 2 skipped**（两条既有 Gymnasium 提示，235.45 s），
分层 **27 passed**。规定 compileall 及共享包、控制器、仿真编译检查通过，
123 个本地文档链接和差异检查通过；收尾无本轮测试进程残留、真实串口无占用。
完整回归日志为 `log/p005_batch2_20261103/full_tests_final.log`。

非软链接构建 13 包通过，隐藏源码和原 build/install 后，仅从新安装区启动 MoveIt、
准备服务、MuJoCo 和 Dashboard；`/dev` 隔离真实串口，localhost 独立域运行。
Fast DDS 在该隔离方式下须共享 `/dev/shm`，否则跨隔离客户端发现不到完整反馈。
完整操作链通过实际 MoveIt 规划执行、网页点对点、网页/终端键盘、C0/C1 兼容、
固定夹爪忙拒绝、执行中取消与场景变化停止、原始 jerk 样本以及环境/附着物碰撞拒绝。
终端经 PTY 真正读键，第二键在首步结束前明确拒绝，最终只完成一个步长。

| 冻结物理对照 | 旧阶梯 | prepared_v1 | 要求 |
| --- | --- | --- | --- |
| 0.02 rad / 0.4 s 最大跟踪误差 | 0.020000 rad | 0.003398 rad | ≤0.005 rad 且改善 |
| 结束后达到并保持 0.5 s 的观察耗时 | 0.842 s | 0.504 s | ≤5 s |
| position→hold 后 1 s 最大漂移 | 0.0000486 rad | 0.0000266 rad | ≤0.001144 rad |
| 切换瞬时最大支撑力矩变化 | 0.007958 Nm | 0.004352 Nm | 不被清零 |

正式 ROS 首个单步独立物理记录：最大跟踪误差 0.002812 rad，结束后 0.504 s
内满足相同收敛观察要求。冻结模型、增益、限值和 MoveIt 预算未修改。
真实 MoveIt .03 加速度缩放规划被 jerk 限值拒绝；独立 .01 规划通过原路径实际执行，
未修改其时间。该结果不要求所有 MoveIt 规划均能通过五次重建检查。

证据在 `log/p005_batch2_20261103/`；先前安装区及默认启动验证保留在
`installed_acceptance_v8/`、`default_acceptance/`。控制频率复核发现目标写入额外推进 PI，
修复后以原配置重新物理对照及默认链验收，最终结果在 `validated_acceptance/`，
逐物理步记录在 `validated_physics.jsonl`，完整链日志在 `validated_chain.log`。启动阻断硬件控制器、
MotorBridge 和串口模块导入。
安装区导入、新接口及源码/真实串口隔离检查通过；轻量 preview 的迟到订阅确认收到 legacy 能力。
负例日志及首次因隔离通信、反馈年龄、测试场景清理或终端发现时机导致的失败均保留。
反馈过期和准备超时保持明确拒绝，不通过放宽门限、重打时间戳或自动重试解决。
复核中发现 rclpy 回调池竞争延迟，准备节点改为单执行器，场景摘要定时器不可重入；
它们仍通过同一场景锁串行，不降低碰撞采样或场景摘要频率。最终链 13 次准备请求的
快照输入年龄最大 19.5 ms；此单次结果不是最坏耗时保证。
