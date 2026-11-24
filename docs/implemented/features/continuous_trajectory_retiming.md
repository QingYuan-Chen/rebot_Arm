# 连续轨迹曲线整定与执行

> 状态：IMPLEMENTED；类型：连续轨迹功能说明；适用范围：`rebotarm_trajectory`、`rebotarm_motion`、`rebotarmcontroller`、`rebotarm_simulation`；软件与仿真验证不等于真机定量精度验收。

轨迹执行统一使用 `continuous`。旧 legacy 离散执行模式与轻量预演后端已退役，模式话题仅保留为只读能力声明，启动与节点参数不再提供模式选择。本页记录当前生效的连续轨迹曲线整定、起点绑定、碰撞准备、组合执行与退出兼容契约；真机定量精度与整机硬件验收状态统一维护在[项目状态](../../reference/project_status.md)。

## 协同接口

公共 `/rebotarm/follow_joint_trajectory` 仅接受完整六轴 `joint1..joint6`（单位 rad）；七轴、单手指及携带右指的旧 FJT 目标明确拒绝，不静默投影。六轴 `q`/`q+v`/`q+v+a` 的 C0/C1/C2 分级兼容与版本化系数载荷保留。

网页 continuous 的关节、键盘和组合操作统一使用 `/rebotarm/execute_teleop_motion`（[ExecuteTeleopMotion](../../../src/rebotarm_msgs/action/ExecuteTeleopMotion.action)）：

- **Goal**：完整六轴轨迹、`move_gripper`、`gripper_position`（双指总开口 m）、`gripper_max_effort`。
- **Feedback**：`preparing`/`executing`/`stopping` 状态、机械臂进度、夹爪状态、实际开度及六轴参考/反馈。
- **Result**：整体终态、`arm_state`、`gripper_state` 和失败原因；提交或单侧完成不作为整体成功。

后端准备通过后在同一控制周期登记六轴与独立夹爪目标。夹爪沿用原有独立限速、到位、超时与中性释放规则，不参与 FJT 插值或机械臂重定时。两侧分别到位并完成清理后整体成功，不承诺同时到达或总线原子同步；机械臂正常结束后继续保持最终命令终点。取消、停止、单侧失败或重连统一终止组合任务，停止确认前保留预约；健康可恢复失败保持使能。

`/rebotarm/trajectory_capabilities` 是版本 3 的只读 JSON 快照，声明规划绑定 v1、六轴 FJT、网页 Action v1、独立位置夹爪、系数版本、生效夹爪速度/力矩上限、模型限值摘要和后端会话身份。规划结果绑定其 revision，缺失、过期、重连或配置不一致时明确拒绝；该快照不代表使能或运动许可。

规划阶段通过 `/rebotarm/retime_arm_trajectory` 缩放六轴轨迹的时间、速度与加速度：

- 最终执行计划先读取后端只读规划快照，将首点规范化到已发送保持目标，保留其余节点与相对时序；已有首末导数归零，中间导数保留。
- 自动模式返回最终时长；固定时长模式超限时返回所需时长并拒绝。原始节点先校验，位置越限或非法输入不能靠起点替换或重定时掩盖。
- 按实际多项式曲线解析计算极值，并在纳秒编码后重建复核；执行准备服务只复查已绑定的权威曲线，不再改写首点或时长。
- 重定时服务总期限 `2.0 s`，准备服务期限 `min(2.0 s, 0.2T + 0.25 s)`，MoveIt 执行预算 `1.2T + 0.5 s`（`T` 为最终规划时长）。

MoveIt 响应链为 `TOTG → Ruckig → 执行曲线重定时 → ValidateSolution → DisplayMotionPath`。`rebotarm_motion` 采用混合构建，C++ 响应插件调用 Python 重定时服务并缓存 MoveIt 匹配值与权威纳秒轨迹，由控制器管理装饰器匹配后下发权威轨迹及绑定；缓存缺失时要求重新规划。`ompl_reference` 仅供示教起点对齐等中间段保留虚拟起点，不可直接执行。

执行准备绑定两种碰撞策略：网页 Action 固定使用 `WEB_CLOSED`（计算副本中双指闭合为零）；普通 FJT、MoveIt 机械臂、示教和视觉查询使用 `FIXED_CURRENT`（准备时获取的真实开度，全段固定）。替代开度只作用于本次计算副本，不改变反馈、界面显示、公共规划场景或附着物。同一 MoveItPy 只读场景快照检查完整机器人及附着物，覆盖全部六轴节点及段内位置极值，相邻跨度 ≤ `0.02 rad`；超预算、超时或覆盖不完整均拒绝，不抽稀，也不宣称连续碰撞证明。执行期观察到相关场景变化时整体停止保持。

MoveIt 保留 `arm`（六轴 FJT）与 `gripper`（`/rebotarm/gripper/moveit_command` 独立 `GripperCommand`，左指 m ×2 映射到总开口，反馈/结果 ÷2）两个独立规划组，不提供 `arm_with_gripper`。MoveIt 执行起点容差与实机普通轨迹终点到位容差均为 `0.03 rad`（前者比对规划首点与 MoveIt 当前状态，后者比对实测反馈与命令终点），独立夹爪到位容差为电机角 `0.12 rad`。关节状态桥分开维护机械臂与夹爪源时间戳，避免新机械臂帧刷新过期手指。

网页仅编辑六轴关键点与单次夹爪目标，组合按钮交接一次性 `plan_id`；失能态仅允许预览，使能后须重新预览；会话、代次、保持目标或能力变化均使旧预览失效。终端键盘与网页键盘点动默认 `0.02 rad / 0.4 s`，独立起停、忙时拒绝、不排队。

## 规划起点绑定

内部服务 `/rebotarm/get_arm_planning_state` 只读复制后端会话、跨任务执行代次与已发送保持目标，不预约也不下发运动命令（硬件控制器在设备队列内复制一致快照，MuJoCo 按目标锁→仿真锁复制）。服务失败、忙、断流或超时不可降级为预览；仅明确失能态返回仅预览结果。

`RetimeArmTrajectory.planning_purpose` 区分 `FINAL_EXECUTION` 与 `REFERENCE` 并返回 `executable`。最终计划通过 `rebotarm-plan:v1:` 绑定能力版本、会话、代次、保持目标与完整 `q/v/a`/整数纳秒摘要。任何其他六轴运动、Stop（含空闲 Stop）、失能或重连都会使旧计划失效，后端预约前即拒绝，无自动重规划或重试。无绑定的旧六轴 FJT 保留原分级准备兼容，不自动重定时；已声明的参考计划或失能预览不能借此降级执行。

规划重定时节点收到 `SIGINT`/`SIGTERM` 后，先停止自旋并在有效 ROS context 内排空工作线程回调与服务回复，再释放执行器句柄与节点，避免 Jazzy 多线程执行器关闭竞态。可通过显式设置 `planning_trace_path` 记录完整输入/快照/输出用于诊断，默认关闭。

## 组件与契约

- `rebotarm_trajectory`：纯标准库数值内核，提供不可变轨迹点、分段系数、结构/有限性/时间校验、系数解码、能力与规划绑定编码以及 `q/v/a/jerk` 求值。
- `rebotarm_motion`：系数生成、边界规范化、重定时、段内极值校验与 MoveItPy 场景碰撞检查（[PrepareArmTrajectory.srv](../../../src/rebotarm_msgs/srv/PrepareArmTrajectory.srv)）。
- `rebotarmcontroller`：唯一设备线程、发送确认回执、组合预约与停止确认。
- `rebotarm_simulation`：物理时间步进、仿真控制模式、轨迹预约、采样执行与生命周期管理。

控制器与仿真均单向依赖 `rebotarm_trajectory`，不得导入 `rebotarm_motion` 实现。

| 输入字段 | 插值方式 | 连续性等级 |
| --- | --- | --- |
| `q` | 线性 | C0 |
| `q / v` | 三次 Hermite | C1 |
| `q / v / a` | 五次 Hermite | C2（不保证 jerk 连续） |

同一轨迹混合缺失导数时明确拒绝。网页点对点与键盘点动由 motion 构造首末完整 `q/v/a` 约束；`PoseCompatNode` 保留 `q`-only；示教保留输入等级（缺少加速度的 `q/v` 记录采用 C1 拼接，详见[示教 C1 处理](teach_replay.md#缺少加速度的记录与-c1-拼接)）。

执行期实机按单调时间、MuJoCo 按仿真时间直接采样当前参考，不补发过期点；仅完整发送确认后才推进进度。`position -> hold` 保留位置控制器积分与上一拍支撑力矩，避免切换跳变；底层位置命令越界在修改目标前拒绝。

## 模式与启动

硬件与 MuJoCo 统一固定为 `continuous`，后端持续发布只读 `/<ns>/trajectory_execution_mode=continuous` 及版本化能力快照。完整真机、MuJoCo、键盘与 RViz 组合自动包含 MoveIt 场景、重定时与准备服务；单独运行底层控制器或 `mujoco_sim.launch.py` 时需同域提供外部准备服务。

构建并启动 MuJoCo + MoveIt + 可选 Dashboard 仿真组合示例：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rebotarm_bringup
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 launch rebotarm_bringup mujoco_moveit_sim.launch.py \
  use_rviz:=false use_mujoco_viewer:=false \
  use_dashboard:=true dashboard_port:=18088 \
  trajectory_trace_path:=/tmp/continuous_physics.jsonl
```

真机操作顺序与安全门见[网页与键盘命令](../../reference/commands/rebotarm_common_commands.md#连续轨迹曲线整定用户实机功能验证)及[MoveIt 操作命令](../../reference/commands/rebotarm_feature_commands.md#真机-rviz-拖动)。

## MoveIt/MoveItPy 退出段错误兼容

配套 Jazzy MoveIt 2.12.4 / `rclcpp` 28.1.22 在退出阶段存在插件卸载顺序缺陷：轨迹执行管理器析构时先 `dlclose` 卸载 `libmoveit_simple_controller_manager`，随后 `rclcpp::CallbackGroup` 释放 Action 客户端弱引用控制块仍访问已卸载插件虚表，导致 `move_group` 与 MoveItPy 准备节点触发 `SIGSEGV`（上游同类问题参考 [moveit2#1597](https://github.com/moveit/moveit2/issues/1597)、[moveit2#3721](https://github.com/moveit/moveit2/issues/3721)）。

工程兼容处理集中位于唯一共用的 `moveit_demo.launch.py`：

- 仅对 `move_group` 与准备节点设置进程级 `LD_PRELOAD`，通过 ament 索引定位 `moveit_simple_controller_manager` 插件，使只读映射覆盖完整析构期；
- 保留用户已有 `LD_PRELOAD` 项，插件缺失在启动前报错，不影响硬件控制器或其他节点，也不修改系统 ROS 二进制或跳过正常析构；
- `rebotarm_bringup` 已显式声明该插件运行依赖；上游修复并通过退出回归后可移除此兼容处理。

## 验证与边界

软件与仿真验证覆盖共享内核数值等价性、分级连续性、段内解析极值、规划起点绑定、假驱动周期发送/取消/故障隔离，以及 MuJoCo + MoveIt + Dashboard 全链路操作。离线与仿真验收入口：

```bash
# 单条轨迹离线连续性与极值分析（不连接 ROS 或设备）
python3 scripts/analyze_p005_trajectory.py \
  tests/fixtures/p005_moveit_trajectory.json \
  --output /tmp/trajectory_analysis_report.json

# 冻结物理对照与 ROS 仿真链路验收（需先启动隔离域 MuJoCo/MoveIt/Dashboard）
python3 scripts/validate_p005_simulation.py \
  --output log/continuous_trajectory_acceptance --ros --dashboard http://127.0.0.1:18088
```

验证边界：

- 连续轨迹曲线整定功能已按用户实机功能反馈关闭，稳定代码已合入主干；
- 历史阶段排查日志与批次记录归档于 Git 历史，最新回归与验收结论以[项目状态](../../reference/project_status.md)为准；
- 软件与仿真指标不替代真机定量连续性/跟踪精度或整机硬件全项验收。
