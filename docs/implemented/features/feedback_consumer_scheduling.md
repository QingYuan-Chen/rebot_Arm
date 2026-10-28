# 反馈消费端接收调度

运动、视觉与标定节点使用同进程内的独立接收节点和 `SingleThreadedExecutor`。
原业务节点保留多线程执行器，用于同步等待规划、Action 和 Service；接收节点不处理这些请求，
也不访问电机 SDK。组件分别由所属包拥有，不放入硬件控制器，不新增跨包依赖。

| 消费方 | 接收所有者 | 交接与约束 |
| --- | --- | --- |
| `pose_compat_node`、`pose_execution_node` | motion 的 `feedback_receiver.py` | 关节、控制器状态及按需夹爪状态；复制快照与独立停止证据，不获取运动业务锁 |
| `visual_grasp_executor_node` | vision 的 `grasp_input_receiver.py` | 有界计划/候选邮箱及 TF Buffer；业务侧按接收次序交接，输入接收不获取执行状态锁 |
| `handeye_capture_node` | calibration 的 `capture_input_receiver.py` | 有界图像/CameraInfo 缓存及 TF Buffer；检测、稳定性判断与落盘在业务侧完成 |

## 时间与安全语义

默认订阅 QoS 事件诊断保留。接收节点沿用父节点的有效话题重映射、命名空间、ROS 上下文
和 `use_sim_time`；节点名称独立，不继承父节点的名称重映射。快照复制消息或转移邮箱消息
的所有权，调用方不得修改接收缓存。接收时间使用单调时钟，消息头保留源时间，恢复时不刷新旧帧。
运动接收器用自身执行器推进的 ROS 时钟校验源年龄，父节点的同步等待不阻塞其 `/clock` 接收。

位姿兼容的关节新鲜度仍为 **150 ms**，控制器状态心跳仍为 **0.6 s**。
接收缓存每次替换旧帧前检查其年龄；查询快照也检查当前源年龄。关节空档或状态心跳中断
递增连续性代次，模式/使能往返变化也保留代次变化。即使业务线程没有在空档中运行，恢复后
收到的新鲜帧也不能让旧规划或旧执行请求继续。FJT 终态到达后仍检查该代次；取消继续要求
动作终态与新鲜静止反馈，失败只保持/阻断，不附带使能、失能或回位。

各用途的门限继续独立：运动层 `StopFeedback`、控制器 `ArmStop`、现场静止验收、视觉计划
有效期及标定采集窗口均保持原值。不能用删除 QoS 诊断、放宽门限或重新标记时间来替代调度修复。

视觉邮箱最多保存最新计划、最新候选和最近一次空候选撤销；撤销事件不会被随后非空候选
覆盖，从而重新激活旧计划。停止时关闭邮箱，确认停止后清空积压再开放，必须等新输入。
工作流在等待新计划时主动交接邮箱，不依赖多线程业务定时器及时调度。

标定话题配置通过 guard condition 交给接收线程：先创建新订阅，再交换代次、清空缓存及销毁
旧订阅。旧代次回调即使晚到，也不能填入新缓存。源图像、内参匹配、历史 TF 查询与连续稳定
窗口检查保持；快照复制、ArUco 计算与文件写入不占接收锁。

接收线程异常由快照/邮箱读取报告，位姿兼容和视觉输入保持执行门控；显式关闭先唤醒执行器并等待线程退出，再销毁
订阅和 TF 句柄。线程没有退出时保留句柄并报告失败。接收器关闭可重复调用。

## 无硬件验证

加载 Jazzy 和工作区 overlay 后，在已核对空闲的 localhost ROS 域执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  tests/test_feedback_consumers.py tests/test_pose_compat_protocol.py \
  tests/test_motion_stop_confirmation.py tests/test_visual_stop_confirmation_runtime.py \
  tests/test_handeye_capture_runtime.py -q
```

`tests/fixtures/feedback_consumers_probe.py` 拒绝导入电机 SDK/串口，只发布合成数据并使用内存
FJT 服务端。它检查同步服务持有业务锁时的持续接收、正式位姿服务的接纳与终态、独立观测
接收空档、恢复后旧代次拒绝、话题切换、重映射、独立仿真时钟及线程清理。探针同时断言业务锁
在整个观察窗口持续占用，避免服务提前结束造成假通过；第二参数可延长持锁观察窗口：

```bash
ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1 \
  python3 tests/fixtures/feedback_consumers_probe.py /tmp/feedback-probe-session 30
```

运行前确认域与目录用途；不并行运行使用同一域的测试。此入口不启动硬件控制器、不接触真实
通道，不证明实机长期反馈连续性。当前软件结果和硬件待验收边界以
[项目状态](../../reference/project_status.md) 为准。历史订阅空档的隔离证据见
[反馈延迟说明](hardware_execution.md#ros-订阅延迟的离线隔离)，不能将其中对照结果推广为全部
历史真机过期的唯一原因。
