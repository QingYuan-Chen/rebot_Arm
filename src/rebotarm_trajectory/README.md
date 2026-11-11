# rebotarm_trajectory

控制器、motion 和 MuJoCo 共用的纯数值轨迹契约包。模块平铺包根目录，使用 Python
标准库；ROS 消息适配、线程、时钟、设备和物理引擎由调用方拥有。

| 模块 | 职责 |
| --- | --- |
| `polynomial.py` | 不可变轨迹点、具名分段系数、维度/有限性/时间校验及 q/v/a/jerk 求值 |
| `sampling.py` | 离线位置记录的样本校验和线性采样 |
| `joint_contract.py` | 完整六轴关节集合及顺序规范化 |
| `capability.py` | 生效约束摘要、continuous 版本化能力编码/校验 |
| `planning.py` | 规划起点快照、最终曲线摘要、执行/参考/失能预览用途与上下文绑定 |

motion 负责生成系数、重定时、段内限值及碰撞检查；控制器和仿真后端负责资源预约、
执行时间轴、取消和最终接纳。调用方取得数值载荷后仍须执行自己的状态及资源检查。

接口关系见[架构说明](../../docs/implemented/architecture.md#共享轨迹内核)，
操作与准备流程见[P005 执行说明](../../docs/implemented/features/p005_prepared_trajectory.md)。
