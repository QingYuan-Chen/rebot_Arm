# 节点与包所有权

> 状态：REFERENCE；类型：节点归属和 launch 拓扑；适用范围：当前活动入口。

| 入口/节点 | 所属包 | 作用 | 主要依赖 |
|---|---|---|---|
| `reBotArmController` | `rebotarmcontroller` | 真机通信、反馈和执行安全 | MotorBridge/串口、`rebotarm_trajectory`、`rebotarm_force_control` |
| `PoseCompatNode` | `rebotarm_motion` | 位姿兼容 Service/Action（IK/笛卡尔转 FJT） | MoveIt、控制器 Action |
| `TrajectoryPreparationNode` / `TrajectoryRetimingNode` | `rebotarm_motion` | 连续轨迹曲线重定时、规划起点绑定与场景碰撞准备 | MoveItPy、`rebotarm_trajectory` |
| `TeachRecorderNode` | `rebotarm_teach` | 示教录制 | 控制器反馈 |
| `TeleopKeyboardNode` | `rebotarm_teleop` | 终端键盘点动与仿真分模式力控调参 | ROS services/actions、`/rebotarm/arm_status` |
| `TeleopStatusPanelNode` | `rebotarm_dashboard` | Web 页面、HTTP/SSE 和状态聚合 | teleop/teach/calibration clients |
| `move_group` | MoveIt | 规划、IK 和状态有效性检查 | `rebotarm_moveit_config` |
| `rebotarm_vision_node` / GraspNet nodes | `rebotarm_vision` | RGB-D、检测、候选和 Marker | D405/YOLO/GraspNet |
| `rebotarm_mujoco_node` | `rebotarm_simulation` | MuJoCo 状态、连续轨迹、重力补偿与双频 MIT 关节阻抗仿真后端 | MJCF、`rebotarm_trajectory`、`rebotarm_force_control` |
| `rebotarm_handeye_capture` | `rebotarm_calibration` | 标定采样和会话 | Image/CameraInfo/TF |

> 无节点共享库包：`rebotarm_msgs`（消息/服务/动作契约）、`rebotarm_trajectory`（纯数值连续轨迹内核）和 `rebotarm_force_control`（共享重力补偿与关节阻抗数值核心）不直接启动 ROS 节点。

唯一真机控制器所有者是 `rebotarm_bringup/launch/hardware_controller.launch.py`；其他 launch 只能 include 或转发参数。
