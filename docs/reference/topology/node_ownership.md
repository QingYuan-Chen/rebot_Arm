# 节点与包所有权

> 状态：REFERENCE；类型：节点归属和 launch 拓扑；适用范围：当前活动入口。

| 入口/节点 | 所属包 | 作用 | 主要依赖 |
|---|---|---|---|
| `reBotArmController` | `rebotarmcontroller` | 真机通信、反馈和执行安全 | MotorBridge/串口 |
| `PoseExecutionNode` | `rebotarm_motion` | 位姿规划/执行协调 | MoveIt、控制器 Action |
| `TeachRecorderNode` | `rebotarm_teach` | 示教录制 | 控制器反馈 |
| `TeleopKeyboardNode` | `rebotarm_teleop` | 键盘命令适配 | ROS services/actions |
| `TeleopStatusPanelNode` | `rebotarm_dashboard` | Web 页面、HTTP/SSE 和状态聚合 | teleop/teach/calibration clients |
| `move_group` | MoveIt | 规划、IK 和状态有效性检查 | `rebotarm_moveit_config` |
| `rebotarm_vision_node` / GraspNet nodes | `rebotarm_vision` | RGB-D、检测、候选和 Marker | Gemini 2/YOLO/GraspNet |
| `rebotarm_mujoco_node` | `rebotarm_simulation` | MuJoCo 状态和仿真执行 | MJCF、ROS messages |
| `rebotarm_handeye_capture` | `rebotarm_calibration` | 标定采样和会话 | Image/CameraInfo/TF |

唯一真机控制器所有者是 `rebotarm_bringup/launch/hardware_controller.launch.py`；其他 launch 只能 include 或转发参数。
