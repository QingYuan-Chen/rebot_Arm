# rebotarm_teleop

操作者命令适配包。它把键盘、Web 点动和夹爪显示输入转换成 ROS 命令，集中做参数、限位、轨迹插值和服务调用适配；不渲染 Dashboard，不实现示教回放算法，也不直接访问硬件。

## 目录结构

```text
rebotarm_teleop/
├── rebotarm_teleop/
│   ├── teleop_core.py                  # 键盘/Web 命令映射、限位和限速纯逻辑
│   ├── teleop_keyboard_node.py         # 终端按键 -> 关节增量轨迹
│   ├── web_execute.py                  # Web 关节/夹爪请求校验和插值
│   ├── web_teleop_client.py            # Web 命令 -> controller Action/service
│   ├── gripper_visual_model.py         # 夹爪开口 -> 左右手指位移
│   ├── gripper_visual_joint_state_node.py # RViz 可视化关节状态桥
│   ├── service_call_helpers.py         # Trigger 服务调用
│   ├── parameter_helpers.py             # 参数结构与传感器 QoS
│   └── __init__.py
├── setup.py / package.xml / resource/*
```

## 关键流程

- `teleop_core.py` 是键盘和 Web 共用的纯逻辑层，负责步长、目标关节、限位和限速判断。
- `teleop_keyboard_node.py` 轮询终端输入，调用 `TeleopTargetPlanner` 组装目标轨迹。
- `web_execute.py` 先校验 Web payload，再用 `smoothstep`/插值生成 `JointTrajectory`；`web_teleop_client.py` 负责把结果送到控制器并跟踪 future。
- `gripper_visual_joint_state_node.py` 只为 RViz 把真实夹爪的一路开口反馈映射成 URDF 的左右手指关节，不会改变真实夹爪。
- `service_call_helpers.py` 统一停止轨迹、使能/失能等 Trigger 调用的等待和错误返回。

## 对外入口

```bash
ros2 run rebotarm_teleop TeleopKeyboardNode
ros2 run rebotarm_teleop GripperVisualJointStateNode
```

`GripperVisualJointStateNode` 也被 `rebotarm_bringup/moveit_demo.launch.py` 复用。Web 适配通常由 `rebotarm_dashboard` 通过对象接口调用，不在本包提供 HTTP server。

## 边界与安全

```text
键盘/Web/RViz 操作意图 -> rebotarm_teleop -> motion/controller ROS 接口
```

限位校验不是碰撞检查，也不是硬件最后防线；真实执行仍需经过 `rebotarm_motion`、`rebotarmcontroller` 的状态/反馈/使能门。

