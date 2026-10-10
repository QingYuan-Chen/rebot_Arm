# reBotArm Real2Sim 实时跟随

## 目标

该模块复现“拖动真实 reBotArm，MuJoCo 机械臂实时跟随并与虚拟物体交互”的效果。
它不是强化学习训练，也不要求保存数据。当前 Bridge 默认只读 ROS 2 状态，不调用服务、
Action、CAN 或串口，不向实机发送运动指令。

```text
/rebotarm/joint_states ─┐
/rebotarm/gripper/state ├→ 映射/滤波 → MuJoCo → /real2sim/joint_states
/rebotarm/arm_status ───┘                         → /real2sim/status
```

仿真输出使用 `/real2sim/*`，不能与输入 `/rebotarm/*` 相同，防止节点订阅自己的输出。

## 两种模式

`mirror`：

- 每次收到状态后直接同步 MuJoCo 的关节位置和速度；
- 仿真仍推进自由物体和接触，然后再次对齐机械臂；
- 视觉跟随误差接近零，适合复现视频中的丝滑效果；
- 运动学强制同步可能向接触系统注入额外能量，接触力不能直接当作实机真值。

`physics`：

- 实机位置作为 MuJoCo POS_VEL 控制目标；
- 控制器计算力矩，机械臂通过动力学跟随；
- 会出现合理跟踪误差和延迟，适合检查抓取、碰撞和控制参数。

推荐先用 `mirror` 校验方向、零位和实时显示，再用 `physics` 检查动力学。

## 映射与滤波

配置文件：

```text
src/rebotarm_simulation/config/real2sim_mapping.yaml
```

可配置：

- `source_joint_names`：实机消息中的六关节名称；
- `position_scale`：方向映射，反向关节可使用 `-1.0`；
- `position_offset`：实机零位到 MuJoCo 零位的偏移；
- `filter_alpha`：低通滤波系数；
- `max_position_jump_rad`：单帧最大允许位置跳变；
- `gripper_scale`、`gripper_offset_m`：夹爪宽度映射。

正式连接实机前必须逐关节校验这些值，不能默认认为全为 `1/0` 就一定正确。

## 无实机验收

在 Ubuntu VM 中执行：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/rebotarm_simulation:$PYTHONPATH"

python3 -m rebotarm_simulation.diagnostics.real2sim_acceptance \
  --mode mirror \
  --steps 200

python3 -m rebotarm_simulation.diagnostics.real2sim_acceptance \
  --mode physics \
  --steps 200
```

报告必须满足 `ok=true`、`hardware_connected=false` 和 `finite_state=true`。

## 启动只读 Bridge

只启动无界面 Bridge，不会启动 `rebotarmcontroller`：

```bash
ros2 launch rebotarm_simulation real2sim_bridge.launch.py
```

Bridge 没有状态输入时只等待，不会打开硬件设备。

启动带界面的实时跟随：

```bash
python3 -m rebotarm_simulation.apps.real2sim_viewer \
  --ros-args \
  --params-file src/rebotarm_simulation/config/real2sim_bridge.yaml \
  -p mode:=mirror
```

按 `Q` 或 `Esc` 关闭 Viewer。

不连接实机时，可以在另一个终端发布测试状态：

```bash
ros2 topic pub -r 30 /rebotarm/joint_states sensor_msgs/msg/JointState \
  "{name: [joint1, joint2, joint3, joint4, joint5, joint6], position: [0.0, -0.8, -1.0, 0.3, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

夹爪测试状态：

```bash
ros2 topic pub -r 30 /rebotarm/gripper/state rebotarm_msgs/msg/JointMotorState \
  "{joint_name: gripper, position: 0.05, velocity: 0.0, torque: 0.0, status_code: 0}"
```

## 实机联调边界

完整视频效果最终需要实机控制器发布状态。Bridge 本身仍保持只读：

- 不创建控制 service client；
- 不创建 trajectory action client；
- 不发布实机目标 topic；
- 不切换实机控制模式；
- 不启动 `rebotarmcontroller`。

实机进入重力补偿拖动模式属于独立的硬件操作，必须在逐关节映射、急停和空间安全检查
通过后由操作者明确执行。建议联调顺序：

```text
1. 实机保持静止，只读检查六关节和夹爪方向
2. 单关节小范围移动，校准 scale/offset
3. mirror 模式检查 Viewer 跟随
4. 确认急停和超时保持
5. 再由操作者进入重力补偿拖动
6. 最后测试 physics 模式和虚拟抓取
```

## 当前未包含

- 默认不保存操作轨迹或训练数据；
- 不把 MuJoCo 接触力反馈给实机电机；
- 不自动进入重力补偿模式；
- 不声明当前 identity 映射已经过实机标定。
