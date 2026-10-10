# reBotArm MuJoCo 仿真验收

本文档用于验收当前 MuJoCo 仿真底座。正式验收以 Ubuntu VM 为准；Windows 只作为代码保存、Git 管理、快速单元测试和 MJCF 一致性检查环境。

当前验收目标是：模型、场景、无实机 ROS 2 接口、Python API、以及仿真电机控制层可用；不连接真实机械臂，不做强化学习训练。

## 安全边界

所有命令都必须保持无实机：

```text
use_hardware:=false
不启动 rebotarmcontroller 实机驱动
不打开 CAN、串口或真实夹爪
不发送实机使能、回零、轨迹命令
```

## 环境分工

```text
Windows：
保存代码、编辑代码、Git 操作、快速 pytest、URDF/MJCF 一致性检查。
Windows 上的 MuJoCo 运行只算可选快速检查，不作为正式验收结果。

Ubuntu VM：
正式运行验收环境。MuJoCo headless、GUI/EGL、ROS 2、MoveIt 联调都以 VM 结果为准。
```

## 当前应验收的功能

```text
1. MuJoCo 模型可加载，joint/actuator 数量正确
2. robot.xml 由 URDF 自动生成，且与 URDF 一致
3. 底层 actuator 使用 motor，ctrl 表示力矩/力
4. 上层 API 仍使用位置目标和夹爪宽度目标
5. joint1-3 使用 DM4340P，峰值 27 N·m
6. joint4-6 使用 DM4310 V1.2，峰值 12.5 N·m
7. POS_VEL 仿真控制器能把位置目标转为 MuJoCo joint torque
8. 夹爪 MIT 控制器能把开合宽度转为滑动指等效力
9. reset、reset_home、save_state、restore_state 可用
10. CLI、Viewer、ROS 2 action/service 可用
```

## Windows 可选快速检查

这部分只用于提交前快速发现代码错误，不是正式运行验收。

```powershell
cd "C:\Users\Green Bone\.config\superpowers\worktrees\reBotArmController_ROS2-main\tabletop-scene"
python -m pytest tests/test_mujoco_motor_control.py tests/test_mujoco_sim_core.py tests/test_mujoco_model_contract.py tests/test_urdf_to_mjcf.py -q
```

当前预期：

```text
64 passed
```

完整回归：

```powershell
python -m pytest -q
```

当前预期：

```text
635 passed, 1 skipped
```

## MJCF 一致性检查

Windows 和 Ubuntu VM 都可以执行。正式验收时建议在 VM 也跑一次。

检查仓库里的 `robot.xml` 是否仍由当前 URDF 生成：

```powershell
$env:PYTHONPATH="src/rebotarm_simulation"
python -m rebotarm_simulation.model_tools.urdf_to_mjcf --repo-root . --output src/rebotarm_simulation/models/rebotarm/robot.xml --check
```

Ubuntu 等价命令：

```bash
export PYTHONPATH=src/rebotarm_simulation
python3 -m rebotarm_simulation.model_tools.urdf_to_mjcf --repo-root . --output src/rebotarm_simulation/models/rebotarm/robot.xml --check
```

如果 URDF 改过，需要重新生成：

```bash
export PYTHONPATH=src/rebotarm_simulation
python3 -m rebotarm_simulation.model_tools.urdf_to_mjcf --repo-root . --output src/rebotarm_simulation/models/rebotarm/robot.xml
```

重点检查：

```text
joint4/joint5/joint6 actuatorfrcrange = -12.5 12.5
joint4/joint5/joint6 motor ctrlrange = -12.5 12.5
joint4/joint5/joint6 motor forcerange = -12.5 12.5
```

## Ubuntu VM 基础验收

在 VM 中进入同步后的仓库：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=src/rebotarm_simulation
```

运行核心测试：

```bash
python3 -m pytest tests/test_mujoco_motor_control.py tests/test_mujoco_sim_core.py tests/test_mujoco_model_contract.py tests/test_urdf_to_mjcf.py -q
```

Headless 运行：

```bash
python3 -m rebotarm_simulation.apps.mujoco_cli run --duration 5
```

预期：

```text
输出 JSON
simulation_time >= 5
joint_positions、joint_velocities、actuator_forces 都是有限数
无 Python 异常
```

推荐的基础总验收入口：

```bash
python3 -m rebotarm_simulation.diagnostics.mujoco_acceptance --skip-renderer
```

预期输出 JSON，`ok=true`，并包含 `health`、`headless_reach_batch` 和
`cube_contact` 三个 step。若当前终端已 source ROS 2 和 `install/setup.bash`，
可追加 ROS 2 接口验收：

```bash
python3 -m rebotarm_simulation.diagnostics.mujoco_acceptance --skip-renderer --include-ros --timeout 30
```

EGL 渲染健康检查：

```bash
MUJOCO_GL=egl python3 -m rebotarm_simulation.diagnostics.mujoco_health --renderer-timeout 30
```

如果 VM 没有可用 EGL，允许：

```text
ok = true
renderer_available = false
renderer_error 有明确说明
```

但必须满足：

```text
model_loaded = true
physics_step_finite = true
joint_count = 8
actuator_count = 8
```

## Ubuntu GUI 验收

在 VM 图形桌面终端运行，不建议在普通无显示 SSH 中运行：

```bash
python3 -m rebotarm_simulation.apps.mujoco_viewer --duration 60
```

手动检查：

```text
1. 机械臂位于桌面场景
2. R 能回零位，T 能回到 home 姿态
3. G/H/P 能切换重力补偿、保持、Position（内部 POS_VEL）模式
4. Z/X 选择关节，按住 J/K 能连续移动当前关节
5. C/O 能连续闭合/打开夹爪，夹爪 visual 可见且能接近/接触方块
6. 终端状态栏显示 mode、当前关节实际角 q、速度 dq、target、夹爪实际/目标宽度、接触数量和最大接触力
7. 右侧原始 Control 面板默认隐藏；窗口叠加层显示请求/施加力矩、饱和和模式
8. 空格暂停，. 单步，Q 退出
9. 机械臂不飞、不爆、不明显穿桌
```

注意：现在底层是 motor 力矩控制，刚度和响应由仿真控制器与标定参数决定。如果感觉软、慢、抖，这是后续调参验收项，不代表模型加载失败。
模型没有叠加旧的手写机械臂；场景只 include `robot.xml`。如果看起来有透明外壳或边缘重叠，通常是 MuJoCo Viewer 打开了 collision
group，collision STL 仍参与碰撞但现在显示为半透明调试层，正常观察以 visual STL 为准。

## Python API 验收

在 Ubuntu VM 仓库根目录执行：

```bash
export PYTHONPATH=src/rebotarm_simulation
python3 - <<'PY'
from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

with RebotArmMujoco() as sim:
    state0 = sim.reset_home(seed=7)
    print("home", state0.joint_positions)
    print("targets0", sim.control_targets)

    sim.command_joint_positions([0.1, -0.2, -0.2, 0.2, 0.0, 0.0])
    sim.command_gripper_width(0.05)
    saved = sim.save_state()
    state1 = sim.step(100)
    print("time1", state1.simulation_time)
    print("forces", state1.actuator_forces)
    print("targets1", sim.control_targets)

    sim.restore_state(saved)
    state2 = sim.step(100)
    print("time2", state2.simulation_time)
    print("ok")
PY
```

预期：

```text
能打印 home、targets、forces
forces 不全为 0
无异常
最后输出 ok
```

## Ubuntu ROS 2 接口验收

先构建：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
python3 -m colcon build --symlink-install --packages-select rebotarm_simulation
source install/setup.bash
```

终端 1 启动 MuJoCo ROS 2 后端：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

终端 2 检查接口：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/a/project/rebot_Arm
source install/setup.bash

ros2 node list
ros2 topic echo /rebotarm/joint_states --once
ros2 topic echo /clock --once
```

发送六轴轨迹：

```bash
ros2 action send_goal /rebotarm/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6], points: [{positions: [0.1, -0.2, -0.2, 0.2, 0.0, 0.0], time_from_start: {sec: 3}}]}}" \
  --feedback
```

发送夹爪目标：

```bash
ros2 service call /rebotarm/gripper/set rebotarm_msgs/srv/SetGripper \
  "{position: 0.05, max_effort: 0.5}"
```

停止轨迹：

```bash
ros2 service call /rebotarm/trajectory_stop std_srvs/srv/Trigger "{}"
```

预期：

```text
/rebotarm/joint_states 持续发布
/clock 持续发布
action 能收到反馈和结果
夹爪 service 返回成功
全程不需要 use_hardware:=true
```

自动化等价命令：

```bash
python3 -m rebotarm_simulation.diagnostics.mujoco_ros_acceptance --timeout 15
```

## MoveIt 联调验收

终端 1：

```bash
ros2 launch rebotarm_simulation mujoco_sim.launch.py
```

终端 2：

```bash
ros2 launch rebotarm_bringup interactive_system.launch.py \
  use_moveit_preview:=true \
  use_hardware:=false \
  use_moveit_fake_joint_states:=false \
  start_passive_joint_state_publisher:=false \
  use_sim_time:=true
```

验收点：

```text
1. /rebotarm/joint_states 只有 MuJoCo 后端作为物理状态来源
2. MoveIt 控制器仍映射到 /rebotarm/follow_joint_trajectory
3. RViz 中规划后执行，MuJoCo 机械臂状态变化
4. 没有启动任何实机驱动
```

自动化等价命令需要先保持终端 1 和终端 2 正在运行，然后在第三个终端执行：

```bash
python3 -m rebotarm_simulation.diagnostics.mujoco_moveit_acceptance --timeout 30
```

预期输出 JSON，`moveit_plan_success=true`、`trajectory_action_success=true`、
`ok=true`。

## 不通过时如何判断问题

```text
模型加载失败：
检查 robot.xml 是否重新生成，STL 路径是否存在。

joint4-6 力矩仍是 7：
检查 rebotarm.urdf、reBot-DevArm_fixend.urdf、robot.xml 是否都已更新到 12.5。

机械臂软绵绵：
这是控制标定问题，优先看 motor_control_calibration.yaml 的 firmware_to_torque_scale、积分限幅和 effort 是否打满。

机械臂抖或飞：
优先降低 firmware_to_torque_scale，检查 dt、控制频率、碰撞穿插和惯量。

ROS 2 action 重复：
检查是否有多个 mujoco_sim.launch.py 或旧控制节点未退出。

GUI 无法打开：
优先用 headless 和 mujoco_health --skip-renderer 验证物理层。
```
