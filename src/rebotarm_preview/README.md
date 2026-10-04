# rebotarm_preview

轻量 RViz 预演后端，不加载 MuJoCo、不计算动力学、不检查物理碰撞，也不连接真实硬件。

`rebotarm_sim_trajectory_controller` 提供 `FollowJointTrajectory`、关节状态、停止、
safe home 和夹爪设置接口，供 `rebotarm_bringup` 的键盘、RViz 拖动和视觉无硬件流程使用。
它只用于 ROS/MoveIt/视觉流程联调；需要物理、接触、执行器力或碰撞证据时，必须使用
`rebotarm_simulation` 的 MuJoCo 后端。

保持可执行名与 ROS 接口不变，直接启动命令改为：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rebotarm_preview rebotarm_sim_trajectory_controller
```

现有 bringup 入口无需更改使用方式。同一命名空间只能启动一个执行后端。
包本身不依赖 MuJoCo、NumPy、mjlab 或 rebotarm_simulation；bringup 同时组合其他后端，
其包级依赖仍包含 rebotarm_simulation。
