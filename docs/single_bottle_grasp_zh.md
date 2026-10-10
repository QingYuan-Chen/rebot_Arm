# 单次瓶体视觉抓取

`rebotarm_single_bottle_grasp` 是从本地旧项目迁入的单次瓶体抓取功能入口。旧项目的实机测试
不能代替在这个上游新基线上的重新验收；当前迁移只做软件测试，不启动实机。它消费
`/grasp/filtered_plan` 的新鲜瓶体候选，执行以下固定流程：

```text
捕获本轮六轴基线
-> MoveIt 规划预抓取与抓取位姿
-> 使能并张开夹爪
-> 预抓取
-> 从真实预抓取反馈重新规划接近段
-> 接近
-> 限力闭合并保持
-> 张开释放
-> 返回本轮基线
-> 位置、速度与稳定时间验收
-> 失能
```

该功能不执行抬升、撤退或固定 safe-home，也不把夹爪堵转判定当成成功夹住物体的
证明。发生可恢复失败时，它停止轨迹并尝试受控返回本轮基线；只有回位验收通过后
才失能。控制器、电机或反馈不健康时走保护性失能；硬件健康但回位失败时保持使能，
等待人工恢复。

运行参数默认从随包安装的
`rebotarm_vision/config/single_bottle_grasp.yaml` 加载。这个配置保留已经实测的
`10/3/3/10 s` 四段时长、`0.080 m` 张开宽度和夹爪力矩参数；这些属于待复核的旧项目配置，
在新基线未经实机复验，不应直接用于执行。证据输出路径与逐次
真机确认不能写入配置，必须由每次命令显式提供。

实机视觉配置只允许 `bottle` 检测进入 GraspNet，并在规划前执行以下验收：抓取点
在 `base_link` 中不得低于 `0.03 m`；候选中心必须位于该瓶子分割点云的三维范围
内；TF 必须按深度图采集时间查询。单次抓取入口还要求连续 3 个不同传感器时间戳
的计划稳定，窗口内水平位置差不超过 `0.015 m`、高度差不超过 `0.010 m`、夹爪
宽度差不超过 `0.010 m`，并按传感器时间检查计划年龄不超过 `1.5 s`。

先启动视觉、MoveIt 和硬件支撑链。串口仍由启动命令显式指定：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_FASTRTPS_USE_SHM=0

python3 scripts/setup_motorbridge_fresh_feedback.py --check-installed || exit 1
fuser -v /dev/ttyUSB0 /dev/ttyACM0

ros2 launch rebotarm_bringup visual_grasp_system.launch.py \
  channel:=/dev/ttyACM0 use_hardware:=true
```

确认现场安全、候选正确并取得本轮实机动作授权后，在第二个终端执行一次：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rebotarm_vision rebotarm_single_bottle_grasp \
  --output "log/visual_grasp/single_bottle_$(date +%Y%m%d_%H%M%S).json" \
  --confirm REAL_SINGLE_BOTTLE_GRASP
```
