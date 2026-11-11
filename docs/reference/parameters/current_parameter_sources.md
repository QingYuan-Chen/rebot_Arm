# 当前参数来源

> 状态：REFERENCE；类型：参数来源和维护边界；适用范围：当前 launch/config 参数。

## 参数来源层级

```text
launch 参数覆盖
  > 包内 config/*.yaml
  > 节点代码默认值
```

MuJoCo、视觉和 GraspNet 解释器另遵循显式参数/环境变量优先级，见 [启动解释器显式配置](../../setup/launch_python_configuration.md)。

## 主要参数文件

| 参数域 | 权威文件/入口 | 所有者 |
|---|---|---|
| URDF/SRDF、关节限位、规划器 | `src/rebotarm_moveit_config/config/` | `rebotarm_moveit_config` |
| 真机设备配置、安全增益和时序 | `rebotarmcontroller/config/arm.yaml`、`gripper.yaml`、`configuration.ProtocolTiming`；bringup 显式覆盖 `rebotarm_bringup/config/` 的配置 | `rebotarmcontroller` |
| 真机反馈和夹爪运行参数 | `hardware_controller.launch.py`、`rebotarmcontroller` 参数 | `rebotarmcontroller` |
| 轨迹安全和运动规划 | `rebotarm_motion` 与各组合参数 | `rebotarm_motion` |
| 视觉节点组合和后端选择 | `src/rebotarm_bringup/launch/visual_perception_system.launch.py` 及 `launch/includes/` | `rebotarm_bringup` |
| D405、YOLO、GraspNet | `src/rebotarm_vision/config/` | `rebotarm_vision`；新手眼结果显式传入 |
| 视觉候选跨节点接口 | `src/rebotarm_bringup/config/visual_grasp_interfaces.yaml` | `rebotarm_bringup` |
| MuJoCo 模型、动力学和 CPU RL | `src/rebotarm_simulation/config/`、`docs/setup/dependencies/mujoco.md`、`docs/setup/dependencies/rl.md` | `rebotarm_simulation` |
| 标定会话和手眼输入 | `src/rebotarm_vision/config/handeye.yaml`、标定 launch 参数 | `rebotarm_calibration` |

直接构造控制器的默认周期为 500 Hz，bringup 配置为 100 Hz。内存 channel 覆盖
同时作用于共享机械臂/夹爪总线，原配置中两者 channel 不一致时会在设备打开前拒绝。

## 视觉感知入口层级

| 入口 | 用途 | 参数范围 |
|---|---|---|
| `visual_perception_system.launch.py` | 唯一视觉入口 | `robot_context:=offline|external`、显示开关与配置文件 |

维护规则：新 topic/frame/service/action 先加入 `visual_grasp_interfaces.yaml`；
视觉策略归 rebotarm_vision/config，运动策略归 rebotarm_motion/config。
筛选策略继续使用 `visual_strategy_profile.yaml` 的有效参数；Marker 输入与 `filtered_plan_topic` 一致。

CPU Reach 工具仍归 `rebotarm_simulation`；MJLab 任务与训练参数归现有独立工程
`/home/a/project/rebot_Arm_rl/MJLab/`，见[独立工程入口](../commands/mjlab_rl.md)。
