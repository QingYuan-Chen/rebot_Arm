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
| 真机反馈、夹爪和控制器 | `hardware_controller.launch.py`、`rebotarmcontroller` 参数 | `rebotarmcontroller` |
| 轨迹安全、视觉准备位和执行 | `src/rebotarm_motion/config/`、`visual_grasp_system.launch.py` | `rebotarm_motion` |
| Gemini 2、YOLO、GraspNet | `src/rebotarm_vision/config/` | `rebotarm_vision` |
| MuJoCo 模型、动力学和 RL | `src/rebotarm_simulation/config/`、`docs/setup/dependencies/rl.md` | `rebotarm_simulation` |
| 标定会话和手眼输入 | `src/rebotarm_vision/config/handeye.yaml`、标定 launch 参数 | `rebotarm_calibration` |

视觉抓取参数数量较多，具体默认值以 `visual_grasp_system.launch.py` 和对应 config 为准；本页不复制整张参数表。
