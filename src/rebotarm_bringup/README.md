# rebotarm_bringup

`rebotarm_bringup` 是 reBotArm 的启动组合包。它负责把各个功能包的
launch、配置和 RViz 资源组合成可运行入口，并选择真机、仿真或无硬件预览后端。
本包不实现电机控制、运动规划、视觉算法、示教业务或 Dashboard 业务逻辑。

## 目录结构

```text
rebotarm_bringup/
├── launch/                 # ROS 2 启动入口和启动结构说明
├── config/                 # 硬件、夹爪、遥操作和示教组合参数
├── rviz/                   # 各入口使用的 RViz 配置
├── package.xml
├── setup.py
└── resource/rebotarm_bringup
```

启动文件的包含关系、功能边界和参数维护原则见
[`launch/README.md`](launch/README.md)。

## 主要入口

| 入口 | 用途 |
|---|---|
| `hardware_controller.launch.py` | 唯一直接启动真实 `reBotArmController` 的底层入口 |
| `bringup.launch.py` | 硬件、状态发布、TF 和基础 RViz |
| `moveit_hardware.launch.py` | 真机 MoveIt 规划/执行入口 |
| `rebotarm_app.launch.py` | 真机 MoveIt、示教录制和 Dashboard 工作台 |
| `teleop_keyboard.launch.py` | 键盘点动和基础 RViz |
| `teleop_system.launch.py` | 键盘点动、示教和 Dashboard 组合 |
| `rviz_ee_drag_sim.launch.py` | 无硬件仿真规划与 RViz 拖动 |
| `visual_grasp_system.launch.py` | 视觉只读、plan-only 和受控执行组合 |

日常可复制命令统一维护在 [`docs/reference/commands/`](../../docs/reference/commands/README.md)，
本包不在多个文档中重复维护同一套命令。

## 依赖边界

```text
rebotarm_bringup
  ├── 组合 rebotarmcontroller / rebotarm_motion / rebotarm_vision 等包的入口
  ├── 选择 hardware / simulation / preview 后端
  └── 安装 launch、config、rviz 资源
```

- 硬件访问只属于 `rebotarmcontroller`。
- 轨迹生成、MoveIt 适配和执行期校验属于 `rebotarm_motion`。
- 感知与抓取候选属于 `rebotarm_vision`。
- MuJoCo 后端属于 `rebotarm_simulation`。
- 本包只能组合这些能力，不能复制其实现。
- 同一机械臂命名空间只能有一个真实或仿真轨迹执行后端。

## 安全边界

- 真机启动默认保持失能；必须先确认新鲜反馈和现场安全条件，再显式调用 `/rebotarm/enable`。
- `Plan` 成功不等于 `Execute` 成功；执行还必须具备正确的 MoveIt 控制器接口。
- 健康真机发生可恢复任务失败时，不得把机械臂直接失能在未知姿态。
- 无硬件视觉和仿真结果不能作为真实抓取或实体验收证据。

## 验证

```bash
python3 -m pytest tests/test_package_layering.py -q
python3 -m compileall src/rebotarm_bringup/launch -q
colcon build --symlink-install --packages-up-to rebotarm_bringup
```

