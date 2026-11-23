# rebotarm_force_control

共享重力补偿与关节力控算法包。硬件控制器和仿真单向调用本包；输入为关节数值、模型、配置和时间戳，输出为待发送命令与后端无关的算法状态。零 `rclpy`、`mujoco`、`motorbridge` 或 `threading` 依赖。

## 模块职责（单一职责拆分）

| 模块 | 职责 | 核心对外符号 |
|---|---|---|
| `contracts.py` | 共享基础契约、向量/限位校验、包资源与 URDF 限位读取 | `JOINT_NAMES`、`POSITION_LIMITS`、`vector`、`six_vector`、`finite_scalar`、`validate_position_limits`、`package_file`、`read_position_limits` |
| `mit_command.py` | 公共 MIT 协议上限、总力矩与变化率裁剪、内环求值与周期计时 | `PROTOCOL_KP_MAX`、`PROTOCOL_KD_MAX`、`bounded_mit_command`、`evaluate_mit_torque`、`LoopTiming` |
| `gravity_model.py` | 重力参数配置、Pinocchio 六轴降维动力学模型与前馈缩放 | `GravitySettings`、`GravityModel`、`load_gravity_model`、`apply_gravity_compensation_tau_scale` |
| `gravity_control.py` | 重力补偿运行期抗漂移积分与拖动跟随状态 | `GravityExecutionState` |
| `joint_impedance.py` | 关节阻抗不可变值对象与无状态单拍控制律 | `JointImpedanceCommand`、`ImpedanceOutput`、`ImpedanceMitOutput`、`compute_joint_impedance`、`scale_impedance_command` |
| `joint_impedance_settings.py` | 阻抗准入安全上限配置（无默认刚度/阻尼回退）与 YAML 加载 | `JointImpedanceSettings` |
| `joint_impedance_budget.py` | 离线增益预算公式与违规判定（不依赖 MuJoCo） | `REFERENCE_DEFLECTION_RAD`、`DAMPING_RATIO`、`evaluate_gains`、`budget_violations` |
| `joint_impedance_session.py` | 后端无关的两阶段提交状态机（准入/热更新/MIT 锁存/交接/快照） | `JointImpedanceSession` |

`config/gravity_compensation.yaml` 保存既有重力补偿参数；`config/joint_impedance.yaml` 仅声明安全准入上限（`Kp<=500 / Kd<=5` 与力矩变化率上限），不预设默认刚度或阻尼，后续真机参数与仿真整定参数完全分开。

MuJoCo 接入和 Python、CLI、ROS 用法见 [仿真说明](../rebotarm_simulation/README_mujoco.md)。
仿真示例刚度/阻尼按电机额定力矩除以参考偏角与目标阻尼比推导并受达妙 MIT 协议 `Kp<=500 / Kd<=5` 约束；整定依据、力矩预算和同类实现对照见
[关节阻抗增益整定](../../docs/implemented/features/joint_impedance_gains.md)，
可用 `python3 -m rebotarm_simulation.diagnostics.impedance_gain_budget` 复算。

六轴顺序为 `joint1..joint6`，单位为 rad、rad/s、N·m 和 s。模型使用 `rebotarm_moveit_config` 的规范 URDF。`GravityModel` 内部计算缓存与 `JointImpedanceSession` 实例须由调用方串行使用；每个执行实例单独持有各自状态。

`rebotarmcontroller/gravity/gravity_controller.py` 持有硬件重力补偿适配与保护。仿真后端持有物理时间、生命周期、执行互斥和快照接线。现有重力补偿服务与参数数值保持；真机力控需单独验收。
