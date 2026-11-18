# 关节阻抗增益整定

> 状态：IMPLEMENTED；类型：参数整定依据与力矩预算；适用范围：仿真关节阻抗的默认示例增益。
> 最后核对：2026-11-18（仿真推导与定向回归通过；真机未验证）。

本文说明 `rebotarm_force_control` 的关节阻抗控制律为什么采用当前示例增益，以及这些数值
如何从电机额定力矩、权威 URDF 限幅和模型惯量推导出来。运行期控制律、限幅与拒绝逻辑
不因本文改动；本文约束的是示例、文档和测试里的**推荐增益**。

## 控制律与限幅来源

`joint_impedance.py` 计算：

```text
tau = g(q) + K * (q_ref - q) - D * q_dot
```

- `K`：刚度，单位 N·m/rad；`D`：阻尼，单位 N·m·s/rad。
- 总力矩先按 **URDF `limit@effort`**（joint1–3 = 27 N·m，joint4–6 = 7 N·m）裁剪，
  再按变化率裁剪（180 / 90 N·m/s）。
- 这个限幅是**仿真安全基线**，不是电机额定力矩：电机额定更低（9 / 3.5 N·m），
  所以增益过大时请求会先超过额定、再被 URDF 限幅截断，期间不会报错。
  这就是本次整定的起因。

## 电机官方参数

| 型号 | 额定力矩 | 峰值力矩 | 减速比 | 额定转速 |
|---|---|---|---|---|
| DM-J4340P-2EC V1.2（24 V） | **9 N·m** | **27 N·m** | 40:1 | 36 rpm |
| DM-J4340P-2EC V1.1（24 V） | 12 N·m | 40 N·m | 40:1 | 36 rpm |
| DM-J4310-2EC V1.2（24 V） | **3.5 N·m** | **12.5 N·m** | 10:1 | 120 rpm |

来源：[DM-J4340P-2EC 说明书 V1.2](https://raw.githubusercontent.com/dmBots/DM-J4340P-2EC/HEAD/manual/DM-J4340P-2EC%20Geared%20Motor%20User%20Manual%20V1.2%202026-09-14.pdf)、
[DM-J4340P-2EC 说明书 V1.1](https://raw.githubusercontent.com/dmBots/DM-J4340P-2EC/HEAD/manual/DM-J4340P-2EC%20V1.1%20Geared%20Motor%20User%20Manual%20V1.4%202026-09-14.pdf)、
[DM-J4310-2EC 说明书 V1.2](https://raw.githubusercontent.com/dmBots/DM-J4310-2EC/HEAD/manual/DM-J4310-2EC%20V1.2%20Geared%20Motor%20User%20Manual%20V1.4%202026-09-14.pdf)、
[达妙官网 DM-J4340-2EC](https://www.damiaokeji.com/index.php?c=show&id=100)。

本机 joint1–joint3 经用户确认为 **V1.2（9 / 27 N·m）**，`motor_control_calibration.yaml`
的 `motor_specs.DM4340P` 已按该版本记录。V1.1 的 12/40 N·m 保留在本文对照，便于日后换型。

两份手册都写明 MIT 帧的增益范围：**`Kp range: [0,500]`、`Kd range: [0,5]`**，
Kp 与 Kd 在帧内各占 12 bit。这是硬约束，与力矩无关。

## 当前模型的重力与惯量

参考姿态取零位、Home `[0,-0.8,-1,0.3,0,0]` 和腕部前伸 `[0,-1.2,-0.6,0.6,0,0]`，
用 MuJoCo 3.3.0 在零速度下求 `qfrc_bias`：

| 姿态 | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| 零位 | 0.00 | −0.78 | −8.45 | −2.55 | 0.00 | 0.00 |
| Home | 0.00 | 1.64 | −8.05 | −2.56 | 0.00 | 0.00 |
| 腕部前伸 | 0.00 | 3.63 | −6.94 | −1.20 | 0.00 | 0.00 |

Home 姿态的对角惯量（`mj_fullM`，单位 kg·m²）：

```text
M = [0.1248, 0.3719, 0.3307, 0.0475, 0.0176, 0.0035]
```

重力项只用于第二条边界（弹簧项加重力不得超过 URDF 限幅）的计算，不单独判违规。
仿真模型的 joint3 重力读数偏高（最不利 8.45 N·m），但现场实测未观察到超负载，
因此不作为问题项，也不据此调整任何参数。

## 整定规则

```text
K_i = rated_i / θ_ref                       θ_ref = 0.2 rad
D_i = min(5.0, round(2·ζ·sqrt(K_i·M_i), 2))  ζ = 0.7，M 取 Home 姿态对角惯量
```

- 刚度由「额定力矩 ÷ 参考偏角」决定，含义是：偏差达到 0.2 rad 时弹簧项正好用满额定力矩。
- 阻尼由目标阻尼比换算；若超过协议上限 5，则截断到 5，代价是实际阻尼比低于 0.7。
- 参考偏角与阻尼比是本项目的设计选择，不是厂商数据；改动它们需要重跑诊断并复验响应。

## 推荐增益

| 轴 | K (N·m/rad) | D (N·m·s/rad) | 实际 ζ | ωn (Hz) |
|---|---|---|---|---|
| J1 | 45 | 3.32 | 0.70 | 3.02 |
| J2 | 45 | 5.00（协议上限截断） | 0.61 | 1.75 |
| J3 | 45 | 5.00（协议上限截断） | 0.65 | 1.86 |
| J4 | 17.5 | 1.28 | 0.70 | 3.05 |
| J5 | 17.5 | 0.78 | 0.70 | 5.02 |
| J6 | 17.5 | 0.34 | 0.69 | 11.34 |

力矩预算（`spring = K·θ_ref`，`gravity` 取三个姿态的最不利值）：

| 轴 | spring | + 重力 | spring/额定 | (spring+重力)/URDF 限幅 | D·1 rad/s 占额定 |
|---|---|---|---|---|---|
| J1 | 9.00 | 9.00 | 1.00 | 0.33 | 37% |
| J2 | 9.00 | 12.63 | 1.00 | 0.47 | 56% |
| J3 | 9.00 | 17.45 | 1.00 | 0.65 | 56% |
| J4 | 3.50 | 6.06 | 1.00 | 0.87 | 37% |
| J5 | 3.50 | 3.50 | 1.00 | 0.50 | 22% |
| J6 | 3.50 | 3.50 | 1.00 | 0.50 | 10% |

## 与旧默认值的对比

旧默认 `K = [80,80,80,20,10,5]`、`D = [8,8,8,1,0.5,0.1]` 的问题：

- 偏差 0.2 rad 时弹簧项为 16 / 16 / 16 / 4 N·m，分别是 J1–J4 额定的
  **1.78 / 1.78 / 1.78 / 1.14 倍**；叠加重力后 J3 需求 24 N·m，接近 URDF 限幅 27。
- `D = 8` 使 1 rad/s 的阻尼项就用掉 J1–J3 额定的 **89%**，几乎不留重力余量。
- **J1–J3 的 `kd = 8` 超出 DM MIT 协议上限 5**，属于协议外取值。

## 同类开源实现对照

| 仓库 | 关节刚度 K | 阻尼 D | 说明 |
|---|---|---|---|
| [Seeed-Projects/reBotArm_control_py](https://github.com/Seeed-Projects/reBotArm_control_py) 重力补偿例 | 0 | 1.0 | 纯重力漂浮；README 另写 kp=2、kd=1 |
| [YaolinDong/rebotarm_control_rt](https://github.com/YaolinDong/rebotarm_control_rt) `10_gravity_compensation_lock.py` | 8.0 | 1.0 | MIT 锁定的命令行默认值 |
| [EclipseaHime017/reBotArmController_ROS2](https://github.com/EclipseaHime017/reBotArmController_ROS2) `rebotarm_hardware.yaml` | DM 版 `[120,120,120,18,18,18]`；重力补偿 2.0 | `[8,8,8,2,2,2]`；重力补偿 1.5 | 同样使用 kd=8，同样超出协议上限 |
| [HJX-exoskeleton/reBotArm_develop_hjx](https://github.com/HJX-exoskeleton/reBotArm_develop_hjx) 关节阻抗保持 | `[10.5,7.5,10.5,4.5,3,3]` | `[1.0,2.5,1.5,1.5,0.8,0.6]` | 力矩限幅 `[10,10,10,5,5,5]` |
| [enactic/openarm](https://github.com/enactic/openarm_description/blob/HEAD/assets/arm/openarm_v2.0/config/control/control_gains.yaml) | 4340 关节（J3/J4）70/60；4310 关节（J5–J7）10 | 4340：2.0/2.0；4310：0.7/0.6/0.5 | J1–J2 用 DM8009（effort 40），不能与本机 J1 直接对照 |
| [p20030920p/miku-arm-ros2](https://github.com/p20030920p/miku-arm-ros2) | 6.0 | 0.6 | 达妙六轴臂 |

同类实现的阻尼基本都在 **0.5–3 N·m·s/rad** 区间，且不超过协议上限 5。
本次推荐值落在该区间内，刚度则按本机额定力矩显式推导，不照抄他人数值。

## 仿真响应

Home 姿态下 J1 单轴阶跃（100 Hz 控制拍，力矩与变化率限幅生效）：

| 阶跃 | 峰值力矩 | 2 s 误差 | 6 s 误差 | 15 s 误差 |
|---|---|---|---|---|
| 0.05 rad | 2.2 N·m | 0.0134 | 0.0012 | 0.0003 |
| 0.10 rad | 4.5 N·m | 0.0269 | 0.0020 | 0.0003 |
| 0.20 rad | 9.0 N·m | 0.0554 | 0.0037 | 0.0004 |
| 0.30 rad | 13.5 N·m | 0.0858 | 0.0055 | 0.0004 |

其他实测结果：

- 三个姿态的纯重力保持误差 ≤ 2e-6 rad。
- J1 受 2 N·m 外力时静态偏移 0.0364 rad（理论 2/K = 0.0444），撤力后回到平衡点。
- 0.3 rad 阶跃的峰值需求由旧增益的 24 N·m 降到 13.5 N·m。

**代价**：增益变软后回稳变慢。2 s 处的误差从约 0.003 rad 增至 0.014 rad，回稳时间从约
4.5 s 增至约 6 s。这是「不超额定」的直接结果，不是缺陷；需要更快响应时应优先提高
控制频率或改用前馈，而不是重新抬高刚度。

## 复算与检查

```bash
PYTHONPATH=src/rebotarm_simulation:src/rebotarm_force_control \
  python3 -m rebotarm_simulation.diagnostics.impedance_gain_budget
```

命令打印 JSON 预算报告。以下三条边界任一不满足时以非零退出码失败：

1. `K·θ_ref ≤ 额定力矩`（弹簧项不超额定）；
2. `|g| + K·θ_ref ≤ URDF effort`（瞬时总需求不超仿真限幅）；
3. `K ≤ 500`、`D ≤ 5`（达妙 MIT 协议范围）。

若推导结果与模块常量
`RECOMMENDED_STIFFNESS` / `RECOMMENDED_DAMPING` 不一致（例如换了电机型号或模型惯量变了），
命令同样失败，提示同步文档、示例与测试。

`tests/test_joint_impedance_gain_budget.py` 覆盖上述边界、旧增益反例，以及
`README_mujoco.md` 三处示例与常量的一致性。

## 边界与未决项

- 以上全部是**仿真整定**结果，不构成真机验收。真机摩擦、回差、温升与 MIT 链路的
  实际增益编码未验证。
- J6 的固有频率约 11 Hz，在 100 Hz 控制拍下每周期约 9 个采样点，须在真机上做振荡检查。
- 换末端负载会改变惯量，必须重跑诊断后重新整定。
- 真机 `rebotarmcontroller/config/arm.yaml` 与 `rebotarm_bringup/config/arm.yaml` 中
  joint1–joint3 的 MIT `kd = 8.0` 同样超出协议上限 5。该值经 MIT 直通路径下发，
  本次只记录、未修改，需单独授权并在现场验证。
