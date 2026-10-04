# reBotArm 项目实施完成度

> 本文件主要保留 P0-P6 的历史验收清单；这些阶段已按用户确认的工程范围关闭，不代表当前 RL 任务或实时硬件状态。
> 根目录两份旧规划已清理，历史内容可从 Git 追溯；新规划由用户另行提出。
> 基线日期：2026-08-06
> 说明：这里衡量的是有验收证据的实施完成度，不是代码量、文档完成度或主观进度。

## 当前阶段（2026-10-04）

- 当前主线是纯仿真的 MuJoCo Reach 正确性基准与 mjlab + MuJoCo Warp + RSL-RL GPU Reach 训练线。
- RL 已迁移到独立仓库 https://github.com/huangbinai/rebotarm_rl，使用自己的 .venv；ROS CPU MuJoCo 使用独立轻量环境 third_party/rebotarm_mujoco_venv。模型源由本仓库维护，训练按固定提交与哈希下载资源。
- mjlab 训练链路已完成 GPU smoke 和集成验证；这不代表策略已经稳定收敛，也不构成 sim-to-real 或硬件验收。
- 当前不授权真实机械臂或夹爪动作；P0-P6 只作为已关闭历史基线保存。
- 当前源码中的真机视觉 `tcp_offset_xyz` 为 `[-0.04, 0.0, 0.0] m`；旧 `-0.105 m` 仅为历史 upstream nominal 值，不能写成当前实测 TCP。

## 计分规则

- 每个 P 阶段具有固定权重，总计 100；
- 阶段完成率 = 已勾选验收项 / 该阶段全部验收项；
- 总完成率 = 各阶段完成率乘以阶段权重后求和；
- checkbox 只有在代码、测试、运行记录或硬件验收证据成立后才能勾选；
- `STATE.json` 由 `update_state.py` 自动计算，不手工填写百分比；
- 硬件未连接或未实测时，相关项保持未完成。

## P0: 机械臂安全启动与执行门控 [weight=20]

- [x] 已存在显式 `/rebotarm/enable` 和 `/rebotarm/disable` ROS service；证据：`rebotarmcontroller/ros_services.py`。
- [x] `HardwareManager.connect()` 只建立通信，不调用 enable 或启动控制循环；证据：P0 hardware manager 测试。
- [x] enable 前主动刷新反馈并检查在线状态、有限值和软限位；证据：严格 feedback refresh、URDF 软限位一致性测试。
- [x] enable 设置当前关节为 hold target，并确认全部电机状态；证据：当前位置 hold 与 status 测试。
- [x] 部分电机使能失败时回滚所有已使能电机；证据：部分失败回滚测试。
- [x] 完整视觉抓取默认 `use_hardware=false`、`plan_only`、不自动 visual-ready/safe-home/enable；证据：launch wiring 测试。
- [x] 通过失能 joint states、显式 enable、当前位置保持、disable 和失败回滚验收；证据：Gate A 记录、`gate-bc-20260806-211949.json` 及此前失败后的安全 cleanup 报告。

## P1: MuJoCo 基线巩固与差距整合 [weight=20]

- [x] 已有 MuJoCo 模型生成、model profile 和 headless smoke 基础设施。
- [x] 已有独立 MuJoCo ROS adapter 和 `FollowJointTrajectory` 接口。
- [x] 已有 trajectory metrics、summary 和 step-response suite。
- [x] 已有 path/goal tolerance 失败记录和返回逻辑。
- [x] 已有 URDF position limit 与 MuJoCo ctrlrange 第一层测试。
- [x] 已有 launch 可选 viewer，并保留 RViz fake sim。
- [x] 固定并分类上游 repository URL、branch/tag、commit SHA 和 license；证据：`docs/mujoco_upstream_sources.md`。无许可证文件的 HJX 来源标记 `NOASSERTION` 并从默认模型依赖移除。
- [x] 完成 current/upstream gap matrix，确认只选择性整合缺失能力；证据：`docs/mujoco_gap_matrix.md`、`tests/test_mujoco_gap_matrix.py`。
- [x] 修正 grasp scene keyframe 的不合理起始姿态；scene home `qpos` 与 `ctrl` 已同步，证据：`Agent/evidence/P1/2026-08-06-grasp-keyframe.md`、`tests/test_mujoco_model_profile.py`。
- [x] 固定 `huangbinai/robotarm_ros2` MuJoCo 快照并完成目标文件与 license evidence 审计；证据：`Agent/evidence/P1/2026-08-06-robotarm-ros2-preflight.md`、`Agent/evidence/P1/2026-08-07-upstream-snapshot-localization.md`，固定 `main@fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`，license 结果为 `PROVISIONAL`。
- [x] 独立运行上游 health/headless/model/ROS/motor/collision 测试并形成双基线差异报告；证据：`Agent/evidence/P1/2026-08-07-baseline-comparison.md` 与 JSON。上游为 `219 passed, 1 skipped, 1 failed`，唯一失败已分类为嵌套 `mapping -> tuple` 的测试断言缺陷，未宣称上游全绿。
- [x] 将固定上游 MuJoCo runtime、模型、ROS adapter 和 launch 直接本地化到 `src/rebotarm_simulation`；切换前的 current source 已归档到 `third_party/rebotarm_simulation_current_baseline`。最终 active runtime 收口为 upstream-only，安全边界与单 Action server 约束通过测试和 ROS preflight 验证。证据：`Agent/evidence/P1/2026-08-07-upstream-src-swap.md`、`third_party/rebotarm_simulation_current_baseline/ARCHIVE_MANIFEST.json`。
- [x] 将 timeout 接入执行循环；使用 monotonic wall-clock deadline，超时返回非成功结果并 hold 当前位置，证据：`tests/test_mujoco_adapter_core.py`、`tests/test_mujoco_ros_adapter_launch.py`。
- [x] execute-loop 集成测试覆盖成功、取消、停止、path/goal 容差失败和超时；ROS 2 + MuJoCo 环境 `6 passed`，证据：`Agent/evidence/P1/2026-08-07-execute-loop-integration.md`。
- [x] 补齐 effort、velocity、acceleration 和 jerk 限制一致性；URDF effort / MuJoCo forcerange 一致，MoveIt planner limits 保守且由 `rebotarm_motion` runtime guard 执行，证据：`Agent/evidence/P1/2026-08-07-runtime-limit-consistency.md`。
- [x] 按用户最终决策将 current fallback 退出 active runtime 和 P1 验收范围；未把其 joint4-6 tracking/contact 标定误记为成功。旧实现仅保存在 `third_party/rebotarm_simulation_current_baseline` 归档，active setup/launch 不再暴露 current adapter、legacy CLI 或 backend selector；证据：`Agent/evidence/P1/2026-08-07-p1-upstream-only-closeout.md`。

## P2: Gemini 2 SDK 与真实 RGB-D 验收 [weight=15]

- [x] 已有 Ubuntu 视觉环境、Orbbec SDK 安装和 udev 脚本。
- [x] `Gemini2Driver` 已有 RGB、uint16 depth 和硬件对齐采集代码。
- [x] 真实 Gemini 2 在当前 Ubuntu 上稳定输出 RGB 和 depth；生产 driver 默认 profile 连续 `120/120` 同时出帧，证据：`Agent/evidence/P2/2026-08-07-gemini2-sdk-ros-acceptance.md`。
- [x] 从 SDK 读取设备信息和实际 stream profiles；已记录 serial/firmware/USB、48 color 与 36 depth profiles 以及 selected profiles，证据同上。
- [x] 从 SDK 读取 intrinsics、distortion 和 RGB-depth extrinsics；已进入 driver metadata 与双 CameraInfo，证据同上。
- [x] 获取并正确应用 depth scale，明确 ROS 与内部深度单位；`DepthFrame.get_depth_scale()=1.0 mm/unit`，driver 已应用后发布 `mono16` millimeter，证据同上。
- [x] 验证 RGB-D 对齐、分辨率变化和 CameraInfo 一致性；当前 HW alignment、双 CameraInfo、分辨率和 change-mask correspondence 已有硬件证据，用户明确接受沿用同一相机既有 pixel-level calibration，不要求重复受控边缘复测。证据：`Agent/evidence/P2/2026-08-07-p2-closeout-accepted-calibration.md`。
- [x] 深度距离验收关闭；用户确认同一相机既有测试/calibration 无问题并豁免本轮重复多个精确距离 ground-truth 表。本轮约 300 mm 样本只作为稳定性证据，未补造精确 bias/RMSE。acceptance 与残余风险见同一 closeout evidence。

P2 已按用户 acceptance decision 以 `8/8` 关闭：当前 Ubuntu 已完成设备/profile/calibration metadata/depth scale/连续 RGB-D/ROS CameraInfo/timestamp/short-soak 和 gross alignment 硬件验证；同一 serial 的既有测试记录作为补充 provenance。pixel-level 受控目标和精确多距离 error table 没有在本轮重做，已作为明确豁免和残余风险记录，不触发重新标定。证据：`Agent/evidence/P2/2026-08-07-gemini2-readonly-preflight.md`、`Agent/evidence/P2/2026-08-07-gemini2-sdk-ros-acceptance.md`、`Agent/evidence/P2/2026-08-07-gemini2-obstruction-removal-differential.md`、`Agent/evidence/P2/2026-08-07-gemini2-2min-short-soak.md`、`Agent/evidence/P2/2026-08-07-p2-closeout-accepted-calibration.md`。

## P3: 完整单 Ubuntu 视觉链路 [weight=15]

- [x] 已有 `camera_ubuntu.yaml` 和 `vision_ubuntu.launch.py`。
- [x] 已有 Ubuntu 本地 CUDA YOLO 环境和独立启动脚本。
- [x] 完整 bringup 支持显式 `vision_profile:=ubuntu_native`；仅接受 `network`/`ubuntu_native`，非法 profile fail closed。证据：`Agent/evidence/P3/2026-08-07-ubuntu-native-preflight.md`。
- [x] 一次 camera-only 安全 launch 发布 RGB、depth、双 CameraInfo、detections 和 annotated topics；主 bringup 的 native 分支也已运行。证据同上。
- [x] 完整 Ubuntu native camera path 不依赖 Windows 路径或 Windows 服务；native config 固定 Gemini 2 SDK + local YOLO。证据同上。
- [x] 时间戳、frame_id、TF 和候选新鲜度检查通过；Ubuntu native Image publishers 使用显式 `RELIABLE`，真实 Gemini 2 默认 15 Hz 的 11.6 秒 rosbag2 为 color/depth `175/175`、0 transport loss，8 秒 consumer 为 `120/120` 且 timestamp 单调、freshness/frame_id/TF 正常。证据：`Agent/evidence/P3/2026-08-07-ubuntu-native-preflight.md`。
- [x] 视觉失效或过期候选 fail closed：empty/partial camera frame 发布空 detections，网络异常不复用 stale payload，executor 拒绝 unset/stale plan；focused P3 tests 通过。证据同上。

## P4: Ubuntu 本地 GraspNet [weight=10]

- [x] 已有 GraspNet candidate topic、network client 和 localhost 配置入口。
- [x] 已有 GraspNet baseline inference/bridge 的参考工具代码。
- [x] 建立独立 `.venv-graspnet` 和固定依赖；PyTorch `2.11.0+cu128`、Open3D `0.19.0`、NumPy `1.26.4`、CUDA import 与隔离性通过。证据：`Agent/evidence/P4/2026-08-07-graspnet-environment.md`。
- [x] 新增 Ubuntu GraspNet setup/health-check 脚本与安装文档；脚本不自动下载未审查的模型资产。证据：`Agent/evidence/P4/2026-08-07-graspnet-environment.md`。
- [x] localhost GraspNet 服务提供 `/health` 并可独立启停；未配置模型时明确报告 `unconfigured` 且 inference fail closed。证据：`Agent/evidence/P4/2026-08-07-graspnet-environment.md`。
- [x] service contract `1.0` 包含 `bgr8` RGB、`32FC1` 米制 depth、intrinsics、bbox、timestamp 和 frame_id；非法 contract 拒绝且响应保留 header。证据同上。
- [x] 候选稳定性、过期、无候选和 TF 失败的 fail-closed 验收通过；真实 Gemini 2 六帧为 `8–10` candidates/frame，ROS 2 最终 probe 的 29 条 timestamp 全部非零且严格递增、freshness 最大 `686.5 ms`，stale/no-depth/TF failure 均为空结果。证据：`Agent/evidence/P4/2026-08-08-graspnet-real-inference.md`。

## P5: 标定 [weight=10]

- [x] 已有 hand-eye 配置加载与 TF 发布机制，但数据仅视为历史配置。
- [x] 验证 SDK 相机内参、distortion、RGB-depth extrinsics、depth scale 和不同 profile；复用 P2 的同一 serial Gemini 2 硬件证据，不代表重新标定。证据：`Agent/evidence/P2/2026-08-07-gemini2-sdk-ros-acceptance.md`。
- [x] 对当前实际安装完成 hand-eye 多姿态稳定性验收；SUBPIX DANIILIDIS candidate 部署后，三个全新 temporal holdouts 相对固定 training reference 的 position/rotation RMS为 `2.495 mm/0.359 deg`，rotation span `38.792 deg`，全部既定门通过。证据：`Agent/evidence/P5/2026-08-09-handeye-postdeploy-holdout-final.{md,json}`。
- [x] 历史阶段曾接受 upstream explicit nominal TCP `[-0.105, 0.0, 0.0] m` 作为工程口径；当前 active local source 已改为 `tcp_offset_xyz=[-0.04, 0.0, 0.0] m`，且没有新增 physical TCP measurement。旧值仅保留作历史 provenance，不表述为当前实测 TCP。
- [x] 完成 MuJoCo 关节轴、零位、link、mesh、夹爪和场景几何验收。Robot body geometry / 机器人本体几何由active/upstream/CAD文件一致性及用户对当前装机revision、六轴zero convention和夹爪CAD结构参数的权威确认闭环；`scene.xml`按用户工程接受保留为canonical simulation workbench / 规范仿真工作台，不要求拟合真实工作场景，也不得当作真实场景标定证据。
- [x] 接受 active upstream MuJoCo dynamics / 当前上游动力学参数为经过实机标定的权威基线。该 provenance / 来源由用户（硬件来源方）明确确认；仓库未保存原始标定数据，因此不表述为本轮独立重做标定。废弃的 downstream/current baseline 未经电机标定，不参与当前验收。active model 的 joint2 margin 与 joint4-6 `7 N.m` 仅是本地安全约束，不取代或否定上游动力学标定。

2026-08-11 MuJoCo geometry/dynamics/contact read-only audit已完成。active MJCF由active URDF确定性生成且`--check`通过，focused模型/来源测试`17 passed`；axis/link transform/mesh/finger/`ee_site=-0.105 m`均可追溯到upstream显式值或CAD-export候选，paired safe-posture也支持局部joint-coordinate mapping。用户随后补充来源事实：废弃的downstream/current baseline没有经过电机标定，而active upstream参数已经过实机标定，可以作为本项目权威基线。因此先前`4.929100 kg`与CAD CSV`2.741131 kg`差异保留为历史数据源差异，不再作为active dynamics blocker；dynamics checkbox按用户来源确认关闭，但不冒充本轮独立复现了原始标定。审计当时记录的zero、装机revision、夹爪range与scene scope缺口，已由后续权威确认和canonical simulation workbench / 规范仿真工作台范围决策闭环，见下一段。Contact force/material calibration已从项目规划剔除。完整审计及后续决策：`Agent/evidence/P5/2026-08-11-mujoco-geometry-dynamics-contact-readonly-audit.md`。

2026-08-11 Geometry authoritative-values / 几何权威值表已完成。active URDF、generated MJCF、fixed upstream snapshot和CAD-export的六轴joint origin/axis/link transform逐项一致；base/link1-6 mesh在active sim/bringup/upstream/CAD四处逐文件一致，gripper三件mesh在active sim/bringup/upstream三处一致。唯一geometry覆盖是有实测安全来源的joint2 upper `0 -> +0.02 rad`；`urdf_to_mjcf --check`通过。用户确认当前装机无换件/垫片/机械改装、与upstream/CAD revision一致，上游实机标定包含六轴zero convention，夹爪`0..90 mm`是CAD显式结构参数；robot body geometry据此闭环。用户进一步确认`scene.xml`无需拟合真实现场，可直接作为规范仿真工作台；其验收含义是仿真内部几何一致和可复现，不是real-scene calibration / 真实场景标定。Geometry checkbox据此关闭。证据：`Agent/evidence/P5/2026-08-11-mujoco-geometry-authoritative-values.md`。

以下 TCP 接触/pivot 段落仅保留已取消路线的历史证据，不再表示当前待执行计划。TCP 标定预检已完成但 checkbox 保持未勾选：标定纸中心在当前 hand-eye 下 30/30 稳定，base-frame center std最大 `0.062 mm`；但纸面不是可重复触碰的三维尖点，且按纸面法向构造的接触姿态全部 MoveIt `NO_IK_SOLUTION(-31)`。工具已迁移到 `rebotarm_calibration` 并增加 classic unknown-reference pivot 求解及 rank/condition/residual gates。该路线后来由用户取消；候选从未通过全部 gates、从未部署。证据：`Agent/evidence/P5/2026-08-09-tcp-calibration-preflight.md`。

用户现已固定约 `10 mm` 钢球并移开标定纸，且明确授权安全范围内持续执行 TCP 低速接触采样；该授权不扩展到 P6 approach/gripper/lift/retreat。最新 10 mm 球心只读定位与 5/5 代表路线 plan-only 证据记录于 `Agent/evidence/P5/2026-08-09-tcp-10mm-moved-route-preflight.json`。由于立杆/底座尚未作为完整碰撞几何验收，接触批次开始前仍需做现场净空/碰撞确认；采样失败必须保持 enabled hold 或受控回 baseline 后再 disable，不能突然失能。TCP checkbox 仍未勾选，待正式多轴 pivot 样本和残差门通过后再评审部署。授权及安全边界记录于 `Agent/evidence/P5/2026-08-09-tcp-contact-authorization.md`。

首次低速接触验证记录为安全失败：baseline→precontact `25 s`、precontact→nominal-contact `12 s` 均成功且无 guard stop，但低力闭合 `contact_detected=false`、`reached_position=0.0876619377 m`，未碰球，不能计入 TCP pivot 样本。失败后保持健康 enabled hold 并受控回 baseline，最大回位误差 `0.000381 rad`，确认到位后才 disable；最终六轴 status 全 0、无 error、`IDLE/disabled`。证据：`Agent/evidence/P5/2026-08-09-tcp-first-contact-sample.{md,json}`。用户汇报的 `0.10` 低夹持力与原始 JSON 的 `hold_force=0.05` 存在口径差异，下一轮前需核对接口参数；TCP checkbox 继续未勾选。

第二次重试在 baseline→precontact 阶段被 tracking guard 安全取消：joint2 tracking error 持续 `0.0530177984 rad > 0.05 rad` 共 `0.5805779 s`，夹爪仍保持打开，未接触钢球。控制器健康 enabled hold 后执行独立 `15 s` guarded return，回基线最大误差 `0.000381 rad`，确认后才 disable，最终六轴 status 全 0、无 error。该轮归类为轨迹跟踪失败，不是 TCP 接触样本；禁止直接重试同一轨迹，先完成 joint2 跟踪根因审查和新路线 plan-only。证据：`Agent/evidence/P5/2026-08-09-tcp-first-contact-sample-retry.{md,json}`、`Agent/evidence/P5/2026-08-09-tcp-retry-guarded-return.json`；TCP checkbox 继续未勾选。

之后的分段预接近 `4/4` action 成功且无 guard stop，但回基线段触发 `window_velocity_sustained`：joint2 `0.1036026011 rad/s > 0.1` 持续 `0.2508651 s`。系统先保持健康 enabled hold，再成功受控回 baseline，最大回位误差 `0.000763 rad`，确认后才 disable，最终六轴 status 全 0、无 error；controller/MoveIt 已停止、tty 无占用。该轮仍未接触钢球，真实 TCP 运动暂停，等待 joint2 轨迹/速度根因审查；禁止直接重试同一分段路线，TCP checkbox 继续未勾选。证据：`Agent/evidence/P5/2026-08-09-tcp-segmented-precontact.{md,json}`。

标定纸移除后完成新鲜复核：91 帧球心像素 `[377.5,228.5]`，base 球心 `[-0.01364,-0.33370,0.20997] m`；4×12 s forward + 15 s contact route 均成功、无 guard stop并到达 contact pose。但 `GraspGripper(0.20/0.10)` 在接触位仍 `contact_detected=false`、`reached_position=0.0875864 m`，且 correct-float free-space A/B 同样超时，确认该服务路径不适用于本流程；没有闭合夹爪或触碰钢球。失败后35 s受控回 baseline，最大误差`0.000763 rad`，确认后才 disable，最终status全0/no error。`SetGripper` free-space target `0.006 m` 可到 `0.008051 m`，下一步仅允许定 gap + actual-position 判据，禁止盲目 force grasp；TCP checkbox继续未勾选。证据：`Agent/evidence/P5/2026-08-09-tcp-fresh-contact-run.{md,json}`、`2026-08-09-grasp_service_free_test.json`、`2026-08-09-setgripper_gap_free_test.json`。

用户现决定取消实物 TCP 接触/pivot 标定，采用上游 active MuJoCo 的显式 `ee_site=-0.105 m` 作为 nominal TCP；真实视觉配置、GraspNet profile、规划默认值和 launch 默认值已统一到 `[-0.105, 0.0, 0.0]`。仿真执行 launch 保持 `0.0` 参数，因为 upstream MuJoCo 已在模型内部表达 `ee_site` 偏移，避免 double offset。2026-08-11用户进一步明确确认：以该 upstream explicit value / 上游显式值完成当前项目 TCP 工程验收，取消实物 TCP 标定要求。因此 TCP checkbox 按工程接受关闭；证据边界仍是不冒充实机 TCP pivot calibration / 枢轴标定通过。证据：`Agent/evidence/P5/2026-08-09-tcp-upstream-explicit-decision.md`。

用户随后要求暂停 TCP 标定并保存现场。已停止 controller/MoveIt/RSP，`/dev/ttyACM0` 无占用；暂停前最后一次已验证状态为受控回 baseline 后 `status_code=0`、无 error、`enabled=false`、`state_machine=IDLE`。控制器停止后 ROS 状态 topic 无 publisher，因此不将其当作新的在线状态采样。暂停封存记录见 `Agent/evidence/P5/2026-08-09-tcp-pause-closeout.md`；恢复前必须先完成无物体夹爪限力/位置语义验证，TCP checkbox保持未勾选。

P5 第二检查点已完成：真实 Gemini 2 + local GraspNet 候选使用当前 upstream hand-eye/TCP，通过 MoveIt IK/collision filter，并以 scaling `0.1` 成功完成 `/plan_kinematic_path` plan-only；MuJoCo joint state 无运动且 trajectory action 无 client。该证据只证明链路连通和候选存在可达解，不替代 hand-eye 多姿态 residual、真实 TCP 接触标定或 MuJoCo sim-to-real 标定，因此本阶段 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-real-vision-mujoco-plan-only.md`。

用户确认当前安装相对 upstream visual-ready 旋转约 `-90 deg` 后，视觉 task-space 已从 base `+X` 同步到 `-Y`，hand-eye/TCP/camera calibration 不变。真实 camera/GraspNet 10 秒产生 38 candidates 和 7 valid filtered plans，旧的负 Y workspace blocker 已解除；软件归一 joint2 到边界 `0` 后 MoveIt plan-only 成功。该结果只证明安装方向与几何链路连通，不替代标定 residual，P5 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-y-negative-installation-workspace.md`。

用户确认当前机械位置正确后，已只对 disabled joint2 执行一次 motorbridge `set_zero_position()`；反馈从 `-0.0020980835` 归零到 `-0.0001907349 rad`，标准 driver 成功进入 `CONNECTED_DISABLED`，六轴 status 全 0。未 enable、未执行、未 store parameters。原 raw start-state soft-limit blocker 当前解除，但 Damiao zero 不跨断电保留，且反馈距上限只有 0.00019 rad；断电后必须复测。证据：`Agent/evidence/P5/2026-08-08-joint2-zero-calibration.md`。

校零后的真实 disabled start state 已直接用于 real vision -> MoveIt plan-only：真实候选 pre-grasp 位于 `base_link [0.053028, -0.418949, 0.227397] m`，`/plan_kinematic_path` 返回 `SUCCESS(1)`、62 points，规划后 2.048 秒 joint delta 为 `0.0 rad`。未启动 executor，hardware trajectory action server 为 0；该结果关闭 P6 的安全实机 plan-only 项，但不替代 P5 hand-eye/TCP 标定验收。证据：`Agent/evidence/P5/2026-08-08-real-disabled-start-state-plan-only.md`。

用户确认当前 joint2 机械位置为零点并允许覆盖原零点后，已在 disabled 状态再次只对 joint2 调用 `set_zero_position()`；feedback 保持 `-0.0001907349 rad`、status 0。joint2 software range 已在 hardware guard、MoveIt/bringup URDF 和 MuJoCo model/profile 中统一为 `[-3.14, +0.02] rad`，其中正向 `0.02 rad` 仅作零点附近量化/回差 margin。标准 driver 复核为 `CONNECTED_DISABLED`。Damiao zero 仍不跨断电保留，且本项不等于六轴/MuJoCo 几何标定完成，P5 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-joint2-zero-margin.md`。

用户明确授权后，已完成 joint2 新零位/limit 下的 5 秒实机 enable-current-position-hold 回归：119 个有效 joint samples，最大 position jump `0.0003815 rad`、最大反馈速度 `0.0073261 rad/s`，最终自动 disable、六轴 status 全 0。状态转换附近的历史固定边界异常帧被 hardware guard 拒绝，没有进入 joint states；本轮未发 trajectory，因此 P6 分级运动 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-joint2-enable-hold-regression.md`、`Agent/evidence/P5/gate-bc-20260808-202559.json`。

第一次 joint1 `+0.02 rad` 低速往返在途中检测到 `0.051282 rad/s > 0.05 rad/s` 后 fail closed；用户随后说明测试期间手动移动了机械臂，因此该 sample 不能归因于 controller，自此本轮 tracking 数据也不能作为验收。joint1 最终停在 baseline 正向 `+0.008774 rad`，其余五轴不变，最终 disabled/IDLE。取消与自动 disable 同时发生还暴露了 action cleanup 的 status-transition race；修复前不重试，P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-joint1-micro-move-interrupted.md`。

`FollowJointTrajectory` cancel/disable status-transition race 已完成纯软件修复：real hardware manager 现在以同一 motor lifecycle `RLock` 串行化 `enable/disable/stop/get_joint_state`，disabled transition 后的重复 stop 不再按 enabled feedback 执行 hold。两个 event-controlled 并发回归、focused `108 passed`、完整仓库 `544 passed, 14 skipped` 和 compile checks 均通过；本次未访问硬件。该修复只解除重试前的软件 blocker，不补足被人工干扰的单关节 tracking 证据，P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-follow-trajectory-cancel-disable-race-fix.md`。

用户授权后的 joint1 重试再次在 raw feedback `0.051281929 rad/s > 0.05 rad/s` 时 fail closed；本次达到 `+0.01983643 rad`，其他关节 delta 0，最终自动 disabled、六轴 status 0。4340P `VMAX=10` 与反馈格点支持 12-bit velocity quantization inference，`0.05` 位于一个 quantization bin 内，故单 sample 既不能证明实际超速也不能满足严格验收。另发现本轮 driver 使用 stale copy-install，随后已 symlink rebuild 并确认加载 race fix，但重建后未再次运动。P6 checkbox 保持未完成。证据：`Agent/evidence/P5/2026-08-08-joint1-micro-move-retry.md`。

rebuilt runtime 的 joint1 降速微动通过：正向 `+0.02 rad`，2 秒 ramp、1 秒 hold、2 秒 return，action `SUCCESSFUL(0)`；255 samples，raw feedback/window velocity peak 为 `0.03662968/0.01801461 rad/s`，其他关节最大偏移 `0.00038147 rad`，joint1 返回误差 `0.00114441 rad`。最终自动 disabled、六轴 status 0、无 error。本结果关闭 P6 staged real-hardware acceptance 的 single-joint sub-gate，但同一 checkbox 还包含安全姿态、pre-grasp、approach、gripper、lift 和 retreat，故暂不勾选。证据：`Agent/evidence/P5/2026-08-08-joint1-micro-move-slow-pass.md`。

safe-posture 只读预检通过 software gates：从最新真实 disabled state 到 `[-pi/2,-0.1,-0.2,+0.2,0,0]` 的 MoveIt explicit-start plan-only 为 `SUCCESS(1)`，production direct interpolation 的 101/101 sampled states 通过 limits/self-collision。默认 4 秒理论峰值约 `0.09456 rad/s`，实机提案改为 20 秒、峰值 `0.018911 rad/s`。当前 scene 没有真实桌面/支架/障碍物，尚未执行 hardware，P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-safe-posture-readonly-preflight.md`。

用户授权的 20 秒 safe-posture 实机动作在 raw feedback `0.051283 rad/s > 0.05` 时 fail closed，action canceled 并自动 disable；最大实际位移为 joint4 `+0.00724792 rad`，其余更小，最终六轴 status 0。monitor failure path 未保存触发轴和 per-joint/window peaks，因此不能把结果直接归因于 4310 quantization，也不放宽 threshold。safe-posture sub-gate 未通过，P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-safe-posture-execution-interrupted.md`。

用户随后批准各 joint raw feedback gate 放宽到 `0.07 rad/s`；timestamped position-window physical gate 保持 `0.05 rad/s`，safe-posture commanded peak 仍约 `0.019 rad/s`。model-aware helper/test 已覆盖 4340P/4310 half-LSB、boundary/next-bin 和 operator floor。该软件策略不自动授权重试，safe-posture sub-gate/P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-model-aware-velocity-gates.md`。

用户授权后使用 raw/window `0.07/0.05 rad/s` 重试 20 秒 safe-posture。第一次 preflight 因 enable service 期间 joint states 暂停而在 goal 前退出，修正为等待 3 个 fresh samples 后第二次发送 goal；joint4 raw feedback 在第 323 个 motion sample 以 `0.095238 rad/s` 触发，cancel transient peak `0.153847 rad/s`，而 0.20 秒 position-window peak为 `0.018793 rad/s`、max adjacent jump 为 `0.00152588 rad`。action canceled，最终自动 disabled、六轴 status 0、串口释放。`0.07` raw gate 未通过，不能继续放宽或关闭 safe-posture sub-gate，P6 checkbox 不变。证据：`Agent/evidence/P5/2026-08-08-safe-posture-raw-007-retry.md`。

后续无硬件审计定位到 trajectory speed 与 firmware speed limit 脱节：ROS action 每 50 ms 仅更新 position target，`ArmEndPos._loop_cb()` 仍按 `arm.yaml` 向 joint4 发送固定 `vlim=3.0 rad/s`，未使用本轮 theoretical peak `0.018267 rad/s`。joint4 50 ms 早期 target increment 小于一个 position LSB，而观测 position/raw feedback 均落在协议量化格点，支持离散 servo correction burst。建议先实现 per-trajectory `vlim override` 并完成 mock tests，再申请 joint4 隔离微动；未获授权前不修改控制语义或继续运动。证据：`Agent/evidence/P5/2026-08-08-joint4-raw-spike-software-audit.md`。

按用户指出的 joint4 torque/小幅动作变量进一步复核：4310 feedback `TMAX=10 N.m`、URDF effort limit `7 N.m`，均明显低于 joint1-3 的 `28/27 N.m`；Pinocchio 在当前 pose 的 joint4 gravity demand约 `2.07 N.m`。这证明末端静态负载不可忽略，但没有运动期间 torque/following-error/saturation 数据，尚不能宣称 motor capacity不足。控制方案不再局限于低 vlim override：先实施 continuous trajectory governor、torque/error telemetry 和 per-joint persistent gates，再通过 joint4 正反向 characterization 选择 POS_VEL tuning 或 MIT gravity-compensated tracking。P6 checkbox不变。证据：`Agent/evidence/P5/2026-08-08-joint4-control-strategy-review.md`。

多信号 guard 实机重试运行到 742 motion samples：joint4 raw peak `0.2564 rad/s` 但未持续，torque/following-error peaks仅 `0.769 N.m / 0.00683 rad`，没有 torque saturation 或跟踪失控；最终由 joint6 单次 0.20 秒 window `0.05487 > 0.05 rad/s` 取消。最终自动 disabled、六轴 status 0。该证据把 blocker进一步收敛到 window gate 单次边界语义；guard已改为 window `0.05` warning、`>0.10` 持续 0.20 秒或 `>0.20` immediate stop，full tests `558 passed, 14 skipped`。新的实机重试仍需确认，P6 checkbox不变。证据：`Agent/evidence/P5/2026-08-08-safe-posture-multisignal-retry.md`。

用户明确授权后，新 window persistence policy 的同一 20 秒 safe-posture trajectory 本身通过：action `SUCCEEDED/error_code=0`，2108 motion samples 无 raw/window/tracking/effort guard trigger；joint4 effort/tracking peaks为 `2.3565 N.m / 0.01089 rad`，没有 torque saturation 或无法到位证据。但验收 client 在成功后直接 disable，Joint4 失去 holding torque 并从目标附近下坠到 `0.27142 rad`。用户明确要求成功后必须保持 enabled hold，或先受控回到可失能位置再 disable；因此本轮只确认 trajectory/guard，safe-posture 完整操作子门仍不关闭。最终六轴 status 0、IDLE、串口释放，P6 checkbox不变。证据：`Agent/evidence/P5/2026-08-08-safe-posture-window-persistence-pass.md`。

随后按用户授权完成正确收尾的完整低速往返：当前 disabled state → 20 秒 safe-posture → enabled hold → 20 秒返回已知 baseline → 到位检查 → disable。两段 action 均 `SUCCEEDED/error_code=0`，1740/1781 samples均无 guard trigger；safe-posture 最大误差 `0.00297 rad`，回位最大误差 `0.000763 rad`。最终六轴 status 0、IDLE、串口释放。正式通过前两次启动异常分别在 enable 前退出、以及 monitor callback 异常后立即 stop+disable，均未混入 pass metrics。safe-posture 子门关闭，但 staged hardware checkbox还包含 pre-grasp、approach、gripper、lift、retreat，故仍不勾选。证据：`Agent/evidence/P5/2026-08-08-safe-posture-roundtrip-pass.md`。

pre-grasp 当前场景只读预检正确 fail closed：真实 Gemini 2/YOLO/local GraspNet 20 秒产生 190 candidates，但 20/20 filtered plans 均无 IK。当前每帧最高 confidence detection 是左上 `umbrella`，bbox depth约 `705 mm`，转换后典型 base Y 达 `-0.85~-1.00 m`；`bowl` bbox depth中位数约 `2115 mm`，也不是可用近距目标。隔离 MuJoCo 已对齐真实 disabled start，probe max joint delta `1.45e-6 rad`；未打开机械臂串口。等待用户在中央摆放 depth `300–450 mm` 的独立目标并移走/遮挡 umbrella 后重做 plan-only，pre-grasp 子门保持未完成。证据：`Agent/evidence/P5/2026-08-08-pregrasp-current-scene-blocked.md`。

用户摆放瓶子后，pre-grasp 真实视觉/规划前置门通过：bottle bbox depth median `334 mm`；20 秒 170 candidates 形成 5 条 fresh、IK-reachable、collision-valid plans。选取 fresh bottle pre-grasp 后，MoveIt `/plan_kinematic_path` 返回 `SUCCESS(1)`、65 points、planning time `57.7 ms`；MuJoCo max joint delta `1.47e-7 rad`，hardware action server 为 0，未打开串口或执行。该结果解除场景 blocker，但不等于实机 pre-grasp 运动通过；下一步需用户明确授权一次“pre-grasp → enabled hold → 受控返回 baseline → disable”的实机往返，且不包含 approach/gripper/lift/retreat，因此 P6 staged hardware checkbox 仍不勾选。证据：`Agent/evidence/P5/2026-08-08-bottle-pregrasp-plan-only-pass.md`。

用户随后授权上述受限 pre-grasp 往返，但执行前重取的 fresh plan 显示当前瓶位需要大幅六轴折叠：joint2/joint3/joint5 分别约 `-99.0/-61.8/+56.7 deg`，最大 joint delta `1.72851 rad`，超过 runner 的 `1.2 rad` 总位移门。两次 runner 均未发送 trajectory；唯一一次 enable 只在 baseline hold 10 秒，之后 stop + disable，最终位置未变、六轴 status 0。将每帧候选限制为 1 后 plan age 改善至 `0.827–1.032 s`，但 40 秒 disabled-only 搜索仍未找到 `max joint delta <=1.2 rad` 的 plan。P6 staged hardware checkbox 保持未完成，等待调整瓶位后 plan-only 复测，或针对该大动作与实际扫掠路径重新授权。证据：`Agent/evidence/P5/2026-08-08-bottle-pregrasp-execution-gated.md`。

用户移近原瓶并更换新瓶后的复测发现目标选择 blocker。首次 MoveIt 启动遗漏 `use_fake_joint_states=false`，fake/real joint-state 混流段已明确作废；纠正后仅有真实 controller publisher，机械臂全程 disabled。新 bottle bbox depth median `241 mm`、YOLO confidence median `0.3445`，低于左侧 umbrella 常见 `0.42–0.55`；而当前 GraspNet node 固定选择全帧最高 confidence detection，因此正常链路实际优先推理 umbrella。临时 bottle-only relay 的 20 秒为 298 candidate output frames、0 non-empty、0 valid plan，不能形成 joint-delta 审查目标。P6 checkbox 不变；下一步需要改善瓶子在画面中的完整性/距离，并补 target-class selection policy 后再 plan-only。证据追加：`Agent/evidence/P5/2026-08-08-bottle-pregrasp-execution-gated.md`。

用户将原瓶放回并明确接受约 `99 deg` joint2 大动作后，完成 pre-grasp 正式低速往返。纠正后的运行使用 `use_fake_joint_states=false` 和唯一真实 controller joint-state publisher；fresh plan 在 final gate age `0.980 s`，实际 joint2 delta `1.61765 rad/92.68 deg`。去程与回程各 45 秒，action 均 `SUCCEEDED/error_code=0`，5336/5318 samples 无 guard trigger；pre-grasp 最大误差 `0.005083 rad`，回位最大误差 `0.001526 rad`。只在回到本轮 baseline 后 disable，约 15 秒复测无材料级下坠，最终六轴 status 0、IDLE、串口释放。pre-grasp 子门关闭；P6 staged hardware checkbox仍包含 approach、gripper、lift、retreat，故保持未勾选。证据：`Agent/evidence/P5/2026-08-08-bottle-pregrasp-roundtrip-pass.md`。

2026-08-09 交接决策：下一优先任务不是直接进入 approach，而是补齐 P5 MuJoCo–真机 paired sim-to-real 对比。现有 P1 证据是 MuJoCo 自测及 upstream/current 两个仿真 backend 的 comparison；现有 P5 真机证据主要保存每轴 peak/error/warning 等汇总 metrics，两者没有使用同一 command 的 time-aligned raw samples，不能视为 sim-to-real calibration。下一任务应先定义统一 command、采样字段、时间基准和 comparison metrics，完成 software-only active upstream MuJoCo replay；如需重采真机 raw trajectory，继续遵守单次明确授权、enabled hold/受控回位后 disable。P5 MuJoCo geometry/dynamics/contact checkbox与 P6 staged hardware checkbox均保持不变。

用户授权并确认系统上电后，已完成同一 exact command 的 active upstream MuJoCo–真机 paired raw 采集：实时 disabled baseline 生成 401 点/20 秒 quintic 去程与回程，`rebotarm_mujoco_node` 与真机四段 action 均 `SUCCEEDED` 且无 guard stop；真机回位最大误差 `0.000763 rad` 后才 disable，最终六轴 status 0、无 error、串口释放。joint4 real tracking RMS 去/回为 `0.004346/0.004200 rad`，real-sim position RMS 为 `0.004316/0.004092 rad`。这关闭了“无同 command/time-aligned raw pair”的证据缺口，但只覆盖局部 safe-posture；effort measurement semantics 尚未标定且无共同 contact topic，因此 P5 geometry/dynamics/contact checkbox 与 P6 staged hardware checkbox仍保持不变。证据：`Agent/evidence/P5/2026-08-09-mujoco-real-trajectory-comparison.md`、`.json`。

hand-eye 多姿态 residual 前置检查已 fail closed：Gemini 2 正常，但当前画面没有有效标定板；`DICT_ARUCO_ORIGINAL id=0` 的零星框经图像复核是货架纹理误检。另确认 RGB marker PnP 使用 color intrinsics，而当前 hand-eye child 命名为 `camera_depth_frame`；Gemini 2 factory color/depth translation 约 14 mm，后续必须显式统一 frame convention。独立 `rebotarm_calibration` residual analyzer、姿态覆盖和 provisional acceptance gate 已落地；等待刚性固定 `DICT_4X4_50 id=0`、黑色边长 100.0 mm 的 marker 后再做 disabled detection stability 和多姿态轨迹审查。本轮未启动 controller/enable/trajectory，hand-eye checkbox不变。证据：`Agent/evidence/P5/2026-08-09-handeye-residual-target-preflight.md`。

用户随后放置并调整标定纸；marker 完整进入画面后 `DICT_4X4_50 id=0` 连续 `89/89` 帧检出，center std `0.112/0.122 px`，按 100 mm 边长计算的 color-frame PnP translation std 最大 `0.392 mm`、span 最大 `0.875 mm`。目标 detection stability 前置门已通过；由于机械臂尚未改变姿态，该结果不等于 hand-eye residual，checkbox仍不变。下一步是 6–8 个安全姿态的 plan-only/轨迹审查和单独运动授权。证据同上。

六姿态 hand-eye residual 实机验收现已形成明确 FAIL 结论：replacement pose 补齐后 translation/rotation span 为 `69.046 mm / 22.191 deg`，diversity gate 通过；两种 color/depth frame hypothesis 的 rotation RMS/max 约 `1.43/1.92 deg`，但 position RMS/max 均约 `15.9–16.0 / 32.2–32.8 mm`，超过 `5/10 mm` 门。加入 Gemini factory depth-to-color 只改善约 `0.125 mm RMS`，不能解释误差。五种 OpenCV AX=XB 方法训练 RMS 最好仍约 `14.62 mm`，方法间 translation 可相差 `>100 mm`，leave-one-out 最好约 `25.8 mm`；因此当前数据足以拒绝旧配置，但不足以部署新外参。`handeye.yaml` 未改，P5 checkbox保持未勾选，下一步为更大且多轴的 10–15 姿态重标定与独立 hold-out 验收。证据：`Agent/evidence/P5/2026-08-09-handeye-multipose-residual.{md,json}`。

15-pose 重标定首轮只形成 3 个合格 captures；`train_02` 三次 stability gate 均因 marker center std 约 `4 px`、joint5 span 最高 `0.015259 rad` 失败。自动回 reference 被 joint6 单次 raw velocity gate 取消后，系统按永久规则保持健康 enabled hold；30 s operator recovery 随后成功，失能前最大 reference error `0.001144 rad`，确认后才 disable。数据集未完成、`handeye.yaml` 未改，P5 checkbox保持未勾选；下一步先做 resume route 与替代姿态的软件审查，再单独申请新实机授权。证据：`Agent/evidence/P5/2026-08-09-handeye-recalibration-interrupted.md`。

用户授权续采后又新增 3 个 accepted captures，当前累计 `6/15`；但 joint6 raw immediate gate 在两轮分别以 `0.315018/0.344322 rad/s` 取消，且第二次 window peak达到 `0.11028 rad/s`，不能未经审查直接放宽。`holdout_03/train_05` 的 stability capture 也各三次失败，证据同时指向关节 hold 与目标/支架/相机振动的未排除影响。所有失败后均保持健康 enabled hold并受控回 reference，验证后才 disable；最终 status0/no error/tty free，handeye未改，P5 checkbox保持未勾选。证据：`Agent/evidence/P5/2026-08-09-handeye-recalibration-resume-interrupted.md`。

用户确认 marker 固定并授权剩余 9 姿态后，9/9 captures、11/11 motion legs 全部成功，无 guard stop；最终受控回 reference 的 enabled max error `0.001907 rad` 后才 disable，status0/no error/tty free。累计虽达到 15/15，但前 6 个采于最后一次重新固定之前，不能与固定后 9 个合并；固定后解反算旧 cohort 的 rotation residual 为 `6.630/7.277 deg RMS/max`，而单姿态 30 帧内部 rotation jitter RMS 仅 `0.322–0.948 deg`。固定后 8 training + 1 holdout 明显更一致，但 holdout 数量/diversity不足，且 training position RMS仍为 `5.305 mm > 5 mm`。`handeye.yaml` 未改，checkbox保持未勾选；补采原 6 姿态的同固定状态方案已通过 MoveIt `707/707`，新实机 motion 待明确授权。证据：`Agent/evidence/P5/2026-08-09-handeye-recalibration-complete-dataset.{md,json}`。

用户确认旧 6 姿态期间标定纸确有移动并授权补采后，已在同一固定状态补齐 `12 training + 3 holdout`。两轮共 11/11 motion legs success、无 guard stop，均在 reference max error `0.000763 rad` 后才 disable，最终 status0/no error/tty free。PARK training position通过，但 training rotation RMS `1.715 deg`、holdout position/rotation RMS `5.634 mm/1.732 deg` 和 holdout rotation span `17.850 deg` 未过门，故不部署。根因进一步收敛到旧 detector 的 `CORNER_REFINE_NONE`：同一 90 帧 SUBPIX A/B 将 rotation jitter `0.704 -> 0.079 deg RMS`、reprojection `0.197 -> 0.031 px`。已启用 SUBPIX；旧 raw无RGB不能离线修复，后续需 marker 不动条件下重新采完整数据并扩展 holdout，checkbox保持未勾选。证据：`Agent/evidence/P5/2026-08-09-handeye-recalibration-final-fixed.{md,json}`。

完整 SUBPIX 重采已通过离线 acceptance：同一固定 marker 下 12 training + 3 independent holdout 均使用 `CORNER_REFINE_SUBPIX`，三轮 23/23 motion legs、8,777 samples 无 guard stop，并在 reference max error `<=0.001907 rad` 后才 disable。DANIILIDIS training position/rotation RMS 为 `2.267 mm/0.496 deg`，holdout为 `2.088 mm/0.369 deg`，holdout rotation span `38.905 deg`，所有既定 residual/diversity gates 通过；四种非离群方法候选spread `0.487 mm/0.488 deg`。但本轮未获配置部署授权，当前 runtime仍使用旧 `handeye.yaml`，也尚未做部署后真实TCP residual，因此 hand-eye checkbox暂不勾选。证据：`Agent/evidence/P5/2026-08-09-handeye-subpix-final.{md,json}`。

用户随后授权部署上述候选。旧配置已留 hash 备份，active/source 与 install `handeye.yaml` 已一致更新为 DANIILIDIS candidate；官方 Ubuntu vision 入口成功连接真实 Gemini 2、运行 YOLO、发布 RGB/depth/CameraInfo/detections，并持续发布新 `end_link -> camera_depth_frame` 静态 TF。runtime frame convention 复核确认 3D aligned RGB-D path 使用 color coordinates/intrinsics，但沿用 `camera_depth_frame` working-frame 名称，未重复施加 factory extrinsic。full suite `584 passed, 14 skipped`、layering 18、compile/diff通过，全程未启动机械臂或占用串口。部署后 independent real TCP residual 仍未执行，因此 hand-eye checkbox继续保持未勾选。证据：`Agent/evidence/P5/2026-08-09-handeye-subpix-deployment.md`。

部署后 independent temporal holdout 首轮取得 1 个新稳定 capture，相对原 training reference 的残差为 `2.837 mm/0.463 deg`，单点通过；但去第二点的 joint6 `-0.42 -> +0.21 rad` 直接跨越被 raw `0.315018 > 0.30 rad/s` immediate gate取消。系统保持健康 enabled hold，随后两段 operator recovery 共 10,025 samples无guard stop，enabled回位最大误差 `0.001907 rad` 后才disable，最终status0/no error/tty free。当前仅 `1/3` 新 holdouts，不能关闭hand-eye checkbox。删除大跨越、从reference进入剩余joint6同簇姿态的30秒路线已MoveIt `303/303` valid，实际重试待新授权。证据：`Agent/evidence/P5/2026-08-09-handeye-postdeploy-holdout-interrupted.md`。

用户再次授权改线重试后，剩余两个 temporal holdouts 与首轮首点合计形成完整三点独立验收。相对原 training reference 的 position RMS/max为 `2.495/2.837 mm`、rotation RMS/max为 `0.359/0.463 deg`；三个新点自身 RMS/max为 `2.478/2.972 mm`、`0.356/0.491 deg`，rotation span `38.792 deg`。sample count、固定reference residual、内部一致性与diversity全部通过。retry 4/4 action、20,046 samples无guard stop，enabled回位max error `0.000763 rad` 后才disable，最终status0/no error/tty free。因此当前安装hand-eye多姿态稳定性checkbox关闭；物理TCP仍未标定。证据：`Agent/evidence/P5/2026-08-09-handeye-postdeploy-holdout-final.{md,json}`。

## P6: 系统集成与最终验收 [weight=10]

2026-08-11 P6 entry cleanup / 进入P6前清理已完成：项目运行进程为空，`/dev/ttyACM0`存在但无人占用，`/tmp`无匹配的P5/MuJoCo临时项；源码侧两个`__pycache__`与仓库根`.pytest_cache`共`80 KiB`已移入桌面回收站。保留全部用户修改、`Agent/evidence`、模型、虚拟环境与`build/install/log`。该清理只建立P6干净起点，不增加验收checkbox，也不授权任何真实运动。

2026-08-11 P6 software-only/read-only gap audit完成。当前hybrid launch仍默认network vision/network GraspNet并启动RViz-only `rebotarm_sim_trajectory_controller`，没有接入唯一active `rebotarm_mujoco_node`，因此“当前Ubuntu-native感知 + MuJoCo”不能勾选。`scene.xml`虽有`fixed_camera`，但尚无ROS RGB-D/CameraInfo/annotation publisher；离线闭环、四类抓取结果和全异常安全停止均只有分散模块证据。下一软件优先级为修复hybrid composition并确保simulation backend互斥。证据：`Agent/evidence/P6/2026-08-11-p6-software-gap-audit.md`。

2026-08-11 P6-A hybrid composition已现代化：专用入口显式使用Ubuntu-native vision、localhost GraspNet service与active `rebotarm_mujoco_node`，并关闭RViz-only trajectory controller以避免同namespace接口冲突；MuJoCo-only launch支持`rebotarm_sim` namespace与初始姿态覆盖。Focused `100 passed`，顺序两包build成功，installed `--show-args`可展开，full `590 passed, 15 skipped`、compile/diff通过。仓库存在`simulation -> MoveIt config -> bringup`既有package cycle，不能通过反向依赖修补，已记录为packaging debt。该结果只证明composition ready，尚未启动真实Gemini 2/local service/MuJoCo runtime，因此P6保持`2/8`。证据：`Agent/evidence/P6/2026-08-11-hybrid-composition-modernization.md`。

2026-08-11 P6-B Ubuntu-native perception + active MuJoCo no-motion runtime acceptance通过。隔离domain内真实Gemini 2/YOLO、localhost GraspNet、MoveIt IK/collision filter与唯一active `rebotarm_mujoco_node`同轮运行，持续产生detections、candidates与valid filtered plans；五个只读plan-age样本为`0.578–2.108 s`，均低于仅hybrid使用的`3.0 s`门限。相机、plan与MuJoCo joint-state header已统一到simulation clock；GraspNet contract以独立`sent_at_unix_ns`保留HTTP freshness检查。`/rebotarm_sim/follow_joint_trajectory`只有一个server，旧namespace为零server，无真实/RViz-only controller，未调用start service或发送trajectory。final full suite `594 passed, 15 skipped`、layering `18 passed`、compile/diff通过；退出后进程、8081与串口均清理。该证据只关闭当前真实视觉+MuJoCo项，不覆盖虚拟RGB-D、完全离线闭环、自动抓取或新增实机动作。证据：`Agent/evidence/P6/2026-08-11-ubuntu-native-mujoco-runtime.md`。

P6-B final cleanup / 最终清理随后完成：仓库根`.pytest_cache`及`src/`、`tests/`、`tools/`、`Agent/`、`scripts/`内共22个`__pycache__`，合计23个cache目录、约`4.5 MiB`，已可恢复地移至桌面回收站`/home/a/.local/share/Trash/files/rebot_Arm-cache-p6b-20260811-1559`与`/home/a/.local/share/Trash/files/rebot_Arm-cache-p6b-final-20260811-1605`。未触碰用户修改、验收证据、模型、虚拟环境、`.local-models`、`third_party`或`build/install/log`；清理后项目runtime为空、8081关闭、串口无人占用。为保持清理结果，不在此后重复运行会重建cache的pytest/compile。

2026-08-11 P6-C MuJoCo virtual RGB-D/CameraInfo/object annotations验收通过。active `rebotarm_mujoco_node`新增默认关闭的EGL offscreen sensing；显式启用后唯一节点同步发布`rgb8`、毫米`mono16`、两路`CameraInfo`与MuJoCo segmentation ground-truth `Detection2DArray`。隔离domain实测320x240五路消息同stamp，depth有效范围`740..1999 mm`，`test_cube` bbox/mask有效，16帧simulation-time平均`10.027 Hz`；sim action唯一server、旧namespace零server。首轮EGL跨线程`EGL_BAD_ACCESS`已通过dedicated renderer worker修复；default-off对照无camera/detection topics。final full suite `601 passed,15 skipped`、layering18、build/compile/show-args/diff通过。最后将本轮生成的20个限定cache目录（`4344 KiB`）可恢复地移至桌面回收站，保留全部证据/模型/环境/build/install/log。该证据只关闭virtual sensing checkbox；camera-to-base TF、offline YOLO/GraspNet/MoveIt/MuJoCo闭环和outcome matrix仍未关闭。证据：`Agent/evidence/P6/2026-08-11-mujoco-virtual-rgbd-annotations.md`。

2026-08-11 P6-D fixed-camera optical TF与ground-truth-assisted offline plan-only链路通过。active MuJoCo按`+X right,+Y up,-Z forward`到ROS optical `+X right,+Y down,+Z forward`的显式转换发布`base_link -> mujoco_fixed_camera_optical_frame`；depth反投影点在40 mm cube体积内。新增无真实driver、无RViz-only controller、无executor的离线组合入口，virtual RGB-D/ground-truth annotation经localhost GraspNet、TF、MoveIt IK/collision filter在干净启动45秒内连续产生3个valid plans，并保留nominal TCP`[-0.105,0,0] m`。ROS 2 Jazzy节点专用YAML覆盖launch wildcard参数的问题已通过candidate filter单一完整launch profile修复。final full suite`605 passed,15 skipped`、layering18、build/compile/show-args/diff通过。该里程碑未运行rendered RGB YOLO、未发送MuJoCo trajectory、未覆盖outcome/failure matrix，因此完全离线闭环checkbox保持未勾选，P6仍为`4/8`。证据：`Agent/evidence/P6/2026-08-11-mujoco-offline-tf-plan-only.md`。

P6-D final cleanup随后将仓库内`.pytest_cache`及20个project `__pycache__`共21目录、`4.6 MiB`可恢复地移至桌面回收站`/home/a/.local/share/Trash/files/rebot_Arm-cache-p6d-20260811-1647`。保留用户修改、证据、模型、虚拟环境及`build/install/log`；清理后无项目runtime/8081 listener，`/dev/ttyACM0`存在但无人占用。为保持清理结果，不在清理后重复运行测试。

2026-08-11 P6-E rendered RGB offline YOLO fail-closed子门完成。新增独立`rebotarm_offline_yolo_node`与纯函数helpers：订阅active MuJoCo virtual`rgb8`，发布raw detections与显式`target_classes=['cube']`过滤后的detections；隔离domain运行中frame `20/40/60/80/100`均为`raw=2,target=0`，raw仅见stock COCO `cup/cake`误检，filtered best-effort echo为`detections: []`。该结果证明输入、推理、allowlist和fail-closed行为，不证明stock COCO能识别canonical`test_cube`，也没有把raw误检接入GraspNet。focused offline tests `4 passed`、layering `18 passed`、全量`609 passed,15 skipped`、两包symlink build、完整package/launch compileall及launch show-args通过；未发送MuJoCo trajectory，真实硬件未触碰。验证后 scoped cache 共20个目录、约`4.1 MiB`已移入可恢复回收站；按规则补跑compileall新增的7个目录、约`988 KiB`也移入`/home/a/.local/share/Trash/files/rebotarm-cache-p6e-final-20260811-1717`，当前scoped cache为0，用户资产、证据、模型、虚拟环境及`build/install/log`保留。P6仍为`4/8`，完全离线checkbox保持未勾选；证据：`Agent/evidence/P6/2026-08-11-rendered-rgb-offline-yolo-fail-closed.md`。

2026-08-11 P6-F active MuJoCo arm-only trajectory execution / 仿真机械臂轨迹执行子门通过。隔离`ROS_DOMAIN_ID=103`启动唯一`rebotarm_mujoco_node` action server，以5秒小幅 joint1 `+0.05 rad` roundtrip 往返 baseline；`FollowJointTrajectory` goal accepted，result `error_code=0 / simulation trajectory finished / SUCCEEDED`，结束样本相对 baseline joint1误差约`1.30e-5 rad`，其余轴约`1e-7 rad`量级。未启动虚拟相机、GraspNet、executor或夹爪/contact，未连接真机；该子门不等于rendered RGB YOLO→GraspNet→MoveIt→MuJoCo完全闭环，也不改变P6总计`4/8`。后续状态刷新产生的1个`__pycache__`（约`224 KiB`）已移入可恢复回收站`/home/a/.local/share/Trash/files/rebotarm-cache-p6f-final-20260811-1721`，当前scoped cache为0。证据：`Agent/evidence/P6/2026-08-11-mujoco-trajectory-roundtrip-software-only.md`。

2026-08-11 P6-F2 integrated plan-to-MuJoCo arm-only execution / 规划到仿真机械臂专用执行子门通过。隔离`ROS_DOMAIN_ID=104`中，带simulation-time时间戳的 P6-D valid `GraspPlan` 经现有`VisualGraspExecutorNode`、`PoseExecutionNode`和MoveIt进入唯一active`/rebotarm_sim/follow_joint_trajectory`；`move_to_pregrasp`、`approach_grasp`、`lift`、`safe_retreat`全部`trajectory executed`，夹爪日志明确`simulation: gripper command skipped`，未做contact/接触验证。首轮复用错误姿态四元数的precheck得到`99999 / Unable to sample any valid states for goal tree`，独立`/compute_ik` sweep找到IK-valid top-down quaternion后重试成功；说明不能只复用位置而猜测orientation provenance。随后受控回MuJoCo baseline，收敛后六轴最大误差`4.35e-5 rad`；action server唯一为`rebotarm_mujoco_node`，旧RViz-only controller不存在，8081/串口均未占用。该子门关闭plan-only→active MuJoCo arm-only execution，但本轮正向plan是P6-D valid plan的带时间戳软件回放，仍不关闭fully offline checkbox；stock COCO对canonical`test_cube`的正向目标模型/场景证据与unified outcome/failure matrix仍缺，P6保持`4/8`。证据：`Agent/evidence/P6/2026-08-11-mujoco-integrated-plan-arm-only.md`。

2026-08-11 范围确认：暂不推进额外的RViz `MotionPlanning`面板接线或一键演示入口；当前仍按原P6 baseline推进，不新增P7，不回退现有代码或用户修改。用户确认原 `test_cube` 只是范例，active canonical MuJoCo target / 当前规范仿真目标切换为 `bottle` proxy / 瓶子代理；下一步使用 bottle 场景继续分级实机验收。Contact calibration已按用户决定移出规划，approach/gripper/lift/retreat仍不构成硬件授权。

- [x] 已有真实感知 + 仿真执行的历史 launch/benchmark 基础。
- [x] 使用当前 Ubuntu native 感知完成真实视觉 + MuJoCo 验收；同轮真实Gemini 2/YOLO/local GraspNet/MoveIt/active MuJoCo无运动runtime通过，证据：`Agent/evidence/P6/2026-08-11-ubuntu-native-mujoco-runtime.md`。
- [x] 增加并验收 MuJoCo 虚拟 RGB-D、CameraInfo 和物体标注；默认关闭、显式EGL启用，RGB/depth/intrinsics/ground-truth mask同simulation stamp，证据：`Agent/evidence/P6/2026-08-11-mujoco-virtual-rgbd-annotations.md`。
- [范围收口] 不再要求以 `tools/yolo26s-seg.pt` 重新完成 bottle 的 YOLO/GraspNet/MoveIt/MuJoCo software-only positive loop。该项未通过；2026-09-06 operator 关闭当前 P6 并决定后续另建规划，因此从当前计分范围移除。历史 YOLO-World、bottle proxy、TensorRT 与 plan-only 证据继续保留。
- [x] 完成安全实机 plan-only 验收；真实 driver 保持 disabled，真实 Gemini 2/YOLO/GraspNet 候选从真实 joint start state 完成 MoveIt `SUCCESS(1)` 规划，规划后 joint delta `0.0 rad`，未发送 action goal。证据：`Agent/evidence/P5/2026-08-08-real-disabled-start-state-plan-only.md`。
- [范围收口] 分级实机验收不再要求继续完成当前 P6 的全部 lift/retreat 等剩余动作。该组合项未通过；single-joint、safe-posture、pre-grasp、approach 及既有夹爪动作只保留各自历史证据，不把未执行的 lift/retreat 或抓取结果补记为成功。
- [范围外] 无力反馈条件下不建立空抓、滑落、未抬起和成功的非显式自动分类；仅记录实际动作、反馈和安全门结果。
- [范围外] 不建立统一失败矩阵；现有各单点 `fail-closed / 失败即安全停止` 安全门继续保留。

2026-09-06 P6 closeout / 阶段收口：operator 明确关闭当前 P6，后续另行加入新的规划。上面两项由未完成改为“范围收口”，不是验收通过；当前计分项因此由 `4/6` 调整为 `4/4`。这只关闭阶段，不产生任何新的硬件动作授权。

同日完成 operator 授权的真实视觉 plan-only 测试。独立 `codex/gripper-unified-bus-fix` worktree 中，真实 controller 保持 `disabled/IDLE`，Ubuntu-native Gemini 2、TensorRT YOLO、in-process GraspNet 与 MoveIt 产生 fresh `valid=true` bottle plan（confidence `0.9310057`、jaw `0.0749830 m`、frame `base_link`、capture age `2.946 s`）；MoveIt `execute=false` 的 pregrasp/grasp 分别成功生成 `55` 点/`5.33271924 s` 和 `82` 点/`8.086942298 s` 轨迹。调用前后六轴 status 全 0、无 error，gripper status0 且 position 均为 `0.0013012104 rad`。未启动 VisualGraspExecutor、未 enable、未发送 trajectory 或夹爪命令；退出后项目进程、ROS daemon、8081/8088 与 `/dev/ttyACM0` 占用均为空。该测试是关闭前的运行态确认，不把范围收口项改写为通过。

2026-08-11 P6 bottle target验证后的现场已按用户要求清理：无项目runtime、无8081 listener、`/dev/ttyACM0`无人占用、无旧`rebotarm_sim_trajectory_controller`。源码/测试范围内21个`__pycache__/.pytest_cache`目录、10个P6临时文件和491个ROS `launch_params_*`均可恢复移入`/home/a/.local/share/Trash/files/rebotarm-cache-p6-clean-20260811-Pjqfpk`；`.local-models`、weights、`build/install/log`、证据和用户修改保留。下一步准备staged hardware acceptance / 分级实机验收，只做只读预检，不新增硬件动作授权。

2026-08-11 用户决定退回本轮 unified outcome/failure matrix / 统一结果失败矩阵软件项，不将其作为当前P6验收项；下一步切换为 staged hardware acceptance / 分级实机验收只读准备。当前不启动真机、不打开串口、不发送trajectory或夹爪动作，下一硬件门按既有顺序为 approach，需单独明确授权。

2026-08-12 用户确认将 unified failure matrix / 统一失败矩阵从当前P6范围剔除，不作为待办或验收项；保留历史证据，不删除模型、代码或既有记录。当前重点为 staged hardware acceptance / 分级实机验收。

2026-08-12 主视觉模型切换为 `tools/yolo26s-seg.pt`：`rebotarm_vision` setup 在构建时从该源文件安装 package-share 模型，Ubuntu-native、visual bringup 与 MuJoCo offline launch 默认均改为 `yolo26s-seg.pt`。Focused tests `13 passed`，`rebotarm_vision` 与 `rebotarm_bringup` symlink build通过，安装模型 SHA-256 与源文件一致；尚未启动相机或真机，需用新模型重新做 bottle software-only positive loop。

2026-08-31 主视觉默认进一步切换为 package-share `yolo26m-seg-fp16-b1-640-linux.engine`，并保留 `yolo26s-seg.pt` 作为显式CPU/兼容回退；TensorRT runtime `10.13.3.9.post1` 已进入 `.venv-vision`。真实 Gemini 2 → engine → localhost GraspNet → candidate IK/collision filter → MoveIt 2 plan-only 同轮通过：`/visual_grasp/execute` 成功，pregrasp/approach/lift/safe-retreat 四段均为 `moveit trajectory planned`，夹爪命令均跳过，两次joint-state采样完全一致且未打开串口。集成时detections约`15 Hz`，fresh image/filtered plan约`1.4–1.6 Hz`；因默认`1.0 s` freshness会拒绝部分plan，测试显式使用既有`3.0 s` software-only override，未放宽生产默认。Focused `90 passed`、build、layering18、compileall通过；全量`631 passed,15 skipped,1 failed`，唯一失败为既有upstream snapshot license状态断言。该证据关闭本次TensorRT→MoveIt plan-only请求，但未启动active MuJoCo执行，因此上方完整positive-loop checkbox保持未勾选。证据：`Agent/evidence/P6/2026-08-31-yolo26m-tensorrt-moveit-plan-only.md`。

2026-08-11 用户授权 staged hardware acceptance 的 approach 单阶段后，fresh-plan 安全门 fail-closed：bottle confidence `0.855`、plan age `0.821 s`；MoveIt plan-only 的 pregrasp 最大关节变化 `1.981618 rad`，从 baseline 预检的 grasp/approach 最大变化 `2.249726 rad`，其中 joint3 `2.184955 rad`。此前已审查的 bottle pre-grasp 路线 joint2 最大变化为 `1.61765 rad / 92.68 deg`；当前路线为未经现场扫掠净空审查的 materially larger route。未 enable、未发送 trajectory、未执行 gripper/lift/retreat/contact，故不计 approach 通过，也不构成真实运动失败。运行结束后项目进程、8081、串口和旧 controller 均清理。下一步需用户明确接受该具体大路线并完成净空审查，或调整瓶位后重规划。证据：`Agent/evidence/P6/2026-08-11-approach-gated-large-joint-delta.md`。

2026-08-11 用户随后明确接受该具体大位移路线；受保护 runner 实际发送 pregrasp 去程，但 joint4 raw velocity `0.3003654479980469 rad/s` 触发 `raw_velocity_corroborated_immediate`（limit `0.30 rad/s`），action `status=5/error_code=-1/canceled`。去程仅 `147` samples，未发送 approach、夹爪、抬升、撤退或接触。controller/motor status 保持健康 enabled hold，随后按规则执行 `25 s` guarded return，回程 `5011` samples、`SUCCEEDED/error_code=0`，enabled baseline 最大误差 `0.00038147 rad`，确认后才 disable；最终六轴 status0/no error、runtime/8081/tty均清理。该结果计为 approach 尝试被 joint4 安全门中止，不计 approach 通过；禁止直接重复同一路线，下一步先做 joint4 trajectory/control root-cause review。证据：`Agent/evidence/P6/2026-08-11-approach-guarded-joint4-stop.md`。

2026-08-11 用户授权慢速重试：pregrasp 与 approach 各 `45 s`、接近位保持 `20 s`、回 baseline `45 s`。fresh bottle confidence `0.8004082`、age `0.9302 s`；pregrasp `91` points/`9009` samples、approach `53` points/`9008` samples、return `9005` samples，三段均 `SUCCEEDED/error_code=0`，无 guard stop。pregrasp baseline max delta `2.00476864 rad`，approach 从实际 pregrasp 起点 max delta `0.58265462 rad`；回位最大误差 `0.00152588 rad`，确认后才 disable，最终 status0/no error/runtime/8081/tty clean。该结果关闭 approach staged hardware sub-gate，但组合 checkbox 仍因 gripper/lift/retreat 未完成而保持未勾选；未执行夹爪、抬升、撤退或接触。证据：`Agent/evidence/P6/2026-08-11-approach-45s-roundtrip-pass.md`。

2026-08-11 用户要求识别当前桌面瓶子并按 `45/20/45 s` 执行一套完整抓取。只读 real-mode runtime 复核显示 bottle confidence `0.874–0.885`，中心 depth `364 mm`、bbox median `373 mm`（p10/p90 `363/382 mm`），TF 反投影到 `base_link` 约 `[-0.0281,-0.5444,0.1456] m`；局部深度统计未证明 depth 错误。当前 GraspNet best candidate confidence 仅 `0.2335`，candidate IK filter 持续 `error_code=-31`，`/grasp/filtered_plan` 为 `valid=false/no IK-reachable grasp candidates`，因此按 fail-closed 未 enable、未发 trajectory、未执行夹爪/lift/retreat/contact。该请求未形成完整抓取证据，当前阻塞收敛为 depth-to-grasp/GraspNet candidate/IK profile mismatch，需 fresh valid plan 后再执行。证据：`Agent/evidence/P6/2026-08-11-full-grasp-perception-gated.md`。

2026-08-11 用户移开瓶子后的障碍物后要求重新读取。8 个 bottle detection confidence `0.8759–0.8862`，中心 depth `364–365 mm`、中央 ROI median `367–372 mm`，live TF 反投影约 `[-0.02815,-0.54452,0.14598] m`；GraspNet raw candidates 已恢复到每帧 `1–9` 个，best raw score 采样最高约 `1.03`，但 filtered candidates 连续为 `0`，filtered plan 仍 `valid=false/no IK-reachable grasp candidates`，固定 `base_axis` grasp/pregrasp 持续 MoveIt `error_code=-31`。障碍物移开不是充分条件，当前不证明深度错误已解决，也不授权盲目完整抓取；未 enable、未发 trajectory、未执行夹爪/lift/retreat/contact，控制器只读为 disabled/status0。证据：`Agent/evidence/P6/2026-08-11-full-grasp-obstacle-cleared-reread.md`。

2026-08-11 用户更换瓶子后要求查看是否可行。新瓶子检测代表性 confidence `0.8969967`，中心 depth `395 mm`、中央 ROI median `401 mm`，TF 反投影到 `base_link` 约 `[-0.0280,-0.5729,0.1331] m`；10 秒窗口 GraspNet 有 `18` 个 non-empty 输出、单帧 `1–10` 个候选、best raw score 约 `1.23`，但 `17` 个 filtered plans 全部 `valid=false/no IK-reachable grasp candidates`，当前 `base_axis` grasp/pregrasp 持续 MoveIt `error_code=-31`。新瓶子改善了 raw perception 但未形成可执行计划，未 enable、未发 trajectory、未执行夹爪/lift/retreat/contact，控制器保持 disabled/status0。证据：`Agent/evidence/P6/2026-08-11-new-bottle-reread.md`。

2026-08-11 用户将新瓶子移近后要求重新查看。代表性 bottle confidence `0.9256485`，中心 depth `285 mm`、中央 ROI median `301 mm`（p10/p90 `284/346 mm`），TF 反投影到 `base_link` 约 `[0.00009,-0.45886,0.13810] m`；10 秒窗口 GraspNet 有 `19` 个 non-empty 输出、单帧多为 `10` 个候选、best raw score 约 `1.08`，但 `16` 个 filtered plans 全部 `valid=false/no IK-reachable grasp candidates`，当前 `base_axis` grasp/pregrasp 持续 MoveIt `error_code=-31`。瓶子位置已改变但 profile 仍无可执行计划，未 enable、未发 trajectory、未执行夹爪/lift/retreat/contact，控制器保持 disabled/status0。证据：`Agent/evidence/P6/2026-08-11-new-bottle-moved-closer-reread.md`。

2026-08-11 完成只读 IK 诊断扫描。`move_group`/KDL `/compute_ik` 在 `group=arm`、`ik_link=end_link`、`avoid_collisions=false` 下正常响应；fixed top-down quaternion 在代表性 `y=-0.596 m` 的 9 个 z 高度全部 `NO_IK_SOLUTION(-31)`，旧 z-rotation quaternion 9/9 `SUCCESS(1)`；fixed quaternion 网格成功集中在 `y=-0.48/-0.44 m`。按最新瓶子近似目标与 nominal TCP/TCP offset、`0.08 m` grasp z-offset，pregrasp 可解而 grasp 不可解；去掉 z-offset 或换旧 z-rotation可解。结论收敛为姿态/TCP/z-offset组合问题，不是 KDL/MoveIt IK backend整体故障。full real-mode复启还出现 `joint1 feedback unavailable after 3 attempts` 并导致 `base_link`/`camera_depth_frame`断树，作为独立现场阻塞记录；未 enable、未发 trajectory。证据：`Agent/evidence/P6/2026-08-11-ik-diagnostic-sweep.md`。

2026-08-11 参数来源核对确认：P5 hand-eye 只负责 `end_link -> camera_depth_frame` 相机外参；`tcp_offset_xyz=[-0.105,0,0]` 是用户接受的 upstream `ee_site` nominal TCP 工程值，不是 hand-eye 或实物 pivot 结果；此前实机命令显式传入的 q_y fixed top-down quaternion `[0,0.707106781,0,0.707106781]` 属于 P6 bottle software profile，覆盖 launch 原默认 q_z `[0,0,-0.707106781,0.707106781]`。用户观察的瓶子竖直/夹爪平行与该姿态语义错配一致；未在确认真实末端坐标方向前重新部署或执行实机动作。证据：`Agent/evidence/P6/2026-08-11-grasp-orientation-provenance.md`。

2026-08-11 旧 z-rotation quaternion plan-only 验证通过：显式回读 `fixed_grasp_orientation_xyzw=[0,0,-0.707106781,0.707106781]`，`candidate_pose_policy=base_axis` 与 `tcp_offset_xyz=[-0.105,0,0]` 下，对上一轮瓶子位置近似 candidate 生成 `/grasp/filtered_plan valid=true`；pregrasp/grasp orientation 均为旧四元数，grasp 位置约 `(-0.000,-0.564,0.113) m`。本轮为软件隔离姿态验证，未启动真实相机、串口、真机 controller、夹爪或 trajectory，不宣称实物方向/抓取成功。证据：`Agent/evidence/P6/2026-08-11-old-z-rotation-plan-only.md`。

2026-08-11 单次瓶子抓取返回尝试未形成验收通过：第一次夹爪开位 `0.090 m` 实际只到 `0.085 m`，在 pregrasp 前 fail-closed 并回 baseline；第二次 fresh bottle plan（confidence `0.426`、旧 z-rotation、jaw `0.0813 m`）完成 pregrasp plan-only，按 `45 s` retime 发出 pregrasp，但未得到可追溯的 approach/close/return action result，故不宣称抓取成功。最终受控回 baseline，最大 joint 误差约 `0.00267 rad`，controller `disabled/IDLE/status0/no error`。staged hardware checkbox 不变，仍不勾选 gripper/pickup；证据：`Agent/evidence/P6/2026-08-11-single-grasp-return-fail-safe.md`。

2026-08-11 上游 hand-eye 参数切换与单次抓取准备的 plan-only 复核完成。`src/rebotarm_vision/config/handeye.yaml` 已对齐 `/home/a/project/rebot_refer@152f8f14ceec5cc8fc61664ff63915187a412800`，TF 为 `end_link -> camera_depth_frame`、平移 `[-0.085,0.010,0.045] m`、四元数 `[-0.523092811,0.578134825,-0.405717914,0.476997914]`；nominal TCP `[-0.105,0,0] m` 与旧 z-rotation 保持不变。fresh real-mode domain `125` 只读状态为 controller disabled/IDLE/status0/no error；瓶子 live candidate 可间歇性形成 `valid=true`，MoveIt execute=false 对 pregrasp/grasp 均规划成功（代表性 `73/76` points）。未 enable、未发送 trajectory、未操作夹爪，故不勾选任何新增 hardware acceptance 项，也不宣称手眼物理对齐或抓取成功。证据：`Agent/evidence/P6/2026-08-11-upstream-handeye-bottle-plan-only.md`。当前仍需清理 runtime，后续若用户明确执行才重新做 fresh-plan 安全门。

2026-08-11 用户因未观察到上一轮动作要求立即重试。live perception 在 fresh 窗口持续 `valid=false/no IK-reachable grasp candidates`，未直接复用旧 joint trajectory；按用户要求复用上一轮已验证 Cartesian bottle pose，并在当前 start state 重新 MoveIt execute=false 规划（pregrasp/grasp `83/87` points，最大关节变化 `2.272753/0.294408 rad`）。受保护重试按 `45 s pregrasp → 45 s approach → close → 20 s hold → 45 s return baseline` 完成，三段 action 均 `SUCCEEDED/error_code=0`、无 guard stop。夹爪开位 `0.08290 m`；闭合请求 `0.05844 m`、实际 `0.06056 m`，不推断抓取或滑落。回 baseline 最大误差 `0.000381 rad` 后 disable，最终 `IDLE/disabled/status0/no error`；未 lift/retreat/contact，现场 runtime/8081/tty/旧 sim controller清理。证据：`Agent/evidence/P6/2026-08-11-upstream-handeye-single-grasp-return-repeat.md`、`.json`。

2026-08-12 夹爪重新安装后完成 gripper zero calibration / 夹爪零位校准。只读预检确认机械闭合对应 feedback `position=0.0`、status0；arm controller disabled/IDLE、六轴 status 全 0、无 error。原 `/rebotarm/set_zero` 只支持 arm motor，已在 `rebotarmcontroller.HardwareManager` 增加显式 `joint_name=gripper` 分支，要求 disabled、停止 gripper loop、验证 gripper status0 后仅调用 gripper motor `set_zero_position()`；没有调用 arm-wide zero。实际服务调用返回 `success=True/set_zero complete`，校准后 feedback仍 `position=0.0`、status0，arm无运动。focused safety tests `21 passed`、controller symlink build通过；最终 controller/串口已清理。该项只证明 encoder zero 写入，不证明物理指宽或抓取效果。证据：`Agent/evidence/P6/2026-08-12-gripper-zero-calibration.{md,json}`。

2026-08-11 用户明确授权执行一次上游 hand-eye bottle grasp。fresh bottle plan confidence `0.73182195`、jaw `0.07044034 m`、旧 z-rotation；MoveIt plan-only pregrasp/grasp 成功，最大关节变化 `2.271795/0.288131 rad`。受保护实机动作按 `45 s pregrasp → 45 s approach → close → 20 s hold → 45 s return baseline` 完成：pregrasp、approach、return 均 `SUCCEEDED/error_code=0`，`guard_stop=null`。夹爪打开实际 `0.08297 m`；闭合请求 `0.05844 m`、实际 `0.06055 m`，仅记录位置服务结果，不判断瓶子是否夹住或滑落。回 baseline 最大误差 `0.000381 rad`，确认后 disable；最终 `IDLE/disabled/status0/no error`。未执行 lift、retreat、接触标定；runtime、8081、`/dev/ttyACM0`、旧 sim controller 均清理。该项证明上游参数下受保护动作链完成，不证明 bottle pickup 成功。证据：`Agent/evidence/P6/2026-08-11-upstream-handeye-single-grasp-return.md`、`.json`。

## 2026-08-13 current-source correction / 当前源码更正

本节以当前 source 为准，并保留上文 `-0.105 m` 相关 P5/P6 evidence 为当时的历史记录。当前真机视觉的 `tcp_offset_xyz` 与 active local MuJoCo `ee_site` 均为 `[-0.04,0,0] m`。历史上 `real_perception_sim_execution.launch.py` 曾传 `[0,0,0]` 以避免 local `ee_site` 重复施加，该顶层组合入口现已删除。`-0.105 m` 仅存在于固定 third-party upstream MuJoCo snapshot，不再是 active runtime/model 值；当前审计没有新增 physical TCP measurement / 物理 TCP 测量证据。

当前 controller source 已无 persistent zero record / 持久零位、startup rehome / 启动回零和 `NaN unknown / 未知` 分支；磁盘遗留的 `~/.local/state/rebotarm/gripper_zero.json` 不被读取，本轮不删除。普通非零夹爪位置目标仍是 upstream continuous position hold / 上游持续位置保持，而非历史中已回退的 neutral/idle 修复。

`gripper_position_torque_cap_nm` 的配置界面仍为默认 `0.40 N.m`、允许 `0.05..1.20 N.m`，但当前 non-zero normal position target / 非零普通位置目标没有把它传入 `_gripper_safe_mit(tau_limit)`；该函数实际采用 `_G_TAU_MAX=1.5 N.m` 作为合成 MIT 命令限幅。故 `0.40/1.20 N.m` 不能作为该路径的有效电机侧硬上限或夹爪动作安全验收依据。后续真实夹爪动作前必须先处理此 source safety gap / 源码安全缺口；本次仅同步文档，没有改源码、启动硬件、enable 或发送 trajectory / 轨迹。

2026-08-14 已按用户授权在现默认 `gripper_position_torque_cap_nm=1.5 N.m`、仅 `driver_only` 下执行一次无负载 `0 → 10 mm → 0`。启动前夹爪10个样本稳定在`1.8299 mm/status0`，六轴status全0；开夹服务返回`7.9618 mm`，其后约12秒位置稳定`8.0236 mm`、torque为`-0.0024..-0.0122 N.m`，未继续向10mm爬升。闭合服务返回`2.0771 mm`，但第一个返回后样本已继续到`0.7450 mm`，才稳定且扭矩近零；按用户异常即停要求立即disable，不重试。故`1.5 N.m`未消除开夹约2mm残差，且闭合侧仍有返回后位移，不能关闭夹爪 neutral+idle / 中性脱力空闲释放的真机验收；post-check六轴及夹爪均status0，controller/tty已停止释放。下一步只允许软件时序诊断，不自动再次动作或提高力矩。

2026-08-14 大行程复核：operator确认机械闭合后，仅 `driver_only` 执行 `0 → 50 mm → 0`，默认`1.5 N.m`；零位预检10样本均`0mm/status0`。开夹返回`48.0007 mm`，后续约12秒固定`48.0831 mm`且torque `-0.0024..-0.0073 N.m`，没有持续向外开夹。闭合返回`1.8643 mm`，首个返回后样本继续到`0.6695 mm`后才稳定；按停止门立即disable、未重试。post-check六轴和夹爪status全0且controller/tty均清理。开夹侧释放有大行程真机证据；闭合侧返回后位移重复出现，当前仍不得宣称完整夹爪释放/到位时序验收通过。

2026-08-14 首次无物体 `GraspGripper` 低力闭合未满足力控保持验收前提。预检为夹爪`0.0034mm/status0`、默认move cap `1.5N.m`、hold timeout `30s`、六轴status全0；30mm开位返回`27.8679mm`。随后指定`close_force/hold_force=0.4N.m`、`2.0s` close timeout、`0.08s`最短时间、`0.04rad/s`速度阈值、`6mm`最小闭合量的服务返回`success=false/contact_detected=false/contact_position=0/reached=8.8407mm`，message为`grasp close timeout before stall detected`。服务未进入holding，故30秒自动neutral释放及5秒后的`SetGripper`命令接管均未执行；按异常即停立即disable，post-check六轴/夹爪status全0、controller/tty清理。此结果不表示夹到或未夹到物体，只说明当前无物体2秒闭合不满足既定stall detection / 堵转检测判据。

2026-08-14 用户确认首次动作无异常声响后授权相同低力参数二次复测。夹爪零位预检稳定`0.4086mm/status0`，30mm开位返回`27.8542mm`；同一`GraspGripper(0.4/0.4N.m,2.0s,0.08s,0.04rad/s,6mm)`再次返回`success=false/contact_detected=false/contact_position=0/reached=8.9300mm`和同一close-timeout message。二次可重复地未进入holding，故hold timeout/主动命令接管仍无真机证据；立即disable并清理controller/tty。后续只能先进行软件级的close-loop速度/位置判据诊断，不能自动提高力、重复或接瓶子。

2026-08-31：Gemini 2 点云显示阻塞已修复。Image/CameraInfo QoS 已统一，Jazzy `depth_image_proc` 实测发布 `/camera/depth/points`，捕获到 640x480、frame `camera_depth_frame` 的 `PointCloud2`；证据见 `Agent/evidence/P6/2026-08-31-gemini2-pointcloud-qos.md`。该验证仅使用相机与视觉节点，不构成 P6 真机运动验收。

2026-08-31：增加 GraspNet 采样前完整 XYZ+RGB 场景的 Open3D 只读查看器，实测复用同一 `build_scene_cloud()` 后输出约 12 万点、z 上限受同一 `1.5 m` 深度门控制；证据见 `Agent/evidence/P6/2026-08-31-graspnet-full-scene-viewer.md`。不改变 P6 或硬件验收状态。

2026-09-01：candidate IK filter 已从忙时直接丢帧改为 latest-wins 单槽调度，并把 `0.4` confidence、`0.006..0.085 m` jaw 与 geometry/workspace gate 前移到 IK/collision 服务之前。60秒 software-only plan-only 实测收到150 raw并发布149个filter结果，coverage由上一轮`40/122=32.8%`提高到`149/150=99.3%`；首条P6 eligible由`14.899s`缩短为`4.849s`，3597个joint samples变化`0rad`。本轮只关闭IK filter frame-starvation软件缺陷，不勾选新增P6硬件验收项；剩余瓶颈为实时score>=0.4候选稀疏及geometry/IK可达性。证据：`Agent/evidence/P6/2026-09-01-candidate-filter-latest-wins.md`。

2026-09-01：latest-wins修改后第二次60秒完整候选链复测为164 raw、153 filter results、8 IK/collision valid且P6 eligible；通过率`4.88% raw / 5.23% filtered / 30.77% score-qualified`，首条`4.748s`，1798 joint samples变化`0rad`。另取fresh eligible执行最终MoveIt `execute=false`，pregrasp/grasp均规划成功（`53/66` points、约`102.5/102.1ms`），额外382 joint samples变化`0rad`。该结果继续只计software-only plan availability，不勾选硬件验收项。证据：`Agent/evidence/P6/2026-09-01-candidate-filter-latest-wins-repeat.md`。

2026-09-02 范围决策：后续将 GraspNet 从独立 localhost HTTP/`8081` 迁入 Ubuntu 视觉抓取链并统一管理生命周期；迁移完成前保留现有 HTTP 路径作为基线。另因用户观察当前 TCP 中心可能有小幅偏差，后续先做受控、可量化的 TCP 对位/残差测试，再依据证据决定是否调整 active `[-0.04,0,0] m`。本条仅记录决策，未改代码/配置、未启动 runtime 或硬件，也不改变 P4/P5/P6 checkbox。

2026-09-03 GraspNet in-process integration / 进程内直连整合完成。Ubuntu active launch不再包含`local_service`、`local_infer_url`或`graspnet_local_infer_url`，同一visual-grasp launch通过`.venv-graspnet/bin/python`管理GraspNet ROS进程，完整RGB-D/CameraInfo/detection直接进入既有full-scene runner。真实Gemini 2 + TensorRT YOLO只读runtime确认`backend_available=True`并发布`source=ubuntu_inprocess_graspnet`、同stamp/frame的bottle candidate；同轮8081无listener且串口无人占用。preview launch同时补齐默认TensorRT模型参数透传。两包build、focused、layering、full suite与compile/diff通过；未启动MoveIt/controller/action或发送trajectory，不改变P6硬件checkbox。Windows/network兼容模式和旧HTTP工具作为回退保留。证据：`Agent/evidence/P6/2026-09-03-graspnet-inprocess-integration.md`。
