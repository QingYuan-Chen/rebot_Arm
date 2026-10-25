# D405 腕部相机模型

2026-10-25 按用户确认，将腕部装配改为 D405，沿用原 Gemini 2 的腕部安装位置和朝向。
统一模型归属 `rebotarm_moveit_config`；RViz、Dashboard 和 MuJoCo 读取同一套装配。
实际 RGB-D 驱动、CameraInfo、手眼标定与视觉抓取仍需后续接入和现场验收。

## 来源与转换

- 用户原始底座保存于 `src/rebotarm_moveit_config/cad/d405/D405_305_Mount.step`，未改孔位或轮廓。
  STEP 为毫米，单个有效实体，尺寸约 46 × 70.905 × 33.156 mm。
- 相机使用 [RealSense 官方模型](https://github.com/realsenseai/realsense-ros/tree/7a87097c499e3b0b69518d54a8e7a9ac1924e87c/realsense2_description)，
  固定提交 `7a87097c499e3b0b69518d54a8e7a9ac1924e87c`。
  原始 STL、参考 Xacro、Apache-2.0 许可证和摘要保存在同一 CAD 目录。
- `scripts/import_d405_meshes.py` 校验输入 SHA256，将毫米统一转为米，保留各自 CAD 坐标轴。
  底座 2006 个三角形，相机 3892 个三角形。网格是官方简化外壳，不含镜头纹理。
  网格及许可证安装到模型资源中，MuJoCo 使用相同网格副本。

使用[可选 CAD 环境](../../setup/dependencies/cad.md)运行：

```bash
python3 scripts/import_d405_meshes.py \
  --output-dir src/rebotarm_moveit_config/meshes \
  --output-dir src/rebotarm_simulation/models/rebotarm/assets
ros2 run rebotarm_simulation rebotarm_urdf_to_mjcf --repo-root "$PWD"
```

## 装配与坐标

URDF 固定链为 `end_link -> d405_mount_link -> d405_link`，没有新增控制关节。
这两个新增 link 是装配 CAD 坐标，不是已标定的相机光学坐标，不能当作手眼标定结果使用。

- `end_link -> d405_mount_link`：平移
  `[-0.08630933, 0.0002079068932795, 0.0000007587263523] m`，
  RPY `[pi/2, 0, pi/2]`。腕部孔距 22 mm，与旧底座完全相同。
- `d405_mount_link -> d405_link`：平移
  `[0.0010014555241297094, 0.06301963080951393, 0.011604510071599904] m`，
  绕 X 旋转 `0.2617962523055204 rad`，约 15°。
  官方 STL 的后背 `z=-0.023 m` 两个 M3 孔与底座 20 mm 孔距同心，前面法向为局部 +Z。
- 保留既有 `wrist_camera_mount` 仿真 site 的接口含义，它不是 D405 光学坐标。

底座的容纳槽宽 43 mm，孔组中心相对槽中心偏移 1 mm；对孔装配后，官方外壳与单侧壁约有
0.5 mm 重叠。用户确认后按原始尺寸导入，没有平移相机来掩盖此偏差。此模型不构成实物配合验收。
详细离线分析在本机 `log/d405_mount_inspection_20261025/`。

## 碰撞与仿真质量

MoveIt 使用原始三角网格；MuJoCo 将底座分为托盘底板、四壁和三个腕部连接盒体，
相机使用 42 × 42 × 23 mm 盒体。近似忽略螺孔、圆角、局部窗口，底座空腔没有被单个凸包填满。
固定装配的四个配对（末端与底座、腕部法兰 link6 与底座、底座与相机、末端与相机）排除内部接触，
对其余机械臂和场景物体的碰撞保持有效。

`config/d405_payload.yaml` 属于仿真包，只在临时 URDF 中注入动力学估计：

- 相机暂取 72 g，参考官方 Xacro 的占位值；上游将这组惯性数据标为不可靠，故此处也仅作临时估计。
  惯量重新按均匀包络盒计算，真实质量与惯量仍需测量。
- 底座暂取 35 g（约等于 CAD 实体体积乘 1200 kg/m³），质心和每单位质量惯量来自 STEP 积分。
  打印填充率、线缆与实物质量尚未测量。

权威 URDF 没有新增估算惯量，真实硬件的重力补偿负载没有由本次导入重新调节。
旧 Gemini 2 仿真载荷注入器已移除，其历史 CAD 资源仅作来源参考。

测试覆盖源网格一致性、独立 CAD 孔位配准、镜头方向、固定链、载荷隔离、零位/home 无假接触、
相机对场景物体碰撞以及有限物理步进。渲染与本次回归证据位于
`log/d405_model_import_20261025/`，不代表实机驱动或标定通过。

隔离 ROS 域 112 的 MoveIt 实测中，`safe_home` 和 `home` 都通过状态有效性检查；
在相机位置加入测试障碍物后，检查正确报告 `d405_link` / `d405_mount_link` 与障碍物碰撞。
启动仅包含假关节状态、TF 与 MoveIt，测试结束后已清理本轮进程。
本次完整回归为 **1250 passed / 2 skipped**，分层 **26 passed**，模型与仿真两包构建通过。
一次性检查/渲染脚本及临时 CAD 环境已清理；正式导入工具、回归测试、源 CAD 和最终验证结果保留。
