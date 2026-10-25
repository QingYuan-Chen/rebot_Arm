# 软指模型替换

> 状态：IMPLEMENTED（模型资源）；实机安装、开口/TCP 标定及柔性接触仍需验收。

## 装配范围

按用户确认，保留原夹爪电机、主体、齿条和滑块，左右各装一套
`Soft_Gripper_Mount.step` 与 `Soft_Gripper_Finger.step`。
源文件归属 `src/rebotarm_moveit_config/cad/soft_gripper/`；保留的原始左右 STL
仅用于可复现提取传动件，不作为当前 URDF 的旧指体继续显示。

旧左右 STL 各有三块连通网格：齿条 2148 三角形、滑块 1080 三角形、旧指体
1542 三角形。当前 `left_finger.stl` / `right_finger.stl` 仅保留前两者的原始顶点。
新增 `left/right_soft_gripper_mount.stl` 和 `left/right_soft_gripper_finger.stl`。
每侧三份 visual/collision 共用原 finger link 和移动关节，固定零件之间不新增自由度。

## CAD 尺寸与坐标

两个 STEP 均以毫米为单位，且各包含一个有效实体。
底座包围盒为 29 × 42.04169 × 33.5 mm；软指为 30.00432 × 87.48928 × 28.5 mm。
按三处连接孔对齐后，组合件总长 93.38928 mm；CAD 布尔交集体积为零。
三个孔轴在 XY 平面形成 20 × 14 mm 的直角三角形。

以下变换均先旋转再平移，平移单位为毫米：

- Finger → Mount：绕 X 旋转 180°，平移 `[-94.209253443758, 141.125760638401, 0]`。
- Mount → 左指 link：绕 Z 旋转 -90°，平移 `[-55.60932368, 19.65, 0]`。
- 右侧新零件：将左侧新零件绕 link 原点的 X 轴旋转 180°。

底座背面 `Y=-15` 对齐原左滑块前面 `X=-70.60932368`。
底座背面的四孔间距 10 × 15 mm，与原左滑块的孔心
`Y=14.8/24.8, Z=±7.5` 对应。右侧采用设计对称装配，原右齿条与滑块顶点原样保留；
原右滑块 STL 存在小幅不对称，最终贴合和紧固仍以实物核验为准。

STL 顶点统一乘 `0.001` 写成米，已包含安装变换；URDF 使用单位缩放和零 visual origin。
网格零位下，两软指包围盒最小间距约 3.766 mm，沿指长的间距还会变化。
因此现有关节单侧 45 mm、双侧 90 mm 表示行程，不能直接当作新软指已标定的净开口。

## 显示、规划与物理边界

- `rebotarm_moveit_config/config/rebotarm.urdf` 是唯一规范模型；RViz 和 Dashboard
  共用新资源。主体、六轴关节、左右移动关节和驱动接口保持原有定义。
- 夹爪底座、保留传动件、安装座和左右软指的可视材质统一为与机械臂一致的银灰色；颜色只影响 visual，
  不改变 collision 几何或接触参数。
- MoveIt 为每侧传动件、底座和软指分别加载三角网格碰撞体。
- MuJoCo 从规范 URDF 生成，使用每侧三份独立凸包，避免一个凸包跨过整侧传动件与指体。
  同一侧齿条与滑块仍共用一个保守凸包。软指镂空被凸包填充，仅为未变形刚体近似。
- 软指摩擦沿用原模拟值，其他新部件使用普通接触值；材料参数未实测。
  手指未配置实测惯性；MuJoCo 仅用可视组的凸包和默认密度推断，碰撞副本不重复贡献质量。
  该刚体估计填充了镂空，不作为真实载荷。
- 不用 CAD 猜测覆盖真机 TCP、开口映射、力矩或零位参数。
  现有 `end_link -> grasp_tcp = [-0.04, 0, 0] m` 和历史开口标定尚未对新夹指验收。
  实际抓取前应重新测量 TCP、净开口、质量/质心与夹持力；柔性变形需单独建模。
- 独立 MJLab 的冻结模型快照不随本次资源替换自动更新。

## 重新生成

可选 CAD 环境依赖见 [cad.md](../../setup/dependencies/cad.md)。先生成两套同源网格：

```bash
/path/to/cad-env/bin/python scripts/import_soft_gripper_meshes.py \
  --output-dir src/rebotarm_moveit_config/meshes \
  --output-dir src/rebotarm_simulation/models/rebotarm/assets
```

生成器校验四个源文件 SHA256；源模型变化后需要重新核对安装变换，不能沿用旧坐标。
再用现有仿真环境生成 MJCF：

```bash
PYTHONPATH=src/rebotarm_simulation python3 -m rebotarm_simulation.model_tools.urdf_to_mjcf \
  --repo-root "$PWD"
```

更新安装资源：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rebotarm_moveit_config rebotarm_simulation
```

检查包含：旧传动件顶点保留、米制尺寸和左右开合、三部分 visual/collision 覆盖、
仿真资产一致性、软指与物体的接触/张开分离，以及仓库要求的完整检查。
运行结果记录于[项目状态](../../reference/project_status.md)，不据此宣称实机或柔性仿真验收。
