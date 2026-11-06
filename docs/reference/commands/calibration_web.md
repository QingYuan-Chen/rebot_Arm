# 网页标定操作说明

> 状态：REFERENCE；类型：标定软件操作手册；适用范围：Dashboard `/calibration`；真实硬件动作仍需单独授权。

本功能位于现有 Dashboard 的 `/calibration`，共用端口。相机必须已经通过原生 ROS
驱动发布 Image/CameraInfo。标定节点不打开串口、不启动相机、不自动使能或部署结果。

## 前置条件与启动

如果工作台已经运行，不要再次启动工作台或控制器。可以在已加载 ROS 环境的另一终端
仅启动只读标定节点：

```bash
cd /home/a/project/rebot_Arm
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rebotarm_calibration rebotarm_handeye_capture
```

在现有 Dashboard 地址后加 `/calibration`，默认地址为 `http://127.0.0.1:8088/calibration`。
节点启动后提供 `/rebotarm_handeye_capture/command`。话题不是默认值时显式指定：

```bash
ros2 run rebotarm_calibration rebotarm_handeye_capture --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info \
  -p session_directory:="$HOME/.ros/rebotarm_calibration"
```

同一个会话目录只允许一个标定节点写入。工作台下一次正常启动时，也可在原启动命令
加 `calibration:=true`；默认关闭。不要同时运行独立节点和工作台内的标定节点。
`rebotarm_app.launch.py` 为真机工作台，本文不自动启动它；保持原硬件现场检查流程。

## 操作步骤：眼在手上

1. 固定相机安装和标定板，实测黑色标记外边框边长（米）。板面需平整，采集中不可移动。
2. 选择“眼在手上”，填写实际 base/end 和图像 optical frame、marker id/字典/边长。
   camera_frame 必须与 Image 和 CameraInfo 的 header.frame_id 一致，不能仅凭名称猜轴向。
3. 创建会话，保留会话 ID。先检查机器人 TF，再刷新快照和执行只读预检。
   预检检查同步图像、内参、标记质量与稳定机器人位姿；不是物理标定精度证明。
4. 操作者检查反馈、净空和支撑后，分别点击明确使能和进入重力补偿。网页会再次确认。
   无硬件/执行开关关闭时按钮请求会被后端拒绝。重力补偿不保证松手静止。
5. 人工缓慢调整到不同位置和旋转方向，保持稳定后采样。只绕单轴、重复姿态或不足覆盖
   不能通过。建议15–20个训练姿态，独立采集至少5个验证姿态；软件下限为5+5。
6. 验证集不参与算法选择。固定板和相机参数全程保持一致；必要时另建新会话。
7. 点击求解并验证，检查算法表中的位置mm/角度deg误差、失败门、逐样本残差与
   bootstrap诊断。bootstrap只描述样本敏感度，不覆盖内参/打印尺寸等系统误差。
8. 通过后填写操作者并确认已检查报告，再记录人工接受。accepted不等于部署。
9. 推荐点击“下载服务器保存的数据包”；“导出当前快照”可能不是服务器最新状态。
   离线重复求解可运行：

```bash
ros2 run rebotarm_calibration rebotarm_handeye_calibration \
  --input session.json --output recalculated_report.json
```

如只需复核已有会话的残差和姿态多样性，可运行独立分析入口；它不连接相机、机械臂，
也不会修改原始会话：

```bash
ros2 run rebotarm_calibration rebotarm_handeye_residual \
  --input session.json --output residual_report.json
```

结束或关闭网页不会改变机器人模式。退出重力补偿会由当前验证位置接管位置保持；
等待新鲜IDLE反馈后，再点一次退出可解除网页标定占用，不重复下发stop。
之后按既有受控回位、到位/支撑确认、失能顺序处理，不能把关闭网页当作停止或失能。

## 实时距离与采样反馈

手眼会话的“预检与姿态采样”页每 0.5 秒刷新一次“相机到标记中心”估测，显示直线距离、
光学坐标系 XYZ、光学轴深度 Z、当前会话边长与重投影误差。计算使用同步彩色图像、
CameraInfo 和 ArUco/PnP，距离起点为相机光学中心，终点为标记中心。当前标记实测
75 mm，新会话边长默认 `0.075 m`；恢复会话沿用其保存值。换用其他标记时填写对应实测边长，
估测的平移尺度随此值变化。

该只读估测通过独立的 `/rebotarm_handeye_capture/observation` 服务刷新，复用采集的图像、
内参和标记质量门；无需机器人 TF 或稳定窗口，采集等待期间仍更新。图像过期、遮挡、
内参不匹配或质量失败时清除数值并显示原因；离开采样页或隐藏网页时暂停刷新。
此服务只接受 `observe`，读取当前会话和订阅快照，保留样本数、会话版本及相机订阅。
恢复会话会切换至该会话的图像与内参话题；多个页面选择不同话题时，显示话题不匹配的
页面需恢复相应会话。

点击采集后，按钮附近显示等待状态和最终结果；按钮停用时显示停用原因。采集请求
默认最多等待 `capture_timeout_sec=5.0` 秒，持续重试图像、ArUco、TF 和稳定窗口。
等待结束后返回最后一次观测原因；失败请求跳过样本写入，并在标定节点日志中记录
命令、会话 ID、请求 ID、原因码和原因。请求进行中的重复调用会显示等待提示。
页面底部“运行状态”保留结果，距离自动刷新单独更新距离区域。

单次采集检查当前观测质量；跨样本姿态覆盖和求解残差仍由覆盖统计及求解报告评估。

手眼采集要求机器人与标记观测同时稳定：窗口默认 0.4 秒、至少 3 帧、相邻源时间
最大间隔 0.2 秒，两侧平移跨度均至多 1 mm、旋转跨度均至多 0.5°。图像和内参使用
RELIABLE 接收；自定义相机发布端也需提供可靠传输。成功时选择窗口内总位姿距离最小
的实测配对帧，保留该帧图像时刻的机器人与标记位姿。重复机器人姿态在写入前返回
已有样本 ID，调整姿态后再采；同一请求的重试沿用已有结果。

## 基坐标系与采样证据

当前机器人推荐配置为 `base_frame=base_link`、`end_link_frame=end_link`、
`camera_frame=camera_color_frame`。`base_link` 是六轴运动链根坐标系，MoveIt 的 `arm`
组也从此处起始；`end_link` 通过固定 `end_joint` 连接 `link6`，包含现有法兰偏置。
采集读取 `lookup_transform(base_link, end_link, 图像时间)`，得到末端在基座中的位姿。
求解使用 `base_to_end × end_to_camera × camera_to_marker = base_to_marker`。

现有启动组合的 `world → base_link` 为固定恒等变换。固定更换基座参考只改变固定板在
基座中的表达，相机安装外参保持一致；同一会话应持续使用所选参考系。
若机器人反馈中断，动态 TF 也会中断，先按控制器具体故障处理并恢复新鲜反馈，再检查同期 TF。

新样本中的 `capture_window` 保存各帧两侧位姿、角点、内参及时间，`selected_index`
标明代表帧；`image` 保存该帧无损 PNG/base64 及 SHA256，支持离线重检角点和 PnP。
`joint_state_evidence` 保存图像时刻两侧最近的原始 JointState 与时间差，订阅话题为
`/<arm_namespace>/joint_states`；独立采集节点默认 `arm_namespace=rebotarm`，工作台
自动传入机械臂命名空间。关节源消息缺失时 `samples=[]`，可结合保留的 TF 与窗口记录复核。
这些内容随服务器会话 JSON 原子保存。采集实现和条件已变化，下一次采集请创建新会话。

## 操作步骤：TCP 枢轴法

选择TCP模式，只需base/end TF；无需图像。工具的同一个物理尖点必须在整个训练和验证
期间保持同一空间点。手扶改变工具方向，不能移动固定点。训练联合求TCP和空间固定点，
验证使用训练得到的固定点而非重新拟合。它只标定TCP位置，不标定工具旋转方向。

需要独立运行传统 TCP 交互节点时使用：

```bash
ros2 run rebotarm_calibration rebotarm_tcp_calibration
```

该节点只负责采样/计算接口，仍需由操作者确认 TF、固定点和现场安全；通常优先从
Dashboard 的 `/calibration` 工作流进入，不要同时启动两个 TCP 标定节点。

## 失败处理与恢复

刷新页面后点击恢复/刷新按ID读服务器会话。create/capture等请求携带ID和revision，
旧页面版本会被拒绝；超时表示结果未知，先等待并刷新，勿反复创建新的采样请求。
服务不在线时显示不可用。采样或求解失败不自动退出重力补偿、不自动disable。
出现“故障保持”时先查看页面中的具体原因并扶稳机械臂。仅有重力保持错误且反馈正常时，
可确认退出补偿，待成功交接位置保持后重新进入；失能操作仍按原有现场支撑流程执行。
恢复条件和失败处理见[重力补偿控制](../../implemented/features/hardware_execution.md#重力补偿控制)。
solved会话可重新开放采样，旧报告失效；accepted/aborted为只读终态，需另建会话。
原始数据位于session_directory，原子JSON写入；不同采样保存实际阈值、源码指纹和时间。

## 验证结果与边界

软件测试使用隔离ROS域、合成图像/TF、控制器替身和canonical MuJoCo模型。
真实相机/电机重力补偿/物理精度仍需要独立现场验收，不因网页软件测试通过而视为完成。
详细证据与限制见 [标定软件验收边界](../../implemented/features/calibration_acceptance.md)。

同一会话首个样本会固定采集条件：CameraInfo、有效阈值和软件来源。重启后若改变
内参、分辨率、采集阈值或标定代码版本，再采样会被拒绝，须新建会话。不要通过修改
旧JSON绕过检查；旧会话仍可只读导出和复核。

网页可填写每个手眼会话的image_topic和camera_info_topic（绝对ROS话题名）。
采样/预览切换会话时节点重新订阅并清空旧缓存，旧订阅排队回调被代次标识拒绝；
切换后立即预览可能提示暂无图像，稍后刷新即可。TCP不需要切换图像订阅。

数据包包含created_at/updated_at（UTC）、逐样本sample_index/recorded_at和逐操作audit。
样本序号跨训练/验证统一递增；重试同一request_id不重复记录。audit记录revision、
request_id、操作与接受操作者，用于追溯，不替代设备时钟或防篡改签名。
