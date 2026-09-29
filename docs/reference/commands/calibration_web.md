# 网页标定操作说明

> 状态：REFERENCE；类型：标定软件操作手册；适用范围：Dashboard `/calibration`；真实硬件动作仍需单独授权。

本功能位于现有 Dashboard 的 `/calibration`，共用端口。相机必须已经通过原生 ROS
驱动发布 Image/CameraInfo。标定节点不打开串口、不启动相机、不自动使能或部署结果。

## 前置条件与启动

如果工作台已经运行，不要再次启动工作台或控制器。可以在已加载 ROS 环境的另一终端
仅启动只读标定节点：

```bash
cd /home/huangbin/robotarm_ros2
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

结束或关闭网页不会改变机器人模式。退出重力补偿会由控制器最后目标角接管位置保持；
等待新鲜IDLE反馈后，再点一次退出可解除网页标定占用，不重复下发stop。
之后按既有受控回位、到位/支撑确认、失能顺序处理，不能把关闭网页当作停止或失能。

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
