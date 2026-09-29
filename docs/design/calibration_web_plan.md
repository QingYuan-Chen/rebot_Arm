# 网页手眼标定技术路线

> 状态：DESIGN；类型：网页手眼标定技术路线；适用范围：真实相机、真实机械臂和物理精度缺口；软件链路已实现并通过合成/替身验证，但不代表所有阶段已经结束。

## 目标和边界

网页用于把人工标定流程变成可追踪的向导：显示状态、提示操作者、采集当前样本、
展示质量门、比较求解结果和导出候选配置。网页不实现 PnP、手眼求解、TCP 数学、TF
查询或硬件控制；这些能力分别属于 `rebotarm_calibration` 与现有 controller/motion
包。真实硬件默认保持失能，网页初期只支持人工移动后的只读采样。

## 分层方案

```text
Browser wizard
    -> local Dashboard HTTP/SSE
    -> calibration ROS adapter (rebotarm_calibration)
    -> TF/Image/CameraInfo read-only capture
    -> SessionStore + handeye/tcp math
    -> versioned report / candidate export
```

Dashboard 只调用 ROS 接口和读取状态，不导入标定算法。标定包不依赖 Dashboard，避免
网页生命周期、HTTP 线程和数学/采集逻辑互相耦合。所有输出先是候选结果，必须人工确认
后才能复制到 `rebotarm_vision` 配置；网页不能自动部署配置。

## 阶段计划

### Phase 0：契约和离线核心

- 固定 session schema、状态机、样本索引和 abort 语义；
- 实现不依赖 ROS/HTTP 的会话状态核心（正式实现为 `SessionStore`）；
- 固定 hand-eye/TCP 报告 schema、阈值和 `accepted` 与 `pass` 的区别；
- 离线 JSON 数据集可重放，确保网页不需要硬件也能开发。

### Phase 1：ROS 只读采集适配器

- 新增 calibration node，负责 TF、Image、CameraInfo 的同步和质量门；
- 提供 `start_session`、`capture_sample`、`finish_session`、`abort_session`、`get_status`；
- 每个样本记录 ROS 时间、单调时间、frame、TF 年龄、图像/内参关联和拒绝原因；
- 不发布运动目标，不 enable，不写部署配置。

### Phase 2：离线求解和验证

- hand-eye 多方法求解（TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS）；
- 训练集/留出集验证；
- observability、残差、离群点和不确定度报告；
- 原子写入 session dataset/report，支持断点恢复和重复分析。

### Phase 3：Dashboard 后端接线

- 新增 `/api/calibration/*` 白名单路由；
- Dashboard 只做 ROS service/action client 和状态聚合；
- SSE 推送 session、capture、quality、solve、abort 状态；
- 所有写操作带 session id，拒绝过期/重复请求。

### Phase 4：网页向导

1. 选择 hand-eye/TCP、frame/topic、marker 参数；
2. 只读预检：TF、相机、时间戳、内参、串口/控制器状态；
3. 逐姿态提示、人工确认、采样和质量反馈；
4. 样本覆盖/可观测性进度；
5. 求解方法比较与训练/验证结果；
6. 候选结果、阈值、原始数据和报告下载；
7. 人工确认记录。默认没有“自动部署”按钮。

### Phase 5：仿真和现场验收

- 用离线录制数据和 MuJoCo 验证整个网页流程；
- 真实相机先做只读 preflight 和 plan-only 复核；
- 任何自动引导运动另行设计 motion 接口、限位和安全授权，不纳入首版网页。

## 首版 API 约定

所有请求包含 `session_id`；所有响应包含 `schema_version`、`session_id`、`state` 和
`message`。`capture_sample` 成功时返回 sample index 和质量摘要，失败时返回结构化
`reason_code`，而不是让网页解析异常字符串。`finish_session` 只生成候选报告；只有
操作者明确确认并且报告所有门通过，session 才能进入 `accepted`，这仍不等于自动部署。

## 当前实现与剩余路线

软件范围的主要链路已完成验收，见 `docs/implemented/features/calibration_acceptance.md`。
以下实现检查点同时保留已完成软件项和仍未完成的现场/物理边界；不能把软件验收扩展为真实设备验收。

当前正式会话实现为 `SessionStore`，负责状态转换、幂等、版本、人工确认和持久化。
Phase 0 的未接线 `CalibrationSession` 原型已移除，避免维护两套接受规则。
采集由 `HandeyeCaptureNode`、页面由 Dashboard 持有。当前验收进度见
[当前软件验收边界](../implemented/features/calibration_acceptance.md)，操作步骤见
[当前网页操作命令参考](../reference/commands/calibration_web.md)。

## 实现检查点：离线求解与持久化

新增 `rebotarm_handeye_calibration --input dataset.json --output report.json`。
输入 schema_version=1，metadata 必须包含 base_frame/end_link_frame/camera_frame/marker_frame；
training_samples 和 validation_samples 各至少五项，每项包含唯一 sample_id、
base_to_end、camera_to_marker（translation/rotation_xyzw）。禁止重复机器人姿态。
五种算法按训练集归一化残差选择；留出集始终对训练得到的固定标记参考进行评估，
不参与选方法。报告保留逐方法结果、输入 SHA256、OpenCV 版本和全部残差门。
passed 不等于 accepted；CLI 不接受、不部署配置。输出用 fsync + 原子替换，
数值失败退出码 2，成功为 0。会话恢复、ROS adapter 与网页仍待实现。

## 实现检查点：持久化会话和 ROS 契约

`SessionStore` 在 calibration 层拥有会话 JSON，重启后可按 session_id 恢复；
create 使用客户端 UUID，可安全重试。写操作携带 request_id 和 expected_revision，
拒绝旧版本或同 ID 不同内容，重复请求不会重复追加采样。求解失败不提交状态，
solved 可 reopen 后补样本并使旧报告失效；接受要求通过报告、confirmed=true 和操作者。
终态 accepted/aborted 不允许继续变更。原始数据和报告同文件原子提交。
当前单节点单写者，进程内互斥锁；不得启动多个节点写同一会话目录。

新增 `rebotarm_msgs/CalibrationCommand` 为网页与标定 ROS 节点的契约：
command/session_id/request_id/expected_revision/payload_json，响应返回
success/reason_code/message/session_json。采样命令只允许指定 training 或 validation；
实际观测必须由节点读取 TF 和图像，不能使用网页传来的机器人/标记变换。
服务定义和纯存储已实现；ROS service 回调、异步采样和前端仍待接线。

## 实现检查点：ROS 采集节点

新增 `ros2 run rebotarm_calibration rebotarm_handeye_capture`，服务解析为
`/rebotarm_handeye_capture/command`（CalibrationCommand）。节点只订阅图像、内参、TF，
无控制器 service/action client。多线程 executor 将命令回调和传感器回调分离；
采样要求请求后的新图像，按拍摄时刻查询 base→end，连续至少3帧且覆盖0.4秒的稳定窗口。
不使用旧手眼外参。create/status/preflight/capture/solve/reopen/accept/abort 已接入存储。
相机未启动时限时返回 CAPTURE_TIMEOUT，网页提交 sample 字段会拒绝。
当前仍需正向合成 ROS 图像/TF 集成测试、标定页面和 Dashboard 适配器；
这不代表真实相机采集或重力补偿验收完成。

## 实现检查点：Dashboard 基础接线

`/calibration` 页面由现有 HTTP server 提供，资源随 dashboard 安装；
`/api/calibration/command` 白名单路由通过 CalibrationClient 调用 ROS service，
HTTP 线程等待 future，ROS executor 继续运行。页面包含创建/恢复、预检、训练/验证采样、
求解、重开、人工接受、终止和 JSON 下载；现有 /events 推送标定请求状态。
当前界面为功能初版：尚未完成浏览器验收、正向 ROS 闭环、重力补偿按钮和控制权互斥、
相机预览、详细质量可视化与 launch 组合。不能宣称网页流程完成。

## 实现检查点：正向 ROS 采集与工作台组合

真实 ROS executor/service 集成测试用合成 ArUco 图像、CameraInfo、动态 base→end TF
完成正向采集；验证图像/TF 时间相同、PnP 距离正确和重复请求不追加样本。
测试位于 tests/test_handeye_capture_runtime.py，独立 ROS domain，不启动硬件。
rebotarm_app 增加 calibration:=false（默认关闭）；显式开启时只启动采集节点，
图像/内参话题和会话目录可经 calibration_* 参数传入，不自动启动相机。
控制器当前 stop_gravity_compensation 使用 _gravity_comp_q_last 接管位置速度保持，
不是重新读取当前姿态；网页说明和停止语义必须遵循该事实。重力补偿网页操作仍待接线。

## 实现检查点：覆盖反馈和不确定度

ROS会话响应增加 training/validation 各自 coverage：有效数量、平移与旋转跨度、
相对旋转约束秩/条件数，网页显示对应门是否满足。报告增加64次固定seed的训练集
bootstrap：平移标准差/经验95%分位区间、旋转偏差P95和失败抽样数。
这仅衡量样本重采样敏感度，不估计系统误差，不替代独立验证或物理验收。
数据集哈希改为仅包含规范化输入和元数据，排除既有报告与会话请求历史。

## 实现检查点：显式重力补偿操作

网页新增独立使能、进入重力补偿、退出并保持按钮。使能复用既有arm_enable入口；
重力补偿入口要求confirmed、hardware/web_execute_enabled/execute、新鲜(0.5秒)arm_status、
enabled且无error。start要求IDLE并拒绝活动示教/回放/键盘/网页目标。
Dashboard请求锁串行处理操作，标定重力补偿占用期间阻止新运动操作；停止类入口仍允许。
超时不自动释放占用、不自动失能；stop成功才解除。退出按控制器最后目标角保持。
这仅覆盖本Dashboard命令边界，不替代控制器对其他ROS客户端的仲裁。
待补：完整HTTP→ROS替身控制测试、重启/外部模式切换状态恢复、浏览器交互验收。

## 浏览器检查记录

实际 in-app 浏览器打开 production calibration.html，由隔离本地HTTP测试服务承载。
先验证未连接错误显示，再使用真实 SessionStore/solve_dataset 与合成姿态依次完成
创建、5训练+5验证采样、求解、人工接受、刷新后恢复。最终 accepted=true、deployed=false。
浏览器可见方法HORAUD、64次bootstrap（54有效/10退化），覆盖门通过。
本轮服务不含ROS或硬件连接，验证范围为浏览器→HTTP→真实标定核心；
完整浏览器→Dashboard ROS client→采集服务仍需联调。临时标签与测试服务已关闭。

## HTTP→ROS采集联调

正向采集测试现通过生产HTTP server和CalibrationClient发起命令，真实ROS service
接收并从合成Image/CameraInfo/TF构造样本。验证HTTP200、相同时戳、PnP距离与幂等重试。
Dashboard在arm_status反馈GRAVITY_COMP时建立冲突操作保护，涵盖外部进入模式与节点重启。
超时请求的迟到结果与退出后占用解除仍需完善，不能仅凭IDLE解除未决start的保护。

## 重力补偿超时恢复

GravityRequestTracker 保留超时future，不把HTTP超时当成控制器取消。未完成时拒绝再发
start/stop；收到结果后记录完成时刻。解除标定占用要求操作者明确stop，且收到完成时刻
之后的新鲜、enabled无错IDLE反馈。stop成功也不立即解除占用，避免响应/状态竞态。
测试覆盖超时重复点击不重复下发、迟到响应、旧反馈和过期反馈拒绝解除。
没有自动模式切换、disable或回位。

## 实现检查点：可读结果与来源

页面增加方法比较表（独立验证位置RMS/最大mm、旋转RMS/最大度、失败门），完整报告
折叠展示。每个实时采样记录实际capture_settings、Python/OpenCV/NumPy版本及标定包
各源文件SHA256，支持dirty工作树/安装包追溯，不暴露存储目录。ROS正向测试验证来源写入。

## 实现检查点：只读相机快照

页面“刷新相机快照”经CalibrationCommand preview获取新鲜图像；calibration节点
检查帧龄后限宽640编码JPEG，响应携带原图时间戳、坐标系、获取时帧龄。
页面明确为非实时快照，不将预览替代同步采样。图像仅返回请求方，不写SSE/会话报告。
实际Dashboard→ROS合成测试解码JPEG并检查尺寸与frame；仍需浏览器视觉复查。

## 独立机器人 TF 检查与导出验证

新增 tf_check 命令和网页按钮，独立查询 base→end 最新TF并校验非零时间戳、帧龄和
有限刚体变换；无需看到标记，不读取待标定的相机外参。真实Dashboard→ROS测试验证输出。
页面脚本测试实际点击下载处理器、读取其Blob URL并逐字段比较原始样本/报告/失败标志；
这验证导出内容，不替代浏览器下载目录的人工文件检查。

## TCP 网页分支

metadata.mode=tcp 选择枢轴法；同一物理尖点在训练/验证全程固定。
采集复用请求后新鲜TF稳定窗口，不取图像。训练组联合求偏移和pivot，验证组按训练
pivot算误差而非重新拟合，默认至少5+5样本、旋转跨度30度、满秩与条件数门。
页面支持模式选择和TCP结果摘要。纯数据/持久工作流和20mm验证点偏移拒绝测试通过；
实际ROS TCP采样与浏览器该模式仍待进一步联调。

## 超时及多页面状态隔离

CalibrationClient保留超时future，未完成时后续请求返回PENDING而不排队重复提交；
原请求结束后可刷新会话查询结果。SSE仅广播session_id、command、revision和简短结果，
不重复广播完整样本/原图/报告。页面只显示当前session_id的进度。
测试覆盖超时后不再次下发和完成后恢复请求；不把超时当作取消。

## MuJoCo模型运动学验证

tests/test_calibration_mujoco.py 通过配置的MuJoCo解释器加载当前canonical scene.xml，
随机选取限位内部的20组关节角，mj_forward读取end_link世界位姿；固定已知相机外参和
标记位置，生成14训练+6验证观测。真实求解工作流恢复外参，验证位置最大误差<1e-7m。
不进行物理动作、不接ROS控制器。此证据为运动学/求解组合，不覆盖渲染PnP或网页整体闭环。

## 连续稳定窗口

采样新增maximum_sample_gap_sec=0.2：有效样本不能跨越断流，时戳倒退、跨度过大、
位姿移动或质量拒绝会重置窗口。等待新图像和TF暂不可用可重试，但下一有效样本仍受
时间间隔门约束。样本记录stability_sample_count和stability_duration_sec。
纯窗口边界测试及手眼/TCP真实ROS正向测试均通过。

## 服务器附件导出

新增GET /api/calibration/export?session_id=<32位hex>，Dashboard经只读ROS status读取
持久会话，以Content-Disposition attachment返回UTF-8 JSON，Cache-Control=no-store。
非法ID不进入后端。页面保留当前Blob快照导出，另给服务器数据包链接；实际HTTP→ROS
测试比较附件内容与已采样数据一致。文件所有权仍在calibration，不由Dashboard读取路径。

## 会话采集条件一致性

样本入库前对照首个样本的camera_info/provenance（包括实际阈值和源码指纹）；
训练/验证跨组、重启恢复都不能混用不同配置。缺失来源与实时来源也不得混入同一会话。
条件不一致时拒绝新样本、不改变revision或既有数据，并提示新建会话。

## 完整证据哈希

手眼和TCP报告dataset_sha256统一覆盖schema、metadata、完整训练/验证样本，包括
CameraInfo、原始时戳和采集来源；仅排除会话状态、请求历史与旧报告。求解软件来源另记
solver_provenance。旧版仅规范化变换哈希不再作为完整数据追溯依据；重算旧数据包会产生
按完整证据计算的新哈希。这是审计标识，不是防篡改数字签名。

## 最终命令命名与响应封装

规划中的start_session/capture_sample/finish_session/abort_session/get_status分别落实为
CalibrationCommand的create/capture/solve/abort/status。另有preview、tf_check、preflight、
reopen、accept。统一HTTP响应包含schema_version/session_id/state/revision/message和
success/reason_code；ROS的session_json提供同样的会话身份。失败回读实际持久化状态，
无会话时state=unavailable，不返回内存中的部分修改。create使用客户端session_id作为
幂等身份；写入还使用request_id与expected_revision。preview允许无会话的只读快照。
