# 系统节点与数据流

> 状态：REFERENCE；类型：系统数据流图；适用范围：当前 Ubuntu/ROS 2 Jazzy 部署。

## 真机工作台

```text
Gemini 2
  -> rebotarm_vision（Image/Depth/CameraInfo）
  -> YOLO detections
  -> local GraspNet candidates
  -> candidate IK / workspace / collision gates
  -> rebotarm_motion / MoveIt
  -> FollowJointTrajectory
  -> rebotarmcontroller
  -> real arm + joint_states / arm_status
```

```text
Browser Dashboard
  -> rebotarm_dashboard HTTP/SSE
  -> rebotarm_teleop command adapters
  -> rebotarm_motion / rebotarm_teach
  -> controller services/actions or MoveIt
```

## 示教数据流

```text
gravity compensation feedback
  -> TeachRecorderNode
  -> raw JSONL
  -> prepare/resample/retime
  -> start alignment + collision precheck
  -> dry-run token
  -> FollowJointTrajectory replay
  -> runtime tracking gate
```

## 仿真替代链路

```text
MoveIt / RViz
  -> simulated FollowJointTrajectory
  -> rebotarm_simulation MuJoCo backend
  -> simulated joint states / metrics
```

视觉总入口、无硬件键盘和 RViz 拖动使用 `rebotarm_preview` 提供轻量轨迹插值及关节状态；
该后端不加载 MuJoCo，不验证动力学或接触。MuJoCo 的独立入口继续提供物理仿真。
两种后端都不得启动 `rebotarmcontroller` 或打开真实串口，且同一命名空间只能有一个
`FollowJointTrajectory` 服务端。预演姿态不能作为真实手眼或实物定位证据。

通用视觉执行器启动后等待显式 execute。stop 会排空旧请求并等待运动层停止确认，
确认失败时继续阻断执行；停止静止判定不等于已回安全基线或允许失能。
