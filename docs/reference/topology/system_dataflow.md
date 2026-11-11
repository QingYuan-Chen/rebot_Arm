# 系统节点与数据流

> 状态：REFERENCE；类型：系统数据流图；适用范围：当前 Ubuntu/ROS 2 Jazzy 部署。

## 真机工作台

```text
D405 / RealSense SDK
  -> rebotarm_vision（Image/Depth/CameraInfo）
  -> YOLO detections
  -> local GraspNet candidates
  -> candidate IK / workspace / collision gates
  -> filtered_candidates / filtered_plan / RViz Marker
  <- MoveIt IK、碰撞查询 / TF / 关节状态
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

无硬件键盘、RViz 拖动及网页执行统一使用 MuJoCo continuous 后端，并复用同一 MoveIt 场景与仿真时钟。视觉离线入口使用被动关节状态和 MoveIt 查询，外部入口复用已有环境。各机器人环境应保持唯一关节状态、TF 和时钟来源。
