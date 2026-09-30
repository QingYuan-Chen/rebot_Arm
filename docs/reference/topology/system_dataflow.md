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

仿真后端不得启动 `rebotarmcontroller`，也不得打开真实串口。视觉 `plan_only` 可使用假关节状态，但不能作为真实手眼或实物定位证据。
