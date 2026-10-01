# reBotArm Context

## Current Scope

Supported deployment is Ubuntu 24.04 / ROS 2 Jazzy:

```text
Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/detections -> local GraspNet
```

Retired Windows, HTTP, MJPEG, remote-JSON, and standalone GraspNet service
paths are unsupported. Dashboard HTTP is local UI/API only. For real hardware,
`connected` means feedback communication is established, `enabled` means motors
provide holding torque, and `ready_for_motion` requires explicit Enable. Startup
is disabled by design. Each motor has an independent feedback sequence; values
across joints do not need to be equal.

## Terms

### Hardware Controller

The ROS2 layer that owns real motor communication and last-line execution
safety. In this repository, this is `rebotarmcontroller`.

### Point-to-Point Execution

Moving the robot from its current state to one target state. It is a motion
problem, not a teach replay problem. The expected output is a valid trajectory
with a safe final state.

### Teach Replay

Reproducing a recorded hand-guided trajectory. Raw teach data is input data; the
robot should execute a prepared and validated trajectory.

### Prepared Trajectory

The filtered, resampled, retimed, and checked trajectory derived from a teach
record. Real replay should prefer this prepared trajectory over raw samples.

### Operator Interaction

Human command input through web, keyboard, RViz marker, or gripper controls.
Operator interaction translates intent into ROS commands but does not own
hardware internals or teach replay algorithms.

### Dashboard

The web-facing UI and status API. It displays state and calls services but does
not own motion planning, teach replay algorithms, or motor SDK calls.

## Current Runtime Contract

The maintained vision path is native Ubuntu only:

```text
Gemini 2 SDK -> YOLO -> ROS Image/CameraInfo/detections
-> local GraspNet -> candidate IK/collision gates -> MoveIt
-> MuJoCo or explicitly enabled hardware
```

Windows/HTTP/MJPEG/remote JSON and standalone GraspNet service paths are retired.
Dashboard HTTP is local UI, not vision transport. `connected` means feedback
communication, `enabled` means holding torque, and `ready_for_motion` requires
both plus a healthy lifecycle. Motor feedback sequence counters are independent.
