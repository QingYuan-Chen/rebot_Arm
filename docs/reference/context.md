# reBotArm Context

## Current Scope

Supported deployment is Ubuntu 24.04 / ROS 2 Jazzy:

```text
D405 -> YOLO -> ROS RGB-D/CameraInfo/detections -> local GraspNet
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
D405 / RealSense SDK -> YOLO -> ROS Image/CameraInfo/detections
-> local GraspNet -> candidate IK/collision queries -> filtered results / RViz markers
```

Windows/HTTP/MJPEG/remote JSON and standalone GraspNet service paths are retired.
Dashboard HTTP is local UI, not vision transport. `connected` means feedback
communication, `enabled` means holding torque, and `ready_for_motion` requires
both plus a healthy lifecycle. Motor feedback sequence counters are independent.

`rebotarm_preview` owns lightweight trajectory interpolation without physics.
MuJoCo remains a separate backend owned by `rebotarm_simulation`; neither backend
opens real hardware. The teach package owns record preparation and replay policy,
while MuJoCo teach-preview execution and Viewer lifecycle belong to simulation.

FollowJointTrajectory accepts exactly the six arm joints (rad); finger and seven-axis
goals are rejected. Prepared web operations use ExecuteTeleopMotion for the arm curve
and an optional independent gripper opening. Motion owns arm generation, retiming and
full-scene collision checks; backends own reservations and the overall result, waiting
for both independent executors. Web checks use closed fingers only in a calculation
copy; other arm requests freeze fresh actual opening. Feedback and the public scene
always remain real, with separate arm/gripper source timestamps. MoveIt arm and gripper
execute independently; teach replay is six-axis. Real
hardware stays disabled by default and uses continuous; legacy remains explicitly selectable.
See [P005 coordination](../implemented/features/p005_prepared_trajectory.md#修订协同接口).

The visual entrypoint owns perception, candidate filtering and display. Offline mode
uses passive robot state and MoveIt queries; external mode reuses an existing robot
environment. D405 hand-eye calibration is supplied explicitly; the default camera-only
launch leaves the arm-to-camera TF unpublished. The old Gemini2 calibration is retained
for provenance. Vision never submits motion targets or accesses motor SDKs directly.
