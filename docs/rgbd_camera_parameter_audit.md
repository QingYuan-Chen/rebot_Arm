# RGB-D / CameraInfo and MuJoCo intrinsics audit

2026-09-26. Software and simulated output audit; Gemini 2 was absent from USB
inventory and no real vision node was running. No actual device calibration or
real image/CameraInfo consistency acceptance is claimed.

## Production data path

`visual_grasp_system.launch.py` → vision launch → `camera_ubuntu.yaml`:
color 640x480 MJPG @30 fps; native depth 640x400 @30 fps; hardware D2C enabled.
The ROS loop is capped at 15 Hz, distinct from sensor stream rate. Production
YOLO receives the color BGR image; its internal inference preprocessing is not
a change to the published camera stream calibration. GraspNet subscribes to
`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/camera_info`.
The depth stream presented to it is aligned-to-color (expected 640x480), not
native depth coordinates. Profile selection and actual image shape still require
runtime verification on the connected camera.

Gemini2Driver reads fx/fy/cx/cy and distortion from the selected SDK profiles;
D2C depth CameraInfo copies color intrinsics. Native-depth intrinsics are stored
separately in driver metadata. ROS distortion is rational_polynomial with
[k1,k2,p1,p2,k3,k4,k5,k6]. The message converter preserves these fields.

Important gaps identified, not silently changed in this audit:
- VisionNode currently accepts SDK info without matching dimensions against the
  actual image; missing metadata has an approximate fallback, not calibration.
- GraspNet's CameraInfo callback retains only fx/fy/cx/cy, not D, dimensions or
  CameraInfo timestamps. Point-cloud construction is a pinhole projection.
- D2C alignment alone does not establish color lens rectification. For nonzero
  real D, raw-image distortion handling must be evaluated before claiming
  geometrically accurate pinhole backprojection.

## Simulation scope (2026-09-27)

At the user's request, virtual RGB-D rendering, CameraInfo, simulated optical TF,
annotation publishers and camera launch parameters were removed. Current training
and simulation work uses numerical state observations. Camera/bracket physical
payload meshes and masses remain. Real Gemini 2 / YOLO / GraspNet are unchanged.
Previous camera implementations and tests can be recovered from Git commit
2f706be9e6ba1ec4d75a4b929d8839faccd9d7a7 if visual simulation is needed later.

## Real-camera read-only metadata check

With an already running real camera publisher in the same ROS domain:

```bash
cd /home/huangbin/robotarm_ros2
source tools/source_local_environment.bash
python3 tools/check_rgbd_camera_info.py --timeout 15 --frames 5
```

The tool only subscribes. Passing proves message metadata consistency, not optical
calibration accuracy or that GraspNet compensates distortion.
