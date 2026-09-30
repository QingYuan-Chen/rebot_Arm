# Repository Instructions for Coding Agents

## Scope

The supported deployment is Ubuntu 24.04 / ROS 2 Jazzy:

```text
Gemini 2 -> YOLO -> ROS RGB-D/CameraInfo/detections -> local GraspNet
```

Do not restore retired Windows, HTTP, MJPEG, remote-JSON, or standalone
GraspNet-service vision paths. Dashboard HTTP is local UI/API only.

Before changing code, read `docs/implemented/architecture.md`, `CONTEXT.md`,
`tests/test_package_layering.py`, `Agent/README.md`, `Agent/MEMORY.md`,
`Agent/PROJECT_STATUS.md`, and `Agent/STATE.json`.

## Hard Rules

- Never automatically disable a healthy real arm away from the verified baseline after a recoverable task failure. Stop and keep enabled hold, or return under guard to the captured baseline, verify it, then disable. Protective disable away from baseline is reserved for controller/motor error, communication loss, stale feedback, or explicit emergency stop.
- Real hardware starts disabled. Require fresh feedback, site-safety checks, and explicit `/rebotarm/enable`; do not restore automatic enabling.
- MoveIt Execute requires `moveit_simple_controller_manager`; Plan success alone is not Execute evidence.
- Keep hardware access in `rebotarmcontroller`.
- Keep trajectory generation, MoveIt adapters, retiming, collision checks, and runtime guards in `rebotarm_motion`.
- Keep teach recording, prepared trajectories, and replay orchestration in `rebotarm_teach`.
- Keep keyboard/Web/RViz operator adapters in `rebotarm_teleop`.
- Keep web UI, HTTP routes, SSE, and dashboard assets in `rebotarm_dashboard`.
- Keep URDF/SRDF/planning configuration in `rebotarm_moveit_config`.
- Keep perception, depth, detection, and grasp candidates in `rebotarm_vision`.
- Keep MuJoCo models, simulated execution, and physics metrics in `rebotarm_simulation`.
- Keep launch composition and backend selection in `rebotarm_bringup`.
- Keep hand-eye, TCP, and TF validation in `rebotarm_calibration`.

## Agent State

`Agent/` is the live project-state source. At task start run:

```bash
python3 Agent/update_state.py --event start --actor <agent> --note "<task>"
```

Update `Agent/MEMORY.md` when facts or blockers change. Update `STATE.json` only
through `Agent/update_state.py`; do not rewrite activity history or mark hardware
acceptance from software tests alone.

## Required Checks

```bash
python3 -m pytest tests/test_package_layering.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion -q
python3 -m compileall src/rebotarm_bringup/launch -q
```

If a feature crosses ownership boundaries, split it into interfaces owned by the
responsible packages instead of copying implementation across layers.
