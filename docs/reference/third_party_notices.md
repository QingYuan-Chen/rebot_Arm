# Third-party notices

## Supported local dependencies

### reBotArm control SDK

- Source: <https://github.com/huangbinai/rebotarm_control.git>
- Pinned revision: `6a49302804f25e624995e771acb6d61896d1856d`
- Used from the ignored `third_party/reBotArm_control_py` checkout for vendor
  adapters, kinematics and configuration. Obtain it under its upstream license.

### MotorBridge

- Source: <https://github.com/motorbridge/motorbridge.git>
- Pinned revision: `2b7b350914ace47ba06e85fcad333143de2b057b`
- Local patch: `patches/motorbridge/0001-add-feedback-sequence-api.patch`.
- Required runtime: `0.4.7+rebotarm.1` with `feedback_sequence=true`; unpatched
  PyPI `0.4.7` is not a valid real-controller runtime. The pinned revision and
  patch digest are checked by `scripts/setup_motorbridge_fresh_feedback.py`.

### Vision assets

`scripts/yolo26s-seg.pt` is tracked. Verify its upstream source and license before
redistribution. GraspNet code, checkpoints, PointNet2/KNN extensions and
TensorRT engines are external runtime assets and are not supplied here.

## MuJoCo snapshot attribution

- Source project: `huangbinai/robotarm_ros2`
- Source URL: <https://github.com/huangbinai/robotarm_ros2>
- Pinned revision: `fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`
- Scope: the historical 57-file simulation comparison snapshot, including code,
  configuration, MJCF/XML, meshes, documentation and tests.
- Current source: the upstream-derived implementation remains under
  `src/rebotarm_simulation`; local changes are tracked in this repository.
- This attribution is not a Git remote relationship; the development remote is
  the repository's `origin`.
- The pinned package metadata declares Apache-2.0 but the snapshot has no root
  LICENSE, COPYING or NOTICE. The maintainer recorded upstream-owner permission
  for public redistribution of this exact snapshot on 2026-08-13. This does not
  authorize other revisions or unrelated third-party material.

## Excluded local material

Not part of the repository release: model checkpoints and weights not tracked by
Git, Python virtual environments, build/install/log outputs, ignored SDK and
MuJoCo checkouts, experiment evidence, and historical implementation plans.
Windows/HTTP/MJPEG vision tools and protocol tests have been removed from the
supported source tree. The tracked YOLO model remains subject to its own
provenance and license review.
