"""Kinematic playback of supplied simulation recordings; no hardware access."""
import math
import time
from pathlib import Path


def validate_recording(qpos, time_s, *, nq):
    import numpy as np
    if qpos.ndim != 2 or qpos.shape[1] != nq or qpos.shape[0] < 2 or time_s.shape != (qpos.shape[0],):
        raise ValueError("recording shape differs from model/time dimensions")
    if not np.isfinite(qpos).all() or not np.isfinite(time_s).all():
        raise ValueError("recording contains non-finite values")
    if time_s[0] < 0 or np.any(np.diff(time_s) <= 0):
        raise ValueError("recording time must be nonnegative and strictly increasing")


def replay(model_path, recording, *, speed=1.0, headless=False):
    import mujoco
    import numpy as np
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("replay speed must be positive and finite")
    model = mujoco.MjModel.from_xml_path(str(Path(model_path).resolve()))
    data = mujoco.MjData(model)
    with np.load(recording, allow_pickle=False) as saved:
        qpos, stamps = saved["qpos"], saved["time_s"]
        if "target_m" in saved:
            target = saved["target_m"]
            if target.shape != (3,) or not np.isfinite(target).all():
                raise ValueError("invalid recorded target_m")
            marker = model.body("reach_goal").mocapid[0]
            if marker < 0:
                raise ValueError("recorded target requires reach_goal mocap marker")
            data.mocap_pos[marker] = target
    validate_recording(qpos, stamps, nq=model.nq)
    if headless:
        for pose in qpos:
            data.qpos[:] = pose
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.xpos).all():
                raise ValueError("replay produced non-finite kinematics")
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(model, data) as viewer:
            start = time.monotonic()
            for pose, stamp in zip(qpos, stamps, strict=True):
                if not viewer.is_running():
                    break
                target_time = start + (stamp - stamps[0]) / speed
                while viewer.is_running() and time.monotonic() < target_time:
                    time.sleep(min(0.01, max(0.0, target_time - time.monotonic())))
                with viewer.lock():
                    data.qpos[:] = pose
                    mujoco.mj_forward(model, data)
                viewer.sync()
    return {"frames": len(qpos), "duration_s": float(stamps[-1] - stamps[0]),
            "mode": "recorded_kinematic_playback", "headless": headless}
