"""Paired held-out evaluation in the same MJLab physics for IK and ONNX policies."""
import json
from pathlib import Path

import mujoco
import numpy as np

from .reach_model import (DELTA, DT, EPISODE_STEPS, HOLD_STEPS, HIGH, LOW, MARGIN,
    deployment_contract, load_catalogue, model_spec, protocol, valid_path, valid_pose)


def solve_ik(model, data, start, target):
    """Position-only damped least squares, independently of the FK witness."""
    q = np.array(start, dtype=float)
    jac = np.zeros((3, model.nv))
    site = model.site("ee_site").id
    for _ in range(250):
        data.qpos[:] = q
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        error = np.asarray(target)-data.site_xpos[site]
        if np.linalg.norm(error) < .001:
            if valid_path(model, data, np.array(start), q):
                return q
            raise ValueError("IK path violates geometric precheck")
        mujoco.mj_jacSite(model, data, jac, None, site)
        dq = jac.T @ np.linalg.solve(jac@jac.T + .0001*np.eye(3), error)
        q = np.clip(q + np.clip(dq, -.05, .05), LOW+MARGIN, HIGH-MARGIN)
    raise ValueError("IK did not converge")


def smooth_command(start, end, time_s, duration):
    u = min(max(time_s/duration, 0.), 1.)
    return start+(end-start)*(10*u**3-15*u**4+6*u**5)


def evaluate(output, *, policy=None, device="cuda:0"):
    output = Path(output)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite evaluation: {output}")
    output.mkdir(parents=True)
    from .reach_workflow import make_env
    env = make_env(num_envs=16, device=device, auto_reset=False)
    import torch
    from .reach import state
    from .contracts import sha256
    bank = load_catalogue()["eval"]
    identity = {"controller": "IK + quintic trajectory + bounded position command"}
    try:
        if policy:
            import onnxruntime as ort
            from .export import verify_export
            verified = verify_export(policy, deployment_contract())
            session = ort.InferenceSession(str(Path(policy)/"policy.onnx"), providers=["CPUExecutionProvider"])
            identity = {"controller": "ONNX", "model_sha256": verified["model_sha256"]}
        model = model_spec().compile()
        data = mujoco.MjData(model)
        starts = np.array([r["start_rad"] for r in bank])
        goals = starts.copy()
        planning_failed = np.zeros(16, dtype=bool)
        if not policy:
            for i, row in enumerate(bank):
                try:
                    goals[i] = solve_ik(model, data, starts[i], row["target_m"])
                except ValueError:
                    planning_failed[i] = True
        duration = np.maximum(3., 1.875*np.max(np.abs(goals-starts), axis=1)/.4)
        obs, _ = env.reset()
        s = state(env)
        initial_tcp = (env.scene["robot"].data.site_pos_w[:, s.site]-env.scene.env_origins).cpu().numpy()
        direction = np.array([r["target_m"] for r in bank])-initial_tcp
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        active = np.ones(16, dtype=bool)
        traces = [{"qpos": [starts[i].copy()], "time_s": [0.], "error": [],
                   "tcp": [], "speed": [], "action": []} for i in range(16)]
        rows = [None]*16
        for step in range(1, EPISODE_STEPS+1):
            if policy:
                action = session.run(None, {"observations": obs["actor"].cpu().numpy()})[0]
            else:
                desired = np.stack([smooth_command(starts[i], goals[i], step*DT, duration[i]) for i in range(16)])
                action = np.clip((desired-s.command.cpu().numpy())/DELTA, -1., 1.)
            action[~active] = 0
            obs, _, terminated, truncated, _ = env.step(torch.as_tensor(action, dtype=torch.float32, device=device))
            done = (terminated | truncated).cpu().numpy()
            q = env.scene["robot"].data.joint_pos.cpu().numpy()
            tcp, errors, speeds = s.tcp.cpu().numpy(), s.error.cpu().numpy(), s.speed.cpu().numpy()
            unsafe, success = s.unsafe.cpu().numpy(), s.success.cpu().numpy()
            clipped = s.raw_action.cpu().numpy()
            for i in np.flatnonzero(active):
                tr = traces[i]
                tr["qpos"].append(q[i].copy())
                tr["time_s"].append(step*DT)
                for key, val in (("error", errors[i]), ("speed", speeds[i]), ("tcp", tcp[i].copy()), ("action", clipped[i].copy())):
                    tr[key].append(val)
                if done[i]:
                    err = np.array(tr["error"])
                    reached = np.flatnonzero(err <= .01)
                    actions = np.vstack([np.zeros(6), tr["action"]])
                    metrics = {"success": int(success[i]), "violation": int(unsafe[i]),
                        "timeout": int(truncated[i] and not terminated[i]), "planning_failure": int(planning_failed[i]),
                        "final_error_m": float(err[-1]), "mean_error_m": float(err.mean()),
                        "last_window_rms_error_m": float(np.sqrt(np.mean(err[-HOLD_STEPS:]**2))),
                        "reach_time_s": float((reached[0]+1)*DT if reached.size else EPISODE_STEPS*DT),
                        "success_time_s": float(step*DT if success[i] else EPISODE_STEPS*DT),
                        "overshoot_m": float(max(0., ((np.array(tr["tcp"])-bank[i]["target_m"])@direction[i]).max())),
                        "action_rate_rms_s_inv": float(np.sqrt(np.mean((np.diff(actions, axis=0)/DT)**2))),
                        "peak_tcp_speed_m_s": float(max(tr["speed"]))}
                    rows[i] = {"id": bank[i]["id"], "metrics": metrics}
                    active[i] = False
            if not active.any():
                break
            if done.any():
                obs, _ = env.reset(env_ids=torch.as_tensor(np.flatnonzero(done), device=device))
        if any(row is None for row in rows):
            raise RuntimeError("evaluation ended without accounting for every trial")
        recordings = {}
        for row, tr in zip(bank, traces, strict=True):
            path = output/(row["id"]+".npz")
            np.savez(path, qpos=tr["qpos"], time_s=tr["time_s"], target_m=row["target_m"])
            recordings[path.name] = sha256(path)
        # Export exact assembled scene: includes task finger/floor/marker adaptation.
        env.scene.write(output/"model")
        report = {"protocol": protocol(), "candidate": identity, "trials": rows,
                  "recordings": recordings, "training_updates": 0}
        (output/"report.json").write_text(json.dumps(report, indent=2)+"\n")
        return {"report": str(output/"report.json"), "successes": sum(r["metrics"]["success"] for r in rows),
                "trials": len(rows), "violations": sum(r["metrics"]["violation"] for r in rows),
                "planning_failures": int(planning_failed.sum()), "training_updates": 0}
    finally:
        env.close()
