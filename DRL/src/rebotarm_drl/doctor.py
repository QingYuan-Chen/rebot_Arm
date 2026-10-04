"""Environment smoke checks only: no task, optimizer or hardware access."""
import importlib.metadata
import json
import platform

from .contracts import sha256
from .runtime import PACKAGE, check_training_host


def doctor(*, gpu=False):
    import mujoco
    import numpy as np

    versions = {name: importlib.metadata.version(name) for name in
                ("mjlab", "mujoco", "mujoco-warp", "warp-lang", "torch", "rsl-rl-lib", "onnx", "onnxruntime")}
    assets = PACKAGE / "assets/rebotarm"
    provenance = json.loads((assets / "provenance.json").read_text())
    for name, digest in provenance["files"].items():
        if sha256(assets / name) != digest:
            raise ValueError(f"model asset checksum mismatch: {name}")
    model = mujoco.MjModel.from_xml_path(str(assets / "robot.xml"))
    data = mujoco.MjData(model)
    for _ in range(10):
        mujoco.mj_step(model, data)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise ValueError("non-finite model smoke check")
    try:
        check_training_host(confirmed=True)
        training = "remote confirmation required"
    except RuntimeError as exc:
        training = str(exc)
    report = {"python": platform.python_version(), "platform": platform.platform(),
              "versions": versions, "assets_verified": len(provenance["files"]),
              "model": {"nq": model.nq, "nv": model.nv, "nu": model.nu, "smoke_steps": 10},
              "training_policy": training, "task": "RebotArm-Reach-Hold-v1"}
    if gpu:
        import torch
        import warp as wp
        import mjlab
        import mujoco_warp
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is unavailable")
        # Small arithmetic check, not a learning update.
        a = torch.arange(256, dtype=torch.float32, device="cuda").reshape(16, 16) / 256
        result = (a @ a.T).cpu()
        torch.testing.assert_close(result, a.cpu() @ a.cpu().T)
        wp.config.quiet = True
        wp.init()
        devices = [str(device) for device in wp.get_cuda_devices()]
        if not devices:
            raise RuntimeError("Warp has no CUDA device")
        with wp.ScopedDevice("cuda:0"):
            gpu_model = mujoco_warp.put_model(model)
            gpu_data = mujoco_warp.put_data(model, data, nworld=2, nconmax=128, njmax=512)
            for _ in range(10):
                mujoco_warp.step(gpu_model, gpu_data)
            wp.synchronize()
            if not np.isfinite(gpu_data.qpos.numpy()).all() or not np.isfinite(gpu_data.qvel.numpy()).all():
                raise ValueError("non-finite GPU physics smoke check")
        report["gpu"] = {"name": torch.cuda.get_device_name(0), "torch_cuda": torch.version.cuda,
                         "capability": list(torch.cuda.get_device_capability(0)),
                         "matrix_check": "passed", "warp_devices": devices,
                         "physics_worlds": 2, "physics_steps": 10}
    return report
