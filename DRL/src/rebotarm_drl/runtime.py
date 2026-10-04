"""Local execution policy and caches, configured before importing GPU libraries."""
import hashlib
import json
import os
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent
ROOT = PACKAGE.parents[1]
POLICY = PACKAGE / "configs/execution_policy.json"


def machine_id():
    value = Path("/etc/machine-id").read_text().strip()
    if not value:
        raise RuntimeError("empty machine-id; cannot establish training host")
    return hashlib.sha256(value.encode()).hexdigest()


def check_training_host(policy=POLICY, *, host_id=None, confirmed=False):
    config = json.loads(Path(policy).read_text())
    hosts = config.get("evaluation_host_ids")
    if not isinstance(hosts, list) or not hosts or not all(isinstance(x, str) and x for x in hosts):
        raise ValueError("missing evaluation host policy")
    if (host_id or machine_id()) in hosts:
        raise RuntimeError("This host is export/evaluation only; training is forbidden")
    if not confirmed:
        raise RuntimeError("remote training requires --confirm-remote-training")


def configure_runtime():
    cache = ROOT / ".cache"
    for name, suffix in {
        "WARP_CACHE_PATH": "warp", "TORCH_HOME": "torch", "TRITON_CACHE_DIR": "triton",
        "XDG_CACHE_HOME": "xdg", "MPLCONFIGDIR": "matplotlib", "CUDA_CACHE_PATH": "cuda",
    }.items():
        path = cache / suffix
        path.mkdir(parents=True, exist_ok=True)
        os.environ[name] = str(path)
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("WANDB_MODE", "disabled")
    os.environ.setdefault("OMP_NUM_THREADS", "4")
