from __future__ import annotations

import importlib
import os
from pathlib import Path


def main() -> int:
    print(f"python={os.sys.executable}")
    for module in ("numpy", "scipy", "cv2", "open3d", "PIL", "transforms3d", "torch"):
        try:
            imported = importlib.import_module(module)
            print(f"{module}={getattr(imported, '__version__', 'present')}")
        except Exception as exc:
            print(f"{module}=MISSING {type(exc).__name__}: {exc}")
            return 1

    import torch

    print(f"cuda_available={torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"cuda_device={torch.cuda.get_device_name(0)}")

    model_root_raw = os.environ.get("GRASPNET_MODEL_ROOT", "").strip()
    checkpoint_raw = os.environ.get("GRASPNET_CHECKPOINT_PATH", "").strip()
    model_root = Path(model_root_raw).expanduser() if model_root_raw else None
    checkpoint = Path(checkpoint_raw).expanduser() if checkpoint_raw else None
    print(f"model_root={'set' if model_root is not None else 'unset'}")
    print(f"checkpoint={'set' if checkpoint is not None else 'unset'}")
    if model_root is not None:
        print(f"model_root_exists={model_root.is_dir()}")
    if checkpoint is not None:
        print(f"checkpoint_exists={checkpoint.is_file()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
