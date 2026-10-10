#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/.venv-graspnet"
python_bin="${venv_dir}/bin/python"
export PYTHONNOUSERSITE=1

if [[ ! -x "${python_bin}" ]]; then
  python3 -m venv "${venv_dir}"
fi

env PYTHONPATH= "${python_bin}" -m pip install --upgrade "pip<27" "setuptools<80" wheel
env PYTHONPATH= "${python_bin}" -m pip install \
  torch==2.11.0+cu128 torchvision==0.26.0+cu128 \
  --index-url https://download.pytorch.org/whl/cu128
env PYTHONPATH= python3 "${repo_root}/scripts/install_python_dependencies.py" graspnet --python "${python_bin}" \
  --extra-index-url https://download.pytorch.org/whl/cu128

env PYTHONPATH= "${python_bin}" - <<'PY'
import importlib
import torch

required = ("numpy", "scipy", "cv2", "open3d", "PIL", "transforms3d")
for name in required:
    importlib.import_module(name)
print(f"PyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")
print("GraspNet environment imports: OK")
PY
env PYTHONPATH= "${python_bin}" -m pip check

python3 "${repo_root}/scripts/patch_graspnet_deprecations.py" \
  --root "${repo_root}/third_party/graspnet-baseline"

echo
echo "GraspNet environment ready: ${venv_dir}"
echo "Model root/checkpoint are intentionally not downloaded by this script."
echo "Set GRASPNET_MODEL_ROOT and GRASPNET_CHECKPOINT_PATH only after provenance/license review."
