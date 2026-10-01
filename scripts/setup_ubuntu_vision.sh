#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_dir="${repo_root}/.venv-vision"
python_bin="${venv_dir}/bin/python"
export PYTHONNOUSERSITE=1

python3 -m venv --system-site-packages "${venv_dir}"
env PYTHONPATH= "${python_bin}" -m pip install --upgrade "pip<27" "setuptools<80" wheel
env PYTHONPATH= "${python_bin}" -m pip install \
  torch==2.11.0+cu128 torchvision==0.26.0+cu128 \
  --index-url https://download.pytorch.org/whl/cu128
env PYTHONPATH= python3 "${repo_root}/scripts/install_python_dependencies.py" vision --python "${python_bin}"
env PYTHONPATH= python3 "${repo_root}/scripts/install_python_dependencies.py" tensorrt --python "${python_bin}" --no-deps

set +u
source /opt/ros/jazzy/setup.bash
set -u

"${python_bin}" - <<'PY'
import cv2
import numpy
import pyorbbecsdk
import tensorrt
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO

print(f"NumPy: {numpy.__version__}")
print(f"OpenCV: {cv2.__version__}")
print(f"PyTorch: {torch.__version__}")
print(f"TensorRT: {tensorrt.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device: {torch.cuda.get_device_name(0)}")
print("ROS cv_bridge, Orbbec SDK and Ultralytics imports: OK")
PY

echo
echo "Environment ready."
echo "Build the ROS workspace separately with system Python; see docs/setup/ubuntu_ros2_jazzy.md."
echo "No models are downloaded or exported. Select a model using yolo_model_path."
echo "Install the Orbbec udev rule once after connecting the camera:"
echo "  ${repo_root}/scripts/install_orbbec_udev_rules.sh"
echo "Start Ubuntu-native vision with:"
echo "  ${repo_root}/scripts/run_ubuntu_vision.sh"
