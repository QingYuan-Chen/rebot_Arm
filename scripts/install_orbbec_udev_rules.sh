#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_root}/.venv-vision/bin/python"

if [[ ! -x "${python_bin}" ]]; then
  echo "Vision environment not found. Run scripts/setup_ubuntu_vision.sh first." >&2
  exit 1
fi

rule_path="$(
  "${python_bin}" - <<'PY'
from importlib.metadata import distribution

print(
    distribution("pyorbbecsdk2").locate_file(
        "pyorbbecsdk/shared/99-obsensor-libusb.rules"
    )
)
PY
)"

if [[ ! -f "${rule_path}" ]]; then
  echo "Orbbec udev rule not found: ${rule_path}" >&2
  exit 1
fi

sudo install -m 0644 "${rule_path}" /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Orbbec udev rule installed. Reconnect the camera before starting ROS."
