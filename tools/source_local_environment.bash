#!/usr/bin/env bash
# Local deployment environment only; never starts nodes or accesses hardware.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Use: source tools/source_local_environment.bash\n' >&2
    exit 1
fi

_rebotarm_local_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
_rebotarm_local_ros="${_rebotarm_local_root}/build_local_dependencies/root/opt/ros/jazzy"
if [[ -d "${_rebotarm_local_ros}/lib/python3.12/site-packages" ]]; then
    export PYTHONPATH="${_rebotarm_local_ros}/lib/python3.12/site-packages:${PYTHONPATH:-}"
    export LD_LIBRARY_PATH="${_rebotarm_local_ros}/lib:${LD_LIBRARY_PATH:-}"
    export AMENT_PREFIX_PATH="${_rebotarm_local_ros}:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="${_rebotarm_local_ros}:${CMAKE_PREFIX_PATH:-}"
fi
if [[ -f "${_rebotarm_local_root}/install/local_setup.bash" ]]; then
    source "${_rebotarm_local_root}/install/local_setup.bash"
fi
export REBOTARM_VISION_PYTHON="${_rebotarm_local_root}/.venv-vision/bin/python"
export GRASPNET_PYTHON="${_rebotarm_local_root}/.venv-graspnet/bin/python"
export GRASPNET_MODEL_ROOT="${GRASPNET_MODEL_ROOT:-${_rebotarm_local_root}/third_party/graspnet-baseline}"
export GRASPNET_CHECKPOINT_PATH="${GRASPNET_CHECKPOINT_PATH:-${_rebotarm_local_root}/.local-models/checkpoints/checkpoint-rs.tar}"
export REBOTARM_MUJOCO_PYTHON="${_rebotarm_local_root}/third_party/rebotarm_mujoco_venv/bin/python"
unset _rebotarm_local_root _rebotarm_local_ros
