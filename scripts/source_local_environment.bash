#!/usr/bin/env bash
# 在当前 shell 加载 ROS 与本地安装覆盖层,仅设置环境;节点由后续命令显式启动.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf 'Use: source scripts/source_local_environment.bash\n' >&2
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
# 各视觉进程通过 launch prefix 选择解释器;此处保留终端当前 Python,供 ROS 构建使用.
export REBOTARM_VISION_PYTHON="${_rebotarm_local_root}/.venv-vision/bin/python"
export GRASPNET_PYTHON="${_rebotarm_local_root}/.venv-graspnet/bin/python"
export GRASPNET_MODEL_ROOT="${GRASPNET_MODEL_ROOT:-${_rebotarm_local_root}/third_party/graspnet-baseline}"
export GRASPNET_CHECKPOINT_PATH="${GRASPNET_CHECKPOINT_PATH:-${_rebotarm_local_root}/.local-models/checkpoints/checkpoint-rs.tar}"
unset _rebotarm_local_root _rebotarm_local_ros
