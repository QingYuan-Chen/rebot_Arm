"""将配置文件先合并为普通参数字典，保证显式覆盖的优先级。"""

from pathlib import Path

import yaml


def read_section(path, section, field):
    path = Path(path).expanduser()
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        values = payload[section] if field is None else payload[section][field]
        if not isinstance(values, dict):
            raise TypeError("configuration section must be a mapping")
        return dict(values)
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise RuntimeError(f"Invalid profile {path} [{section}]: {exc}") from exc


def merged_parameters(context, node_name, profiles, overrides):
    """Profile 顺序合并后，显式 launch 覆盖最后写入；不依赖 ROS YAML 特异性。"""
    values = {}
    for profile in profiles:
        path = profile.perform(context)
        values.update(read_section(path, node_name, "ros__parameters"))
    values.update(overrides)
    return values
