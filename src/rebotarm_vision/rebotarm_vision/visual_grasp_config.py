"""Immutable snapshot of executor parameters used by orchestration."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from .utils.parameter_validation import validate_visual_parameters


@dataclass(frozen=True)
class VisualGraspConfig:
    values: Mapping[str, Any]

    def get(self, name: str, default: Any = None) -> Any:
        return self.values.get(name, default)

    @property
    def input_topic(self) -> str:
        return str(self.get("input_topic", "/grasp/filtered_plan"))

    @property
    def max_plan_age_sec(self) -> float:
        return float(self.get("max_plan_age_sec", 1.0))

    @property
    def auto_retry_enabled(self) -> bool:
        return bool(self.get("auto_retry_enabled", False))

    @property
    def safe_retreat_before_retry(self) -> bool:
        return bool(self.get("safe_retreat_before_retry", True))

    @classmethod
    def from_node(cls, node) -> "VisualGraspConfig":
        try:
            names = node.list_parameters([], depth=10).names
        except AttributeError:
            names = ["input_topic", "max_plan_age_sec", "auto_retry_enabled", "safe_retreat_before_retry"]
        values = {name: (tuple(value) if isinstance(value, (list, tuple)) else deepcopy(value)) for name, value in ((name, node.get_parameter(name).value) for name in names)}
        validate_visual_parameters(values)
        return cls(MappingProxyType(values))
