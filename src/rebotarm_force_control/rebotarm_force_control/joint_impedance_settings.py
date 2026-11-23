"""六轴关节阻抗安全准入上限配置与 YAML 加载校验.

职责与位置
    加载并核对后端无关的关节阻抗准入安全边界(最大允许刚度/阻尼,力矩变化率与超时范围);
    独立于纯控制律公式与运行期状态机.

对外接口
    JointImpedanceSettings: 保存边界值提供 from_yaml() 与 validate_command().

安全与并发约束
    严格拒绝未知 YAML 键与超 MIT 协议范围(Kp<=500, Kd<=5)的上界配置;
    不提供任何默认刚度/阻尼回退,调用方每次必须显式提供完整六轴增益.

文件结构(按定义顺序)
    JointImpedanceSettings.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Mapping

import yaml

from .contracts import JOINT_NAMES, finite_scalar, six_vector
from .joint_impedance import JointImpedanceCommand
from .mit_command import PROTOCOL_KD_MAX, PROTOCOL_KP_MAX


@dataclass(frozen=True)
class JointImpedanceSettings:
    """关节阻抗准入安全边界(不含默认刚度/阻尼);构造与加载即校验."""

    max_stiffness_nm_rad: tuple[float, ...] = (PROTOCOL_KP_MAX,) * 6
    max_damping_nm_s_rad: tuple[float, ...] = (PROTOCOL_KD_MAX,) * 6
    torque_rate_limits_nm_s: tuple[float, ...] = (108.0, 108.0, 108.0, 28.0, 28.0, 28.0)
    min_timeout_s: float = 0.02
    max_timeout_s: float = 10.0

    def __post_init__(self) -> None:
        stiffness = six_vector(self.max_stiffness_nm_rad, "max_stiffness_nm_rad")
        damping = six_vector(self.max_damping_nm_s_rad, "max_damping_nm_s_rad")
        rates = six_vector(self.torque_rate_limits_nm_s, "torque_rate_limits_nm_s")
        if (stiffness <= 0.0).any() or (stiffness > PROTOCOL_KP_MAX).any():
            raise ValueError(f"max_stiffness_nm_rad must be within (0, {PROTOCOL_KP_MAX}]")
        if (damping <= 0.0).any() or (damping > PROTOCOL_KD_MAX).any():
            raise ValueError(f"max_damping_nm_s_rad must be within (0, {PROTOCOL_KD_MAX}]")
        if (rates <= 0.0).any():
            raise ValueError("torque_rate_limits_nm_s must be positive")
        min_timeout = finite_scalar(self.min_timeout_s, "min_timeout_s")
        max_timeout = finite_scalar(self.max_timeout_s, "max_timeout_s")
        if min_timeout > max_timeout:
            raise ValueError("min_timeout_s must not exceed max_timeout_s")
        object.__setattr__(self, "max_stiffness_nm_rad", tuple(float(v) for v in stiffness))
        object.__setattr__(self, "max_damping_nm_s_rad", tuple(float(v) for v in damping))
        object.__setattr__(self, "torque_rate_limits_nm_s", tuple(float(v) for v in rates))
        object.__setattr__(self, "min_timeout_s", min_timeout)
        object.__setattr__(self, "max_timeout_s", max_timeout)

    @classmethod
    def from_yaml(cls, path: str | Path) -> JointImpedanceSettings:
        """读取 YAML 顶层边界字段;拒绝未知键,避免配置拼写错误被静默忽略."""
        values = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(values, Mapping):
            raise ValueError("joint impedance settings must be a mapping")
        unknown = set(values) - {item.name for item in fields(cls)}
        if unknown:
            raise ValueError(f"unknown joint impedance settings: {sorted(unknown)}")
        return cls(**values)

    def validate_command(self, command: JointImpedanceCommand, timeout_s: float) -> None:
        """核对命令增益及看门狗期限不超过安全准入边界."""
        if not isinstance(command, JointImpedanceCommand):
            raise TypeError("command must be JointImpedanceCommand")
        timeout = finite_scalar(timeout_s, "timeout_s")
        if not self.min_timeout_s <= timeout <= self.max_timeout_s:
            raise ValueError(
                f"timeout_s must be within [{self.min_timeout_s}, {self.max_timeout_s}]"
            )
        for index, name in enumerate(JOINT_NAMES):
            if command.stiffness[index] > self.max_stiffness_nm_rad[index]:
                raise ValueError(
                    f"stiffness exceeds limit for {name}: "
                    f"{command.stiffness[index]} > {self.max_stiffness_nm_rad[index]}"
                )
            if command.damping[index] > self.max_damping_nm_s_rad[index]:
                raise ValueError(
                    f"damping exceeds limit for {name}: "
                    f"{command.damping[index]} > {self.max_damping_nm_s_rad[index]}"
                )
