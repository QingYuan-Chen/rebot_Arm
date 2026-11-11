"""回放设置提供者:把节点参数与本次请求 payload 合并成规范化回放设置.

对外只暴露两个查询:

- ``auto_align_duration()``:仅按起始误差推算对齐时长;
- ``from_payload()``:产出最终设置字典,键固定为
  ``replay_speed`` / ``align_duration`` / ``align_steps`` / ``final_hold_sec``,
  取值已按实现内的上下限夹紧,调用方无需再做校验.

本提供者不读文件,不访问硬件,也不决定是否允许回放.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .teach_recording import compute_auto_align_duration, normalize_teach_replay_settings


@dataclass(frozen=True)
class TeachReplaySettingsProvider:
    """回放设置默认值与自动对齐参数(全部来自节点参数,构造后不再变化).

    字段含义与单位:

    - ``replay_speed``:默认回放速度倍率;1.0 = 按录制原速,越小越慢(规范化为 0.1~1.0).
    - ``align_duration``:``align_duration_auto`` 关闭时使用的对齐段时长(s).
    - ``align_duration_auto``:为 True 时对齐时长改由起始误差与目标速度自动推算,
      payload 里的 ``align_duration`` 不再生效.
    - ``align_target_speed_rad_s``:自动推算时假定的对齐运动速度(rad/s),越小对齐越慢.
    - ``align_min_duration`` / ``align_max_duration``:自动推算结果的时长上下限(s).
    - ``align_steps``:对齐段插值步数(含首尾点),规范化为 2~200.
    """

    replay_speed: float
    align_duration: float
    align_duration_auto: bool
    align_target_speed_rad_s: float
    align_min_duration: float
    align_max_duration: float
    align_steps: int

    def from_payload(self, payload: dict, *, max_error: float | None = None) -> dict[str, float | int]:
        """合并 payload 内 ``settings`` 子字典与默认值,返回规范化设置.

        优先级:``payload["settings"]`` 中的同名字段 > 默认值/自动推算值.
        ``max_error`` 是当前位姿与记录首点的最大关节误差(rad),
        仅在自动对齐时长模式下参与计算;其余情况可以传 ``None``.

        返回字典中的 ``final_hold_sec`` 在实现内被固定为 1.0 s(末尾保持时长).
        """
        values = payload.get("settings", {})
        if not isinstance(values, dict):
            raise ValueError("settings must be a mapping")
        for name in ("replay_speed", "align_duration", "align_steps"):
            if name in values:
                if isinstance(values[name], bool):
                    raise ValueError(f"invalid setting {name}")
                try:
                    valid = math.isfinite(float(values[name]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"invalid setting {name}") from exc
                if not valid:
                    raise ValueError(f"invalid setting {name}")
        align_duration = self.auto_align_duration(max_error)
        if not self.align_duration_auto:
            # 手动模式下允许 payload 覆盖对齐时长,缺省则用构造时的参数值.
            align_duration = float(values.get("align_duration", align_duration))
        return normalize_teach_replay_settings(
            replay_speed=float(values.get("replay_speed", self.replay_speed)),
            align_duration=align_duration,
            align_steps=int(values.get("align_steps", self.align_steps)),
            final_hold_sec=1.0,
        )

    def auto_align_duration(self, max_error: float | None) -> float:
        """按起始误差推算对齐时长(s);未启用自动模式时返回配置的固定时长."""
        if self.align_duration_auto:
            # 换算规则:时长 = |误差| / 目标速度,并夹在 [min, max] 之间;
            # 误差为 0 或不可用时取 min_duration.
            return compute_auto_align_duration(
                max_error,
                target_speed_rad_s=float(self.align_target_speed_rad_s),
                min_duration_sec=float(self.align_min_duration),
                max_duration_sec=float(self.align_max_duration),
            )
        return float(self.align_duration)
