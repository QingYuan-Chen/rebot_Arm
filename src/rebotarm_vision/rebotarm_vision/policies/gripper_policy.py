"""平行夹爪开合宽度与夹持力的决策策略（纯函数，不接触硬件）。

输入是候选抓取给出的目标夹持宽度（夹爪两指张合方向上的物体尺寸），输出开爪/闭爪目标
位置与最大夹持力，由运动执行侧组装成夹爪指令下发。

安全语义（改动前务必确认）：

- **fail-closed**：物体宽度超过夹爪容许上限时返回 ``allowed=False`` 并附带英文原因串，
  由调用方拒掉这次抓取，绝不「凑合夹一下」；
- 夹持力恒取 ``default_max_effort`` 并截断到 ``[min_effort, max_effort]``，不按物体类别、
  长度或宽度加权，保证同一组参数下行为可复现；
- 长度单位一律为 m；``max_effort`` 是控制器约定的归一化夹持力（本仓库夹爪取 0~0.6 量纲），
  不是牛顿。

本模块不读 ROS 参数、不打日志：默认值只在节点侧被参数覆盖。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GripperPolicyConfig:
    """夹爪策略配置。

    与 ``gripper_policy.yaml`` 中的参数一一对应，单位 m（夹角/力除外）：

    - ``auto_width``：是否按检测到的物体宽度自动推导开合宽度。关闭（或未测得有效宽度）
      时退回 ``default_open_width_m`` / ``default_close_width_m`` 两个固定值。
    - ``auto_effort``：自动夹持力开关。当前实现不参与计算（夹持力恒取
      ``default_max_effort``）。
    - ``default_open_width_m``：默认开爪宽度，0.09 m，等于夹爪最大开口。
    - ``default_close_width_m``：默认合爪宽度，0.025 m；偏小会把目标压得过紧甚至堵转，
      偏大则夹不到目标。
    - ``default_max_effort``：默认最大夹持力（归一化），越大夹得越紧也越容易压坏目标。
    - ``open_clearance_m``：开爪相对物体宽度的余量，0 表示与物体等宽；留太小接近时容易
      刮碰目标。
    - ``close_margin_m``：合爪相对物体宽度收窄的量，0.012 m；靠它形成夹持预紧力，太小
      夹不牢，太大可能顶开或压坏目标。
    - ``min_open_width_m`` / ``max_open_width_m``：开爪位置的机械行程限幅，0.035 / 0.09 m。
    - ``min_close_width_m`` / ``max_close_width_m``：合爪位置的机械行程限幅，0.006 / 0.08 m。
    - ``min_effort`` / ``max_effort``：夹持力安全区间，0.22 / 0.60；低于下限夹不紧，
      高于上限有过载与压坏风险。
    - ``max_allowed_width_m``：允许抓取的最大物体宽度，0.085 m（夹爪开口上限留余量）；
      超过即拒绝，是宽度方向的硬安全门。
    """

    auto_width: bool = True
    auto_effort: bool = True
    default_open_width_m: float = 0.09
    default_close_width_m: float = 0.025
    default_max_effort: float = 0.4
    open_clearance_m: float = 0.0
    close_margin_m: float = 0.012
    min_open_width_m: float = 0.035
    max_open_width_m: float = 0.09
    min_close_width_m: float = 0.006
    max_close_width_m: float = 0.08
    min_effort: float = 0.22
    max_effort: float = 0.60
    max_allowed_width_m: float = 0.085


@dataclass(frozen=True)
class GripperCommand:
    """一次抓取要下发的夹爪指令。

    ``open_width_m`` / ``close_width_m``：开爪、合爪目标位置，单位 m，指两指间距。
    ``max_effort``：合爪阶段允许的最大夹持力（归一化）。
    ``allowed``：False 表示该抓取被策略拒绝，调用方必须终止这条候选。
    ``reason``：拒绝原因（英文，直接进入日志/诊断，不可翻译）；允许时为 "ok"。
    """

    open_width_m: float
    close_width_m: float
    max_effort: float
    allowed: bool = True
    reason: str = "ok"


def _clamp(value: float, lower: float, upper: float) -> float:
    """把 value 截断到 [lower, upper] 闭区间内，用于夹爪行程与夹持力限幅。"""

    return min(max(float(value), float(lower)), float(upper))


def resolve_gripper_command(
    *,
    jaw_width_m: float,
    object_length_m: float = 0.0,
    class_name: str = "",
    config: GripperPolicyConfig | None = None,
) -> GripperCommand:
    """根据物体夹持宽度解算夹爪开合位置与夹持力。

    参数：
    - ``jaw_width_m``：物体在夹爪张合方向上的尺寸，单位 m；<= 0 或负数表示没有有效测量，
      此时退回默认开合宽度（不会按负宽度计算）；``None`` 由调用方转成 0.0 后传入。
    - ``object_length_m`` / ``class_name``：物体长度与类别；当前实现不使用这两个值
      （不做类别相关的夹持力或宽度修正）。
    - ``config``：为空时使用 ``GripperPolicyConfig()`` 默认值，通常由节点用 ROS 参数填充。

    返回 ``GripperCommand``：
    - 宽度超过 ``max_allowed_width_m`` → ``allowed=False``，拒绝该抓取（fail-closed）；
    - 否则开爪 = 物体宽度 + ``open_clearance_m``，合爪 = 物体宽度 - ``close_margin_m``，
      两者分别截断到各自的行程限幅；最后再保证 ``close_width`` 不大于 ``open_width``，
      避免行程限幅把两者夹成倒挂（合爪比开爪还宽）。
    """

    cfg = config or GripperPolicyConfig()
    # 无效测量统一归零（0 是「没测到宽度」的哨兵值），负宽度按 0 处理。
    width = max(float(jaw_width_m or 0.0), 0.0)

    # 硬安全门：物体比夹爪最大许可宽度还宽，直接拒绝这条抓取（fail-closed）。
    if cfg.auto_width and width > float(cfg.max_allowed_width_m):
        return GripperCommand(
            open_width_m=float(cfg.max_open_width_m),
            close_width_m=float(cfg.max_close_width_m),
            max_effort=_clamp(float(cfg.default_max_effort), float(cfg.min_effort), float(cfg.max_effort)),
            allowed=False,
            reason=f"object too wide for gripper: {width:.3f}m > {cfg.max_allowed_width_m:.3f}m",
        )

    # 仅当拿到有效宽度才自适应开合；_clamp 保证下发位置不超出夹爪机械行程。
    if cfg.auto_width and width > 0.0:
        open_width = _clamp(
            width + float(cfg.open_clearance_m),
            float(cfg.min_open_width_m),
            float(cfg.max_open_width_m),
        )
        close_width = _clamp(
            width - float(cfg.close_margin_m),
            float(cfg.min_close_width_m),
            float(cfg.max_close_width_m),
        )
        # 行程限幅可能把合爪顶到比开爪更宽（窄物体时 max_close 仍较大），兜底保证单调递减。
        close_width = min(close_width, open_width)
    else:
        open_width = _clamp(
            float(cfg.default_open_width_m),
            float(cfg.min_open_width_m),
            float(cfg.max_open_width_m),
        )
        close_width = _clamp(
            float(cfg.default_close_width_m),
            float(cfg.min_close_width_m),
            float(cfg.max_close_width_m),
        )

    # 夹持力固定取默认值并限制在安全区间内：不按类别/长度加权，保证行为可复现。
    max_effort = _clamp(float(cfg.default_max_effort), float(cfg.min_effort), float(cfg.max_effort))

    return GripperCommand(
        open_width_m=open_width,
        close_width_m=close_width,
        max_effort=max_effort,
        allowed=True,
        reason="ok",
    )
