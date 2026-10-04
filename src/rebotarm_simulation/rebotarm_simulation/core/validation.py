import math
from typing import Sequence

def finite_vector(values: Sequence[float], length: int, label: str) -> tuple[float, ...]:
    """把输入校验为指定长度、全部有限的浮点向量。

    label 仅用于构造错误消息。所有对外数值接口（关节角、位姿、四元数）都经此
    把关：非数值、长度不符、含 NaN/Inf 一律拒绝，避免坏值进入物理积分。
    """
    # 显式拦截 str/bytes：它们本身可迭代，否则会被逐字符转换而得到隐蔽的错误值。
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be a numeric sequence") from exc
    if len(result) != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} values must be finite")
    return result
