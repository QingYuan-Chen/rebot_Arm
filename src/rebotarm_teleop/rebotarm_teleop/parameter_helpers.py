"""遥操作节点共用的 ROS 参数→运行时结构转换辅助.

本模块只做纯数据整形:把 ROS 参数里的平行数组/配置字典转成节点内部使用的
映射,不含任何 ROS 调用,便于单独测试.
"""

from __future__ import annotations


def build_joint_limits(
    *,
    joint_names: tuple[str, ...],
    lower_limits: tuple[float, ...],
    upper_limits: tuple[float, ...],
) -> dict[str, tuple[float, float]]:
    """把同名等长的三个平行数组组装成"关节名 → (下限, 上限)"映射.

    ROS 参数不支持嵌套字典,因此关节名与上下限在参数里是三个独立数组,这里按
    下标对应关系重新绑定;单位统一为弧度(rad).三个数组长度不一致时抛
    ``ValueError``-错位绑定会把限位套到错误的关节上,属于必须拦住的配置错误.
    """
    if len(joint_names) != len(lower_limits) or len(joint_names) != len(upper_limits):
        raise ValueError(
            "joint_names, lower_limits, and upper_limits must have the same length"
        )
    return {
        name: (float(lower), float(upper))
        for name, lower, upper in zip(joint_names, lower_limits, upper_limits)
    }


def sensor_qos_kwargs() -> dict[str, object]:
    """返回订阅传感器类话题使用的 QoS 关键参数.

    ``depth``=10:只保留最近 10 条,状态流允许丢旧帧而不排队;
    ``reliability``="best_effort":与控制器发布端保持一致,避免因 QoS 不兼容
    而完全收不到关节状态.返回的是原始字符串取值,由调用方转成 QoSProfile 枚举.
    """
    return {
        "depth": 10,
        "reliability": "best_effort",
    }
