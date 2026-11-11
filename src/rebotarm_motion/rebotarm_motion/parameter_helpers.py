"""节点参数与订阅 QoS 的小工具（无任何 import 的纯函数模块）。

本模块只做"把节点声明的参数拼成便于查询的结构"和"给出与控制器反馈一致的订阅 QoS 约定"，
不持有状态、不依赖中间件类型，因此单元测试可以直接调用。包内的关节限位、关节反馈订阅都
依赖这里保持的字段名与取值口径；同名副本也存在于其它包里，改动需同步核对。
"""

from __future__ import annotations


def build_joint_limits(
    *,
    joint_names: tuple[str, ...],
    lower_limits: tuple[float, ...],
    upper_limits: tuple[float, ...],
) -> dict[str, tuple[float, float]]:
    """把三个平行数组装配成 ``{关节名: (下限, 上限)}`` 查表。

    参数（只允许关键字传入，避免三个等长序列被位置错配）：
        ``joint_names``   关节名，顺序即后两个序列的顺序；
        ``lower_limits``  每个关节的下限（rad，转动关节；若是移动关节则为 m）；
        ``upper_limits``  每个关节的上限，单位同上。

    返回新建的 dict，值统一 ``float`` 化，调用方可安全修改。三者长度不一致时抛 ``ValueError``
    （消息为固定英文常量）：错位会把某个关节的限位套到另一个关节上，属于必须拒绝的配置错误，
    不能按最短序列截断或补默认值。
    """
    if len(joint_names) != len(lower_limits) or len(joint_names) != len(upper_limits):
        raise ValueError(
            "joint_names, lower_limits, and upper_limits must have the same length"
        )
    # zip 在长度一致的前提下逐项配对；转 float 是为了兼容 YAML 传进来的 int 参数值。
    return {
        name: (float(lower), float(upper))
        for name, lower, upper in zip(joint_names, lower_limits, upper_limits)
    }


def sensor_qos_kwargs() -> dict[str, object]:
    """返回"传感器数据 QoS"的关键字段，供调用方展开成 QoSProfile 构造参数。

    ``depth=10``：KEEP_LAST 历史策略的队列深度，按控制器约 100 Hz（10 ms 周期）的关节反馈
    计算，约缓存最近 0.1 s 的数据，足以覆盖一次回调抖动。
    ``reliability="best_effort"``：尽力而为投递，与控制器/仿真后端发布关节反馈时使用的传感器
    QoS 保持一致——发布端与订阅端的可靠性策略必须匹配，否则订阅收不到消息。关节状态是周期
    刷新的数据，丢帧可用下一帧替代，因此不做可靠重传。
    """
    return {
        "depth": 10,
        "reliability": "best_effort",
    }
