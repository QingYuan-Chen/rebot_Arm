"""面板包的参数整形与订阅 QoS 约定辅助(纯函数,无 ROS 调用,无状态).

职责
    把 ROS 参数里的平行数组整理成节点内部使用的查表结构(关节限位),并给出
    读取周期性传感器反馈时统一使用的 QoS 取值.两个函数的返回口径由面板节点
    与同包其它组件共同依赖;本模块不发起任何通信,因此可被单元测试直接调用.

使用位置
    本文件是按面板职责最小复制的副本,调用方见 ``teleop_status_panel_node.py``;
    同名的实现也存在于其它分层包中,修改取值口径时须同步核对,避免各节点对
    同一路反馈使用不同 QoS 而导致订阅静默失配.
"""

from __future__ import annotations


def build_joint_limits(
    *,
    joint_names: tuple[str, ...],
    lower_limits: tuple[float, ...],
    upper_limits: tuple[float, ...],
) -> dict[str, tuple[float, float]]:
    """把三个等长平行序列按同一顺序装配成 ``{关节名: (下限, 上限)}`` 映射.

    ROS 参数不支持嵌套字典,关节名与上下限因此是三个独立数组,这里按**下标**对应关系
    重新绑定:第 i 个关节的限位取两个限位序列的第 i 项.

    参数(仅允许关键字传入,避免三个等长序列被位置错配):
        ``joint_names``   关节名序列,顺序即后两个序列的顺序;
        ``lower_limits``  每个关节的下限;转动关节单位 rad,移动关节为 m;
        ``upper_limits``  每个关节的上限,单位同上.

    返回新建的字典,值统一转为 ``float``(兼容参数里的整数).三个序列长度不一致时抛
    ``ValueError``:错位绑定会把某个关节的限位套到另一个关节上,属于必须拦住的配置错误,
    不能按最短序列截断或补默认值.
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
    """返回订阅周期性传感器反馈所用的 QoS 关键字段,供调用方展开成 QoSProfile 构造参数.

    ``depth``=10:KEEP_LAST 历史策略下的队列深度,只保留最近 10 条;关节状态是持续刷新的
    数据,丢帧可由下一帧替代,因此不排队积压.
    ``reliability``="best_effort":尽力而为投递,必须与控制器/仿真后端发布关节反馈时使用的
    可靠性策略一致,否则订阅端收不到任何消息.

    返回值刻意保留原始的字符串取值(而非 QoSProfile 枚举),由调用方负责转换,使本模块不依赖
    中间件类型,可脱离运行环境测试.
    """
    return {
        "depth": 10,
        "reliability": "best_effort",
    }
