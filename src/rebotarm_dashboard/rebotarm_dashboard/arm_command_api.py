"""整臂命令的白名单、锁定判定与超时策略（面板包的纯逻辑层）。

本模块不含任何 ROS 调用与 IO，只提供一组可单测的纯函数，供同包的整臂控制客户端与
面板节点在执行整臂命令前做校验：哪些命令合法、当前是否被示教回放锁定、该等多久。

术语约定
    整臂命令：使能、失能、回安全原点这三类人工操作，取值为英文字符串常量，同时也是
    面板接口与前端约定的命令名；这些字符串是对外接口，不能改写。
    回放锁定：示教回放（或已请求取消/停止但尚未收敛）期间禁止人工整臂命令，
    避免操作者与回放轨迹同时驱动同一台机械臂。
"""

from __future__ import annotations


# 合法的整臂命令白名单；其它任何取值都会被 normalize_arm_command 判为非法命令。
VALID_ARM_COMMANDS = frozenset(("safe_home", "enable", "disable"))
# 需要锁定整臂命令的回放状态：回放中、已请求取消、已请求停止。后两者是尚未收敛的过渡态，
# 此时控制器仍可能在跟踪轨迹，同样不能接受人工整臂命令。
REPLAY_LOCK_STATES = frozenset(("replaying", "cancel_requested", "stop_requested"))
# 执行前必须先停掉在跟踪轨迹的命令：回原点与失能都会改变控制权，若控制器里还排着旧轨迹，
# 可能在新状态下被继续执行；使能不在此列，因为使能本身不改变已下发的目标。
TRAJECTORY_STOP_COMMANDS = frozenset(("safe_home", "disable"))


def normalize_arm_command(command: object) -> str | None:
    """把外部传入的命令规整为白名单内的字符串；非法输入返回 None。

    先转字符串再去除首尾空白，因此 None 与空串都会得到 None；刻意不做大小写折叠，
    "Enable" 这类大小写变体一律视为非法，避免与前端约定的命令名产生歧义。
    """
    value = str(command or "").strip()
    return value if value in VALID_ARM_COMMANDS else None


def arm_command_is_replay_locked(replay_state: object) -> bool:
    """判断当前回放状态是否应锁定整臂命令。

    比较前统一转小写并去空白；未知状态（含空串、None）返回 False，即锁定只对明确的
    三个回放状态生效，避免状态缺省时把机械臂永久锁死。
    """
    return str(replay_state or "").strip().lower() in REPLAY_LOCK_STATES


def status_state(value: object) -> str:
    """从状态字典或裸字符串中取出规范化的 state 字段。

    面板内部的状态有的存成 {"state": ...} 字典，有的直接就是字符串；这里统一成小写去
    空白的字符串，调用方可以直接与状态常量比较。
    """
    if isinstance(value, dict):
        return str(value.get("state", "") or "").strip().lower()
    return str(value or "").strip().lower()


def should_stop_trajectory_before_arm_command(command: object) -> bool:
    """判断该整臂命令执行前是否必须先请求停止轨迹（见 TRAJECTORY_STOP_COMMANDS）。"""
    normalized = normalize_arm_command(command)
    return normalized in TRAJECTORY_STOP_COMMANDS


def arm_command_timeout_sec(command: object) -> float:
    """返回等待整臂服务响应的超时秒数，按命令类型区分。

    safe_home 需要真正走完回原点动作，给 30 s；enable 只做上使能与反馈检查，8 s；
    disable 需要等待受控下使能，10 s。这些值是"最长等待时间"而非动作时长，宁可给宽
    也不要中途误判超时。非法命令走默认 2 s 的保守值（该情况下通常不会真的发起调用）。
    """
    normalized = normalize_arm_command(command)
    # 说明：normalized 为 None 时 str(None) 得到 "None"，同样落到最后的默认值。
    return {"safe_home": 30.0, "enable": 8.0, "disable": 10.0}.get(str(normalized), 2.0)
