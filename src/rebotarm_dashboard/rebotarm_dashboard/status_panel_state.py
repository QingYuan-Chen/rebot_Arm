"""面板状态存储与 SSE 报文工具（面板包的纯数据层）。

面板节点把来自 ROS 的关节反馈、整臂状态以及遥操作/示教状态写入 TeleopStatusStore；
HTTP 的 /api/status 与 /events 再从存储取一份快照。本模块只做“聚合 + 序列化”，
不订阅话题、不发起任何 ROS 调用，因此可以脱离 ROS 运行时单测。

线程模型
    存储内部用一把互斥锁保护三类字典；所有写入方法与 snapshot() 都会加锁，
    snapshot 复制关节条目和 arm/teleop 顶层字典；嵌套列表及状态 payload 仍共享，
    写入方和读取方均按只读数据交接，后续更新应提交新对象。

数据形状（键名为对外接口，保持英文）
    joints：{关节名：{"position"：float，"velocity"：float，"torque"：float，
                      "status_code": int}}
        position 单位弧度（夹爪为米），velocity 单位 rad/s（夹爪为 m/s），
        torque 沿用电机力矩反馈 N·m，status_code 为底层电机状态码。
    arm：{"mode": str, "enabled": bool, "state_machine": str, "error_codes": [str]}
    teleop：{"status": ..., "recording": ..., "replay": ...}，具体取值由上层写入。
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class StatusSnapshot:
    """一次加锁读取得到的状态快照。

    joints 为逐条复制的关节表，arm 为整臂状态，teleop 为遥操作/示教状态。
    arm/teleop 的嵌套内容共享原对象，跨线程消费时保持只读。
    """

    joints: dict[str, dict[str, float | int]]
    arm: dict[str, Any]
    teleop: dict[str, Any]


class TeleopStatusStore:
    """进程内状态存储：ROS 回调线程写，HTTP/SSE 线程读。

    初始值刻意给出“尚未连接”的确定语义：关节表为空，整臂 mode/state_machine 为空串、
    enabled 为 False（真实硬件默认失能），遥操作三项均为 "idle"，
    这样前端在第一条 ROS 消息到达前也能渲染出稳定的状态。
    各 update_* 方法都是覆盖写，不保留历史，也不做范围校验——校验由写入方负责。
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._joints: dict[str, dict[str, float | int]] = {}
        self._arm: dict[str, Any] = {
            "mode": "",
            "enabled": False,
            "state_machine": "",
            "error_codes": [],
        }
        self._teleop: dict[str, Any] = {
            "status": "idle",
            "recording": "idle",
            "replay": "idle",
        }

    def update_joint_state(
        self,
        *,
        names: tuple[str, ...],
        positions: tuple[float, ...],
        velocities: tuple[float, ...],
        efforts: tuple[float, ...],
    ) -> None:
        """用一条关节状态消息批量更新多个关节。

        names/positions/velocities/efforts 是同一时刻的并列元组，按下标对齐；
        position 必定写入，而 velocity/effort 只在对应下标存在时才写入——消息允许这两个
        数组比 name 短，此时缺失的关节保留上一次的速度/力矩，不会被清零。
        表中不存在的关节会被新建；已有条目按字段就地合并，不会丢掉 status_code 等旧字段。
        """
        with self._lock:
            for index, name in enumerate(names):
                joint = dict(self._joints.get(name, {}))
                joint["position"] = float(positions[index])
                if index < len(velocities):
                    joint["velocity"] = float(velocities[index])
                if index < len(efforts):
                    joint["torque"] = float(efforts[index])
                self._joints[name] = joint

    def update_motor_state(
        self,
        *,
        joint_name: str,
        position: float,
        velocity: float,
        torque: float,
        status_code: int,
    ) -> None:
        """用单个电机的状态反馈整体覆盖该关节条目（含底层状态码）。

        与 update_joint_state 共用同一张关节表，两者谁后写入谁生效；本方法会把条目
        完全替换为这四个字段，因此来自逐电机话题的反馈不会残留旧的速度/力矩。
        位置单位弧度（夹爪为米）、速度 rad/s、力矩 N·m，status_code 为电机状态码。
        """
        with self._lock:
            self._joints[joint_name] = {
                "position": float(position),
                "velocity": float(velocity),
                "torque": float(torque),
                "status_code": int(status_code),
            }

    def update_arm_status(
        self,
        *,
        mode: str,
        enabled: bool,
        state_machine: str,
        error_codes: tuple[str, ...],
    ) -> None:
        """覆盖整臂状态：控制模式、使能标志、状态机名与当前错误码列表。

        error_codes 会转成列表存储、enabled 强制为 bool，保证快照的字段类型稳定，
        前端不必再判断类型。
        """
        with self._lock:
            self._arm = {
                "mode": mode,
                "enabled": bool(enabled),
                "state_machine": state_machine,
                "error_codes": list(error_codes),
            }

    def update_teleop_status(self, key: str, value: Any) -> None:
        """按 key 写入一项遥操作/示教状态（如 status、recording、replay、arm_command）。

        key 由调用方约定，这里不做白名单校验，未知 key 也会被保留；value 原样保存，
        不做深拷贝，因此调用方在写入后不应再修改该对象。
        """
        with self._lock:
            self._teleop[key] = value

    def snapshot(self) -> StatusSnapshot:
        """加锁复制关节条目及 arm/teleop 顶层；调用方保持嵌套 payload 只读。"""
        with self._lock:
            return StatusSnapshot(
                joints={name: dict(data) for name, data in self._joints.items()},
                arm=dict(self._arm),
                teleop=dict(self._teleop),
            )

    def snapshot_dict(self) -> dict[str, Any]:
        """把快照整理成可直接 JSON 序列化的字典（供 HTTP 与 SSE 使用）。"""
        snapshot = self.snapshot()
        return {
            # ROS 允许用 NaN 表示未知反馈，而 JSON 不允许；因此内部仍保留原始快照，
            # 只在序列化时把非有限浮点数（NaN/±Inf）换成 JSON null，前端据此显示“未知”。
            "joints": {
                name: {
                    key: (
                        None if isinstance(value, float) and not math.isfinite(value) else value
                    )
                    for key, value in joint.items()
                }
                for name, joint in snapshot.joints.items()
            },
            "arm": snapshot.arm,
            "teleop": snapshot.teleop,
        }


def encode_sse_event(payload: dict[str, Any], *, event: str = "status") -> str:
    """把状态字典编码成一条 SSE（Server-Sent Events）报文。

    返回依次为事件行、数据行和一个空行；末尾的空行是 SSE 的帧结束标记，缺少它浏览器
    不会派发事件。数据部分用紧凑 JSON（分隔符不带空格）以减小流式传输体积。
    事件名默认 "status"，需与前端 addEventListener 监听的名字一致。
    """
    data = json.dumps(payload, separators=(",", ":"))
    return f"event: {event}\ndata: {data}\n\n"


def format_angle_readout(radians: float) -> dict[str, str]:
    """把弧度角度格式化成同时带 rad 与 deg 两个字符串，便于前端直接显示。

    固定精度：弧度保留 4 位小数、角度保留 1 位小数；返回的是格式化后的字符串而非数值，
    前端不需要再自行换算或截断。仅用于显示，不参与任何控制计算。
    """
    value = float(radians)
    return {
        "rad": f"{value:.4f}",
        "deg": f"{math.degrees(value):.1f}",
    }


def clamp_preview_value(value: float, lower: float, upper: float) -> float:
    """把预览用途的数值钳制到 [lower, upper] 区间。

    上下限写反时会先自动交换；本函数只用于界面预览与自定义控件的取值限制，
    不是安全钳位，真正的安全校验在下发命令的适配层完成。
    """
    low = float(lower)
    high = float(upper)
    if high < low:
        low, high = high, low
    return min(max(float(value), low), high)
