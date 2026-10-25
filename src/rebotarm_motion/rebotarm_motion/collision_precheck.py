"""示教回放/轨迹执行前的碰撞预检工具（运动包的校验能力）。

职责与位置
    在真正下发轨迹之前，把轨迹按固定间隔采样成若干组关节位置，逐组交给 MoveIt 的
    "状态有效性"查询服务做碰撞检查；只要有一个采样点处于碰撞状态，整条轨迹就判为不可
    执行。本模块只做校验，不下发运动指令，也不接触硬件。

对外接口
    调用方（上层示教回放工作流）负责创建 ROS 2 服务客户端与请求对象工厂，通过构造参数
    注入；核心入口是 :meth:`CollisionPrechecker.check_positions`，返回纯字典结果，供上层
    组装状态 payload 并决定是否阻断回放。

结果字典的 ``state`` 取值（这些字符串是上层判定依据，不要改动）
    ``disabled``  预检被配置关闭，未做任何检查；
    ``collision`` 采样点中检测到碰撞（附带接触对信息）；
    ``pass``      计划检查的采样点全部有效；
    ``unknown``   无法得出结论：服务不可用、超时、调用抛异常或检查未覆盖全部采样点。

安全约束
    1. 只有 ``pass`` 表示"没发现问题"；``unknown`` 必须由调用方按"不可放行"处理——本模块
       只如实报告，不做放行决策。
    2. 抽样必须覆盖轨迹首末点，否则端点附近的碰撞会被漏检。
    3. 命中第一个碰撞样本后立即返回，后面的样本不再检查：结果只保证"至少存在这一处碰撞"，
       而不是列出全部碰撞。
    4. 服务不可用、请求异常、超时一律不得被当成"无碰撞"。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable


def select_collision_samples(samples, *, max_samples: int) -> list[tuple[int, object]]:
    """在整条轨迹的样本序列上均匀挑选至多 ``max_samples`` 个样本。

    返回 ``(原始索引, 样本)`` 列表，索引升序，便于上层把碰撞点映射回轨迹点号。
    抽样按 ``round(i * (n - 1) / (limit - 1))`` 均匀铺满整条轨迹，因此首样本与末样本一定
    被包含（"起点是否已经碰撞"和"终点能否到达"是关键判据）。``max_samples`` 会被夹到至少
    1；样本总数不超过上限时全部返回，不做抽样；上限恰为 1 时只返回首样本（此时不覆盖末点）。
    """
    if not samples:
        return []
    # 采样上限至少为 1，避免下面按 (limit - 1) 等分时除零。
    limit = max(int(max_samples), 1)
    if len(samples) <= limit:
        # 样本本来就不多：全部检查，不引入抽样误差。
        return list(enumerate(samples))
    if limit == 1:
        # 只能取一个样本时优先检查轨迹起点（当前状态是否已经处于碰撞）。
        return [(0, samples[0])]
    indices = sorted(
        {
            # 按行程比例取整得到均匀分布的下标；用集合去重后再排序，避免取整产生重复下标。
            round(index * (len(samples) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, samples[index]) for index in indices]


@dataclass(frozen=True)
class CollisionPrecheckConfig:
    """一次碰撞预检的配置（冻结对象，构造后不再变化）。

    ``enabled``     预检总开关；关闭时直接返回 ``disabled``，不访问服务；
    ``service``     状态有效性查询服务名，仅用于回填结果与排查问题；
    ``group_name``  规划组名,MoveIt 依据它决定检查哪些连杆与碰撞对；
    ``max_samples`` 采样上限（个），越大越保险，但每次预检耗时线性增长；
    ``timeout_sec`` 整轮预检的总超时（秒），下限 0.1 s,超时按 ``unknown`` 处理；
    ``default_joint_positions``
                    轨迹样本里没有、但规划组需要的关节及其保持位置（例如夹爪指关节）。
                    检查时补齐到 joint_state,否则服务会因状态不完整而拒绝或给出不可信
                    结果。位置单位随关节量纲（旋转关节 rad,移动关节 m)。
    """

    enabled: bool
    service: str
    group_name: str
    max_samples: int
    timeout_sec: float
    default_joint_positions: tuple[tuple[str, float], ...] = field(default_factory=tuple)


class CollisionPrechecker:
    """对采样后的关节位置执行 MoveIt 状态有效性检查，判断轨迹是否碰撞。

    依赖（服务客户端、请求工厂、时钟、休眠函数）全部由构造参数注入，因此本类不绑定具体
    的 ROS 2 消息类型，可以在没有中间件的情况下用假客户端做单元测试。每个实例对应一个
    服务客户端；检查过程是同步阻塞的（轮询 future 并主动休眠），调用它的回放流程通常也
    是单线程、一次只处理一条轨迹。
    """

    def __init__(
        self,
        *,
        client: Any,
        request_factory: Callable[[], Any],
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """注入外部依赖。

        ``client``          已创建的服务客户端，需提供 ``service_is_ready`` /
                            ``wait_for_service`` / ``call_async``；
        ``request_factory`` 无参可调用对象，每次调用返回一个全新的请求对象（请求对象会被
                            反复填充，不能复用同一实例）；
        ``monotonic``       单调时钟函数（秒），注入后可在测试中伪造超时；
        ``sleep``           休眠函数（秒），注入后测试无需真实等待。
        """
        self._client = client
        self._request_factory = request_factory
        self._monotonic = monotonic
        self._sleep = sleep

    def check_positions(
        self,
        *,
        joint_names: tuple[str, ...],
        positions_list: list[tuple[float, ...]],
        config: CollisionPrecheckConfig,
    ) -> dict:
        """按顺序检查一批关节位置，返回碰撞预检结果字典。

        ``joint_names``    关节名序列，与 ``positions_list`` 中每个元组的元素顺序一一对应；
        ``positions_list`` 待检查的关节位置样本（单位随关节量纲，旋转关节为 rad）。

        返回字段：``state``（见模块文档）、``message``、``checked_samples``（真正拿到结果的
        样本数，含被判定为碰撞的那一个）、``requested_samples``（抽样后计划检查的样本数）、
        ``collisions``（最多一条，含样本下标与前 5 组接触对）、``added_default_joints``
        （本次为凑齐规划组状态而补进来的关节名）。

        安全语义：异常、超时、服务不可用都不抛给调用方，而是降级为 ``unknown``，由调用方按
        "不可放行"处理；只有计划内的样本全部检查且都有效时才返回 ``pass``。
        """
        if not bool(config.enabled):
            # 总开关关闭：不算通过，明确报告 disabled，策略由调用方决定。
            return {"state": "disabled", "message": "collision precheck disabled"}
        if not joint_names or not positions_list:
            # 没有可检查的内容（空关节名或空轨迹），不能当成"无碰撞"。
            return {"state": "unknown", "message": "no trajectory samples to check"}
        if not self._service_available():
            # 服务不可用时同样不给结论，避免把"没查"误当成"没问题"。
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(config.service),
                "checked_samples": 0,
            }
        joint_names, positions_list, added_defaults = self._with_default_joint_positions(
            joint_names,
            positions_list,
            config.default_joint_positions,
        )
        selected = select_collision_samples(positions_list, max_samples=max(int(config.max_samples), 1))
        collisions = []
        checked = 0
        # 整轮检查共用一个截止时间；下限 0.1 s，防止配置成极小值导致必然超时。
        deadline = self._monotonic() + max(float(config.timeout_sec), 0.1)
        for sample_index, positions in selected:
            if self._monotonic() >= deadline:
                # 每个样本前先看总超时，避免单个服务调用把整轮检查拖住。
                return self._unknown_timeout(checked, selected, collisions)
            request = self._request_factory()
            request.group_name = str(config.group_name)
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._client.call_async(request)
            # 本模块不持有 executor，用轮询 + 短休眠等待服务返回；0.01 s 兼顾响应速度与开销。
            while not future.done() and self._monotonic() < deadline:
                self._sleep(0.01)
            if not future.done():
                # 到截止时间仍未返回：跳出循环，后面按"检查不完整"归为 unknown。
                break
            try:
                response = future.result()
            except Exception as exc:
                # 服务调用抛异常（例如服务端中途退出）必须降级为 unknown，而不是放行。
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            # 先计数再判定，保证 collision 结果里的 checked_samples 包含碰撞样本本身。
            checked += 1
            if not bool(getattr(response, "valid", False)):
                collisions.append(
                    {
                        "sample": sample_index,
                        "contacts": self._contacts_to_dicts(getattr(response, "contacts", [])),
                    }
                )
                # 首个碰撞样本即短路：后面的样本再查也不会改变"不可执行"的结论。
                break
        if collisions:
            # 已确认碰撞：附上样本下标与接触对，供界面展示与排查。
            return {
                "state": "collision",
                "message": "collision detected in teach trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            # 循环被提前跳出（服务未在超时前返回）时不允许给出 pass。
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled teach trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
            "added_default_joints": added_defaults,
        }

    def _service_available(self) -> bool:
        """判断状态有效性服务当前是否可调用。

        先查 ``service_is_ready``，再退化为 0 超时的 ``wait_for_service``：服务刚被发现时前者
        可能仍返回 False，用一次不阻塞的等待再确认。任何异常都视为不可用——宁可返回
        ``unknown``，也不要在服务状态不明时继续检查。
        """
        try:
            available = bool(self._client.service_is_ready())
            if not available:
                available = bool(self._client.wait_for_service(timeout_sec=0.0))
            return available
        except Exception:
            return False

    @staticmethod
    def _contacts_to_dicts(contacts) -> list[dict[str, str]]:
        """把服务返回的碰撞接触信息压缩成最多 5 组 ``{body_1, body_2}`` 字典。

        只保留前 5 组：接触对数量可能很多，而结果会进入状态 payload（可能经网络传给界面），
        必须限制体积。键名 ``body_1`` / ``body_2`` 是上层读取用的固定字段名，不要改动。
        """
        result = []
        for contact in list(contacts)[:5]:
            result.append(
                {
                    "body_1": str(getattr(contact, "contact_body_1", "")),
                    "body_2": str(getattr(contact, "contact_body_2", "")),
                }
            )
        return result

    @staticmethod
    def _with_default_joint_positions(
        joint_names: tuple[str, ...],
        positions_list: list[tuple[float, ...]],
        default_joint_positions: tuple[tuple[str, float], ...],
    ) -> tuple[tuple[str, ...], list[tuple[float, ...]], list[str]]:
        """把缺失的默认关节位置补进样本，凑齐规划组要求的状态。

        状态有效性检查要求 joint_state 覆盖规划组内的全部关节；示教轨迹通常只记录 6 个手臂
        关节，夹爪指等关节必须按保持位置补齐，否则检查结果不可信。

        返回 ``(补全后的关节名, 补全后的位置样本, 本次新增的关节名列表)``。新增关节统一追加
        到末尾，保证每个位置元组的元素顺序与关节名一一对应；入参不被修改。
        """
        names = tuple(str(name) for name in joint_names)
        # 已经出现在轨迹里的关节不覆盖，保留记录到的真实位置。
        existing = set(names)
        defaults = [
            (str(name), float(position))
            for name, position in default_joint_positions
            if str(name) not in existing
        ]
        if not defaults:
            # 无需补全时原样返回，避免多余的复制。
            return names, positions_list, []
        added_names = tuple(name for name, _ in defaults)
        added_values = tuple(position for _, position in defaults)
        return (
            names + added_names,
            # 每个样本都追加同一组默认值，长度与 added_names 保持一致。
            [tuple(float(v) for v in positions) + added_values for positions in positions_list],
            list(added_names),
        )

    @staticmethod
    def _unknown_timeout(checked: int, selected, collisions: list) -> dict:
        """构造超时场景的 ``unknown`` 结果（字段与其他分支保持一致）。"""
        return {
            "state": "unknown",
            "message": "collision precheck timed out",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": collisions,
        }
