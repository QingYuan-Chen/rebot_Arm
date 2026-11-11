"""成对轨迹命令协议：真机与仿真共用的关节空间命令格式、构造、校验与采样。

职责与位置
    本模块属于运动层，是一套与 ROS 无关（只用标准库）的纯计算工具：把一条关节空间
    运动表示成可 JSON 序列化的"命令"字典，供真机执行脚本、离线生成脚本与仿真侧分析
    脚本读写同一份数据。真机/仿真对照实验要求两侧执行完全相同的命令，因此字段格式、
    时间栅格与内容哈希都在这里统一定义。

命令字典字段（键名是跨模块接口，不可改动）
    ``schema_version``  格式版本号，当前为 1，版本不符时校验失败；
    ``label``           命令标签，仅用于报告与日志；
    ``joint_names``     固定为 joint1..joint6，``points`` 中的位置按此顺序解释；
    ``duration_sec``    命令总时长，单位 s；
    ``cadence_sec``     采样周期，单位 s（``duration_sec`` 必须是它的整数倍）；
    ``points``          轨迹点列表，每点含 ``elapsed_sec``（相对起点的秒数，首点为 0，
                        末端等于 ``duration_sec``，严格递增）与 ``positions``（6 个关节
                        角，单位 rad）；
    ``source_waypoints`` / ``source_waypoint_indices``
                        仅重定时命令才有：原始规划路径点，及其在 ``points`` 中的下标，
                        用于事后审计"规划器的几何路径是否被完整保留"；
    ``command_sha256``  对除自身外全部字段的规范化 JSON 计算的 SHA-256，任何篡改都会
                        让 :func:`validate_command` 失败。

安全语义
    1. 命令只携带位置，速度由控制器插值产生；真正决定实际速度的是时间栅格
       （``duration_sec`` 与 ``cadence_sec``）。构造阶段不做速度限位，越限由执行侧的
       运行守卫拦截，所以调整时长必须同时考虑守卫阈值。
    2. :func:`validate_command` 是执行前的最后一道格式闸门：调用方必须先校验再下发，
       且不得把校验失败当成"空轨迹"继续执行。
    3. 起止速度为零由五次多项式时间律保证，避免下发瞬间出现阶跃指令。
"""

from __future__ import annotations

from bisect import bisect_right
import hashlib
import json
import math
from typing import Mapping, Sequence


# 手臂 6 个旋转关节的规范顺序：命令里的位置向量、真机反馈与仿真对照分析都按此顺序解释。
ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
# 命令格式版本：校验时用它拒绝版本不符的命令，避免按错误字段解释数据。
SCHEMA_VERSION = 1


def quintic_blend(ratio: float) -> float:
    """五次多项式时间律 ``s(t) = 10t^3 - 15t^4 + 6t^5``（``t`` 为归一化进度）。

    关键性质是两端的一阶、二阶导数都为零：``s(0)=0``、``s(1)=1``、
    ``s'(0)=s'(1)=0``、``s''(0)=s''(1)=0``，因此用它做时间参数化时起止速度与加速度
    都是零，指令不会在起步/停止瞬间跳变；曲线在 [0, 1] 上单调递增。

    ``ratio`` 会被夹到 [0, 1]（超范围按端点处理）；返回值是 0..1 的无量纲进度。
    """
    value = min(max(float(ratio), 0.0), 1.0)
    return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5


def build_quintic_command(
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    *,
    duration_sec: float,
    cadence_sec: float = 0.05,
    label: str,
) -> dict[str, object]:
    """构造"起点 -> 终点"的直线关节空间命令，时间律为五次多项式。

    每个关节独立按 ``start + (target - start) * blend`` 插值，所以在关节空间里是一条
    直线，中途不会绕行；如果终点来自规划器且需要保留其避障路径，必须改用
    :func:`build_retimed_path_command`（本函数会丢弃规划器的全部中间路径点）。

    参数：
    - ``start_positions`` / ``target_positions``：各 6 个关节角，单位 rad，顺序与
      :data:`ARM_JOINT_NAMES` 一致；
    - ``duration_sec``：总时长，单位 s，必须为正，且是 ``cadence_sec`` 的整数倍
      （否则末点时间无法既落在采样栅格上又等于总时长）；
    - ``cadence_sec``：采样周期，单位 s，默认 0.05（20 Hz）；周期越大命令点越稀疏、
      由控制器插值填补，实际速度与总时长成反比；
    - ``label``：命令标签，原样写入返回值，便于在报告里区分不同行程。

    返回填好 ``command_sha256`` 的命令字典（结构见模块 docstring）；参数非法时由
    :func:`_vector6` / :func:`_positive_finite` 抛 ``ValueError``。
    """
    start = _vector6(start_positions, "start_positions")
    target = _vector6(target_positions, "target_positions")
    duration = _positive_finite(duration_sec, "duration_sec")
    cadence = _positive_finite(cadence_sec, "cadence_sec")
    # 采样段数：至少 2 段以保证起点与终点都在命令里；duration 已要求是 cadence 的整数倍，
    # 这里的 round 只是消除浮点除法误差。
    steps = max(2, int(round(duration / cadence)))
    actual_cadence = duration / float(steps)
    # 复核整除关系：不整除时末点时间会偏离 duration_sec，直接拒绝而不是悄悄改周期。
    if not math.isclose(actual_cadence, cadence, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("duration_sec must be an integer multiple of cadence_sec")

    points = []
    for index in range(steps + 1):
        # 归一化进度 t = i / steps；blend 用五次曲线，使首末段自动加减速。
        ratio = index / float(steps)
        blend = quintic_blend(ratio)
        points.append(
            {
                "elapsed_sec": duration * ratio,
                "positions": [
                    float(origin + (destination - origin) * blend)
                    for origin, destination in zip(start, target)
                ],
            }
        )
    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "label": str(label),
        "joint_names": list(ARM_JOINT_NAMES),
        "duration_sec": duration,
        "cadence_sec": cadence,
        "points": points,
    }
    payload["command_sha256"] = command_sha256(payload)
    return payload


def build_retimed_path_command(
    waypoints: Sequence[Sequence[float]],
    *,
    duration_sec: float,
    cadence_sec: float = 0.05,
    label: str,
) -> dict[str, object]:
    """把已有的关节空间路径重定时到五次多项式时间律上。

    :func:`build_quintic_command` 只做起点到终点的直线插值，会丢掉规划器在中途找到的
    一切信息：避障绕行、中间姿态，以及该规划实际所属的逆解分支。拿它替代规划结果去
    执行，末端扫过的笛卡尔路径就与当初被校验过的那条不同了。

    本函数原样保留规划器的几何路径——每个源路径点都按顺序插入下发命令——只把时间参数化
    换成平滑、边界速度为零的曲线。
    弧长在关节空间用无穷范数（max-norm，各关节位移绝对值的最大值）度量，五次曲线作用在
    "沿该弧长的进度"上，因此起点与终点的速度都为零。

    参数：
    - ``waypoints``：规划器给出的关节空间路径，按执行顺序排列，每个元素是 6 个关节角
      （rad）；至少 2 个点，且相邻点不能全部重合（总弧长必须为正）；
    - ``duration_sec`` / ``cadence_sec``：与 :func:`build_quintic_command` 同义，单位 s；
      时长决定峰值速度（越短越快），必须为正且是采样周期的整数倍；
    - ``label``：命令标签，原样写入返回值。

    返回的命令字典额外带 ``source_waypoints`` 与 ``source_waypoint_indices`` 审计字段，
    使 :func:`validate_command` 能验证"规划器的每个路径点都原样出现在命令中"。
    """
    if len(waypoints) < 2:
        # 少于两个点无法构成路径（也没有可插值的区段）。
        raise ValueError("waypoints must contain at least two entries")
    path = [_vector6(point, "waypoint") for point in waypoints]
    duration = _positive_finite(duration_sec, "duration_sec")
    cadence = _positive_finite(cadence_sec, "cadence_sec")
    steps = max(2, int(round(duration / cadence)))
    actual_cadence = duration / float(steps)
    if not math.isclose(actual_cadence, cadence, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("duration_sec must be an integer multiple of cadence_sec")

    # 累计关节空间弧长：用 max-norm 而不是欧氏距离，让运动量最大的关节主导进度。
    cumulative = [0.0]
    for previous, current in zip(path, path[1:]):
        step = max(abs(b - a) for a, b in zip(previous, current))
        cumulative.append(cumulative[-1] + step)
    total = cumulative[-1]
    if total <= 0.0:
        # 全部路径点重合 => 没有可走的路径，重定时无从谈起（同时避免后面除零）。
        raise ValueError("waypoints must describe a non-zero path")

    # 只按固定周期重采样可能"跨过"某个中间路径点，控制器会直接插值切掉拐角。因此把每个
    # 规划器路径点的精确时刻也并入采样栅格，命令折线就不可能切掉规划器的拐角。相邻的重复
    # 路径点会共用一个命令点。
    waypoint_times = [
        # 路径点时刻 = 总时长 × 反解五次曲线（该点累计弧长占总弧长的比例），保证在五次
        # 时间律下走到该比例弧长的时刻正好是它。
        duration * _inverse_quintic_blend(length / total)
        for length in cumulative
    ]
    sample_times = [duration * index / float(steps) for index in range(steps + 1)]
    event_times = sorted([*sample_times, *waypoint_times])
    times: list[float] = []
    for event_time in event_times:
        # 合并同一时刻（1e-12 容差内）的事件，避免生成时间重复的轨迹点。
        if not times or not math.isclose(event_time, times[-1], rel_tol=0.0, abs_tol=1e-12):
            times.append(event_time)

    points = []
    for elapsed in times:
        # 该时刻沿路径已走过的弧长 -> 定位所在区段 -> 区段内线性插值。
        ratio = elapsed / duration
        travelled = total * quintic_blend(ratio)
        segment = bisect_right(cumulative, travelled)
        if segment >= len(cumulative):
            # 已到路径末端（浮点误差下 travelled 可能略超 total）。
            positions = list(path[-1])
        elif segment == 0:
            # 仍在路径起点（travelled <= 0）。
            positions = list(path[0])
        else:
            lower_length = cumulative[segment - 1]
            upper_length = cumulative[segment]
            span = upper_length - lower_length
            # span <= 0 说明该区段是重复点，直接取区段起点，避免除零。
            blend = 0.0 if span <= 0.0 else (travelled - lower_length) / span
            positions = [
                float(start + (end - start) * blend)
                for start, end in zip(path[segment - 1], path[segment])
            ]
        points.append({"elapsed_sec": elapsed, "positions": positions})

    # 审计字段：记录每个源路径点在命令中的下标，并把这些命令点的位置"钉"回路径点的精确值
    # （栅格点的位置是插值出来的，可能与路径点存在浮点级偏差）。
    source_waypoint_indices: list[int] = []
    for waypoint, waypoint_time in zip(path, waypoint_times):
        command_index = min(
            range(len(points)),
            key=lambda index: abs(float(points[index]["elapsed_sec"]) - waypoint_time),
        )
        # 路径点时刻已并入栅格，理应能精确命中；命中不了说明合并逻辑出错，必须报错而不是
        # 悄悄把拐角挪到邻近的采样点上。
        if not math.isclose(
            float(points[command_index]["elapsed_sec"]),
            waypoint_time,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("failed to place planner waypoint on command timeline")
        points[command_index]["positions"] = list(waypoint)
        source_waypoint_indices.append(command_index)

    payload: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "label": str(label),
        "joint_names": list(ARM_JOINT_NAMES),
        "duration_sec": duration,
        "cadence_sec": cadence,
        "points": points,
        "source_waypoints": [list(point) for point in path],
        "source_waypoint_indices": source_waypoint_indices,
    }
    payload["command_sha256"] = command_sha256(payload)
    return payload


def command_sha256(command: Mapping[str, object]) -> str:
    """计算命令的内容哈希，返回 SHA-256 的十六进制字符串。

    规范化方式：先排除 ``command_sha256`` 自身（否则无法自洽），再按键名排序、用紧凑
    分隔符（无多余空格）序列化成 JSON；``ensure_ascii=False`` 保留标签原文，
    ``allow_nan=False`` 直接拒绝 NaN/Infinity（它们不是合法 JSON，出现即说明命令里有
    非法数值）。

    排序加紧凑格式是确定性的，因此真机侧与仿真侧对同一份命令必然得到相同哈希；对照分析
    用它确认两侧执行的是同一条命令，任何字段被改动都会导致哈希变化。
    """
    canonical = {key: value for key, value in command.items() if key != "command_sha256"}
    encoded = json.dumps(
        canonical,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def validate_command(command: Mapping[str, object]) -> None:
    """校验命令字典的完整性与自洽性，不通过就抛 ``ValueError``（通过则返回 ``None``）。

    检查项：
    - ``schema_version`` 必须等于 :data:`SCHEMA_VERSION`；
    - ``joint_names`` 必须恰好是规范的 joint1..joint6；
    - ``points`` 至少 2 个，且每个点都是映射、含 6 个有限位置值；
    - 每点 ``elapsed_sec`` 有限、非负、严格递增，末点时间必须等于 ``duration_sec``
      （容差 1e-9）；
    - 审计字段 ``source_waypoints`` / ``source_waypoint_indices`` 必须成对出现、长度
      一致、下标有序且在范围内，且下标指向的命令点位置与源路径点逐位相等；
    - ``command_sha256`` 必须与重新计算的内容哈希一致。

    只校验、不修改入参；任何一项不通过都直接抛异常，绝不返回"部分可信"的结果，安全路径
    上的调用方必须把异常当作"禁止下发"。
    """
    if int(command.get("schema_version", -1)) != SCHEMA_VERSION:
        raise ValueError("unsupported command schema_version")
    if tuple(command.get("joint_names", ())) != ARM_JOINT_NAMES:
        raise ValueError("command joint_names must be canonical joint1..joint6")
    points = command.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("command must contain at least two points")
    # 时间单调性哨兵：首点允许为 0.0（-1.0 小于任何合法时间），其余必须严格递增。
    previous = -1.0
    for point in points:
        if not isinstance(point, Mapping):
            raise ValueError("command points must be mappings")
        elapsed = float(point["elapsed_sec"])
        _vector6(point["positions"], "point positions")
        if not math.isfinite(elapsed) or elapsed < 0.0 or elapsed <= previous:
            if elapsed == 0.0 and previous < 0.0:
                pass
            else:
                raise ValueError("point elapsed_sec must be finite and strictly increasing")
        previous = elapsed
    # 末点时间与声明的总时长必须一致，否则控制器看到的时长与命令元数据不符。
    if not math.isclose(previous, float(command["duration_sec"]), abs_tol=1e-9):
        raise ValueError("last point must equal duration_sec")
    source_waypoints = command.get("source_waypoints")
    source_indices = command.get("source_waypoint_indices")
    # 两个审计字段要么都有、要么都没有：只出现一个是残缺命令。
    if (source_waypoints is None) != (source_indices is None):
        raise ValueError("source waypoint audit fields must be provided together")
    if source_waypoints is not None and source_indices is not None:
        if not isinstance(source_waypoints, list) or not isinstance(source_indices, list):
            raise ValueError("source waypoint audit fields must be lists")
        if len(source_waypoints) != len(source_indices) or len(source_waypoints) < 2:
            raise ValueError("source waypoint audit fields have invalid lengths")
        previous_index = -1
        for waypoint, point_index in zip(source_waypoints, source_indices):
            expected = _vector6(waypoint, "source waypoint")
            index = int(point_index)
            # 下标必须随路径顺序单调不减，且在命令点范围内。
            if index < previous_index or not 0 <= index < len(points):
                raise ValueError("source waypoint indices must be ordered and in range")
            actual = _vector6(points[index]["positions"], "point positions")
            # 要求逐位相等（不是近似）：确认规划器拐角确实原样留在命令里。
            if actual != expected:
                raise ValueError("source waypoint is not present exactly in command points")
            previous_index = index
    # 哈希覆盖上面所有字段，因此这一条同时等价于"命令内容未被篡改"。
    if str(command.get("command_sha256", "")) != command_sha256(command):
        raise ValueError("command_sha256 mismatch")


def sample_command(command: Mapping[str, object], elapsed_sec: float) -> tuple[float, ...]:
    """取命令在指定时刻的期望关节角，用于把真机/仿真反馈与命令对齐比较。

    先跑一次 :func:`validate_command`（保证时间栅格合法、点序列严格递增），再做分段线性
    插值：命令点之间没有速度信息，只能线性插值；时刻早于首点或晚于末点时按端点值钳位，
    不做外推。

    ``elapsed_sec`` 是相对命令起点的秒数，必须有限；返回 6 个关节角（rad），顺序同
    :data:`ARM_JOINT_NAMES`。命令非法或时刻非有限时抛异常。
    """
    validate_command(command)
    elapsed = float(elapsed_sec)
    if not math.isfinite(elapsed):
        raise ValueError("elapsed_sec must be finite")
    points = command["points"]
    times = [float(point["elapsed_sec"]) for point in points]
    # 越界钳位：命令范围之外不负责外推，返回最近端点的位置。
    if elapsed <= times[0]:
        return tuple(float(value) for value in points[0]["positions"])
    if elapsed >= times[-1]:
        return tuple(float(value) for value in points[-1]["positions"])
    # bisect_right 给出第一个时间大于 elapsed 的下标，因此插值区间是 [upper-1, upper]。
    upper = bisect_right(times, elapsed)
    lower_point = points[upper - 1]
    upper_point = points[upper]
    lower_time = float(lower_point["elapsed_sec"])
    upper_time = float(upper_point["elapsed_sec"])
    ratio = (elapsed - lower_time) / (upper_time - lower_time)
    return tuple(
        float(start + (end - start) * ratio)
        for start, end in zip(lower_point["positions"], upper_point["positions"])
    )


def _vector6(values: Sequence[float], label: str) -> tuple[float, ...]:
    """把输入规范化为"6 个有限浮点数"的元组，非法时抛 ``TypeError`` / ``ValueError``。

    ``label`` 只用于异常文本，便于定位出错的是哪个参数。字符串与字节序列被显式拒绝：
    它们可迭代但迭代结果是字符，属于典型调用错误。
    """
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{label} must be a numeric sequence")
    result = tuple(float(value) for value in values)
    # 长度固定为 6：与 ARM_JOINT_NAMES 绑定，命令的关节顺序是固定契约。
    if len(result) != len(ARM_JOINT_NAMES):
        raise ValueError(f"{label} must contain six values")
    # NaN/Inf 会污染哈希与控制器的插值，一律拒绝。
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain finite values")
    return result


def _positive_finite(value: float, label: str) -> float:
    """校验"有限正数"（用于时长与采样周期，单位 s），返回规范化后的 float。"""
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return result


def _inverse_quintic_blend(progress: float) -> float:
    """反解 [0, 1] 上单调递增的五次混合曲线（:func:`quintic_blend` 的逆函数）。

    没有闭式解，用二分法迭代 64 次，区间宽度收敛到约 2**-64，远小于轨迹所需的精度。
    输入会被夹到 [0, 1]；端点直接返回 0.0 / 1.0，避免无意义的迭代。
    """
    target = min(max(float(progress), 0.0), 1.0)
    if target <= 0.0:
        return 0.0
    if target >= 1.0:
        return 1.0
    lower = 0.0
    upper = 1.0
    for _ in range(64):
        # 在 [lower, upper] 上二分：blend(middle) 小于目标说明解在右半区间。
        middle = (lower + upper) * 0.5
        if quintic_blend(middle) < target:
            lower = middle
        else:
            upper = middle
    return (lower + upper) * 0.5
