"""仿真命令行入口：以文本命令手动操纵 MuJoCo 仿真，或做无头定量步进。

职责与位置
    本模块属于仿真层，是纯离线调试工具：命令行走的是仿真后端，绝不触碰真实电机
    通道，也不参与 ROS 通信。它是同一份模型在不同视角下的"手动示波器"，用于快速
    复现模型加载问题、夹爪/关节响应与接触情况，并把结果以 JSON 打印到标准输出，
    便于脚本收集。

两种运行模式
    1. 交互模式（默认）：逐行读取标准输入，把每行交给 :func:`dispatch_command`
       解析执行，执行结果序列化为 JSON 打印到标准输出，直到 quit 或输入结束；
    2. 无头模式（``--headless``）：跑固定步数或固定时长后打印一次最终状态 JSON
       并退出，用于自动化检查（见 :func:`_run_headless`）。

输入输出约定
    - 支持的命令：``quit``、``state``、``joint NAME VALUE``、``joints J1..J6``、
      ``jog NAME DELTA``、``gripper WIDTH``、``step [N]``、``reset``、
      ``contacts``、``pause``、``resume``；命令名大小写不敏感，参数个数与名字
      不合法时抛 :class:`ValueError`；
    - 交互模式里单个命令的解析/执行错误只打印一行 ``error: ...`` 并继续读下一行，
      不会终止会话；而 :func:`main` 中初始化仿真的失败会打印到标准错误并返回
      退出码 1；
    - 位置单位为 rad、夹爪宽度单位为 m、步数与时长分别以步和 s 计，均为仿真量，
      与真机标定无关。
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping
from dataclasses import fields, is_dataclass
import json
import math
from pathlib import Path
import sys

from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco


def _nonnegative_finite(value: str) -> float:
    """argparse 类型校验：非负且有限的浮点数（用于 ``--duration``，单位 s）。"""
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative finite number")
    return number


def _positive_int(value: str) -> int:
    """argparse 类型校验：正整数（用于 ``--steps``）。0 与负数一律拒绝。"""
    try:
        number = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive integer") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def build_parser() -> argparse.ArgumentParser:
    """构造命令行解析器。

    选项：
    - ``--model``：MJCF 场景文件路径；缺省（None）时由仿真对象自己使用随包安装的
      默认场景，因此不必手写绝对路径；
    - ``--headless``：只跑定量步进、不进入交互提示；
    - ``--duration``：无头模式的仿真时长，单位 s，必须非负有限；
    - ``--steps``：无头模式的物理步数，必须是正整数。
    注意 ``duration`` 与 ``steps`` 互不冲突：给了 steps 就先跑 steps 步，再按时长
    续跑（见 :func:`_run_headless`）。
    """
    parser = argparse.ArgumentParser(description="Run or manually control reBotArm MuJoCo")
    parser.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    parser.add_argument("--headless", action="store_true", help="run without an interactive prompt")
    parser.add_argument("--duration", type=_nonnegative_finite, help="simulation seconds to run")
    parser.add_argument("--steps", type=_positive_int, help="number of physics steps to run")
    return parser


def _plain(value):
    """把任意结果对象递归转换成可 JSON 序列化的普通 Python 类型。

    规则：数据类按字段名展开；映射转成键为字符串的字典；元组/列表转成列表（元组
    在 JSON 里没有对应类型）；路径转字符串；None/布尔/整数/浮点/字符串原样保留；
    其他对象退回其 ``__dict__``。无法处理时抛 :class:`TypeError` 而不是返回字符串，
    避免把类型错误伪装成正常输出。
    """
    if is_dataclass(value):
        return {field.name: _plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if hasattr(value, "__dict__"):
        return {key: _plain(item) for key, item in vars(value).items()}
    raise TypeError(f"cannot serialize {type(value).__name__}")


def _emit(value, stdout) -> None:
    """把结果序列化成一行 JSON 写到指定流。

    固定 ``sort_keys=True`` 让同一状态每次输出字节一致，方便 diff 与回归比较；
    ``ensure_ascii=False`` 保留非 ASCII 原文（真机/仿真标签里可能有中文）。
    """
    print(json.dumps(_plain(value), ensure_ascii=False, sort_keys=True), file=stdout)


def dispatch_command(sim, line: str, *, paused: bool = False):
    """解析并执行一行交互命令，返回 ``(新的暂停状态, 待打印结果, 是否退出)``。

    参数：
    - ``sim``：仿真对象，需提供 state/joint/joints/jog/gripper/step/reset/contacts
      等能力；
    - ``line``：原始输入行，按空白切分；空行直接返回 ``(paused, None, False)``；
    - ``paused``：当前暂停状态。暂停只影响 ``step``（被忽略并提示），不影响读状态
      或设置目标，因此暂停时仍可安全地检查模型。

    命令名统一转小写，关节名按 :data:`ARM_JOINT_NAMES` 精确匹配。用法或参数不合法、
    或命令未知时抛 :class:`ValueError`（消息即用法提示），由调用方决定如何呈现。
    结果值为 ``None`` 表示"无需打印"，退出标志为 True 时调用方应结束会话。
    """
    parts = line.split()
    if not parts:
        return paused, None, False
    command, arguments = parts[0].lower(), parts[1:]
    if command == "quit":
        if arguments:
            raise ValueError("usage: quit")
        return paused, "bye", True
    if command == "state":
        if arguments:
            raise ValueError("usage: state")
        return paused, sim.get_state(), False
    if command == "joint":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: joint NAME VALUE")
        return paused, sim.set_joint_position_targets({arguments[0]: float(arguments[1])}), False
    if command == "joints":
        if len(arguments) != 6:
            raise ValueError("usage: joints J1 J2 J3 J4 J5 J6")
        values = [float(value) for value in arguments]
        return paused, sim.set_joint_position_targets(values), False
    if command == "jog":
        if len(arguments) != 2 or arguments[0] not in ARM_JOINT_NAMES:
            raise ValueError("usage: jog NAME DELTA")
        # 增量基于当前实测位置而非上一个目标：暂停或多次 jog 时不会累积过期目标。
        state = sim.get_state()
        index = ARM_JOINT_NAMES.index(arguments[0])
        target = float(state.joint_positions[index]) + float(arguments[1])
        return paused, sim.set_joint_position_targets({arguments[0]: target}), False
    if command == "gripper":
        if len(arguments) != 1:
            raise ValueError("usage: gripper WIDTH")
        return paused, sim.set_gripper_width(float(arguments[0])), False
    if command == "step":
        if len(arguments) > 1:
            raise ValueError("usage: step [N]")
        count = 1 if not arguments else _positive_int(arguments[0])
        # 暂停时拒绝推进物理：静默多走几步会让"暂停"失去意义，这里显式回话。
        if paused:
            return paused, "paused; step ignored", False
        return paused, sim.step(count), False
    if command == "reset":
        if arguments:
            raise ValueError("usage: reset")
        return paused, sim.reset(), False
    if command == "contacts":
        if arguments:
            raise ValueError("usage: contacts")
        return paused, sim.get_contacts(), False
    if command == "pause":
        if arguments:
            raise ValueError("usage: pause")
        return True, "paused", False
    if command == "resume":
        if arguments:
            raise ValueError("usage: resume")
        return False, "running", False
    raise ValueError(f"unknown command: {command}")


def _run_headless(sim, duration: float | None, steps: int | None):
    """无头定量步进，返回最终状态字典（含请求时长与实际达成时长）。

    执行顺序：先按 ``steps`` 跑指定步数，再按 ``duration`` 补跑到目标仿真时刻；两者
    都给时是"累加"关系而不是二选一。两者都没给时只走一步，保证总有状态可打印。
    ``achieved_duration`` 记录实际达成的时长（未请求时长时为 None），使调用方能区分
    "步数限制先到"与"时长已满足"。
    """
    if steps is not None:
        sim.step(steps)
    requested_duration = duration
    # 以当前仿真时间为基准累计，而不是假设从 0 开始，重复调用时语义才一致。
    duration_start = float(sim.get_state().simulation_time)
    if requested_duration is not None:
        target = duration_start + requested_duration
        # 1e-15 容差用于吸收浮点累加误差，避免永远差最后一步而死循环。
        while float(sim.get_state().simulation_time) + 1e-15 < target:
            sim.step()
    elif steps is None:
        sim.step()
    state = _plain(sim.get_state())
    state["requested_duration"] = requested_duration
    state["achieved_duration"] = (
        float(state["simulation_time"]) - duration_start if requested_duration is not None else None
    )
    return state


def _interactive(sim, stdin, stdout) -> int:
    """交互主循环：逐行读命令、执行、打印结果，返回进程退出码（正常结束为 0）。

    单行命令的类型/取值错误与参数错误都在此捕获并打印 ``error: ...`` 后继续；
    输入流结束或收到 quit 时返回 0。异常信息写 stdout（与正常结果同一流），
    便于交互式终端按顺序阅读。
    """
    paused = False
    for line in stdin:
        try:
            paused, result, should_quit = dispatch_command(sim, line, paused=paused)
            if result is not None:
                _emit(result, stdout)
            if should_quit:
                return 0
        except (argparse.ArgumentTypeError, TypeError, ValueError) as exc:
            print(f"error: {exc}", file=stdout)
    return 0


def main(argv=None, *, sim_factory=RebotArmMujoco, stdin=None, stdout=None, stderr=None) -> int:
    """命令行入口：构造仿真对象并分派到无头或交互模式，返回进程退出码。

    参数：
    - ``argv``：参数列表；None 时取 ``sys.argv[1:]``（便于从控制台脚本调用）；
    - ``sim_factory``：仿真对象工厂，接收 ``--model`` 的值（可能为 None）。可注入
      以便测试替换，默认使用真实仿真后端；
    - ``stdin`` / ``stdout`` / ``stderr``：流注入，None 时使用进程标准流。

    返回 0 表示正常结束；仿真初始化或运行期发生任何异常时打印一行
    ``MuJoCo CLI error: ...`` 到 stderr 并返回 1。无论成功失败都在 finally 中
    ``close()`` 释放仿真资源（未成功创建时跳过），避免遗留查看器窗口或句柄。
    本入口只跑仿真，不会启用任何真实硬件通道。
    """
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    effective_argv = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(effective_argv)
    sim = None
    try:
        sim = sim_factory(args.model)
        if args.headless:
            _emit(_run_headless(sim, args.duration, args.steps), stdout)
            return 0
        return _interactive(sim, stdin, stdout)
    except Exception as exc:
        print(f"MuJoCo CLI error: {exc}", file=stderr)
        return 1
    finally:
        if sim is not None:
            sim.close()


if __name__ == "__main__":
    raise SystemExit(main())
