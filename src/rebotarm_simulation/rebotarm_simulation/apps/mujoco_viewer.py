"""MuJoCo 被动查看器（离线键盘调试工具）。

职责：在弹出的 MuJoCo 被动查看器窗口中手动驱动仿真机械臂与夹爪，用于离线
核对模型、控制器与夹爪行为。本模块不触碰真实电机、不开启硬件通道，也不
发布/订阅任何 ROS 话题，因此可以安全地在无硬件环境下单独运行。

线程模型：查看器内部线程在按键时回调 `on_key`，仅把键码放入 FIFO 队列；
仿真线程每周期取出一个"有限快照"顺序处理，从而避免跨线程直接改动 MuJoCo
原生状态。查看器渲染只通过 `viewer.sync()` 与仿真线程同步。

控制模式（`active_mode`）：
- `pos_vel`：关节位置/速度闭环，默认模式，用于普通点动；
- `gravity_comp`：重力补偿，重置/复位后进入，便于手动拖动示教；
- `hold`：保持当前关节角，切换时会把目标同步为当前角度。

安全要点：退出时必须确认查看器线程已释放 MuJoCo 原生句柄后才能关闭仿真；
若无法确认，则保留完整所有权图（见 `_RETAINED_UNSAFE_VIEWERS`）并抛出异常，
绝不冒险释放，以免查看器仍在使用已释放内存时崩溃进程。
"""

from __future__ import annotations

import argparse
import importlib
import math
import sys
import time
from dataclasses import replace
from queue import SimpleQueue
from typing import Callable, Sequence

from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
from rebotarm_simulation.apps.viewer_interaction import _state_from_sim, overlay_text, process_key_events, apply_continuous_jog
from rebotarm_simulation.apps.viewer_lifecycle import close_passive_viewer_safely

def _positive_float(value: str) -> float:
    # argparse 类型校验：步长/速率/时长/保持时间都必须为有限正数；
    # 0 或负值会让点动无意义或把计时逻辑推向非法分支，NaN/Inf 会污染控制器目标。
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    """构造独立运行时的命令行解析器。

    所有物理量均为正数：角度用弧度、宽度用米、速率按秒计。`--duration` 以
    "仿真时间"（非墙钟时间）计时，便于在无显示环境下自动限时运行。
    """
    parser = argparse.ArgumentParser(description="Control reBotArm in the MuJoCo viewer")
    # 未指定时由仿真对象使用其默认场景文件，避免此处硬编码模型路径。
    parser.add_argument("--model", default=None, help="MuJoCo scene XML path")
    # 单步点动增量：关节 0.01 rad，夹爪 0.001 m。
    parser.add_argument("--joint-step", type=_positive_float, default=0.01, help="joint jog in radians")
    parser.add_argument("--gripper-step", type=_positive_float, default=0.001, help="gripper jog in metres")
    # 按住按键时的连续点动速率。
    parser.add_argument("--joint-rate", type=_positive_float, default=0.08, help="held joint jog rate in rad/s")
    parser.add_argument("--gripper-rate", type=_positive_float, default=0.01, help="held gripper jog rate in m/s")
    # 键盘自动重复有间隔，靠这段"保持时间"把离散按键事件串成连续运动；
    # 调大更顺滑但松手后余量更长，调小更跟手但可能断续。
    parser.add_argument(
        "--jog-hold-time",
        type=_positive_float,
        default=0.18,
        help="seconds to keep jogging after the latest key-repeat event",
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=None,
        help="exit after this many seconds of simulated time",
    )
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    sim_factory: Callable = RebotArmMujoco,
    launch_passive: Callable | None = None,
    sleep: Callable[[float], None] = time.sleep,
    clock: Callable[[], float] = time.monotonic,
    status_stream=None,
) -> int:
    """运行键盘控制循环，返回进程退出码。

    参数：
    - `argv`：命令行参数（None 表示读取 `sys.argv`）；
    - `sim_factory`：仿真对象工厂，接收模型路径并返回仿真实例，便于测试替换；
    - `launch_passive`：被动查看器启动函数，None 时延迟导入 MuJoCo 的查看器模块
      （这样无显示环境下只做其它操作时不会强依赖查看器）；
    - `sleep`/`clock`/`status_stream`：时间与输出注入点，默认真实睡眠/单调时钟/stderr。

    返回值：正常结束 0，`KeyboardInterrupt`（Ctrl-C）返回 130。
    副作用：启动后先把控制模式置为 `gravity_comp`，随后每周期处理按键、推进物理、
    变化时打印状态并 `sync()` 渲染；`--duration` 到达后自动退出。
    """
    args = build_parser().parse_args(argv)
    if status_stream is None:
        status_stream = sys.stderr
    sim = sim_factory(args.model)
    viewer = None
    model = data = None
    try:
        sim.reset()
        # 启动即进入重力补偿：这是最安全的初始状态，控制器不会主动驱动关节。
        if hasattr(sim, "set_control_mode"):
            sim.set_control_mode("gravity_comp")
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        state = _state_from_sim(sim)
        # 以仿真时间作为 `--duration` 的计时基准，与物理推进严格对应。
        start_simulation_time = float(sim.get_state().simulation_time)
        events = SimpleQueue()
        previous_status = overlay_text(state)
        print(previous_status, file=status_stream, flush=True)

        def on_key(keycode: int) -> None:
            # 查看器线程回调：只入队，不做任何 MuJoCo 操作，保证线程安全。
            events.put(keycode)

        # 取出底层原生句柄交给查看器；生命周期由本函数负责，查看器关闭后才释放。
        model, data = sim.borrow_viewer_handles()
        viewer = launch_passive(
            model,
            data,
            key_callback=on_key,
        )
        try:
            while viewer.is_running():
                state = process_key_events(
                    sim,
                    events,
                    state,
                    args.joint_step,
                    args.gripper_step,
                    args.jog_hold_time,
                )
                if state.quit:
                    break
                cycle_start = clock()
                # 暂停时跳过物理步进；单步请求（single_step）时仍执行一步。
                if not state.paused or state.single_step:
                    state = apply_continuous_jog(
                        sim,
                        state,
                        dt=sim.timestep,
                        joint_rate=args.joint_rate,
                        gripper_rate=args.gripper_rate,
                    )
                    sim.step()
                    sim_state = sim.get_state()
                    state = replace(
                        state,
                        joint_positions=tuple(float(value) for value in sim_state.joint_positions[:6]),
                        single_step=False,
                    )
                current_status = overlay_text(state)
                # 仅在文本变化时输出，避免每周期刷屏拖慢主循环。
                if current_status != previous_status:
                    print(current_status, file=status_stream, flush=True)
                    previous_status = current_status
                viewer.sync()
                elapsed = float(sim.get_state().simulation_time) - start_simulation_time
                if args.duration is not None and elapsed >= args.duration:
                    break
                # 按仿真步长节流为实时速度；若本周期已超时则不再睡眠（不补偿欠账）。
                sleep(max(0.0, sim.timestep - (clock() - cycle_start)))
            return 0
        except KeyboardInterrupt:
            # 约定俗成的 SIGINT 退出码。
            return 130
    finally:
        # 无论正常退出还是异常，都在确认查看器放手后释放仿真资源。
        close_passive_viewer_safely(
            viewer,
            sim,
            model,
            data,
            clock=clock,
            sleep=sleep,
        )

if __name__ == "__main__":
    raise SystemExit(main())
