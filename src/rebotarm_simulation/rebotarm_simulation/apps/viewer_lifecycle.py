"""viewer lifecycle; ownership and behavior unchanged."""
from __future__ import annotations
import time
import math
from typing import Callable, Sequence
_RETAINED_UNSAFE_VIEWERS = []


def _close_viewer_then_sim(
    viewer,
    sim,
    model,
    data,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    timeout: float = 5.0,
) -> None:
    """只有在被动查看器线程释放原生状态之后，才释放仿真资源。

    `viewer.m` 是查看器公开的生命周期信号：它变为 None 表示查看器已不再持有
    MuJoCo 模型。在确认之前释放 MjModel/MjData 属于 use-after-free，会直接让
    进程崩溃，因此这里宁可等待或保留引用，也不提前释放。

    `clock`/`sleep`/`timeout` 均可注入，便于测试中不真正睡眠。超时（默认 5 s）
    后抛出 `TimeoutError`，同时把整套所有权图登记到 `_RETAINED_UNSAFE_VIEWERS`
    以保持引用存活。
    """
    if viewer is None:
        # 查看器未启动（例如构造阶段就失败）：没有共享句柄，直接关闭仿真。
        sim.close()
        return
    try:
        viewer.close()
    except BaseException:
        # 关闭本身抛错时，只能依据 `viewer.m` 判断查看器是否已放手；
        # 若连这个公开信号都取不到，释放原生状态就是危险的猜测。
        model_is_released = False
        try:
            model_is_released = viewer.m is None
        except BaseException:
            # 若这个公开的生命周期信号本身不可用，释放原生状态就只是危险的猜测。
            pass
        if model_is_released:
            sim.close()
        else:
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
        raise

    viewer_model = getattr(viewer, "m", None)
    if viewer_model is None:
        sim.close()
        return
    started = clock()
    while viewer_model is not None:
        if clock() - started >= timeout:
            # 查看器仍暴露模型时释放 MjModel/MjData 会让进程崩溃。保留整套所有权图，
            # 并把清理失败上报出去，而不是冒险造成 use-after-free。
            _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data))
            raise TimeoutError("MuJoCo passive viewer did not finish closing")
        sleep(0.01)
        viewer_model = getattr(viewer, "m", None)
    sim.close()


def close_passive_viewer_safely(
    viewer,
    sim,
    model,
    data,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    timeout: float = 5.0,
) -> None:
    """Close a passive viewer before releasing its MuJoCo resources.

    This public wrapper keeps ownership-retention details inside the simulation
    package so callers do not depend on ``_RETAINED_UNSAFE_VIEWERS``.
    """
    _close_viewer_then_sim(
        viewer,
        sim,
        model,
        data,
        clock=clock,
        sleep=sleep,
        timeout=timeout,
    )
