"""viewer lifecycle; ownership and behavior unchanged."""
from __future__ import annotations
import time
import math
from typing import Callable, Sequence
_RETAINED_UNSAFE_VIEWERS = []


def retain_unsafe_resources(viewer, sim, model, data, *owners) -> None:
    """保持仍被原生查看器或执行线程使用的所有权图。"""
    _RETAINED_UNSAFE_VIEWERS.append((viewer, sim, model, data, *owners))


def stop_executor(executor, thread, *, multi_threaded: bool = False) -> None:
    """等待执行器和自旋线程结束，再允许释放其访问的模型。"""
    if executor is None:
        return
    deadline = time.monotonic() + 2.0
    if multi_threaded:
        stopped = executor.shutdown(timeout_sec=2.0, wait_for_threads=False)
    else:
        stopped = executor.shutdown(timeout_sec=2.0)
    if thread is not None:
        thread.join(timeout=max(0.0, deadline - time.monotonic()))
    if not stopped or (thread is not None and thread.is_alive()):
        raise TimeoutError("ROS executor did not finish closing")
    if multi_threaded:
        # Jazzy 的 shutdown 返回值不足以证明线程池中的回调结束；逐个有界等待。
        workers = tuple(executor._executor._threads)
        for worker in workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        if any(worker.is_alive() for worker in workers):
            raise TimeoutError("ROS executor workers did not finish closing")


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
            retain_unsafe_resources(viewer, sim, model, data)
        raise

    try:
        viewer_model = viewer.m
        started = clock() if viewer_model is not None else None
        while viewer_model is not None:
            if clock() - started >= timeout:
                raise TimeoutError("MuJoCo passive viewer did not finish closing")
            sleep(0.01)
            viewer_model = viewer.m
    except BaseException:
        # 无法确认原生线程释放时，保留模型和数据并向调用方传递错误。
        retain_unsafe_resources(viewer, sim, model, data)
        raise
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
