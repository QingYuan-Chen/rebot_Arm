from __future__ import annotations

import argparse
import importlib
from queue import Empty, SimpleQueue
import threading
import time
from typing import Sequence

from rebotarm_simulation.apps.viewer_lifecycle import (
    close_passive_viewer_safely, retain_unsafe_resources, stop_executor,
)
from rebotarm_simulation.ros.real2sim_ros_node import build_node_class


def _positive_float(value: str) -> float:
    result = float(value)
    if result <= 0.0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Show the read-only ROS Real2Sim bridge in MuJoCo Viewer"
    )
    parser.add_argument("--duration", type=_positive_float, default=None)
    return parser


def main(
    argv: Sequence[str] | None = None,
    *,
    node_factory=None,
    launch_passive=None,
) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    args, ros_args = build_parser().parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = executor = viewer = thread = None
    simulation = model = data = None
    quit_events = SimpleQueue()
    worker_errors = []
    try:
        if node_factory is None:
            node_factory = build_node_class()
        node = node_factory()
        simulation = node.simulation
        model, data = simulation._unsafe_viewer_handles()
        if launch_passive is None:
            launch_passive = importlib.import_module("mujoco.viewer").launch_passive

        def on_key(keycode: int) -> None:
            if keycode in (81, 256) or keycode == ord("q"):
                quit_events.put(True)

        viewer = launch_passive(model, data, key_callback=on_key)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        def spin_executor():
            try:
                executor.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            except BaseException as exc:
                worker_errors.append(exc)

        thread = threading.Thread(
            target=spin_executor,
            name="real2sim-ros-executor",
            daemon=True,
        )
        thread.start()
        started = time.monotonic()
        while rclpy.ok() and viewer.is_running():
            if worker_errors:
                raise RuntimeError("real2sim executor failed") from worker_errors[0]
            if not thread.is_alive():
                raise RuntimeError("real2sim executor stopped unexpectedly")
            try:
                quit_events.get_nowait()
                break
            except Empty:
                pass
            with node.simulation_lock:
                viewer.sync()
            if args.duration is not None and time.monotonic() - started >= args.duration:
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            stop_executor(executor, thread)
        except BaseException:
            retain_unsafe_resources(viewer, simulation, model, data, node, executor, thread)
            if rclpy.ok():
                rclpy.shutdown()
            raise
        try:
            if node is not None and executor is not None:
                executor.remove_node(node)
        finally:
            try:
                if simulation is not None:
                    close_passive_viewer_safely(viewer, simulation, model, data)
            finally:
                try:
                    if node is not None:
                        node.destroy_node()
                finally:
                    if rclpy.ok():
                        rclpy.shutdown()
    if worker_errors:
        raise RuntimeError("real2sim executor failed") from worker_errors[0]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
