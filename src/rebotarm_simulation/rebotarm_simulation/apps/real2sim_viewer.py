from __future__ import annotations

import argparse
import importlib
from queue import Empty, SimpleQueue
import threading
import time
from typing import Sequence

from rebotarm_simulation.apps.mujoco_viewer import _close_viewer_then_sim
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
    from rclpy.executors import SingleThreadedExecutor

    args, ros_args = build_parser().parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = executor = viewer = thread = None
    simulation = model = data = None
    quit_events = SimpleQueue()
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
        thread = threading.Thread(
            target=executor.spin,
            name="real2sim-ros-executor",
            daemon=True,
        )
        thread.start()
        started = time.monotonic()
        while viewer.is_running():
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
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=2.0)
        if node is not None and executor is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        if simulation is not None:
            _close_viewer_then_sim(viewer, simulation, model, data)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
