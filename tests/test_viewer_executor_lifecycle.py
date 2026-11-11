from __future__ import annotations

import importlib
import threading
from types import SimpleNamespace

import pytest
import rclpy
import rclpy.executors

from rebotarm_simulation.apps import real2sim_viewer, viewer_lifecycle
from rebotarm_simulation.ros import mujoco_ros_node


class _Executor:
    def __init__(self, *, shutdown_result=True):
        self.finished = threading.Event()
        self.shutdown_result = shutdown_result
        self._executor = SimpleNamespace(_threads=set(), shutdown=lambda **kwargs:None)
        self._shutdown_lock = threading.Lock()
        self._futures = []

    def wake(self):
        pass

    def add_node(self, node):
        pass

    def remove_node(self, node):
        pass

    def spin(self):
        self.worker = threading.current_thread()
        try:
            raise ValueError("physics failed")
        finally:
            self.finished.set()

    def shutdown(self, timeout_sec=None, **kwargs):
        return self.shutdown_result


class _Viewer:
    m = None

    def __init__(self, executor, events):
        self.executor = executor
        self.events = events

    def is_running(self):
        assert self.executor.finished.wait(2.0)
        self.executor.worker.join(2.0)
        return True

    def close(self):
        self.events.append("viewer.close")


def _ros_context(monkeypatch):
    context = SimpleNamespace(ok=True)
    monkeypatch.setattr(rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(rclpy, "ok", lambda: context.ok)
    monkeypatch.setattr(rclpy, "shutdown", lambda: setattr(context, "ok", False))


def test_mujoco_executor_failure_is_reported_after_safe_cleanup(monkeypatch):
    _ros_context(monkeypatch)
    events = []
    executor = _Executor()
    viewer = _Viewer(executor, events)
    node = SimpleNamespace(show_viewer=True, _retain_simulation=False, _closing=threading.Event(),
        viewer_handles=lambda: (object(), object()),
        sync_viewer=lambda _viewer: None,
        close=lambda: events.append("sim.close"),
        destroy_node=lambda: events.append("node.destroy"))
    monkeypatch.setattr(mujoco_ros_node, "create_node_class", lambda: lambda: node)
    monkeypatch.setattr(rclpy.executors, "MultiThreadedExecutor", lambda **kwargs: executor)
    monkeypatch.setattr(importlib, "import_module", lambda name: SimpleNamespace(
        launch_passive=lambda *args: viewer))
    with pytest.raises(RuntimeError, match="executor failed") as failure:
        mujoco_ros_node.main()
    assert isinstance(failure.value.__cause__, ValueError)
    assert events == ["viewer.close", "sim.close", "node.destroy"]


def test_real2sim_executor_failure_is_reported_after_safe_cleanup(monkeypatch):
    _ros_context(monkeypatch)
    events = []
    executor = _Executor()
    viewer = _Viewer(executor, events)
    sim = SimpleNamespace(_unsafe_viewer_handles=lambda: (object(), object()),
                          close=lambda: events.append("sim.close"))
    node = SimpleNamespace(simulation=sim, simulation_lock=threading.Lock(),
                           destroy_node=lambda: events.append("node.destroy"))
    monkeypatch.setattr(rclpy.executors, "SingleThreadedExecutor", lambda: executor)
    with pytest.raises(RuntimeError, match="executor failed") as failure:
        real2sim_viewer.main([], node_factory=lambda: node,
                             launch_passive=lambda *args, **kwargs: viewer)
    assert isinstance(failure.value.__cause__, ValueError)
    assert events == ["viewer.close", "sim.close", "node.destroy"]


def test_executor_timeout_retains_node_and_simulation(monkeypatch):
    _ros_context(monkeypatch)
    events = []
    executor = _Executor(shutdown_result=False)
    viewer = _Viewer(executor, events)
    sim = SimpleNamespace(_unsafe_viewer_handles=lambda: (object(), object()),
                          close=lambda: events.append("sim.close"))
    node = SimpleNamespace(simulation=sim, simulation_lock=threading.Lock(),
                           destroy_node=lambda: events.append("node.destroy"))
    monkeypatch.setattr(rclpy.executors, "SingleThreadedExecutor", lambda: executor)
    before = len(viewer_lifecycle._RETAINED_UNSAFE_VIEWERS)
    with pytest.raises(TimeoutError, match="ROS executor"):
        real2sim_viewer.main([], node_factory=lambda: node,
                             launch_passive=lambda *args, **kwargs: viewer)
    assert events == []
    assert len(viewer_lifecycle._RETAINED_UNSAFE_VIEWERS) == before + 1


def test_worker_timeout_is_not_reported_as_a_clean_shutdown():
    executor = _Executor()
    worker = SimpleNamespace(join=lambda timeout: None, is_alive=lambda: True)
    executor._executor._threads = [worker]
    with pytest.raises(TimeoutError, match="workers did not finish"):
        viewer_lifecycle.stop_executor(executor, None, multi_threaded=True)
