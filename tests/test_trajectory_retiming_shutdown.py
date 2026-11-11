"""使用真实 Jazzy 执行器复现排队回调与唤醒句柄销毁的竞争."""

import threading
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from rebotarm_motion import trajectory_retiming_node


@pytest.mark.parametrize('owner', ['retiming', 'simulation'])
def test_shutdown_drains_queued_callback_before_destroying_guard(monkeypatch, owner):
    context = Context()
    context.init(args=[], domain_id=226)
    node = Node('retiming_shutdown_test', context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    release = threading.Event()
    started = threading.Barrier(3)

    def occupy_worker():
        started.wait(timeout=2.)
        assert release.wait(2.)

    try:
        executor.add_node(node)
        subscription = node.create_subscription(String, 'shutdown_probe', lambda msg: None, 1)
        for _ in range(2):
            executor._executor.submit(occupy_worker)
        started.wait(timeout=2.)
        task = executor._make_handler(subscription, node, executor._take_subscription)
        executor._executor.submit(task)
        executor._futures.append(task)
        pool_shutdown = executor._executor.shutdown

        def drain(*args, **kwargs):
            release.set()
            return pool_shutdown(*args, **kwargs)

        monkeypatch.setattr(executor._executor, 'shutdown', drain)
        if owner == 'retiming':
            trajectory_retiming_node._shutdown_executor(executor)
        else:
            from rebotarm_simulation.apps.viewer_lifecycle import stop_executor
            stop_executor(executor, None, multi_threaded=True)
        assert task.done() and task.exception() is None
    finally:
        release.set()
        executor.shutdown()
        node.destroy_node()
        context.try_shutdown()


def test_shutdown_propagates_finished_callback_failure():
    from rclpy.task import Future
    from types import SimpleNamespace

    failure = Future()
    failure.set_exception(ValueError('retiming callback failed'))
    events = []
    executor = SimpleNamespace(
        _executor=SimpleNamespace(shutdown=lambda **kwargs: events.append('workers')),
        shutdown=lambda: events.append('handles'), _futures=[failure])
    with pytest.raises(ExceptionGroup, match='retiming executor callbacks') as caught:
        trajectory_retiming_node._shutdown_executor(executor)
    assert events == ['workers', 'handles']
    assert str(caught.value.exceptions[0]) == 'retiming callback failed'


@pytest.mark.parametrize('shutdown_signal,busy', [
    (signal.SIGINT, False), (signal.SIGTERM, False),
    (signal.SIGINT, True), (signal.SIGTERM, True),
])
def test_retiming_process_signal_keeps_inflight_response_valid(tmp_path, shutdown_signal, busy):
    from rebotarm_msgs.srv import RetimeArmTrajectory
    from rclpy.executors import SingleThreadedExecutor

    # 子进程使用正式 main;仅在测试中暂停服务回调,精确把信号落在回复前.
    code = '''
from pathlib import Path
import sys, time
from rclpy.node import Node
from rebotarm_motion.trajectory_retiming_node import main
directory = Path(sys.argv[1])
original = Node.create_service
def create_service(self, kind, name, callback, *args, **kwargs):
    if name.endswith('/retime_arm_trajectory'):
        def paused(request, response):
            (directory/'entered').touch()
            deadline = time.monotonic()+3.
            while not (directory/'release').exists():
                if time.monotonic() > deadline:
                    raise TimeoutError('test did not release callback')
                time.sleep(.002)
            return callback(request, response)
        return original(self, kind, name, paused, *args, **kwargs)
    return original(self, kind, name, callback, *args, **kwargs)
Node.create_service = create_service
main(args=[])
'''
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='226', ROS_LOCALHOST_ONLY='1')
    source = Path(__file__).resolve().parents[1]/'src/rebotarm_motion'
    env['PYTHONPATH'] = str(source)+os.pathsep+env.get('PYTHONPATH', '')
    context = Context()
    context.init(args=[], domain_id=226)
    node = Node('retiming_signal_client', context=context)
    executor = SingleThreadedExecutor(context=context)
    client = node.create_client(RetimeArmTrajectory, '/rebotarm/retime_arm_trajectory')
    with (tmp_path/'process.log').open('w') as output:
        process = subprocess.Popen([sys.executable, '-c', code, str(tmp_path)], env=env,
                                   stdout=output, stderr=subprocess.STDOUT)
        try:
            assert client.wait_for_service(timeout_sec=4.)
            if busy:
                response = client.call_async(RetimeArmTrajectory.Request())
                deadline = time.monotonic()+3.
                while not (tmp_path/'entered').exists():
                    assert time.monotonic() < deadline, 'service callback did not start'
                    time.sleep(.002)
            process.send_signal(shutdown_signal)
            if busy:
                # 给主线程进入排空等待的机会;回调回复仍须使用有效的 ROS context.
                time.sleep(.15)
                assert process.poll() is None
                (tmp_path/'release').touch()
                rclpy.spin_until_future_complete(node, response, executor=executor, timeout_sec=4.)
                assert response.done() and not response.result().success
                assert 'fresh trajectory capability unavailable' in response.result().message
            assert process.wait(timeout=4.) == 0
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3.)
            executor.shutdown()
            node.destroy_node()
            context.try_shutdown()
    output = (tmp_path/'process.log').read_text()
    assert 'Destroyable' not in output and 'never retrieved' not in output
    assert 'Traceback' not in output
