"""真实 DDS 验证故障结果,迟到查询,故障准入和重建恢复."""

import threading
import time

import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rebotarm_msgs.action import ExecuteTeleopMotion
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rebotarm_simulation.ros.trajectory_action_server import TrajectoryActionServer


@pytest.mark.parametrize('action_type', [FollowJointTrajectory, ExecuteTeleopMotion])
@pytest.mark.parametrize('callback_raises', [False, True])
def test_persistent_abort_failure_reaches_client_and_requires_restart(
        monkeypatch, action_type, callback_raises):
    context = Context()
    context.init(args=[], domain_id=226)
    node = Node('action_failure_probe', context=context)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    handles, failures, aborts = [], [], []
    fault = True
    stopping = threading.Event()
    original_abort = ServerGoalHandle.abort

    def abort(handle, *args, **kwargs):
        aborts.append(handle)
        raise OSError('persistent abort injection')

    def execute(handle):
        handles.append(handle)
        result = action_type.Result()
        if fault:
            if callback_raises:
                raise RuntimeError('execution failure; hold failure')
            result.error_code = -1
            result.error_string = 'execution failure; hold failure'
            try:
                handle.abort()
            except OSError as error:
                result.error_string += f'; {error}'
        else:
            handle.succeed()
            result.error_code = 0
        return result

    def spin():
        try:
            while not stopping.is_set():
                executor.spin_once(timeout_sec=.02)
        except BaseException as error:
            failures.append(error)

    def wait(predicate):
        deadline = time.monotonic() + 4.
        while not predicate():
            assert not failures, failures
            assert time.monotonic() < deadline, 'DDS operation timed out'
            time.sleep(.002)

    server = TrajectoryActionServer(node, action_type, '/failure_probe', execute,
                                    result_timeout=.5)
    client = ActionClient(node, action_type, '/failure_probe')
    executor.add_node(node)
    thread = threading.Thread(target=spin)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=3.)
        monkeypatch.setattr(ServerGoalHandle, 'abort', abort)
        accepted = client.send_goal_async(action_type.Goal())
        wait(accepted.done)
        handle = accepted.result()
        assert handle.accepted
        response = handle.get_result_async()
        wait(response.done)
        result = response.result()
        assert result.status == GoalStatus.STATUS_UNKNOWN
        assert result.result.error_code == -1
        assert 'execution failure; hold failure' in result.result.error_string
        assert 'persistent abort injection' in result.result.error_string
        assert 'previous status=2' in result.result.error_string
        assert len(aborts) == (1 if callback_raises else 2) and handles[0].is_active
        key = bytes(handle.goal_id.uuid)
        assert server._goal_handles[key].status == GoalStatus.STATUS_EXECUTING
        late = handle.get_result_async()
        wait(late.done)
        assert late.result() == result
        monkeypatch.setattr(ServerGoalHandle, 'abort', original_abort)
        fault = False
        successor = client.send_goal_async(action_type.Goal())
        wait(successor.done)
        assert not successor.result().accepted
        client.destroy()
        server.destroy()
        assert not handles[0].is_active
        server = TrajectoryActionServer(node, action_type, '/failure_probe', execute,
                                        result_timeout=.1)
        client = ActionClient(node, action_type, '/failure_probe')
        assert client.wait_for_server(timeout_sec=3.)
        successor = client.send_goal_async(action_type.Goal())
        wait(successor.done)
        assert successor.result().accepted
        response = successor.result().get_result_async()
        wait(response.done)
        assert response.result().status == GoalStatus.STATUS_SUCCEEDED
        assert response.result().result.error_code == 0
        wait(lambda: not server._goal_handles)
        assert not failures
    finally:
        stopping.set()
        thread.join(timeout=3.)
        assert not thread.is_alive()
        executor._executor.shutdown(wait=True)
        executor.shutdown()
        client.destroy()
        server.destroy()
        node.destroy_node()
        context.try_shutdown()
