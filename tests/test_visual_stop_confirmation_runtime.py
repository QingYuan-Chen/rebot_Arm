"""Isolated ROS stop confirmation; no reset, homing, camera or hardware."""
from pathlib import Path
import os
import subprocess
import sys
import pytest


def test_stop_confirmation_holds_and_allows_next_execution():
    import rclpy
    if not getattr(rclpy, '__file__', None):
        pytest.skip('ROS runtime is not installed')
    result = subprocess.run([sys.executable, str(Path(__file__).resolve())],
        env=dict(os.environ, ROS_DOMAIN_ID='211', ROS_LOCALHOST_ONLY='1'),
        capture_output=True, text=True, timeout=55)
    assert result.returncode == 0, result.stdout + result.stderr
    print(result.stdout)


def run_probe():
    import threading
    import time
    from types import SimpleNamespace
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from rebotarm_msgs.srv import ExecutePose
    from rebotarm_msgs.msg import GraspPlan
    from rebotarm_motion.pose_execution_node import PoseExecutionNode
    from rebotarm_simulation.sim_trajectory_controller_node import SimTrajectoryControllerNode
    from rebotarm_vision.nodes.visual_grasp_executor_node import VisualGraspExecutorNode
    from rebotarm_vision.visual_grasp_runtime import VisualGraspRuntime

    rclpy.init(args=['--ros-args', '-p', 'arm_namespace:=stop_probe',
                     '-p', 'use_hardware:=false', '-p', 'stop_confirmation_timeout_sec:=1.5'])
    sim, motion, visual = SimTrajectoryControllerNode(), PoseExecutionNode(), VisualGraspExecutorNode()
    probe = Node('stop_probe_client')
    executor = MultiThreadedExecutor(num_threads=10)
    for node in (sim, motion, visual, probe):
        executor.add_node(node)
    samples = []
    probe.create_subscription(JointState, '/stop_probe/joint_states',
        lambda m: samples.append((time.monotonic(), list(m.position[:6]))), 100)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def result(future, timeout=20):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done(), 'future did not complete'
        return future.result()

    def client(service, name):
        c = probe.create_client(service, '/stop_probe/' + name)
        assert c.wait_for_service(timeout_sec=3)
        return c
    stop = client(Trigger, 'visual_grasp/stop')
    execute = client(Trigger, 'visual_grasp/execute')
    pose = client(ExecutePose, 'motion_execution/execute_pose')
    mode = ['normal']
    planned = threading.Event()
    dispatches = []
    original_send = motion._trajectory_client.send_goal_async
    def send(goal):
        dispatches.append(time.monotonic())
        return original_send(goal)
    motion._trajectory_client.send_goal_async = send
    def plan(*args, **kwargs):
        planned.set()
        if mode[0] == 'slow_plan':
            time.sleep(.6)
        trajectory = JointTrajectory()
        trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
        start = JointTrajectoryPoint(positions=list(samples[-1][1]))
        start.time_from_start.nanosec = 1_000_000
        end = JointTrajectoryPoint(positions=[.6, -.4, -.5, .5, .2, .1])
        end.time_from_start.sec = 2
        trajectory.points = [start, end]
        return SimpleNamespace(success=True, trajectory=trajectory, message='probe plan')
    motion._planner = SimpleNamespace(plan_pose=plan)
    # Exercise the real visual service ownership/gateway using one controlled
    # motion stage, independent of perception/MoveIt availability in CI.
    def run(runtime, request, response, **kwargs):
        req = ExecutePose.Request()
        req.execute = True
        req.timeout_sec = 3.
        reply, error = visual._io_gateway.call('execute_pose', req, 5.)
        response.success = bool(reply is not None and reply.success)
        response.message = error or reply.message
        return response
    VisualGraspRuntime.execute = run
    def fresh():
        plan = GraspPlan()
        plan.valid = True
        plan.header.stamp = visual.get_clock().now().to_msg()
        visual._on_plan(plan)

    try:
        deadline = time.monotonic()+3
        while not samples and time.monotonic()<deadline:
            time.sleep(.02)
        assert samples
        assert all('/visual_grasp/reset' not in n for n,_ in visual.get_service_names_and_types())
        for cycle in range(2):
            fresh()
            pending = execute.call_async(Trigger.Request())
            time.sleep(.7)
            stopped_at = samples[-1][1]
            before = time.monotonic()
            reply = result(stop.call_async(Trigger.Request()))
            assert reply.success, reply.message
            assert not result(pending).success
            time.sleep(.15)
            after = [q for t,q in samples if t > before+.15]
            assert after
            span = max(max(q[j] for q in after)-min(q[j] for q in after) for j in range(6))
            assert span < .002, span
            assert abs(after[-1][0]) > .05  # No automatic safe_home.
            assert visual._state.phase == 'IDLE'
            assert visual._state.latest_plan is None
            print(f'cycle={cycle+1} stop_sec={time.monotonic()-before:.3f} hold_span={span:.6f}', flush=True)
        # Stop while planning: the late plan cannot dispatch an action.
        count = len(dispatches)
        mode[0] = 'slow_plan'
        planned.clear()
        fresh()
        pending = execute.call_async(Trigger.Request())
        assert planned.wait(2)
        reply = result(stop.call_async(Trigger.Request()))
        assert reply.success, reply.message
        assert not result(pending).success
        assert len(dispatches) == count
        print('stop during planning: no late action dispatched', flush=True)
        # A stale sensor stream cannot reopen either layer, even with an idle action.
        timers = list(sim.timers)
        for timer in timers:
            timer.cancel()
        time.sleep(.6)
        reply = result(stop.call_async(Trigger.Request()))
        assert not reply.success
        fresh()
        assert not result(execute.call_async(Trigger.Request())).success
        assert motion._stopping and visual._state.plans_blocked
        for timer in timers:
            timer.reset()
        # Explicit stop retry obtains new evidence; there is no reset bypass.
        reply = result(stop.call_async(Trigger.Request()))
        assert reply.success, reply.message
        mode[0] = 'normal'
        fresh()
        assert result(execute.call_async(Trigger.Request())).success
        print('stale feedback blocks execution; stop retry and next execute: passed', flush=True)
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        for node in (sim, motion, visual, probe):
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_probe()
