from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from typing import Sequence


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
ACCEPTANCE_TARGET = (0.05, -0.10, -0.10, 0.05, 0.0, 0.0)


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run in-process ROS 2 acceptance for the MuJoCo backend")
    parser.add_argument("--timeout", type=_positive_float, default=15.0)
    parser.add_argument("--gripper-width", type=float, default=0.05)
    return parser


def _wait_until(predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return bool(predicate())


def _wait_future(future, timeout: float) -> bool:
    return _wait_until(lambda: future.done(), timeout)


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def run_acceptance(*, timeout: float = 15.0, gripper_width: float = 0.05) -> dict:
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rebotarm_msgs.srv import SetGripper
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    from rebotarm_simulation.ros.mujoco_ros_node import create_node_class

    if not math.isfinite(gripper_width):
        raise ValueError("gripper_width must be finite")

    started_here = False
    if not rclpy.ok():
        rclpy.init()
        started_here = True

    sim_node = None
    probe = None
    executor = None
    thread = None
    joint_messages: list[JointState] = []
    clock_messages: list[Clock] = []
    try:
        sim_node = create_node_class()()
        probe = Node("rebotarm_mujoco_acceptance_probe")
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(sim_node)
        executor.add_node(probe)
        probe.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            lambda msg: joint_messages.append(msg),
            10,
        )
        probe.create_subscription(
            Clock,
            "/clock",
            lambda msg: clock_messages.append(msg),
            10,
        )
        gripper_client = probe.create_client(SetGripper, "/rebotarm/gripper/set")
        action_client = ActionClient(
            probe,
            FollowJointTrajectory,
            "/rebotarm/follow_joint_trajectory",
        )

        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        topics_ready = _wait_until(
            lambda: len(joint_messages) >= 2 and len(clock_messages) >= 2,
            timeout,
        )
        service_ready = bool(gripper_client.wait_for_service(timeout_sec=timeout))
        action_ready = bool(action_client.wait_for_server(timeout_sec=timeout))

        service_success = False
        gripper_reached = None
        if service_ready:
            request = SetGripper.Request()
            request.position = float(gripper_width)
            response_future = gripper_client.call_async(request)
            if _wait_future(response_future, timeout):
                response = response_future.result()
                service_success = bool(response.success)
                gripper_reached = float(response.reached_position)

        action_accepted = False
        action_success = False
        action_error_code = None
        if action_ready:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
            point = JointTrajectoryPoint()
            point.positions = list(ACCEPTANCE_TARGET)
            point.time_from_start = Duration(sec=1, nanosec=0)
            goal.trajectory.points = [point]
            send_future = action_client.send_goal_async(goal)
            if _wait_future(send_future, timeout):
                goal_handle = send_future.result()
                action_accepted = bool(goal_handle.accepted)
                if action_accepted:
                    result_future = goal_handle.get_result_async()
                    if _wait_future(result_future, timeout):
                        result = result_future.result().result
                        action_error_code = int(result.error_code)
                        action_success = action_error_code == int(
                            FollowJointTrajectory.Result.SUCCESSFUL
                        )

        final_joint = joint_messages[-1] if joint_messages else None
        final_clock = clock_messages[-1] if clock_messages else None
        clock_progress = (
            len(clock_messages) >= 2
            and _stamp_seconds(clock_messages[-1].clock) > _stamp_seconds(clock_messages[0].clock)
        )
        joint_schema_ok = (
            final_joint is not None
            and tuple(final_joint.name) == ARM_JOINT_NAMES + ("left_finger_joint", "right_finger_joint")
            and len(final_joint.position) == 8
            and len(final_joint.velocity) == 8
            and len(final_joint.effort) == 8
            and all(math.isfinite(value) for value in final_joint.position)
        )
        final_error = None
        if final_joint is not None and len(final_joint.position) >= 6:
            final_error = max(
                abs(float(actual) - target)
                for actual, target in zip(final_joint.position[:6], ACCEPTANCE_TARGET)
            )
        ok = bool(
            topics_ready
            and clock_progress
            and joint_schema_ok
            and service_success
            and action_accepted
            and action_success
            and final_error is not None
            and final_error <= 0.04
        )
        return {
            "ok": ok,
            "topics_ready": topics_ready,
            "joint_state_count": len(joint_messages),
            "clock_count": len(clock_messages),
            "clock_progress": clock_progress,
            "joint_schema_ok": joint_schema_ok,
            "gripper_service_ready": service_ready,
            "gripper_service_success": service_success,
            "gripper_reached_position_m": gripper_reached,
            "trajectory_action_ready": action_ready,
            "trajectory_action_accepted": action_accepted,
            "trajectory_action_success": action_success,
            "trajectory_error_code": action_error_code,
            "final_max_joint_error_rad": final_error,
            "final_clock_sec": _stamp_seconds(final_clock.clock) if final_clock is not None else None,
        }
    finally:
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=2.0)
        if probe is not None:
            probe.destroy_node()
        if sim_node is not None:
            sim_node.destroy_node()
        if started_here and rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None, *, stdout=None, stderr=None) -> int:
    args = build_parser().parse_args(argv)
    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    try:
        payload = run_acceptance(timeout=args.timeout, gripper_width=args.gripper_width)
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
