from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Sequence
from collections import deque

ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))

MOVEIT_ACCEPTANCE_TARGET = (0.04, -0.12, -0.12, 0.06, 0.0, 0.0)


def require_simulation_server(node, executor, action_name, timeout):
    """Fail closed unless the unique action owner identifies as MuJoCo.

    The probe must never infer simulation from an action name shared with hardware.
    """
    from rclpy.action import get_action_server_names_and_types_by_node
    from rcl_interfaces.srv import GetParameters
    owners = []
    for name, namespace in node.get_node_names_and_namespaces():
        servers = get_action_server_names_and_types_by_node(node, name, namespace)
        if any(topic == action_name for topic, _types in servers):
            owners.append((name, namespace))
    if len(owners) != 1:
        raise RuntimeError(f"expected one MuJoCo action owner, found {owners}")
    name, namespace = owners[0]
    if name != "rebotarm_mujoco_node":
        raise RuntimeError(f"refusing non-MuJoCo action owner: {name}")
    path = namespace.rstrip("/") + "/" + name + "/get_parameters"
    client = node.create_client(GetParameters, path)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("cannot verify MuJoCo backend parameters")
        request = GetParameters.Request()
        request.names = ["backend", "headless"]
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("MuJoCo backend verification timed out")
        values = future.result().values
        if len(values) != 2 or values[0].string_value != "mujoco" or not values[1].bool_value:
            raise RuntimeError("action owner is not a verified headless MuJoCo backend")
    finally:
        node.destroy_client(client)


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe a running MoveIt + MuJoCo setup without hardware"
    )
    parser.add_argument("--timeout", type=_positive_float, default=30.0)
    parser.add_argument("--planning-service", default="/plan_kinematic_path")
    parser.add_argument("--action", default="/rebotarm/follow_joint_trajectory")
    parser.add_argument("--joint-states", default="/rebotarm/joint_states")
    parser.add_argument("--clock", default="/clock")
    parser.add_argument("--group", default="arm")
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


def _build_motion_plan_request(
    *,
    service_type,
    joint_names: Sequence[str],
    target_positions: Sequence[float],
    group_name: str,
    start_positions: Sequence[float] | None = None,
):
    from moveit_msgs.msg import Constraints, JointConstraint
    from sensor_msgs.msg import JointState

    request = service_type.Request()
    motion_request = request.motion_plan_request
    motion_request.group_name = group_name
    motion_request.pipeline_id = "ompl"
    motion_request.num_planning_attempts = 3
    motion_request.allowed_planning_time = 5.0
    motion_request.max_velocity_scaling_factor = 0.1
    motion_request.max_acceleration_scaling_factor = 0.1
    if start_positions is not None:
        start_state = JointState()
        start_state.name = list(joint_names)
        start_state.position = [
            0.0 if abs(float(value)) < 1e-9 else float(value)
            for value in start_positions
        ]
        motion_request.start_state.joint_state = start_state
    motion_request.start_state.is_diff = start_positions is None
    constraints = Constraints()
    for name, position in zip(joint_names, target_positions):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = str(name)
        joint_constraint.position = float(position)
        joint_constraint.tolerance_above = 0.01
        joint_constraint.tolerance_below = 0.01
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    motion_request.goal_constraints = [constraints]
    return request


def run_acceptance(
    *,
    timeout: float = 30.0,
    planning_service: str = "/plan_kinematic_path",
    action_name: str = "/rebotarm/follow_joint_trajectory",
    joint_states_topic: str = "/rebotarm/joint_states",
    clock_topic: str = "/clock",
    group_name: str = "arm",
) -> dict:
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    started_here = False
    if not rclpy.ok():
        rclpy.init()
        started_here = True

    node = None
    executor = None
    try:
        node = Node("rebotarm_mujoco_moveit_acceptance_probe")
        # This probe only receives state and asynchronous service/action replies.
        # Keep callbacks on the polling thread so shutdown cannot leave queued
        # worker callbacks accessing a node that is already being destroyed.
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        joint_messages = deque(maxlen=1000)
        clock_messages = deque(maxlen=1000)
        node.create_subscription(
            JointState,
            joint_states_topic,
            lambda msg: joint_messages.append(msg),
            10,
        )
        node.create_subscription(
            Clock,
            clock_topic,
            lambda msg: clock_messages.append(msg),
            10,
        )
        planner = node.create_client(GetMotionPlan, planning_service)
        action = ActionClient(node, FollowJointTrajectory, action_name)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and (
            len(joint_messages) < 2 or len(clock_messages) < 2
        ):
            executor.spin_once(timeout_sec=0.05)
        remaining = max(0.1, deadline - time.monotonic())
        planning_ready = bool(planner.wait_for_service(timeout_sec=remaining))
        remaining = max(0.1, deadline - time.monotonic())
        action_ready = bool(action.wait_for_server(timeout_sec=remaining))
        if not (action_ready and len(joint_messages) >= 2 and len(clock_messages) >= 2):
            raise RuntimeError("fresh simulation state, clock and action are required")
        require_simulation_server(node, executor, action_name, timeout)
        deadline = time.monotonic() + timeout

        plan_success = False
        plan_error_code = None
        planned_points = 0
        trajectory = None
        if planning_ready:
            latest_positions_by_name = {
                name: float(position)
                for name, position in zip(joint_messages[-1].name, joint_messages[-1].position)
            } if joint_messages else {}
            start_positions = tuple(
                latest_positions_by_name.get(name, 0.0) for name in ARM_JOINT_NAMES
            )
            request = _build_motion_plan_request(
                service_type=GetMotionPlan,
                joint_names=ARM_JOINT_NAMES,
                target_positions=MOVEIT_ACCEPTANCE_TARGET,
                group_name=group_name,
                start_positions=start_positions,
            )
            future = planner.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if future.done():
                response = future.result()
                plan_error_code = int(response.motion_plan_response.error_code.val)
                trajectory = response.motion_plan_response.trajectory.joint_trajectory
                planned_points = len(trajectory.points)
                plan_success = plan_error_code == 1 and planned_points > 0

        action_accepted = False
        action_success = False
        action_error_code = None
        if action_ready and plan_success:
            require_simulation_server(node, executor, action_name, timeout)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            future = action.send_goal_async(goal)
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            if future.done():
                goal_handle = future.result()
                action_accepted = bool(goal_handle.accepted)
                if action_accepted:
                    result_future = goal_handle.get_result_async()
                    while not result_future.done() and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.05)
                    if result_future.done():
                        result = result_future.result().result
                        action_error_code = int(result.error_code)
                        action_success = action_error_code == int(
                            FollowJointTrajectory.Result.SUCCESSFUL
                        )
                    else:
                        cancel = goal_handle.cancel_goal_async()
                        cancel_deadline = time.monotonic() + 3.0
                        while not cancel.done() and time.monotonic() < cancel_deadline:
                            executor.spin_once(timeout_sec=0.05)

        post_deadline = time.monotonic() + min(2.0, timeout)
        while time.monotonic() < post_deadline:
            executor.spin_once(timeout_sec=0.05)

        clock_progress = (
            len(clock_messages) >= 2
            and _stamp_seconds(clock_messages[-1].clock)
            > _stamp_seconds(clock_messages[0].clock)
        )
        final_joint = joint_messages[-1] if joint_messages else None
        final_error = None
        if final_joint is not None and len(final_joint.position) >= 6:
            by_name = {
                name: float(position)
                for name, position in zip(final_joint.name, final_joint.position)
            }
            if all(name in by_name for name in ARM_JOINT_NAMES):
                final_error = max(
                    abs(by_name[name] - target)
                    for name, target in zip(ARM_JOINT_NAMES, MOVEIT_ACCEPTANCE_TARGET)
                )
        ok = bool(
            len(joint_messages) >= 2
            and clock_progress
            and planning_ready
            and action_ready
            and plan_success
            and action_accepted
            and action_success
            and final_error is not None
            and final_error <= 0.05
        )
        return {
            "ok": ok,
            "joint_state_count": len(joint_messages),
            "clock_count": len(clock_messages),
            "clock_progress": clock_progress,
            "planning_service_ready": planning_ready,
            "moveit_plan_success": plan_success,
            "moveit_plan_error_code": plan_error_code,
            "moveit_planned_points": planned_points,
            "trajectory_action_ready": action_ready,
            "trajectory_action_accepted": action_accepted,
            "trajectory_action_success": action_success,
            "trajectory_error_code": action_error_code,
            "final_max_joint_error_rad": final_error,
        }
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if started_here and rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None, *, stdout=None, stderr=None) -> int:
    args = build_parser().parse_args(argv)
    if stdout is None:
        stdout = sys.stdout
    if stderr is None:
        stderr = sys.stderr
    try:
        payload = run_acceptance(
            timeout=args.timeout,
            planning_service=args.planning_service,
            action_name=args.action,
            joint_states_topic=args.joint_states,
            clock_topic=args.clock,
            group_name=args.group,
        )
    except Exception as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=stderr)
        return 1
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if payload["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
