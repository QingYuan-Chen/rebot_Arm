"""连接已启动的隔离 MuJoCo/MoveIt 图，验证连续轨迹及夹爪公共接口。

启动组合拥有节点生命周期；本工具只创建探针，并在唯一仿真执行服务端上
完成重定时、规划绑定、六轴 FJT 和夹爪测试。独立非零 ROS 域为必需条件。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Sequence


ARM_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))


def _positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Verify an existing isolated MuJoCo/MoveIt ROS 2 graph")
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
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('use an isolated nonzero ROS_DOMAIN_ID for simulation acceptance')
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rebotarm_msgs.srv import SetGripper, GetArmPlanningState, RetimeArmTrajectory
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint

    if not math.isfinite(gripper_width):
        raise ValueError("gripper_width must be finite")

    started_here = False
    if not rclpy.ok():
        rclpy.init()
        started_here = True

    probe = None
    executor = None
    thread = None
    joint_messages: list[JointState] = []
    clock_messages: list[Clock] = []
    try:
        probe = Node("rebotarm_mujoco_acceptance_probe")
        executor = MultiThreadedExecutor(num_threads=4)
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
        state_client = probe.create_client(GetArmPlanningState, '/rebotarm/get_arm_planning_state')
        retime_client = probe.create_client(RetimeArmTrajectory, '/rebotarm/retime_arm_trajectory')
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
        owners = [(name, namespace) for name, namespace in probe.get_node_names_and_namespaces()
            for action, _types in get_action_server_names_and_types_by_node(probe, name, namespace)
            if action == '/rebotarm/follow_joint_trajectory']
        if owners != [('rebotarm_mujoco_node', '/')]:
            raise RuntimeError(f'unique MuJoCo FJT owner required: {owners}')
        if probe.count_publishers('/rebotarm/joint_states') != 1:
            raise RuntimeError('unique simulation joint state publisher required')
        if not topics_ready or not state_client.wait_for_service(timeout_sec=timeout) or not retime_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('simulation feedback and continuous planning services required')

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
        target_positions = None
        if action_ready:
            # 夹爪动作后等待新物理帧满足准备准入，再获取规划起点。
            deadline = time.monotonic()+timeout
            while True:
                state_future = state_client.call_async(GetArmPlanningState.Request())
                if not _wait_future(state_future, max(0., deadline-time.monotonic())):
                    raise TimeoutError('planning state timeout')
                state_response = state_future.result()
                if state_response.success:
                    snapshot = json.loads(state_response.state_json)
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(state_response.message)
                time.sleep(.02)
            if not snapshot['executable']:
                raise RuntimeError('simulation planning state is not executable')
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM_JOINT_NAMES)
            target_positions = list(snapshot['hold'])
            target_positions[0] += .02
            goal.trajectory.points = [JointTrajectoryPoint(positions=list(q),
                velocities=[0.]*6, accelerations=[0.]*6, time_from_start=Duration(sec=stamp))
                for stamp, q in ((0,snapshot['hold']),(1,target_positions))]
            request = RetimeArmTrajectory.Request(trajectory=goal.trajectory,
                velocity_scaling=1., acceleration_scaling=1.,
                capability_revision=snapshot['capability_revision'])
            planned = retime_client.call_async(request)
            if not _wait_future(planned, timeout):
                raise TimeoutError('continuous planning timeout')
            plan = planned.result()
            if not plan.success or not plan.executable:
                raise RuntimeError(plan.message)
            goal.trajectory = plan.trajectory
            send_future = action_client.send_goal_async(goal)
            if _wait_future(send_future, timeout):
                goal_handle = send_future.result()
                action_accepted = bool(goal_handle.accepted)
                if action_accepted:
                    result_future = goal_handle.get_result_async()
                    if _wait_future(result_future, timeout):
                        terminal = result_future.result()
                        result = terminal.result
                        action_error_code = int(result.error_code)
                        action_success = terminal.status == 4 and action_error_code == int(
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
        if final_joint is not None and target_positions is not None and len(final_joint.position) >= 6:
            final_error = max(
                abs(float(actual) - target)
                for actual, target in zip(final_joint.position[:6], target_positions)
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
            "target_positions": target_positions,
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
