from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest


rclpy = pytest.importorskip("rclpy")

try:
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rebotarm_msgs.msg import JointMotorState
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ImportError:
    pytest.skip("real ROS 2 Python runtime is unavailable", allow_module_level=True)

from rebotarm_simulation.ros.real2sim_ros_node import build_node_class


HOME = (0.0, -0.8, -1.0, 0.3, 0.0, 0.0)


class FakeSimulation:
    def __init__(self, _model=None):
        self.positions = list(HOME)
        self.velocities = [0.0] * 6
        self.width = 0.06
        self.time = 0.0

    def mirror_joint_state(self, positions, velocities, *, gripper_width=None):
        self.positions[:] = positions
        self.velocities[:] = velocities
        if gripper_width is not None:
            self.width = gripper_width
        return self.get_state()

    def step(self, count):
        self.time += int(count) * 0.002
        return self.get_state()

    def get_state(self):
        return SimpleNamespace(
            joint_positions=tuple(self.positions) + (self.width / 2.0, -self.width / 2.0),
            joint_velocities=tuple(self.velocities) + (0.0, 0.0),
            actuator_forces=(0.0,) * 8,
            gripper_width=self.width,
            simulation_time=self.time,
        )

    def close(self):
        pass


def test_ros_bridge_consumes_source_and_publishes_separate_read_only_state():
    rclpy.init()
    bridge = probe = None
    executor = SingleThreadedExecutor()
    try:
        bridge = build_node_class()(sim_factory=FakeSimulation)
        probe = Node("real2sim_test_probe")
        joint_pub = probe.create_publisher(
            JointState, "/rebotarm/joint_states", qos_profile_sensor_data
        )
        gripper_pub = probe.create_publisher(
            JointMotorState, "/rebotarm/gripper/state", qos_profile_sensor_data
        )
        received_joint = []
        received_status = []
        probe.create_subscription(
            JointState,
            "/real2sim/joint_states",
            received_joint.append,
            qos_profile_sensor_data,
        )
        probe.create_subscription(
            String, "/real2sim/status", received_status.append, 10
        )
        executor.add_node(bridge)
        executor.add_node(probe)

        for _ in range(5):
            gripper = JointMotorState()
            gripper.position = 0.04
            gripper_pub.publish(gripper)
            joint = JointState()
            joint.header.stamp = probe.get_clock().now().to_msg()
            joint.name = [f"joint{i}" for i in range(1, 7)]
            joint.position = list(HOME)
            joint.velocity = [0.0] * 6
            joint_pub.publish(joint)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                if received_joint and received_status:
                    break
            if received_joint and received_status:
                break

        assert received_joint
        assert received_joint[-1].name[:6] == [f"joint{i}" for i in range(1, 7)]
        assert received_joint[-1].position[:6] == pytest.approx(HOME)
        status = json.loads(received_status[-1].data)
        assert status["ok"] is True
        assert status["read_only"] is True
        assert status["hardware_commands_sent"] is False
    finally:
        if bridge is not None:
            bridge.close()
            executor.remove_node(bridge)
            bridge.destroy_node()
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
