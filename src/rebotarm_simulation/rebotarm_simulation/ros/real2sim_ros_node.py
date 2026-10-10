"""Read-only ROS 2 state bridge from reBotArm feedback into MuJoCo."""

from __future__ import annotations

import json
import math
import threading
import time

from rebotarm_simulation.core.mujoco_sim import JOINT_NAMES, RebotArmMujoco
from rebotarm_simulation.core.real2sim import (
    JointMappingConfig,
    Real2SimMapper,
    Real2SimSynchronizer,
    RobotStateSample,
    default_real2sim_mapping_path,
)


def stamp_to_seconds(stamp) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("message timestamp must be finite and non-negative")
    return value


def joint_state_message_to_sample(
    message,
    *,
    gripper_width: float | None,
    fallback_timestamp: float,
) -> RobotStateSample:
    timestamp = stamp_to_seconds(message.header.stamp)
    if timestamp == 0.0:
        timestamp = float(fallback_timestamp)
    velocities = tuple(message.velocity)
    if velocities and len(velocities) != len(message.name):
        raise ValueError("joint state velocity length does not match names")
    return RobotStateSample(
        timestamp=timestamp,
        joint_names=tuple(message.name),
        positions=tuple(message.position),
        velocities=velocities,
        gripper_width=gripper_width,
    )


def validate_topic_separation(source_topic: str, output_topic: str) -> tuple[str, str]:
    source = str(source_topic).strip()
    output = str(output_topic).strip()
    if not source.startswith("/") or not output.startswith("/"):
        raise ValueError("source and output topics must be absolute")
    if source == output:
        raise ValueError("source and output joint state topics must be different")
    return source, output


def build_node_class():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rebotarm_msgs.msg import ArmStatus, JointMotorState
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class Real2SimBridgeNode(Node):
        def __init__(self, *, sim_factory=RebotArmMujoco) -> None:
            super().__init__("rebotarm_real2sim_bridge")
            self.declare_parameter("model_path", "")
            self.declare_parameter("mapping_path", "")
            self.declare_parameter("mapping_profile", "rebotarm")
            self.declare_parameter("mode", "mirror")
            self.declare_parameter("update_rate_hz", 100.0)
            self.declare_parameter("physics_steps_per_update", 5)
            self.declare_parameter("source_joint_states_topic", "/rebotarm/joint_states")
            self.declare_parameter("source_gripper_state_topic", "/rebotarm/gripper/state")
            self.declare_parameter("source_arm_status_topic", "/rebotarm/arm_status")
            self.declare_parameter("output_joint_states_topic", "/real2sim/joint_states")
            self.declare_parameter("output_status_topic", "/real2sim/status")
            self.declare_parameter("source_timeout_sec", 0.2)
            self.declare_parameter("require_gravity_comp", False)

            model_path = str(self.get_parameter("model_path").value or "")
            mapping_path = str(self.get_parameter("mapping_path").value or "")
            mapping_profile = str(self.get_parameter("mapping_profile").value)
            mode = str(self.get_parameter("mode").value)
            update_rate = float(self.get_parameter("update_rate_hz").value)
            physics_steps = int(self.get_parameter("physics_steps_per_update").value)
            source_timeout = float(self.get_parameter("source_timeout_sec").value)
            if not math.isfinite(update_rate) or not 0.0 < update_rate <= 500.0:
                raise ValueError("update_rate_hz must be in (0, 500]")
            if not math.isfinite(source_timeout) or source_timeout <= 0.0:
                raise ValueError("source_timeout_sec must be positive")
            source_joint_topic, output_joint_topic = validate_topic_separation(
                self.get_parameter("source_joint_states_topic").value,
                self.get_parameter("output_joint_states_topic").value,
            )
            source_gripper_topic = str(
                self.get_parameter("source_gripper_state_topic").value
            )
            source_status_topic = str(self.get_parameter("source_arm_status_topic").value)
            output_status_topic = str(self.get_parameter("output_status_topic").value)
            self._require_gravity_comp = bool(
                self.get_parameter("require_gravity_comp").value
            )
            self._source_timeout = source_timeout
            self._simulation = sim_factory(model_path or None)
            self._simulation_lock = threading.RLock()
            config = JointMappingConfig.from_yaml(
                mapping_path or default_real2sim_mapping_path(),
                profile=mapping_profile,
            )
            self._synchronizer = Real2SimSynchronizer(
                self._simulation,
                Real2SimMapper(config),
                mode=mode,
                physics_steps_per_update=physics_steps,
            )
            self._lock = threading.Lock()
            self._pending_sample = None
            self._latest_gripper_width = None
            self._last_receipt_monotonic = None
            self._source_gravity_comp = False
            self._last_status_reason = None
            self._output_joint_pub = self.create_publisher(
                JointState, output_joint_topic, qos_profile_sensor_data
            )
            self._status_pub = self.create_publisher(String, output_status_topic, 10)
            self._joint_sub = self.create_subscription(
                JointState,
                source_joint_topic,
                self._on_joint_state,
                qos_profile_sensor_data,
            )
            self._gripper_sub = self.create_subscription(
                JointMotorState,
                source_gripper_topic,
                self._on_gripper_state,
                qos_profile_sensor_data,
            )
            self._arm_status_sub = self.create_subscription(
                ArmStatus,
                source_status_topic,
                self._on_arm_status,
                qos_profile_sensor_data,
            )
            self._timer = self.create_timer(1.0 / update_rate, self._update)
            self.get_logger().info(
                f"Real2Sim read-only bridge started: {source_joint_topic} -> "
                f"{output_joint_topic}, mode={mode}"
            )

        def _on_joint_state(self, message) -> None:
            now = time.monotonic()
            try:
                with self._lock:
                    gripper_width = self._latest_gripper_width
                sample = joint_state_message_to_sample(
                    message,
                    gripper_width=gripper_width,
                    fallback_timestamp=now,
                )
            except (TypeError, ValueError) as exc:
                self._publish_status(False, f"invalid_source_state: {exc}")
                return
            with self._lock:
                self._pending_sample = sample
                self._last_receipt_monotonic = now

        def _on_gripper_state(self, message) -> None:
            width = float(message.position)
            if not math.isfinite(width):
                self._publish_status(False, "invalid_gripper_width")
                return
            with self._lock:
                self._latest_gripper_width = width

        def _on_arm_status(self, message) -> None:
            gravity_comp = str(message.state_machine) == "GRAVITY_COMP"
            with self._lock:
                self._source_gravity_comp = gravity_comp

        def _update(self) -> None:
            now = time.monotonic()
            with self._lock:
                sample = self._pending_sample
                receipt = self._last_receipt_monotonic
                gravity_comp = self._source_gravity_comp
                self._pending_sample = None
            if sample is None:
                if receipt is not None and now - receipt > self._source_timeout:
                    self._publish_status(False, "source_timeout")
                return
            if now - float(receipt) > self._source_timeout:
                self._publish_status(False, "source_timeout")
                return
            if self._require_gravity_comp and not gravity_comp:
                self._publish_status(False, "source_not_in_gravity_comp")
                return
            try:
                with self._simulation_lock:
                    result = self._synchronizer.apply(sample)
                    state = self._simulation.get_state()
            except Exception as exc:
                self._publish_status(False, f"synchronization_rejected: {exc}")
                return
            output = JointState()
            output.header.stamp = self.get_clock().now().to_msg()
            output.name = list(JOINT_NAMES)
            output.position = list(state.joint_positions)
            output.velocity = list(state.joint_velocities)
            output.effort = list(state.actuator_forces)
            self._output_joint_pub.publish(output)
            self._publish_status(
                True,
                "tracking",
                extra={
                    "mode": result.mode,
                    "source_timestamp": result.source_timestamp,
                    "simulation_time": result.simulation_time,
                    "max_tracking_error_rad": result.max_tracking_error_rad,
                    "source_gravity_comp": gravity_comp,
                },
            )

        def _publish_status(self, ok: bool, reason: str, *, extra=None) -> None:
            if not ok and reason == self._last_status_reason:
                return
            self._last_status_reason = None if ok else reason
            payload = {
                "ok": bool(ok),
                "reason": str(reason),
                "read_only": True,
                "hardware_commands_sent": False,
            }
            if extra:
                payload.update(extra)
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
            self._status_pub.publish(message)

        @property
        def simulation(self):
            return self._simulation

        @property
        def simulation_lock(self):
            return self._simulation_lock

        def close(self) -> None:
            with self._simulation_lock:
                if self._simulation is not None:
                    self._simulation.close()
                    self._simulation = None

    return Real2SimBridgeNode


def main(args=None) -> None:
    import rclpy

    node_class = build_node_class()
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
