"""把 GraspNet 原始候选转换为 RViz MarkerArray（只读显示）。"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rebotarm_msgs.msg import GraspCandidateArray

from ..utils.tf_message_adapter import transform_from_msg, transform_pose_message


def _rotate(v, q):
    x, y, z = v
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-9:
        return v
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    t = (2 * (qy * z - qz * y), 2 * (qz * x - qx * z), 2 * (qx * y - qy * x))
    return (x + qw * t[0] + qy * t[2] - qz * t[1],
            y + qw * t[1] + qz * t[0] - qx * t[2],
            z + qw * t[2] + qx * t[1] - qy * t[0])


class GraspCandidateMarkerNode(Node):
    """显示前 N 个原始候选；不参与 IK、规划或执行。"""

    def __init__(self):
        super().__init__("rebotarm_grasp_candidate_markers")
        self.declare_parameter("input_topic", "/grasp/graspnet_candidates")
        self.declare_parameter("output_topic", "/grasp/raw_candidate_markers")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("max_candidates", 5)
        self._target = str(self.get_parameter("target_frame").value).strip()
        self._max = max(1, int(self.get_parameter("max_candidates").value))
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter("input_topic").value),
            self._on_candidates,
            10,
        )

    def _marker(self, frame_id, stamp, marker_id, candidate, pose, color):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "graspnet_raw_candidates"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.scale.x = 0.006
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        width = max(0.006, min(float(candidate.jaw_width), 0.2))
        length = 0.06
        # 在候选局部坐标中画两指和根部横梁，再旋转/平移到 target_frame。
        local = [
            ((-length, -width / 2, 0), (0, -width / 2, 0)),
            ((-length, width / 2, 0), (0, width / 2, 0)),
            ((-length, -width / 2, 0), (-length, width / 2, 0)),
        ]
        for start, end in local:
            for point in (start, end):
                rotated = _rotate(point, q)
                out = Point()
                out.x = pose.position.x + rotated[0]
                out.y = pose.position.y + rotated[1]
                out.z = pose.position.z + rotated[2]
                marker.points.append(out)
        return marker

    def _on_candidates(self, message):
        output = MarkerArray()
        if not message.candidates:
            clear = Marker()
            clear.action = Marker.DELETEALL
            clear.ns = "graspnet_raw_candidates"
            output.markers.append(clear)
            self._pub.publish(output)
            return
        source_frame = str(message.header.frame_id or "")
        if not source_frame:
            return
        try:
            transform = self._tf.lookup_transform(self._target, source_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().debug(f"raw candidate marker TF unavailable: {exc}")
            return
        tf = transform_from_msg(transform)
        for index, candidate in enumerate(list(message.candidates)[: self._max]):
            pose = transform_pose_message(candidate.pose, tf)
            color = (0.1, 1.0, 0.2, 0.9) if index == int(message.best_index) else (1.0, 0.65, 0.05, 0.75)
            output.markers.append(self._marker(self._target, message.header.stamp, index, candidate, pose, color))
        self._pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = GraspCandidateMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
