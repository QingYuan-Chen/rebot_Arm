""":mod:`candidate_ik_runtime` 使用的 ROS 与 MoveIt 接口边界。"""

from __future__ import annotations

import time
from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity


def _pose_from_target(target):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in target.position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(v) for v in target.orientation
    )
    return pose


class CandidateIkGateway:
    """通过注入使用 TF 和 MoveIt 客户端，无需依赖 ROS 节点对象。"""

    def __init__(self, *, config, ik_client, validity_client, tf_buffer,
                 joint_state, logger, publish_ranked, publish_empty,
                 monotonic=time.monotonic, sleep=time.sleep, context_ok=rclpy.ok):
        self.monotonic = monotonic
        self.sleep = sleep
        self.context_ok = context_ok
        self.config = config
        self.ik_client = ik_client
        self.validity_client = validity_client
        self.tf_buffer = tf_buffer
        self.joint_state = deepcopy(joint_state)
        self.logger = logger
        self._publish_ranked = publish_ranked
        self._publish_empty = publish_empty
        self._transform_cache = {}

    def begin_frame(self) -> None:
        """清除上一帧候选对应的坐标变换缓存。"""
        self._transform_cache.clear()

    def publish_ranked(self, message, ranked) -> None:
        self._publish_ranked(message, ranked)

    def publish_empty(self, message) -> None:
        self._publish_empty(message)

    def check_target(self, target, label):
        solution = self.solve_ik(target, label)
        if solution is None or not self.check_state_validity(solution, label):
            return None
        return solution

    def lookup_transform(self, target_frame: str, source_frame: str, stamp=None):
        stamp_ns = 0 if stamp is None else int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        key = (target_frame, source_frame, stamp_ns)
        if key in self._transform_cache:
            return self._transform_cache[key]
        query_time = rclpy.time.Time()
        if stamp_ns > 0:
            query_time = rclpy.time.Time.from_msg(stamp)
        result = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            query_time,
            timeout=rclpy.duration.Duration(seconds=0.2),
        )
        self._transform_cache[key] = result
        return result

    def solve_ik(self, target, label: str):
        config = self.config
        timeout = config.service_timeout_sec
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            self.logger.warn("candidate IK filter IK service unavailable")
            return None
        seed = self.joint_state
        if seed is None or not seed.name or len(seed.position) < len(seed.name):
            self.logger.warn("candidate IK filter has no valid joint state yet; skipping IK")
            return None
        import math
        if not all(math.isfinite(float(v)) for v in seed.position[:len(seed.name)]):
            self.logger.warn("candidate IK filter has nonfinite joint state; skipping IK")
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = config.moveit_group_name
        request.ik_request.ik_link_name = config.ee_frame_id
        request.ik_request.robot_state.joint_state = deepcopy(seed)
        request.ik_request.pose_stamped = PoseStamped()
        request.ik_request.pose_stamped.header.frame_id = config.target_frame
        request.ik_request.pose_stamped.pose = _pose_from_target(target)
        request.ik_request.avoid_collisions = False
        future = self.ik_client.call_async(request)
        deadline = self.monotonic() + max(timeout, 0.1)
        while self.context_ok() and not future.done() and self.monotonic() < deadline:
            self.sleep(0.01)
        if not self.context_ok() or not future.done():
            future.cancel()
            self.logger.warn(f"candidate IK filter IK timed out for {label}")
            return None
        result = future.result()
        if result is None or int(getattr(getattr(result, "error_code", None), "val", 99999)) != 1:
            self.logger.warn(f"candidate IK filter IK failed for {label}")
            return None
        return getattr(result, "solution", None)

    def check_state_validity(self, robot_state, label: str) -> bool:
        config = self.config
        if not config.collision_check_enabled:
            return True
        if robot_state is None or not self.validity_client.wait_for_service(
            timeout_sec=config.service_timeout_sec
        ):
            self.logger.warn(f"candidate IK filter state validity unavailable for {label}")
            return False
        request = GetStateValidity.Request()
        request.group_name = config.collision_group_name
        request.robot_state = robot_state
        future = self.validity_client.call_async(request)
        deadline = self.monotonic() + max(config.service_timeout_sec, 0.1)
        while self.context_ok() and not future.done() and self.monotonic() < deadline:
            self.sleep(0.01)
        if not self.context_ok() or not future.done():
            future.cancel()
            self.logger.warn(f"candidate IK filter state validity timed out for {label}")
            return False
        result = future.result()
        if result is None:
            self.logger.warn(f"candidate IK filter state validity timed out for {label}")
            return False
        valid = bool(getattr(result, "valid", False))
        if not valid:
            self.logger.warn(f"candidate IK filter state validity failed for {label}: state invalid")
        return valid
