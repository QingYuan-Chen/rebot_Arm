from geometry_msgs.msg import Pose, TransformStamped
import pytest

from rebotarm_vision.utils.tf_message_adapter import (
    apply_tcp_offset_to_pose,
    transform_from_msg,
    transform_pose_message,
)


def test_tf_message_adapter_transforms_pose_without_mutating_input():
    message = TransformStamped()
    message.transform.translation.x = 1.0
    message.transform.translation.y = 2.0
    message.transform.translation.z = 3.0
    message.transform.rotation.z = 0.7071067811865476
    message.transform.rotation.w = 0.7071067811865476
    pose = Pose()
    pose.position.x = 1.0
    pose.orientation.w = 1.0

    transformed = transform_pose_message(pose, transform_from_msg(message))

    assert (transformed.position.x, transformed.position.y, transformed.position.z) == pytest.approx(
        (1.0, 3.0, 3.0)
    )
    assert transformed.orientation.z == pytest.approx(0.7071067811865476)
    assert transformed.orientation.w == pytest.approx(0.7071067811865476)
    assert pose.position.x == 1.0
    assert pose.position.y == 0.0
    assert pose.orientation.z == 0.0


def test_tf_message_adapter_uses_canonical_public_module():
    from rebotarm_vision.utils.tf_message_adapter import (
        transform_from_msg as canonical_transform_from_msg,
        transform_pose_message as canonical_transform_pose_message,
    )

    assert canonical_transform_from_msg is transform_from_msg
    assert canonical_transform_pose_message is transform_pose_message
