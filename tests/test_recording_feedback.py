from types import SimpleNamespace

import pytest

from rebotarm_teach.recording_feedback import (
    feedback_teach_sample,
    ordered_feedback_vector,
)


def sample_fixture():
    names = tuple(f"joint{i}" for i in range(1, 7))
    msg = SimpleNamespace(
        name=list(reversed(names)), position=[6., 5., 4., 3., 2., 1.],
        velocity=[0.] * 6, effort=[0.] * 6,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=20_000_000)),
    )
    options = dict(
        joint_names=names, motor_status=dict.fromkeys(names, 1),
        motor_stamps=dict.fromkeys(names, 10_020_000_000), arm_state="GRAVITY_COMP",
        now_ns=10_030_000_000, started_ns=10_000_000_000, last_stamp_ns=None,
        timeout_sec=.15, require_motor_status=True,
    )
    return msg, options


def test_recording_uses_source_stamp_and_orders_joint_vectors():
    msg, options = sample_fixture()
    sample = feedback_teach_sample(msg, **options)
    assert sample.stamp == .02
    assert sample.positions == (1., 2., 3., 4., 5., 6.)


def test_ordered_feedback_vector_is_independent_of_ros_messages():
    assert ordered_feedback_vector(
        [30.0, 10.0, 20.0],
        source_names=("joint3", "joint1", "joint2"),
        source_indices=(1, 2, 0),
    ) == (10.0, 20.0, 30.0)


@pytest.mark.parametrize("values", [[1.0], [1.0, float("nan"), 3.0]])
def test_ordered_feedback_vector_rejects_invalid_values(values):
    with pytest.raises(ValueError):
        ordered_feedback_vector(
            values,
            source_names=("joint1", "joint2", "joint3"),
            source_indices=(0, 1, 2),
        )


def test_recording_does_not_resample_duplicate_or_prestart_feedback():
    msg, options = sample_fixture()
    options["last_stamp_ns"] = 10_020_000_000
    assert feedback_teach_sample(msg, **options) is None
    options["last_stamp_ns"] = None
    options["started_ns"] = 10_040_000_000
    assert feedback_teach_sample(msg, **options) is None


@pytest.mark.parametrize("fault", ["stale", "future", "status", "batch", "nan", "missing"])
def test_recording_rejects_unverified_feedback(fault):
    msg, options = sample_fixture()
    if fault == "stale":
        options["now_ns"] = 10_200_000_000
    elif fault == "future":
        options["now_ns"] = 10_000_000_000
    elif fault == "status":
        options["motor_status"]["joint1"] = 255
    elif fault == "batch":
        options["motor_stamps"]["joint1"] = 10_000_000_000
    elif fault == "nan":
        msg.position[0] = float("nan")
    else:
        msg.name.pop()
    with pytest.raises(ValueError):
        feedback_teach_sample(msg, **options)
