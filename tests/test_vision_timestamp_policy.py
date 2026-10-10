from rebotarm_vision.policies.timestamp_policy import select_frame_timestamp_ns


def test_wall_clock_prefers_positive_camera_system_timestamp() -> None:
    assert select_frame_timestamp_ns(
        1_786_433_839_297_082_000,
        80_000_000_000,
        use_sim_time=False,
    ) == 1_786_433_839_297_082_000


def test_sim_time_uses_receipt_timestamp_in_ros_clock_domain() -> None:
    assert select_frame_timestamp_ns(
        1_786_433_839_297_082_000,
        80_000_000_000,
        use_sim_time=True,
    ) == 80_000_000_000


def test_missing_camera_timestamp_falls_back_to_receipt_time() -> None:
    assert select_frame_timestamp_ns(
        None,
        80_000_000_000,
        use_sim_time=False,
    ) == 80_000_000_000
