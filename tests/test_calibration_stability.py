import numpy as np
from rebotarm_calibration.stability_window import StabilityWindow


def test_continuous_window_rejects_gap_and_clock_regression():
    window=StabilityWindow(.4,.2,.001,.5);pose=np.eye(4)
    assert not window.add(1_000_000_000,pose)
    assert not window.add(1_100_000_000,pose)
    assert not window.add(2_000_000_000,pose)  # no spanning a dropped stream
    assert len(window.values)==1
    assert not window.add(1_500_000_000,pose)
    assert len(window.values)==1
    for stamp in [1_600_000_000,1_700_000_000,1_800_000_000]:
        assert not window.add(stamp,pose)
    assert window.add(1_900_000_000,pose)


def test_motion_and_rejected_observation_reset_window():
    window=StabilityWindow(.4,.2,.001,.5);pose=np.eye(4)
    window.add(1_000_000_000,pose);window.add(1_100_000_000,pose)
    shifted=pose.copy();shifted[0,3]=.01
    assert not window.add(1_200_000_000,shifted)
    assert len(window.values)==1
    window.reset()
    assert not window.add(1_300_000_000,shifted)


def test_marker_motion_and_rotation_restart_paired_window():
    from scipy.spatial.transform import Rotation
    for change in ('translation', 'rotation'):
        window = StabilityWindow(.4, .2, .001, .5)
        robot, marker = np.eye(4), np.eye(4)
        moved = marker.copy()
        if change == 'translation':
            moved[0, 3] = .02
        else:
            moved[:3, :3] = Rotation.from_euler('x', 2., degrees=True).as_matrix()
        for index in range(10):
            assert not window.add(10**9+index*100_000_000, robot, marker if index % 2 else moved)
            assert len(window.values) == len(window.markers) == 1
        for index in range(10, 14):
            assert not window.add(10**9+index*100_000_000, robot, moved)
        assert window.add(2_400_000_000, robot, moved)


def test_representative_frame_keeps_pair_and_avoids_last_outlier():
    window = StabilityWindow(.4, .2, .001, .5)
    for index, offset in enumerate([0., .0001, 0., 0., .0008]):
        robot, marker = np.eye(4), np.eye(4)
        marker[0, 3] = offset
        window.add(10**9+index*100_000_000, robot, marker)
    selected = window.representative_index()
    assert selected in (0, 2, 3)
    assert window.markers[selected][0, 3] == 0.
    window.reset()
    assert not window.values and not window.markers
