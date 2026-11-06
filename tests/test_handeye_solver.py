from __future__ import annotations

import cv2
import numpy as np
import pytest


def _transform(rotation_vector, translation):
    from rebotarm_calibration.handeye_residual import matrix_transform

    value = np.eye(4, dtype=np.float64)
    value[:3, :3] = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64))[0]
    value[:3, 3] = np.asarray(translation, dtype=np.float64)
    return value, matrix_transform(value)


def _perfect_samples():
    from rebotarm_calibration.handeye_residual import matrix_transform

    end_to_camera, _ = _transform([0.22, -0.31, 0.17], [-0.075, 0.012, 0.041])
    base_to_marker, _ = _transform([-0.16, 0.08, 0.24], [0.31, -0.12, 0.27])
    samples = []
    poses = [
        ([0.00, 0.00, 0.00], [0.05, -0.18, 0.29]),
        ([0.24, 0.03, -0.02], [0.09, -0.15, 0.31]),
        ([-0.06, 0.29, 0.05], [0.03, -0.10, 0.34]),
        ([0.04, -0.08, 0.31], [0.12, -0.08, 0.27]),
        ([-0.21, 0.13, -0.18], [0.07, -0.21, 0.36]),
        ([0.15, 0.20, -0.27], [0.01, -0.14, 0.25]),
    ]
    for index, (rotation_vector, translation) in enumerate(poses):
        base_to_end, _ = _transform(rotation_vector, translation)
        camera_to_marker = np.linalg.inv(end_to_camera) @ np.linalg.inv(base_to_end) @ base_to_marker
        samples.append(
            {
                "label": f"pose_{index}",
                "base_to_end": matrix_transform(base_to_end),
                "camera_to_marker": matrix_transform(camera_to_marker),
            }
        )
    return samples, end_to_camera, base_to_marker


def test_eye_in_hand_solver_recovers_exact_transform() -> None:
    from rebotarm_calibration.handeye_solver import solve_eye_in_hand

    samples, expected, _ = _perfect_samples()
    solved = solve_eye_in_hand(samples, method="PARK")

    assert np.allclose(solved, expected, atol=1e-8)


@pytest.mark.parametrize('method', ['TSAI','PARK','HORAUD','ANDREFF','DANIILIDIS'])
def test_fixed_base_frame_change_keeps_same_camera_mount(method):
    from rebotarm_calibration.handeye_residual import matrix_transform,transform_matrix
    from rebotarm_calibration.handeye_solver import solve_eye_in_hand
    samples,expected,_ = _perfect_samples()
    world_to_base,_ = _transform([.3,-.2,.4],[.25,-.1,.3])
    rebased = [dict(sample,base_to_end=matrix_transform(world_to_base@transform_matrix(sample['base_to_end'])))
               for sample in samples]
    assert np.allclose(solve_eye_in_hand(rebased,method=method),expected,atol=1e-8)


def test_eye_in_hand_evaluation_can_use_training_reference() -> None:
    from rebotarm_calibration.handeye_solver import evaluate_eye_in_hand

    samples, handeye, marker = _perfect_samples()
    report = evaluate_eye_in_hand(
        samples[-2:], handeye, reference_base_to_marker=marker
    )

    assert report["reference_source"] == "provided"
    assert report["position_residual"]["max_m"] < 1e-12
    assert report["rotation_residual"]["max_deg"] < 1e-6


def test_eye_in_hand_solver_rejects_short_or_unknown_method() -> None:
    from rebotarm_calibration.handeye_solver import solve_eye_in_hand

    samples, _, _ = _perfect_samples()
    with pytest.raises(ValueError, match="at least three"):
        solve_eye_in_hand(samples[:2])
    with pytest.raises(ValueError, match="unsupported"):
        solve_eye_in_hand(samples, method="unknown")


def test_solver_accepts_matrices_and_rejects_single_axis():
    from rebotarm_calibration.handeye_solver import solve_eye_in_hand
    from rebotarm_calibration.handeye_residual import transform_matrix, analyze_handeye_residual, matrix_transform
    samples, expected, _ = _perfect_samples()
    report = analyze_handeye_residual({'schema_version': 1, 'samples': samples,
                                      'end_to_camera': matrix_transform(expected)})
    assert report['acceptance']['pass']
    matrices = [{k: transform_matrix(s[k]) for k in ('base_to_end', 'camera_to_marker')} for s in samples]
    assert np.allclose(solve_eye_in_hand(matrices, method='PARK'), expected)
    repeated = [matrices[0]] * 5
    with pytest.raises(ValueError, match='unobservable'):
        solve_eye_in_hand(repeated)
