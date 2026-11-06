"""Offline hand-eye comparison. Selection uses training data, never the holdout."""
from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile

import cv2

from .handeye_residual import analyze_handeye_residual, matrix_transform, transform_matrix, robot_pose_key
from .handeye_solver import HAND_EYE_METHODS, evaluate_eye_in_hand, solve_eye_in_hand


def atomic_write_json(path, payload):
    """Serialize before touching destination; publish a complete fsynced file."""
    data = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name, delete=False) as stream:
            temporary = stream.name
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)


def solve_dataset(dataset):
    """Schema 1: metadata + explicitly separate training/validation samples.

    Require independent sample IDs and reject copied poses across splits. A pass
    is numerical validation only, never operator acceptance or deployment.
    """
    if not isinstance(dataset, dict) or dataset.get('schema_version') != 1:
        raise ValueError('unsupported dataset schema')
    metadata = dataset.get('metadata', {})
    for key in ('base_frame', 'end_link_frame', 'camera_frame', 'marker_frame'):
        if not isinstance(metadata.get(key), str) or not metadata[key].strip():
            raise ValueError(f'metadata.{key} is required')
    excluded = dataset.get('excluded_sample_ids', [])
    if not isinstance(excluded, list) or any(not isinstance(item, str) for item in excluded) or len(set(excluded)) != len(excluded):
        raise ValueError('invalid excluded_sample_ids')
    known_ids = {sample.get('sample_id') for name in ('training_samples', 'validation_samples')
                 for sample in dataset.get(name, []) if isinstance(sample, dict)}
    if not set(excluded) <= known_ids:
        raise ValueError('unknown excluded sample')
    groups = []
    identities, poses = set(), set()
    for name in ('training_samples', 'validation_samples'):
        samples = dataset.get(name)
        if not isinstance(samples, list):
            raise ValueError(f'{name} requires at least five samples')
        samples = [sample for sample in samples if not isinstance(sample, dict) or sample.get('sample_id') not in excluded]
        if not isinstance(samples, list) or len(samples) < 5:
            raise ValueError(f'{name} requires at least five samples')
        normalized = []
        for sample in samples:
            if not isinstance(sample, dict):
                raise ValueError('sample must be a mapping')
            identity = sample.get('sample_id')
            if not isinstance(identity, str) or not identity.strip() or identity in identities:
                raise ValueError('sample_id must be nonempty and unique across splits')
            identities.add(identity)
            normalized_sample = {'sample_id': identity, 'label': identity}
            for field in ('base_to_end', 'camera_to_marker'):
                normalized_sample[field] = matrix_transform(transform_matrix(sample[field]))
            pose = robot_pose_key(sample['base_to_end'])
            if pose in poses:
                raise ValueError('duplicate robot pose; use independent diverse samples')
            poses.add(pose)
            normalized.append(normalized_sample)
        groups.append(normalized)
    training, validation = groups
    cutoff = dataset.get('final_validation_after_index')
    fresh_ids = {sample['sample_id'] for sample in dataset['validation_samples']
                 if cutoff is None or sample.get('sample_index', -1) > cutoff}
    final_validation = [sample for sample in validation if sample['sample_id'] in fresh_ids]
    methods = {}
    for method in HAND_EYE_METHODS:
        try:
            candidate = solve_eye_in_hand(training, method=method)
            transform = matrix_transform(candidate)
            train = analyze_handeye_residual({'schema_version': 1, 'samples': training,
                                             'end_to_camera': transform})
            diversity = analyze_handeye_residual({'schema_version': 1, 'samples': validation,
                                                 'end_to_camera': transform})
            holdout = evaluate_eye_in_hand(validation, candidate, reference_base_to_marker=
                                          transform_matrix(train['mean_base_to_marker']))
            from .calibration_quality import residual_diagnostics
            train['diagnostics'] = residual_diagnostics(train['samples'])
            holdout['diagnostics'] = residual_diagnostics(holdout['samples'])
            limits = train['acceptance']
            gates = {
                'training': train['acceptance']['pass'],
                'validation_diversity': diversity['pose_diversity']['pass'],
                'validation_position_rms': holdout['position_residual']['rms_m'] <= limits['max_position_rms_m'],
                'validation_position_max': holdout['position_residual']['max_m'] <= limits['max_position_residual_m'],
                'validation_rotation_rms': holdout['rotation_residual']['rms_deg'] <= limits['max_rotation_rms_deg'],
                'validation_rotation_max': holdout['rotation_residual']['max_deg'] <= limits['max_rotation_residual_deg'],
            }
            methods[method] = {'end_to_camera': transform, 'training': train,
                               'validation': holdout, 'gates': gates, 'passed': all(gates.values())}
            full = [dict(sample, label=sample['sample_id']) for sample in dataset['validation_samples']]
            methods[method]['full_validation'] = evaluate_eye_in_hand(full, candidate,
                reference_base_to_marker=transform_matrix(train['mean_base_to_marker']))
            if cutoff is not None:
                final_pass = False
                if len(final_validation) >= 5:
                    final = evaluate_eye_in_hand(final_validation, candidate,
                        reference_base_to_marker=transform_matrix(train['mean_base_to_marker']))
                    diversity_final = analyze_handeye_residual({'schema_version': 1, 'samples': final_validation, 'end_to_camera': transform})
                    final_pass = (diversity_final['pose_diversity']['pass'] and
                        final['position_residual']['rms_m'] <= limits['max_position_rms_m'] and
                        final['position_residual']['max_m'] <= limits['max_position_residual_m'] and
                        final['rotation_residual']['rms_deg'] <= limits['max_rotation_rms_deg'] and
                        final['rotation_residual']['max_deg'] <= limits['max_rotation_residual_deg'])
                    methods[method]['final_validation'] = final
                gates['fresh_final_validation'] = final_pass
                methods[method]['passed'] = all(gates.values())
        except (ValueError, cv2.error) as exc:
            methods[method] = {'passed': False, 'error': str(exc)}
    eligible = [name for name, result in methods.items()
                if 'training' in result and result['training']['acceptance']['pass']]
    # Dimensionless score normalized by gates; holdout never influences selection.
    selected = min(eligible, key=lambda name:
                   methods[name]['training']['position_residual']['rms_m'] / .005 +
                   methods[name]['training']['rotation_residual']['rms_deg'] / 1.5) if eligible else None
    from .calibration_quality import bootstrap_diagnostic
    uncertainty = bootstrap_diagnostic(training, selected,
        transform_matrix(methods[selected]['end_to_camera'])) if selected else None
    from .provenance import dataset_fingerprint, runtime_provenance
    return {'schema_version': 1, 'metadata': deepcopy(metadata),
            'dataset_sha256': dataset_fingerprint(dataset), 'excluded_sample_ids': list(excluded),
            'solver_provenance': runtime_provenance({}), 'opencv_version': cv2.__version__,
            'methods': methods, 'selected_method': selected,
            'selection_source': 'training_only', 'uncertainty': uncertainty,
            'final_validation_required': cutoff is not None,
            'final_validation_sample_count': len(final_validation),
            'passed': bool(selected and methods[selected]['passed']),
            'accepted': False, 'deployed': False}
