"""Durable hand-eye sessions; adapters supply already gated observations.

No HTTP, ROS, or motor dependencies. Every mutation has an idempotency key and
an expected revision; commit uses atomic persistence before replacing memory.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import threading
from uuid import uuid4

from .handeye_residual import matrix_transform, transform_matrix, robot_pose_key
from .handeye_workflow import atomic_write_json, solve_dataset


def _reject_nonfinite(value):
    raise ValueError(f'invalid saved session number: {value}')


class SessionStore:
    def __init__(self, directory, *, image_topic='/camera/color/image_raw',
                 camera_info_topic='/camera/color/camera_info'):
        self.directory = Path(directory).expanduser()
        self.lock = threading.Lock()
        self.metadata_defaults = {'mode': 'handeye', 'image_topic': image_topic,
                                  'camera_info_topic': camera_info_topic,
                                  'dictionary': 'DICT_4X4_50', 'marker_id': 0}

    def normalize_metadata(self, metadata):
        if not isinstance(metadata, dict):
            raise ValueError('invalid session metadata')
        metadata = deepcopy(metadata)
        metadata.setdefault('mode', 'handeye')
        mode = metadata['mode']
        if mode not in ('handeye', 'tcp'):
            raise ValueError('unsupported calibration mode')
        if mode == 'handeye':
            for name in ('image_topic', 'camera_info_topic', 'dictionary', 'marker_id'):
                metadata.setdefault(name, self.metadata_defaults[name])
            for name in ('image_topic', 'camera_info_topic', 'dictionary'):
                if not isinstance(metadata[name], str) or not metadata[name].strip():
                    raise ValueError(f'invalid {name}')
            if type(metadata['marker_id']) is not int or metadata['marker_id'] < 0:
                raise ValueError('invalid marker_id')
            if 'marker_length_m' in metadata:
                length = metadata['marker_length_m']
                if isinstance(length, bool) or not isinstance(length, (int, float)) or not math.isfinite(length) or length <= 0:
                    raise ValueError('invalid marker_length_m')
        fields = ('base_frame', 'end_link_frame') if mode == 'tcp' else (
            'base_frame', 'end_link_frame', 'camera_frame', 'marker_frame')
        for name in fields:
            if not isinstance(metadata.get(name), str) or not metadata[name].strip():
                raise ValueError(f'missing {name}')
        return metadata

    def _path(self, session_id):
        if not isinstance(session_id, str) or not re.fullmatch(r'[0-9a-f]{32}', session_id):
            raise ValueError('invalid session_id')
        return self.directory / (session_id + '.json')

    def read(self, session_id):
        data = json.loads(self._path(session_id).read_text(encoding='utf-8'),
                          parse_constant=_reject_nonfinite)
        if not isinstance(data, dict) or data.get('schema_version') != 1 or data.get('session_id') != session_id:
            raise ValueError('invalid saved session')
        if (type(data.get('revision')) is not int or data['revision'] < 0
                or data.get('state') not in ('active', 'solved', 'accepted', 'aborted')
                or not isinstance(data.get('requests'), dict)
                or not isinstance(data.get('training_samples'), list)
                or not isinstance(data.get('validation_samples'), list)
                or 'report' not in data
                or (data['report'] is not None and not isinstance(data['report'], dict))
                or (data['state'] in ('solved', 'accepted') and not isinstance(data['report'], dict))):
            raise ValueError('invalid saved session fields')
        data['metadata'] = self.normalize_metadata(data.get('metadata'))
        return data

    def create(self, metadata, *, session_id=None):
        """Caller-generated ID makes retrying creation safe after HTTP timeout."""
        session_id = session_id or uuid4().hex
        path = self._path(session_id)
        metadata = self.normalize_metadata(metadata)
        json.dumps(metadata, allow_nan=False)
        with self.lock:
            if path.exists():
                existing = self.read(session_id)
                if existing['metadata'] != metadata:
                    raise ValueError('session_id already used with different metadata')
                return existing
            created_at = datetime.now(timezone.utc).isoformat()
            data = {'created_at': created_at, 'updated_at': created_at, 'audit': [], 'schema_version': 1, 'session_id': session_id, 'revision': 0,
                    'state': 'active', 'metadata': metadata, 'training_samples': [],
                    'validation_samples': [], 'report': None, 'requests': {}}
            atomic_write_json(path, data)
            return deepcopy(data)

    def mutate(self, session_id, *, request_id, revision, operation, payload=None):
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise ValueError('request_id required (max 128 characters)')
        if type(revision) is not int or revision < 0:
            raise ValueError('revision must be a non-negative integer')
        payload = deepcopy(payload or {})
        digest = hashlib.sha256(json.dumps([revision, operation, payload], sort_keys=True,
                                          allow_nan=False).encode()).hexdigest()
        with self.lock:
            data = self.read(session_id)
            previous = data['requests'].get(request_id)
            if previous:
                if previous != digest:
                    raise ValueError('request_id reused with different content')
                return deepcopy(data)
            if data['revision'] != revision:
                raise ValueError('stale revision; refresh session')
            if data['state'] in ('accepted', 'aborted'):
                raise ValueError('session is terminal')
            if operation == 'capture':
                if data['state'] != 'active':
                    raise ValueError('capture requires active session')
                split = payload.get('split')
                if split not in ('training', 'validation'):
                    raise ValueError('split must be training or validation')
                sample = deepcopy(payload.get('sample', {}))
                fields = ('base_to_end',) if data['metadata']['mode'] == 'tcp' else ('base_to_end', 'camera_to_marker')
                for field in fields:
                    sample[field] = matrix_transform(transform_matrix(sample[field]))
                # Freeze acquisition conditions across both splits and restarts.
                # Legacy offline datasets may omit provenance, but cannot be mixed
                # with live observations inside an existing session.
                conditions = {key: sample.get(key) for key in ('camera_info', 'provenance')}
                existing = data['training_samples'] + data['validation_samples']
                if existing:
                    reference = {key: existing[0].get(key) for key in conditions}
                    if conditions != reference:
                        raise ValueError('acquisition conditions changed; create a new session')
                if data['metadata']['mode'] != 'tcp':
                    key = robot_pose_key(sample['base_to_end'])
                    duplicate = next((item for item in existing if robot_pose_key(item['base_to_end']) == key), None)
                    if duplicate is not None:
                        raise ValueError(f"duplicate robot pose matches sample {duplicate['sample_id']}; change robot pose")
                sample['sample_id'] = request_id
                sample['sample_index'] = len(existing)
                sample['recorded_at'] = datetime.now(timezone.utc).isoformat()
                data[split + '_samples'].append(sample)
                if split == 'training' and data.get('validation_review_started'):
                    data['final_validation_after_index'] = sample['sample_index']
            elif operation == 'solve':
                from .tcp_workflow import solve_tcp_dataset
                effective = deepcopy(data)
                excluded = set(data.get('excluded_sample_ids', []))
                if data['metadata']['mode'] == 'tcp':
                    for split in ('training', 'validation'):
                        key = split + '_samples'
                        effective[key] = [sample for sample in data[key] if sample['sample_id'] not in excluded]
                data['report'] = solve_tcp_dataset(effective) if data['metadata']['mode'] == 'tcp' else solve_dataset(data)
                if data['metadata']['mode'] == 'tcp' and 'final_validation_after_index' in data:
                    final = deepcopy(effective)
                    final['validation_samples'] = [sample for sample in effective['validation_samples']
                        if sample['sample_index'] > data['final_validation_after_index']]
                    count = len(final['validation_samples'])
                    passed = False
                    if count >= 5:
                        final_report = solve_tcp_dataset(final)
                        passed = final_report['passed']
                        data['report']['final_validation'] = final_report['validation']
                    data['report']['final_validation_required'] = True
                    data['report']['final_validation_sample_count'] = count
                    data['report']['passed'] = data['report']['passed'] and passed
                data['report']['excluded_sample_ids'] = sorted(excluded)
                data['validation_review_started'] = True
                data['state'] = 'solved'
            elif operation == 'exclude_samples':
                ids = payload.get('sample_ids')
                reason = payload.get('reason')
                if not isinstance(ids, list) or any(not isinstance(item, str) for item in ids) or len(set(ids)) != len(ids):
                    raise ValueError('sample_ids must be a unique list of strings')
                existing = {sample['sample_id'] for split in ('training', 'validation') for sample in data[split + '_samples']}
                if not set(ids) <= existing:
                    raise ValueError('unknown sample_id')
                old = set(data.get('excluded_sample_ids', []))
                added = set(ids) - old
                flagged = set()
                report = data.get('report') or {}
                methods = report.get('methods', {})
                selected = methods.get(report.get('selected_method')) or next(
                    (method for method in methods.values() if 'training' in method), {})
                for split in ('training', 'validation'):
                    flagged.update(item['label'] for item in selected.get(split, {}).get('diagnostics', {}).get('flagged_samples', []))
                if added and (data['state'] != 'solved' or data['metadata']['mode'] == 'tcp' or added - flagged):
                    raise ValueError('only reported outliers can be newly excluded')
                if set(ids) == old:
                    raise ValueError('sample selection unchanged')
                if not isinstance(reason, str) or not reason.strip():
                    raise ValueError('exclusion change requires a reason')
                validation_ids = {sample['sample_id'] for sample in data['validation_samples']}
                if (set(ids) ^ old) & validation_ids and not str(payload.get('evidence', '')).strip():
                    raise ValueError('validation exclusion requires acquisition evidence')
                data['final_validation_after_index'] = max(
                    (sample['sample_index'] for split in ('training', 'validation') for sample in data[split + '_samples']), default=-1)
                data['excluded_sample_ids'] = sorted(ids)
                if data.get('report'):
                    data.setdefault('report_history', []).append({'revision': data['revision'], 'report': data['report']})
                data['report'] = None
                data['state'] = 'active'
            elif operation == 'reopen':
                if data['state'] != 'solved':
                    raise ValueError('reopen requires solved session')
                data.setdefault('report_history', []).append({'revision': data['revision'], 'report': data['report']})
                data['report'] = None
                data['state'] = 'active'
            elif operation == 'accept':
                if data['state'] != 'solved' or data['report'].get('passed') is not True:
                    raise ValueError('accept requires passing report')
                operator = payload.get('operator')
                if payload.get('confirmed') is not True or not isinstance(operator, str) or not operator.strip():
                    raise ValueError('explicit confirmation and operator required')
                data['state'] = 'accepted'
                data['operator'] = operator
                data['report']['accepted'] = True
            elif operation == 'abort':
                reason = payload.get('reason')
                if not isinstance(reason, str) or not reason.strip():
                    raise ValueError('abort reason required')
                data['state'] = 'aborted'
                data['abort_reason'] = reason
            else:
                raise ValueError('unknown operation')
            data['revision'] += 1
            data['updated_at'] = datetime.now(timezone.utc).isoformat()
            data.setdefault('audit', []).append({
                'revision': data['revision'], 'request_id': request_id,
                'operation': operation, 'at': data['updated_at'],
                'operator': payload.get('operator') if operation == 'accept' else None,
                **({'excluded_sample_ids': data['excluded_sample_ids'], 'reason': reason.strip(), 'evidence': payload.get('evidence', '')}
                   if operation == 'exclude_samples' else {}),
            })
            data['requests'][request_id] = digest
            data.setdefault('request_inputs', {})[request_id] = {'revision': revision, 'operation': operation, 'payload': payload}
            atomic_write_json(self._path(session_id), data)
            return deepcopy(data)
