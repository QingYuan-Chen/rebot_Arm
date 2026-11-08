"""HTTP-thread to ROS adapter; calibration implementation remains in its owner."""
import json
import threading


class CalibrationClient:
    def __init__(self, client, request_type, publish, timeout=30.):
        self.client, self.request_type = client, request_type
        self.publish, self.timeout = publish, timeout
        self.lock = threading.Lock()
        self.pending = None

    def command(self, payload):
        result = self._command(payload)
        if result['success']:
            return result
        session = result.get('session')
        session = session if isinstance(session, dict) else {}
        return dict(result, schema_version=1,
                    session_id=session.get('session_id', payload.get('session_id', '')),
                    state=session.get('state', 'unavailable'), revision=session.get('revision'))

    @staticmethod
    def _success_response(payload, result):
        session = result.get('session')
        command = payload['command']
        try:
            if not isinstance(session, dict) or session['schema_version'] != 1:
                raise ValueError('invalid response schema')
            session_id = session['session_id']
            state = session['state']
            if not isinstance(session_id, str) or not isinstance(state, str):
                raise ValueError('invalid session identity or state')
            if command == 'preview':
                if not isinstance(session['preview'], dict) or not session['preview']['jpeg_base64']:
                    raise ValueError('invalid preview')
                revision = None
            elif command == 'observe':
                if not isinstance(session['observation'], dict):
                    raise ValueError('invalid observation')
                revision = None
            else:
                revision = session['revision']
                if type(revision) is not int or revision < 0:
                    raise ValueError('invalid revision')
        except (KeyError, TypeError, ValueError) as exc:
            return {'success': False, 'reason_code': 'INVALID_RESPONSE',
                    'message': f'标定响应损坏: {exc}', 'session': None}
        return dict(result, schema_version=1, session_id=session_id,
                    state=state, revision=revision)

    def _command(self, payload):
        allowed = {'observe', 'tf_check', 'preview', 'create', 'status', 'preflight', 'capture', 'solve', 'reopen', 'exclude_samples', 'accept', 'abort'}
        if not isinstance(payload, dict) or payload.get('command') not in allowed:
            raise ValueError('invalid calibration command')
        data = payload.get('payload', {})
        if not isinstance(data, dict):
            raise ValueError('calibration payload must be an object')
        for name in ('session_id', 'request_id'):
            if name in payload and not isinstance(payload[name], str):
                raise ValueError(f'{name} must be a string')
        if 'expected_revision' in payload and (type(payload['expected_revision']) is not int
                                                 or not 0 <= payload['expected_revision'] < 2**63):
            raise ValueError('expected_revision must be a non-negative integer')
        try:
            payload_json = json.dumps(data, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'invalid calibration payload: {exc}') from exc
        if not self.lock.acquire(blocking=False):
            return {'success': False, 'reason_code': 'BUSY', 'message': '标定请求正在处理'}
        try:
            if self.pending is not None:
                if not self.pending.done():
                    return {'success': False, 'reason_code': 'PENDING',
                            'message': '上次请求尚未完成，请稍后刷新会话'}
                self.pending = None
            if not self.client.service_is_ready():
                return {'success': False, 'reason_code': 'UNAVAILABLE', 'message': '标定节点尚未启动'}
            request = self.request_type()
            request.command = payload['command']
            request.session_id = payload.get('session_id', '')
            request.request_id = payload.get('request_id', '')
            request.expected_revision = payload.get('expected_revision', 0)
            request.payload_json = payload_json
            self.publish({'busy': True, 'command': request.command, 'session_id': request.session_id})
            future = self.client.call_async(request)
            self.pending = future
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(self.timeout):
                result = {'success': False, 'reason_code': 'TIMEOUT',
                          'message': '请求超时，结果未知；请刷新会话，勿创建重复请求'}
            else:
                self.pending = None
                response = future.result()
                try:
                    session = json.loads(response.session_json) if response.session_json else None
                except (TypeError, json.JSONDecodeError) as exc:
                    result = {'success': False, 'reason_code': 'INVALID_RESPONSE',
                              'message': f'标定响应损坏: {exc}', 'session': None}
                else:
                    result = {'success': response.success, 'reason_code': response.reason_code,
                              'message': response.message, 'session': session}
                if result['success']:
                    result = self._success_response(payload, result)
            # Broadcast compact session-scoped progress, never datasets or images.
            session = result.get('session')
            session = session if isinstance(session, dict) else {}
            self.publish({'busy': self.pending is not None, 'command': request.command,
                          'session_id': request.session_id, 'revision': session.get('revision'),
                          'success': result['success'], 'message': result['message']})
            return result
        except Exception as exc:
            result = {'success': False, 'reason_code': 'TRANSPORT_ERROR',
                      'message': '标定通信失败，结果可能未知：' + str(exc)}
            self.publish(dict(result, busy=False))
            return result
        finally:
            self.lock.release()
