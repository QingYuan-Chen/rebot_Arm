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
        session = result.get('session') or {}
        return dict(result, schema_version=1,
                    session_id=session.get('session_id', payload.get('session_id', '')),
                    state=session.get('state', 'unavailable'), revision=session.get('revision'))

    def _command(self, payload):
        allowed = {'observe', 'tf_check', 'preview', 'create', 'status', 'preflight', 'capture', 'solve', 'reopen', 'exclude_samples', 'accept', 'abort'}
        if not isinstance(payload, dict) or payload.get('command') not in allowed:
            raise ValueError('invalid calibration command')
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
            request.payload_json = json.dumps(payload.get('payload', {}), allow_nan=False)
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
                result = {'success': response.success, 'reason_code': response.reason_code,
                          'message': response.message,
                          'session': json.loads(response.session_json) if response.session_json else None}
            # Broadcast compact session-scoped progress, never datasets or images.
            session = result.get('session') or {}
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
