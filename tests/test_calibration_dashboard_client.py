from concurrent.futures import Future
from types import SimpleNamespace as NS
from rebotarm_dashboard.calibration_client import CalibrationClient


def test_unavailable_and_success_are_explicit():
    class Client:
        ready = False
        def service_is_ready(self): return self.ready
        def call_async(self, request):
            assert request.command == 'status'
            f = Future()
            f.set_result(NS(success=True, reason_code='OK', message='ok',
                session_json='{"schema_version":1,"session_id":"one","state":"active","revision":2}'))
            return f
    client = Client()
    events = []
    adapter = CalibrationClient(client, NS, events.append)
    assert adapter.command({'command':'status'})['reason_code'] == 'UNAVAILABLE'
    client.ready = True
    assert adapter.command({'command':'status'})['session']['revision'] == 2
    assert events[0]['busy'] and not events[-1]['busy']


def test_transport_exception_clears_busy():
    def fail(_): raise RuntimeError('connection lost')
    events=[]
    adapter=CalibrationClient(NS(service_is_ready=lambda:True,call_async=fail),NS,events.append)
    assert adapter.command({'command':'status'})['reason_code']=='TRANSPORT_ERROR'
    assert events[-1]['busy'] is False
    assert not adapter.lock.locked()


def test_invalid_payload_is_not_reported_as_transport_failure():
    import pytest
    adapter = CalibrationClient(NS(service_is_ready=lambda: True), NS, lambda _: None)
    for payload in ({'command': 'status', 'payload': None},
                    {'command': 'status', 'payload': {'value': float('nan')}},
                    {'command': 'status', 'expected_revision': -1},
                    {'command': 'status', 'expected_revision': 2**63}):
        with pytest.raises(ValueError):
            adapter.command(payload)
    assert not adapter.lock.locked()


def test_invalid_response_json_is_reported_separately():
    future = Future()
    future.set_result(NS(success=True, reason_code='OK', message='ok', session_json='{'))
    adapter = CalibrationClient(NS(service_is_ready=lambda: True, call_async=lambda _: future), NS, lambda _: None)
    assert adapter.command({'command': 'status'})['reason_code'] == 'INVALID_RESPONSE'


def test_calibration_wait_does_not_block_operator_stop():
    import threading
    from rebotarm_dashboard.status_panel_api import dispatch_post_request
    entered, release = threading.Event(), threading.Event()
    def slow(payload):
        entered.set()
        assert release.wait(2)
        return {}
    node=NS(_operator_request_lock=threading.RLock(),_calibration_gravity_owned=False,_handle_calibration_command=slow,
            _handle_stop_execute=lambda:{'stopped':True})
    thread=threading.Thread(target=lambda:dispatch_post_request(node,'/api/calibration/command',lambda:{}))
    thread.start()
    try:
        assert entered.wait(1)
        assert node._operator_request_lock.acquire(timeout=.1)
        node._operator_request_lock.release()
        assert dispatch_post_request(node,'/api/stop_execute',lambda:{})['stopped']
    finally:
        release.set();thread.join()


def test_preview_bytes_are_not_broadcast_in_status():
    f=Future();f.set_result(NS(success=True,reason_code='OK',message='ok',
        session_json='{"schema_version":1,"session_id":"","state":"preview","preview":{"jpeg_base64":"test-image"}}'))
    events=[]
    adapter=CalibrationClient(NS(service_is_ready=lambda:True,call_async=lambda _:f),NS,events.append)
    result=adapter.command({'command':'preview'})
    assert result['session']['preview']['jpeg_base64']=='test-image'
    assert 'session' not in events[-1]


def test_timeout_rejects_followup_until_original_request_finishes():
    pending=Future();calls=[]
    def send(request):
        calls.append(request)
        return pending
    events=[]
    adapter=CalibrationClient(NS(service_is_ready=lambda:True,call_async=send),NS,events.append,timeout=.001)
    request={'command':'capture','session_id':'one'}
    assert adapter.command(request)['reason_code']=='TIMEOUT'
    assert events[-1]['busy'] and events[-1]['session_id']=='one'
    assert adapter.command(request)['reason_code']=='PENDING'
    assert len(calls)==1
    pending.set_result(NS(success=True,reason_code='OK',message='ok',
        session_json='{"schema_version":1,"session_id":"one","state":"active","revision":1}'))
    assert adapter.command({'command':'status','session_id':'one'})['success']
    assert not events[-1]['busy']


def test_response_envelope_identifies_failed_session():
    events=[]
    adapter=CalibrationClient(NS(service_is_ready=lambda:False),NS,events.append)
    result=adapter.command({'command':'status','session_id':'a'*32})
    assert result['schema_version']==1
    assert result['session_id']=='a'*32
    assert result['state']=='unavailable' and result['revision'] is None


def test_corrupt_success_response_is_rejected():
    future = Future()
    future.set_result(NS(success=True, reason_code='OK', message='ok', session_json='{"revision":2}'))
    events = []
    adapter = CalibrationClient(NS(service_is_ready=lambda: True, call_async=lambda _: future), NS, events.append)
    result = adapter.command({'command': 'status', 'session_id': 'a' * 32})
    assert not result['success'] and result['reason_code'] == 'INVALID_RESPONSE'
    assert events[-1]['success'] is False


def test_failed_response_keeps_persisted_revision():
    future = Future()
    future.set_result(NS(success=False, reason_code='COMMAND_REJECTED', message='stale revision',
        session_json='{"schema_version":1,"session_id":"one","state":"active","revision":4}'))
    adapter = CalibrationClient(NS(service_is_ready=lambda: True, call_async=lambda _: future), NS, lambda _: None)
    result = adapter.command({'command': 'capture', 'session_id': 'one'})
    assert result['revision'] == 4 and result['state'] == 'active'
