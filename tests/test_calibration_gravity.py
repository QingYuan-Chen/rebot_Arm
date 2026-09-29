from rebotarm_dashboard.calibration_gravity import gravity_rejection


def test_gravity_requires_fresh_explicit_healthy_idle_state():
    args = dict(hardware=True, web_enabled=True, execution_mode='execute',
                arm={'enabled':True,'state_machine':'IDLE','error_codes':[]},status_age=.1,busy=False)
    assert gravity_rejection('start',True,**args) is None
    for key,value in [('hardware',False),('status_age',1.),('busy',True),('execution_mode','dry_run')]:
        assert gravity_rejection('start',True,**dict(args,**{key:value}))
    assert gravity_rejection('start',False,**args)
    assert gravity_rejection('stop',True,**args)
    args['arm']['state_machine']='GRAVITY_COMP'
    assert gravity_rejection('stop',True,**args) is None


def test_dispatch_blocks_conflicting_start():
    from rebotarm_dashboard.status_panel_api import dispatch_post_request
    from types import SimpleNamespace
    node=SimpleNamespace(_calibration_gravity_owned=True)
    assert dispatch_post_request(node,'/api/keyboard_enable',lambda:{})['state']=='blocked'


def test_dispatch_allows_teach_record_stop_during_gravity_compensation():
    from rebotarm_dashboard.status_panel_api import dispatch_post_request
    from types import SimpleNamespace

    node = SimpleNamespace(
        _calibration_gravity_owned=True,
        _handle_teach_record_stop=lambda: {"accepted": True, "state": "stopped"},
    )
    result = dispatch_post_request(node, '/api/teach_record_stop', lambda: {})
    assert result == {"accepted": True, "state": "stopped"}


def test_dispatch_allows_teach_dry_run_during_gravity_compensation():
    from rebotarm_dashboard.status_panel_api import dispatch_post_request
    from types import SimpleNamespace

    node = SimpleNamespace(
        _calibration_gravity_owned=True,
        _handle_teach_dry_run=lambda payload: {"accepted": True, "state": "dry_run"},
    )
    result = dispatch_post_request(node, '/api/teach_dry_run', lambda: {})
    assert result == {"accepted": True, "state": "dry_run"}


def test_timeout_retains_future_and_requires_post_completion_feedback():
    from concurrent.futures import Future
    from types import SimpleNamespace as NS
    from rebotarm_dashboard.calibration_gravity import GravityRequestTracker
    future=Future();calls=[]
    def send(request): calls.append(request);return future
    client=NS(service_is_ready=lambda:True,call_async=send)
    tracker=GravityRequestTracker()
    assert not tracker.call(client,object(),'start',timeout=.001)[0]
    assert not tracker.call(client,object(),'start',timeout=.001)[0]
    assert len(calls)==1
    arm={'enabled':True,'error_codes':[],'state_machine':'IDLE'}
    assert not tracker.idle_confirmed(now=1.,received_at=1.,arm=arm)
    future.set_result(NS(success=False,message='rejected'))
    assert not tracker.refresh(2.)
    assert not tracker.idle_confirmed(now=2.,received_at=1.9,arm=arm)
    assert tracker.idle_confirmed(now=2.2,received_at=2.1,arm=arm)
    assert not tracker.idle_confirmed(now=3.,received_at=2.1,arm=arm)
