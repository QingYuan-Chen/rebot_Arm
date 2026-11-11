import pytest
from rebotarm_calibration.session_store import SessionStore
from test_handeye_workflow import dataset


def test_legacy_metadata_defaults_are_normalized_without_rewriting(tmp_path):
    import json
    store = SessionStore(tmp_path, image_topic='/custom/image', camera_info_topic='/custom/info')
    created = store.create(dataset()['metadata'])
    path = tmp_path / (created['session_id'] + '.json')
    legacy = json.loads(path.read_text())
    for key in ('mode', 'image_topic', 'camera_info_topic', 'dictionary', 'marker_id'):
        legacy['metadata'].pop(key, None)
    path.write_text(json.dumps(legacy))
    before = path.read_bytes()
    restored = store.read(created['session_id'])
    assert restored['metadata']['mode'] == 'handeye'
    assert restored['metadata']['image_topic'] == '/custom/image'
    assert path.read_bytes() == before
    assert store.create(dataset()['metadata'], session_id=created['session_id']) == restored


@pytest.mark.parametrize('field,value', [
    ('metadata', None), ('revision', -1), ('training_samples', None),
    ('requests', []), ('state', 'bogus'), ('report', []),
])
def test_corrupt_saved_session_is_rejected(tmp_path, field, value):
    import json
    store = SessionStore(tmp_path)
    created = store.create(dataset()['metadata'])
    path = tmp_path / (created['session_id'] + '.json')
    data = json.loads(path.read_text())
    data[field] = value
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='invalid'):
        store.read(created['session_id'])


def test_nonfinite_saved_session_is_rejected(tmp_path):
    store = SessionStore(tmp_path)
    created = store.create(dataset()['metadata'])
    path = tmp_path / (created['session_id'] + '.json')
    content = path.read_text().replace('"training_samples": []', '"training_samples": [NaN]')
    assert content != path.read_text()
    path.write_text(content)
    with pytest.raises(ValueError, match='invalid saved session number'):
        store.read(created['session_id'])


def test_durable_workflow_retry_accept_and_restart(tmp_path):
    source = dataset()
    store = SessionStore(tmp_path)
    data = store.create(source['metadata'])
    sid = data['session_id']
    for split in ('training', 'validation'):
        for sample in source[split + '_samples']:
            revision = data['revision']
            payload = {'split': split, 'sample': sample}
            data = store.mutate(sid, request_id=sample['sample_id'], revision=revision,
                                operation='capture', payload=payload)
            assert store.mutate(sid, request_id=sample['sample_id'], revision=revision,
                                operation='capture', payload=payload) == data
    store = SessionStore(tmp_path)
    data = store.mutate(sid, request_id='solve', revision=data['revision'], operation='solve')
    assert data['report']['passed']
    with pytest.raises(ValueError, match='confirmation'):
        store.mutate(sid, request_id='accept', revision=data['revision'], operation='accept')
    accepted = store.mutate(sid, request_id='accept', revision=data['revision'], operation='accept',
                            payload={'confirmed': True, 'operator': 'test'})
    assert accepted['state'] == 'accepted'
    assert accepted['report']['deployed'] is False
    with pytest.raises(ValueError, match='terminal'):
        store.mutate(sid, request_id='again', revision=accepted['revision'], operation='solve')


def test_stale_requests_and_bad_paths_do_not_mutate(tmp_path):
    store = SessionStore(tmp_path)
    data = store.create(dataset()['metadata'])
    sid = data['session_id']
    with pytest.raises(ValueError, match='stale'):
        store.mutate(sid, request_id='bad', revision=1, operation='abort', payload={'reason':'test'})
    with pytest.raises(ValueError):
        store.read('../outside')
    assert store.read(sid) == data
    with pytest.raises(ValueError):
        store.mutate(sid, request_id='solve', revision=0, operation='solve')
    assert store.read(sid) == data


@pytest.mark.parametrize('changed', ['intrinsics', 'settings', 'missing'])
def test_capture_conditions_frozen_across_splits_and_restart(tmp_path, changed):
    from copy import deepcopy
    source=dataset();store=SessionStore(tmp_path)
    session=store.create(source['metadata']);sid=session['session_id']
    sample=deepcopy(source['training_samples'][0])
    sample['camera_info']={'k':[520.,0.,320.,0.,520.,240.,0.,0.,1.],'width':640,'height':480}
    sample['provenance']={'capture_settings':{'maximum_age_sec':.5}}
    session=store.mutate(sid,request_id='first',revision=0,operation='capture',payload={'split':'training','sample':sample})
    next_sample=deepcopy(sample)
    if changed=='intrinsics':next_sample['camera_info']['k'][0]=521.
    elif changed=='settings':next_sample['provenance']['capture_settings']['maximum_age_sec']=1.
    else:next_sample.pop('provenance')
    store=SessionStore(tmp_path)
    with pytest.raises(ValueError,match='conditions changed'):
        store.mutate(sid,request_id='second',revision=1,operation='capture',payload={'split':'validation','sample':next_sample})
    assert store.read(sid)==session
    next_sample=deepcopy(sample)
    next_sample['base_to_end']=source['training_samples'][1]['base_to_end']
    assert store.mutate(sid,request_id='second',revision=1,operation='capture',payload={'split':'validation','sample':next_sample})['revision']==2


def test_failed_report_cannot_be_accepted_and_reopen_invalidates_report(tmp_path):
    from copy import deepcopy
    source=dataset();store=SessionStore(tmp_path)
    current=store.create(source['metadata']);sid=current['session_id']
    for split in ('training','validation'):
        for sample in source[split+'_samples']:
            sample=deepcopy(sample)
            if split=='validation':sample['camera_to_marker']['translation'][0]+=.1
            current=store.mutate(sid,request_id=sample['sample_id'],revision=current['revision'],
                                 operation='capture',payload={'split':split,'sample':sample})
    current=store.mutate(sid,request_id='solve',revision=current['revision'],operation='solve')
    assert not current['report']['passed']
    with pytest.raises(ValueError,match='passing report'):
        store.mutate(sid,request_id='accept',revision=current['revision'],operation='accept',
                     payload={'confirmed':True,'operator':'test'})
    assert store.read(sid)==current
    current=store.mutate(sid,request_id='reopen',revision=current['revision'],operation='reopen')
    assert current['state']=='active' and current['report'] is None
    with pytest.raises(ValueError,match='passing report'):
        store.mutate(sid,request_id='accept',revision=current['revision'],operation='accept',
                     payload={'confirmed':True,'operator':'test'})


def test_returned_session_is_detached(tmp_path):
    store=SessionStore(tmp_path)
    source=dataset();current=store.create(source['metadata']);sid=current['session_id']
    current['metadata']['base_frame']='corrupted'
    assert store.read(sid)['metadata']['base_frame']==source['metadata']['base_frame']
    current=store.mutate(sid,request_id='first',revision=0,operation='capture',
                         payload={'split':'training','sample':source['training_samples'][0]})
    current['training_samples'][0]['base_to_end']['translation'][0]=999
    assert store.read(sid)['training_samples'][0]['base_to_end']['translation'][0]!=999


@pytest.mark.parametrize('split', ['training', 'validation'])
def test_duplicate_pose_rejected_before_persistence_and_retry_stays_idempotent(tmp_path, split):
    from copy import deepcopy
    store = SessionStore(tmp_path)
    source = dataset()
    data = store.create(source['metadata'])
    sid = data['session_id']
    payload = {'split': 'training', 'sample': source['training_samples'][0]}
    saved = store.mutate(sid, request_id='first', revision=0, operation='capture', payload=payload)
    raw = (tmp_path / (sid+'.json')).read_bytes()
    duplicate = deepcopy(payload['sample'])
    duplicate['camera_to_marker']['translation'][0] += .02
    with pytest.raises(ValueError, match='duplicate robot pose matches sample first'):
        SessionStore(tmp_path).mutate(sid, request_id='duplicate', revision=1,
            operation='capture', payload={'split': split, 'sample': duplicate})
    assert (tmp_path / (sid+'.json')).read_bytes() == raw
    assert store.mutate(sid, request_id='first', revision=0, operation='capture', payload=payload) == saved
    duplicate['base_to_end'] = source['training_samples'][1]['base_to_end']
    assert store.mutate(sid, request_id='different', revision=1, operation='capture',
                        payload={'split':split, 'sample':duplicate})['revision'] == 2


def test_audit_indices_and_retries_are_stable(tmp_path):
    from datetime import datetime
    store=SessionStore(tmp_path);source=dataset()
    current=store.create(source['metadata']);sid=current['session_id']
    assert datetime.fromisoformat(current['created_at']).tzinfo is not None
    first={'split':'training','sample':source['training_samples'][0]}
    current=store.mutate(sid,request_id='first',revision=0,operation='capture',payload=first)
    repeated=store.mutate(sid,request_id='first',revision=0,operation='capture',payload=first)
    assert repeated==current and len(current['audit'])==1
    current=store.mutate(sid,request_id='second',revision=1,operation='capture',
        payload={'split':'validation','sample':source['validation_samples'][0]})
    assert current['training_samples'][0]['sample_index']==0
    assert current['validation_samples'][0]['sample_index']==1
    assert [a['revision'] for a in current['audit']]==[1,2]
    assert current['audit'][-1]['at']==current['updated_at']
    assert SessionStore(tmp_path).read(sid)==current


def test_manual_outlier_exclusion_is_reversible_and_keeps_raw_samples(tmp_path):
    source=dataset();store=SessionStore(tmp_path)
    current=store.create(source['metadata']);sid=current['session_id']
    for split in ('training','validation'):
        for sample in source[split+'_samples']:
            current=store.mutate(sid,request_id=sample['sample_id'],revision=current['revision'],
                                 operation='capture',payload={'split':split,'sample':sample})
    current=store.mutate(sid,request_id='solve-1',revision=current['revision'],operation='solve')
    selected=current['report']['selected_method']
    assert selected
    # A valid dataset need not contain any outliers; inject one in a separate session below.
    with pytest.raises(ValueError,match='unknown sample_id'):
        store.mutate(sid,request_id='bad',revision=current['revision'],operation='exclude_samples',
                     payload={'sample_ids':['unknown'],'reason':'test'})
    unflagged=next(sample['sample_id'] for sample in source['training_samples']
                   if sample['sample_id'] not in {item['label'] for item in current['report']['methods'][selected]['training']['diagnostics']['flagged_samples']})
    with pytest.raises(ValueError,match='only reported outliers'):
        store.mutate(sid,request_id='not-flagged',revision=current['revision'],operation='exclude_samples',
                     payload={'sample_ids':[unflagged],'reason':'test'})
    assert store.read(sid)==current


def test_flagged_sample_requires_reason_and_cannot_bypass_minimum(tmp_path):
    from copy import deepcopy
    source=dataset();store=SessionStore(tmp_path)
    current=store.create(source['metadata']);sid=current['session_id']
    for split in ('training','validation'):
        for sample in source[split+'_samples']:
            sample=deepcopy(sample)
            if split=='validation' and sample is not None and sample['sample_id']==source['validation_samples'][0]['sample_id']:
                sample['camera_to_marker']['translation'][0]+=.1
            current=store.mutate(sid,request_id=sample['sample_id'],revision=current['revision'],
                                 operation='capture',payload={'split':split,'sample':sample})
    current=store.mutate(sid,request_id='solve',revision=current['revision'],operation='solve')
    assert current['report']['selected_method']
    suspect=source['validation_samples'][0]['sample_id']
    flags=current['report']['methods'][current['report']['selected_method']]['validation']['diagnostics']['flagged_samples']
    assert suspect in {flag['label'] for flag in flags}
    with pytest.raises(ValueError,match='reason'):
        store.mutate(sid,request_id='no-reason',revision=current['revision'],operation='exclude_samples',
                     payload={'sample_ids':[suspect]})
    with pytest.raises(ValueError,match='evidence'):
        store.mutate(sid,request_id='no-evidence',revision=current['revision'],operation='exclude_samples',
                     payload={'sample_ids':[suspect],'reason':'high residual'})
    changed=store.mutate(sid,request_id='exclude',revision=current['revision'],operation='exclude_samples',
                         payload={'sample_ids':[suspect],'reason':'observed board shift','evidence':'camera inspection log'})
    assert changed['state']=='active' and changed['report'] is None
    assert len(changed['validation_samples'])==len(source['validation_samples'])
    assert changed['excluded_sample_ids']==[suspect]
    assert changed['audit'][-1]['reason']=='observed board shift'
    filtered=store.mutate(sid,request_id='solve-after-exclusion',revision=changed['revision'],operation='solve')
    assert filtered['report']['excluded_sample_ids']==[suspect]
    assert not filtered['report']['passed']
    assert filtered['report']['final_validation_required']
    assert filtered['report']['final_validation_sample_count']==0
    assert changed['report_history'][-1]['report']==current['report']
    method=filtered['report']['selected_method']
    assert filtered['report']['methods'][method]['validation']['sample_count']==len(source['validation_samples'])-1
    restored=store.mutate(sid,request_id='restore',revision=filtered['revision'],operation='exclude_samples',
                          payload={'sample_ids':[],'reason':'restored for inspection','evidence':'reviewed original capture'})
    assert restored['excluded_sample_ids']==[]
    solved=store.mutate(sid,request_id='solve-restored',revision=restored['revision'],operation='solve')
    assert solved['report']['excluded_sample_ids']==[]


def test_excluded_sample_cannot_reduce_validation_below_five(tmp_path):
    from rebotarm_calibration.handeye_workflow import solve_dataset
    source=dataset()
    source['validation_samples']=source['validation_samples'][:5]
    source['excluded_sample_ids']=[source['validation_samples'][0]['sample_id']]
    with pytest.raises(ValueError,match='validation_samples requires at least five'):
        solve_dataset(source)
