"""Execute shipped JS with DOM/transport doubles, including deferred requests."""
from pathlib import Path
import shutil
import subprocess
import pytest


def test_stop_remains_available_while_capture_waits(tmp_path):
    node = shutil.which('node')
    if not node:
        pytest.skip('Node.js unavailable')
    page = (Path(__file__).resolve().parents[1] / 'src/rebotarm_dashboard/rebotarm_dashboard/status_panel_assets/calibration.html').read_text()
    script = page.split('<script>')[1].split('</script>')[0]
    harness = r'''
const assert=require('node:assert/strict');
const elements=new Map();
function element(id){if(!elements.has(id))elements.set(id,{id,value:'',disabled:false,replaceChildren(){this.children=[];},append(...children){this.children=(this.children||[]).concat(children);},textContent:'',dataset:{}});return elements.get(id);}
for(const id of ['create','restore','capture','gravity-stop','gravity-start','enable-arm','safe-home','disable-arm','apply-exclusions','abort','accept'])element(id);
global.document={getElementById:element,createElement:()=>({style:{},replaceChildren(){this.children=[];},append(...children){this.children=(this.children||[]).concat(children);},set id(value){this._id=value;elements.set(value,this);},get id(){return this._id;}}),querySelectorAll:selector=>selector==='button'?[...elements.values()]:selector==='.outlier-choice:checked'?(element('outlier-options').children||[]).flatMap(label=>label.children||[]).filter(input=>input.checked):[]};
element('outlier-options').replaceChildren=function(){this.children=[];};
global.localStorage={getItem:()=>null,setItem(){}};
global.crypto={randomUUID:()=> '12345678-1234-1234-1234-123456789abc'};
const events={};global.EventSource=class{addEventListener(name,callback){events[name]=callback;}};
const timers=[];global.setInterval=(callback,delay)=>{timers.push({callback,delay});return timers.length;};
global.confirm=()=>true;
let resolveCapture;const calls=[];
global.fetch=(path,options)=>{calls.push([path,JSON.parse(options.body)]);if(path==='/api/calibration/command')return new Promise(resolve=>resolveCapture=resolve);return Promise.resolve({json:async()=>({accepted:true,message:'hold'})});};
'''
    assertions = r'''
(async()=>{
 streamFresh=true;feedbackFresh=true;armStatus={enabled:true,state_machine:'IDLE'};session={state:'active'};
 assert.equal(Number(element('marker_length_m').value),.075);
 checklist({image:{state:'passed',detail:'fresh'},camera_info:{state:'failed',detail:'missing'}});
 assert.equal(element('preflight-list').children.length,5);
 assert.equal(element('preflight-list').children[0].className,'preflight-row passed');
 assert.equal(element('preflight-list').children[1].className,'preflight-row failed');
 assert.equal(element('preflight-list').children[2].className,'preflight-row pending');
 const pending=command('capture',{split:'training'});
 assert.equal(element('gravity-stop').disabled,false);
 assert.equal(element('capture').disabled,true);
 assert.ok(element('capture-result').textContent.includes('采集中'));
 await command('capture',{split:'training'});
 assert.equal(calls.length,1);
 assert.ok(element('capture-result').textContent.includes('正在处理'));
 await element('gravity-stop').onclick();
 assert.equal(calls[1][0],'/api/calibration/gravity');
 assert.equal(calls[1][1].command,'stop');
 resolveCapture({json:async()=>({success:false,message:'reprojection error too large'})});
 await pending;
 assert.equal(element('capture').disabled,false);
 assert.equal(calls.length,2);
 assert.ok(element('capture-result').textContent.includes('标记重投影误差过大'));
 const fixture={schema_version:1,session_id:'a'.repeat(32),revision:4,state:'solved',metadata:{camera_frame:'optical',marker_frame:'board_1',marker_length_m:0.08,marker_id:7,dictionary:'DICT_5X5_100'},training_samples:[{sample_id:'t',base_to_end:{translation:[1,2,3]}}],validation_samples:[{sample_id:'v'}],report:{passed:false,accepted:false,deployed:false,methods:{PARK:{passed:false,error:'test rejection'}}}};
 show(fixture);
 assert.equal(element('mode').disabled,true);
 assert.equal(element('marker_length_m').disabled,true);
 assert.ok(element('count').textContent.includes('有效'));
 for(const id of ['marker_frame','marker_length_m','marker_id','dictionary'])assert.ok(elements.has(id),id+' input missing');
 assert.equal(element('marker_frame').value,'board_1');
 assert.equal(element('marker_length_m').value,0.08);
 assert.equal(element('marker_id').value,7);
 assert.equal(element('dictionary').value,'DICT_5X5_100');
 await element('safe-home').onclick();
 assert.equal(calls.at(-1)[0],'/api/arm_safe_home');
 await element('disable-arm').onclick();
 assert.equal(calls.at(-1)[0],'/api/arm_disable');
 assert.equal(elements.has('download'),false);
 element('marker_frame').value='board_2';element('marker_length_m').value='0.12';element('marker_id').value='9';element('dictionary').value='DICT_6X6_250';
 element('create').onclick();
 const created=calls.at(-1)[1];
 assert.equal(created.command,'create');
 assert.deepEqual([created.payload.metadata.marker_frame,created.payload.metadata.marker_length_m,created.payload.metadata.marker_id,created.payload.metadata.dictionary],['board_2',0.12,9,'DICT_6X6_250']);
 resolveCapture({json:async()=>({success:false,message:'test'})});
 await new Promise(resolve=>setImmediate(resolve));
 const flagged={...fixture,state:'solved',report:{passed:false,accepted:false,deployed:false,selected_method:'PARK',methods:{PARK:{training:{diagnostics:{flagged_samples:[{label:'t',reasons:['relative_outlier']}]}},validation:{diagnostics:{flagged_samples:[]}}}}}};
 show(flagged);
 assert.equal(element('exclusion-panel').hidden,false);
 assert.equal(element('outlier-options').children.length,1);
 element('outlier-options').children[0].children[0].checked=true;
 element('exclusion-reason').value='bad observation';
 element('apply-exclusions').onclick();
 const exclusion=calls.at(-1)[1];
 assert.equal(exclusion.command,'exclude_samples');
 assert.deepEqual(exclusion.payload,{sample_ids:['t'],reason:'bad observation',evidence:''});
 resolveCapture({json:async()=>({success:false,message:'test'})});
 await new Promise(resolve=>setImmediate(resolve));
 const countBefore=calls.length;
 armBusy=true;
 updateControls();
 assert.equal(element('capture').disabled,true);
 await element('safe-home').onclick();
 assert.equal(calls.length,countBefore);
 armBusy=false;feedbackFresh=false;updateControls();
 assert.equal(element('safe-home').disabled,true);
 assert.equal(element('gravity-stop').disabled,false);
 await element('safe-home').onclick();
 assert.equal(calls.length,countBefore);
 feedbackFresh=true;armUnknown=true;updateControls();
 assert.equal(element('capture').disabled,true);
 await element('capture').onclick();
 assert.equal(calls.length,countBefore);
 assert.ok(element('capture-result').textContent.includes('未知动作状态'));
 armUnknown=false;
 events.status({data:JSON.stringify({arm:{enabled:true,state_machine:'GRAVITY_COMP',mode:'mit',error_codes:['GRAVITY_HOLD: three consecutive control deadline misses']},teleop:{arm_feedback:{fresh:true},gravity_comp:{active:true}}})});
 assert.ok(element('arm-state').textContent.includes('故障保持'));
 assert.ok(element('arm-state').textContent.includes('连续三个控制周期超时'));
 assert.equal(element('gravity-start').disabled,true);
 assert.equal(element('enable-arm').disabled,true);
 assert.equal(element('gravity-stop').disabled,false);
 await element('gravity-stop').onclick();
 assert.equal(calls.at(-1)[1].command,'stop');
 element('copy-session').onclick();
 assert.equal(session,null);
 assert.equal(element('marker_length_m').disabled,false);

 // 直接运行实际页面轮询：采集忙时仍估测，失效/超时/会话变化均清除旧值。
 session={...fixture,state:'active',metadata:{mode:'handeye'}};activeStep='capture-panel';
 let now=1000;const realNow=Date.now;Date.now=()=>now;
 const observation={distance_m:.3,translation_m:[0,0,.3],age_sec:.01,maximum_age_sec:.5,
   image_stamp_ns:100,frame_id:'optical',marker_length_m:.075,reprojection_rmse_px:.2};
 let distanceCalls=0;
 const reply=p=>({success:true,session_id:fixture.session_id,session:{observation:p}});
 global.fetch=async(path,options)=>{distanceCalls++;assert.equal(JSON.parse(options.body).command,'observe');return {json:async()=>reply(observation)};};
 busy=true;armBusy=true;
 await refreshDistance();
 assert.ok(element('marker-distance').textContent.includes('300.0 mm'));
 assert.ok(element('marker-distance').textContent.includes('75.0 mm'));
 assert.equal(busy,true);assert.equal(armBusy,true);
 now=1600;timers.find(t=>t.delay===100).callback();
 assert.ok(element('marker-distance').textContent.includes('已过期'));
 await refreshDistance();
 assert.ok(!element('marker-distance').textContent.includes('300.0'));
 observation.image_stamp_ns=200;await refreshDistance();
 assert.ok(element('marker-distance').textContent.includes('300.0 mm'));
 global.fetch=async()=>({json:async()=>({success:false,message:'target ArUco marker not detected'})});
 await refreshDistance();
 assert.ok(element('marker-distance').textContent.includes('未检测到目标'));
 assert.ok(!element('marker-distance').textContent.includes('300.0'));
 let releaseDistance;
 global.fetch=()=>{distanceCalls++;return new Promise(resolve=>releaseDistance=resolve);};
 const observing=refreshDistance();const beforeDuplicate=distanceCalls;
 await refreshDistance();assert.equal(distanceCalls,beforeDuplicate);
 element('copy-session').onclick();
 releaseDistance({json:async()=>reply({...observation,image_stamp_ns:300})});await observing;
 assert.ok(!element('marker-distance').textContent.includes('300.0'));
 session={...fixture,metadata:{mode:'tcp'}};
 await refreshDistance();assert.equal(distanceCalls,beforeDuplicate);
 assert.ok(element('marker-distance').textContent.includes('TCP'));
 session={...fixture,metadata:{mode:'handeye'}};
 activeStep='capture-panel';
 document.hidden=true;await refreshDistance();assert.equal(distanceCalls,beforeDuplicate);
 document.hidden=false;
 global.fetch=async()=>({json:async()=>reply({...observation,image_stamp_ns:400,age_sec:.6})});
 await refreshDistance();assert.ok(element('marker-distance').textContent.includes('已过期'));
 global.fetch=async()=>{throw Error('offline');};
 await refreshDistance();assert.ok(element('marker-distance').textContent.includes('请求失败'));
 Date.now=realNow;

})().catch(e=>{console.error(e);process.exitCode=1});
'''
    file = tmp_path / 'page.cjs'
    file.write_text(harness + script + assertions)
    result = subprocess.run([node, str(file)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
