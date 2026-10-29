"""夹爪现场回归客户端：单一监测与目标所有者、独立接收和分项授权请求。
失败停止保持，不自动失能；显式失能仅允许本轮基线新鲜且连续静止一秒后。
"""
import argparse, fcntl, json, math, os, queue, threading, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from rebotarm_msgs.msg import JointMotorState, ArmStatus
from moveit_msgs.srv import GetStateValidity
from control_msgs.action import GripperCommand
from rebotarm_motion.pose_kinematics import JOINT_NAMES
from hardware_regression_io import Audit
from hardware_regression_gate import RegressionGate, SustainedSpeed
from rebotarmcontroller.stationary_evidence import StationaryEvidence, DisabledTransitionEvidence
from rebotarmcontroller.gripper_control import _G_POSITION_MAX_SPEED_RAD_S

if not __debug__:
    raise RuntimeError('现场回归不允许使用 Python 优化模式')

parser=argparse.ArgumentParser(description='单一监测者的现场回归；不会自动使能或运动')
parser.add_argument('session', type=Path, help='包含 baseline.json 和 controller_audit.jsonl 的本轮目录')
ROOT=parser.parse_args().session.resolve()
# 同一用户在本机只允许一个本工具实例；旧临时脚本不能与本工具同时运行。
owner_file=open(f'/tmp/rebotarm-hardware-regression-{os.getuid()}.lock', 'a+')
try: fcntl.flock(owner_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError: raise RuntimeError('已有现场回归所有者')
base=json.loads((ROOT/'baseline.json').read_text());qbase=np.array([base[n] for n in JOINT_NAMES])
if not all(math.isfinite(base[n]) for n in (*JOINT_NAMES, 'gripper')):
    raise ValueError('本轮基线必须包含七路有限数值')
if not 0. <= base['gripper'] <= .085:
    raise ValueError('夹爪基线必须允许在 0–90 mm 行程内张开 5 mm')
audit=Audit(ROOT/'received.jsonl');events=Audit(ROOT/'test_events.jsonl')
rclpy.init();node=rclpy.create_node('guarded_hardware_regression')
feedback_node=rclpy.create_node('guarded_hardware_regression_feedback')
executor=SingleThreadedExecutor();executor.add_node(node)
feedback_executor=SingleThreadedExecutor();feedback_executor.add_node(feedback_node)
lock=threading.RLock();latest={};status=None;phase='discovery';closed=False
handles=[];names=list(JOINT_NAMES)+['gripper'];stop_monitor=threading.Event()
raw_lock=threading.RLock();raw_latest={};raw_errors=[];raw_stop=threading.Event()
feedback_error=[];command_error=[]
critical_events=queue.SimpleQueue();critical_audit_failed=threading.Event()
transition_holder={'value':None}
speed_history=SustainedSpeed(.07, _G_POSITION_MAX_SPEED_RAD_S)
def critical_audit_writer():
    try:
        while True:
            item=critical_events.get()
            if item is None:return
            event,event_phase,at_mono,values=item
            events.record(event,phase=event_phase,at_mono=at_mono,**values)
    except Exception:
        critical_audit_failed.set()
critical_writer=threading.Thread(target=critical_audit_writer,name='critical-audit',daemon=True)
critical_writer.start()
def record_critical(event,**values):
    # 本轮只产生有限次故障交接记录；审计排队不能等待写盘或阻塞停止/取消。
    try:critical_events.put((event,phase,time.monotonic(),values))
    except Exception:critical_audit_failed.set()
def emit(event,**v):
    events.record(event,phase=phase,**v)
    print(json.dumps(dict(event=event,phase=phase,mono=time.monotonic(),**v),ensure_ascii=False),flush=True)
def raw_snapshot_locked():
    if raw_errors:raise RuntimeError(f'原始反馈审计读取失败: {raw_errors}')
    if any(n not in raw_latest for n in names):raise RuntimeError('七路原始反馈未齐')
    values=[dict(raw_latest[n]) for n in names]
    observed=min(x['observed_at'] for x in values)
    now=time.monotonic()
    if not 0 <= now-observed <= .15:raise RuntimeError('原始反馈过期')
    return values,observed,now

def raw_reader():
    try:
        with (ROOT/'controller_audit.jsonl').open('rb') as stream:
            stream.seek(0,2)
            pending=b'';skip_first=True
            while not raw_stop.is_set():
                chunk=stream.read(65536)
                if not chunk:
                    raw_stop.wait(.002);continue
                pending+=chunk
                lines=pending.split(b'\n');pending=lines.pop()
                for line in lines:
                    if skip_first:
                        skip_first=False;continue
                    row=json.loads(line)
                    if row.get('event') != 'verified_feedback':continue
                    x=row['data']
                    if x['name'] not in names:continue
                    with raw_lock:
                        raw_latest[x['name']]=x
                        transition=transition_holder['value']
                        if transition is not None:
                            try:
                                data,stamp,now=raw_snapshot_locked()
                                transition.observe([v['position'] for v in data],
                                    [v['velocity'] for v in data],[v['status'] for v in data],
                                    [v['sequence'] for v in data],stamp,now)
                            except Exception as exc:
                                if not transition.failure:transition.failure=str(exc)
    except Exception as exc:
        with raw_lock:raw_errors.append(str(exc))

raw_thread=threading.Thread(target=raw_reader,name='raw-feedback-audit-reader',daemon=True)
raw_thread.start()
def motor(name):
    def receive(msg,info):
        value=dict(stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
            position=msg.position,velocity=msg.velocity,torque=msg.torque,code=msg.status_code,
            received=time.monotonic(),dds_source=info['source_timestamp'],dds_received=info['received_timestamp'])
        with lock:latest[name]=value
        audit.record('motor',phase=phase,name=name,**value)
    return receive
def arm(msg,info):
    global status
    value=dict(stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,received=time.monotonic(),
        enabled=msg.enabled,loop=msg.control_loop_active,mode=msg.mode,state=msg.state_machine,
        codes=list(msg.per_joint_status_code),errors=list(msg.error_codes))
    with lock:status=value
    audit.record('status',phase=phase,**value)
subscriptions=[feedback_node.create_subscription(JointMotorState,'/rebotarm/gripper/state' if n=='gripper'
    else '/rebotarm/joints/'+n+'/state',motor(n),qos_profile_sensor_data) for n in names]
subscriptions.append(feedback_node.create_subscription(ArmStatus,'/rebotarm/arm_status',arm,
    QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
services={n:node.create_client(Trigger,'/rebotarm/'+n) for n in ('enable','disable','trajectory_stop')}
validity=node.create_client(GetStateValidity,'/check_state_validity')
gripper=ActionClient(node,GripperCommand,'/rebotarm/gripper/command')
def spin_feedback():
    try:feedback_executor.spin()
    except Exception as exc:
        with lock:feedback_error.append(str(exc))
        events.record('feedback_executor_error',phase=phase,error=str(exc))
def spin_commands():
    try:executor.spin()
    except Exception as exc:
        with lock:command_error.append(str(exc))
        events.record('command_executor_error',phase=phase,error=str(exc))
spin=threading.Thread(target=spin_commands,name='regression-commands',daemon=True);spin.start()
feedback_spin=threading.Thread(target=spin_feedback,name='regression-feedback',daemon=True);feedback_spin.start()
def snap(gripper_active=None, inspect_speed=False):
    if gripper_active is None:gripper_active=gate.gripper_motion
    with lock:
        v,s=dict(latest),None if status is None else dict(status)
        receiver_error,commands_error=list(feedback_error),list(command_error)
    now=time.monotonic();wall=feedback_node.get_clock().now().nanoseconds
    ages={n:(wall-x['stamp'])/1e9 for n,x in v.items()}
    issues=[]
    if len(v)!=7 or s is None:issues.append('incomplete')
    if receiver_error or not feedback_spin.is_alive():issues.append('feedback_executor_failed')
    if commands_error or not spin.is_alive():issues.append('command_executor_failed')
    if any(not -.02<=a<=.15 for a in ages.values()):issues.append('source_stale')
    if s is not None and (now-s['received']>.6 or s['errors']):issues.append('status_stale_or_error')
    with raw_lock:
        if raw_errors:issues.append('raw_audit_reader_failed')
        else:
            try:raw_snapshot_locked()
            except RuntimeError:issues.append('raw_feedback_incomplete_or_stale')
    if (audit.failed or events.failed or critical_audit_failed.is_set()
            or not critical_writer.is_alive()):issues.append('client_audit_failed')
    for n,x in v.items():
        if x['code'] not in (0,1) or not all(math.isfinite(x[k]) for k in ('position','velocity','torque')):
            issues.append('invalid_'+n)
        if abs(x['position']-base[n])>(.0065 if n=='gripper' else .005):
            issues.append('position_scope_'+n)
        # 速度峰保留在接收审计；按独立源帧判断持续超限，不能凭重读缓存累计。
        if inspect_speed:
            if speed_history.observe(n,x['stamp'],x['velocity'],gripper_active):
                issues.append('sustained_speed_'+n)
    return v,s,ages,issues
def health(code=None,history=True):
    v,s,ages,issues=snap()
    assert not issues,(issues,ages,s)
    if history:assert not gate.fault,'本轮已记录监测失败，需单独诊断'
    if code is not None:
        assert s['enabled']==bool(code) and s['loop']==bool(code),s
        assert s['codes']==[code]*6 and all(x['code']==code for x in v.values()),(s,v)
    return np.array([v[n]['position'] for n in JOINT_NAMES])
def request_hold():
    # 无论异常发生在动作接纳前后均只请求停止保持，迟到夹爪句柄由 gate 取消。
    record_critical('monitor_failure',issues=gate.fault,fault_mono=gate.fault_at)
    record_critical('monitor_stop_call_start')
    try:
        future=services['trajectory_stop'].call_async(Trigger.Request())
        events.record('monitor_stop_call_return',phase=phase)
        def done(f):
            try:
                result=f.result();events.record('monitor_stop_response',phase=phase,success=result.success,message=result.message)
            except Exception as exc:events.record('monitor_stop_error',phase=phase,error=str(exc))
        future.add_done_callback(done)
    except Exception as exc:events.record('monitor_stop_error',phase=phase,error=str(exc))

def record_cancel(future, error):
    if error is not None:
        events.record('monitor_cancel_error',phase=phase,error=error)
        return
    events.record('monitor_cancel_requested',phase=phase)
    def done(f):
        try:
            result=f.result()
            events.record('monitor_cancel_response',phase=phase,return_code=result.return_code,
                          count=len(result.goals_canceling))
        except Exception as exc:events.record('monitor_cancel_error',phase=phase,error=str(exc))
    future.add_done_callback(done)

def record_cancel_start():
    record_critical('monitor_cancel_call_start')

gate=RegressionGate(request_hold,on_cancel=record_cancel,on_cancel_start=record_cancel_start)
old_names={'current_workspace_hardware_regression','hardware_regression_recovery'}

def observe_faults():
    previous=None
    next_owner_check=0.
    while not stop_monitor.wait(.005):
        def check(gripper_active):
            nonlocal next_owner_check
            v,s,ages,issues=snap(gripper_active,inspect_speed=True)
            if time.monotonic() >= next_owner_check:
                if any(n in old_names for n,_ in node.get_node_names_and_namespaces()):
                    issues.append('legacy_monitor_present')
                next_owner_check=time.monotonic()+.2
            audit.record('health',phase=phase,ages=ages,issues=issues,
                         heartbeat_age=None if s is None else time.monotonic()-s['received'])
            return issues
        issues=gate.monitor_check(check)
        if issues!=previous and issues:emit('health_issues',issues=issues)
        previous=issues

def watchdog():
    while not stop_monitor.wait(.02):gate.watchdog()
def wait(future,timeout=12.,code=None,history=True):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:
        health(code,history);time.sleep(.005)
    assert future.done(),'请求结果未知，禁止重试'
    return future.result()
def unique(path,action=False,expected='reBotArmController'):
    owners=[]
    for name,namespace in set(node.get_node_names_and_namespaces()):
        values=(get_action_server_names_and_types_by_node(node,name,namespace) if action
                else node.get_service_names_and_types_by_node(name,namespace))
        if any(path==p for p,_ in values):owners.append((name,namespace))
    assert owners==[(expected,'/')],(path,owners)
    emit('unique_owner',interface=path,owners=owners)
def call(name,history=True,transition=False):
    client=services[name];unique('/rebotarm/'+name)
    assert client.wait_for_service(timeout_sec=3.)
    emit('service_request',name=name)
    future=(gate.guarded_call(lambda:client.call_async(Trigger.Request())) if name=='enable'
            else client.call_async(Trigger.Request()))
    def record_response(f):
        try:
            result=f.result();emit('service_async_response',name=name,success=result.success,message=result.message)
        except Exception as exc:emit('service_async_error',name=name,error=str(exc))
    future.add_done_callback(record_response)
    result=wait(future,history=history)
    emit('service_result',name=name,success=result.success,message=result.message)
    assert result.success,result.message
    time.sleep(.2)
def observe(seconds,code,history=True):
    end=time.monotonic()+seconds
    while time.monotonic()<end:health(code,history);time.sleep(.005)
def stationary(target=qbase,seconds=1.,history=True,tolerance=.003):
    end=time.monotonic()+10.;evidence=None
    while time.monotonic()<end:
        q=health(1,history)
        with raw_lock:
            data,stamp,now=raw_snapshot_locked()
        positions=[x['position'] for x in data]
        velocities=[x['velocity'] for x in data]
        sequences=[x['sequence'] for x in data]
        if np.max(abs(q-target))>tolerance:
            if evidence is not None:raise RuntimeError('静止确认期间偏离目标范围')
            time.sleep(.005);continue
        if any(x['status']!=1 for x in data):raise RuntimeError('静止确认期间原始状态非使能')
        if evidence is None:
            if max(map(abs,velocities))>.07:
                time.sleep(.005);continue
            evidence=StationaryEvidence(window=seconds,velocity_limit=.07,joints=7)
            evidence.reset(positions)
        reason=evidence.observe(positions,velocities,stamp,now,sequences=sequences)
        if reason in ('invalid_or_stale','source_gap','source_regressed','anchor_displacement'):
            raise RuntimeError('静止证据失败: '+reason)
        if evidence.confirmed(now):
            emit('stationary',target=list(map(float,target)),actual=q.tolist(),duration=seconds,
                 raw_anchor=evidence.anchor.tolist(),raw_sequences=sequences)
            return q
        time.sleep(.005)
    raise RuntimeError('新鲜连续基线/目标静止未确认')
def collision_gripper():
    # 在当前六轴姿态下检查完整夹爪路径；故障锁存后健康检查拒绝后续目标。
    current=health(1)
    actual=snap()[0]['gripper']['position']
    assert abs(actual-base['gripper'])<=.0005,'当前夹爪位置不符合本轮基线'
    assert validity.wait_for_service(timeout_sec=3.)
    for alpha in np.linspace(0.,1.,21):
        health(1)
        request=GetStateValidity.Request();request.group_name='arm_with_gripper'
        request.robot_state.is_diff=False
        request.robot_state.joint_state.name=list(JOINT_NAMES)+['left_finger_joint','right_finger_joint']
        low=min(actual,base['gripper']);high=max(actual,base['gripper']+.005)
        opening=low+(high-low)*float(alpha)
        request.robot_state.joint_state.position=list(map(float,current))+[opening/2,-opening/2]
        result=wait(validity.call_async(request),3.,1)
        assert result.valid,[(c.contact_body_1,c.contact_body_2) for c in result.contacts]
    emit('gripper_collision_pass',samples=21,actual=current.tolist())
def feedback(msg):
    f=msg.feedback
    audit.record('action_feedback',phase=phase,progress=getattr(f,'progress',None),
        actual=list(getattr(getattr(f,'actual',None),'positions',[])))
def send(client,path,goal,expected='reBotArmController'):
    unique(path,True,expected);health(1);assert client.wait_for_server(timeout_sec=3.)
    emit('action_request',action=path)
    pending=gate.submit(lambda:client.send_goal_async(goal,feedback_callback=feedback),
                        gripper=(path=='/rebotarm/gripper/command'))
    wait(pending,6.,1)
    handle,result=gate.result_for(pending)
    emit('action_accepted',action=path,accepted=handle.accepted)
    assert handle.accepted,'Action 未接纳'
    item=(handle,result,path);handles.append(item)
    return item
def finish(item,expected=4,timeout=20.):
    handle,future,path=item
    result=wait(future,timeout,1)
    emit('action_result',action=path,status=result.status,error_code=getattr(result.result,'error_code',None),
        success=getattr(result.result,'success',None),
        message=getattr(result.result,'error_string',getattr(result.result,'message','')))
    assert result.status==expected,result
    if expected==4:
        if hasattr(result.result,'success'):assert result.result.success,result.result
        if hasattr(result.result,'error_code'):assert result.result.error_code==0,result.result
    return result
# 首次发现只允许失能反馈；未就绪前不开始统计，也不发送任何状态命令。
deadline=time.monotonic()+15.;stable=None
while time.monotonic()<deadline:
    try:
        health(0);stable=time.monotonic() if stable is None else stable
        if time.monotonic()-stable>=1.:break
    except AssertionError:stable=None
    time.sleep(.005)
else:raise RuntimeError('新鲜默认失能反馈未就绪')
for name in services:unique('/rebotarm/'+name)
# 拒绝已知历史监测客户端，避免它们继续按旧规则触发停止。
assert not any(n in old_names for n,_ in node.get_node_names_and_namespaces()),'旧回归监测器仍在运行'
phase='ready_disabled';monitor=threading.Thread(target=observe_faults,name='independent-source-age',daemon=True);monitor.start()
deadline=time.monotonic()+1.
while not gate.ready and not gate.fault and time.monotonic()<deadline:time.sleep(.005)
assert gate.ready,'监测器未就绪'
watch=threading.Thread(target=watchdog,name='monitor-watchdog',daemon=True);watch.start()
emit('ready',baseline=base,status=snap()[1])
disabled_verified=False;hold_verified=False
while True:
    try:command=input().strip()
    except (EOFError,KeyboardInterrupt):
        gate.trip('input_closed')
        emit('input_closed',message='会话保持监测；须现场检查并独立处理，不能继续发送目标')
        while True:
            try:time.sleep(1.)
            except KeyboardInterrupt:pass
    try:
        phase=command;emit('phase_started')
        if command=='disabled10':
            observe(10.,0);disabled_verified=True
        elif command=='enable_hold':
            assert disabled_verified,'先完成默认失能观察'
            health(0);call('enable',transition=True);health(1)
            observe(60.,1);stationary();hold_verified=True
        elif command=='gripper_roundtrip':
            assert hold_verified,'先完成使能保持验证'
            stationary(qbase,1.);collision_gripper()
            for target in (base['gripper']+.005,base['gripper']):
                health(1);goal=GripperCommand.Goal();goal.command.position=target;goal.command.max_effort=0.
                before=snap()[0]['gripper']['position'];emit('gripper_target',target=target,before=before)
                item=send(gripper,'/rebotarm/gripper/command',goal)
                def terminal(f):
                    try:
                        r=f.result();emit('gripper_terminal',status=r.status,position=r.result.position,effort=r.result.effort,reached_goal=r.result.reached_goal,stalled=r.result.stalled)
                    except Exception as exc:emit('gripper_terminal_error',error=str(exc))
                item[1].add_done_callback(terminal)
                result=finish(item);assert result.result.reached_goal and not result.result.stalled
                stationary(qbase,1.)
                after=snap()[0]['gripper']['position'];emit('gripper_observed',target=target,before=before,after=after,delta=after-before)
                assert abs(after-before)>.002,'未观察到足够实际位移'
                assert abs(after-target)<=.0023,'夹爪未在生产到位容差附近'
        elif command=='disable':
            stationary(qbase,1.,False,.005)
            assert abs(snap()[0]['gripper']['position']-base['gripper'])<=.0023,'夹爪未返回基线容差'
            assert not gate.has_pending and all(f.done() for _,f,_ in handles),'仍有未知 Action'
            with raw_lock:
                data,_,now=raw_snapshot_locked()
                transition=DisabledTransitionEvidence([v['position'] for v in data],
                    [v['sequence'] for v in data],now)
                transition_holder['value']=transition
            call('disable',history=False,transition=True)
            with raw_lock:transition.service_result(True)
            observe(5.,0,False)
            with raw_lock:
                confirmed=bool(transition.confirmed(time.monotonic()))
                transition_detail=dict(confirmed=confirmed,failure=transition.failure,
                    velocity_events=transition.velocity_events,disabled=transition.disabled,
                    receipt=transition.receipt)
            emit('disabled_transition_evidence',**transition_detail)
            assert confirmed,transition_detail
        elif command=='snapshot':emit('snapshot',status=snap()[1],latest=snap()[0],fault=gate.fault)
        elif command=='quit':
            health(0,False);assert not gate.has_pending and all(f.done() for _,f,_ in handles)
            closed=True;emit('client_closed',fault=gate.fault);break
        else:raise RuntimeError('unknown command')
        emit('operation_pass',operation=command,status=snap()[1],latest=snap()[0])
    except (Exception,KeyboardInterrupt) as exc:
        gate.trip('operation_failed: '+str(exc))
        emit('operation_failed',operation=command,error=str(exc),status=snap()[1],latest=snap()[0])
        emit('kept_current_state',status=snap()[1])
stop_monitor.set();monitor.join(3.);watch.join(3.)
raw_stop.set();raw_thread.join(3.)
executor.shutdown(timeout_sec=3.);spin.join(3.)
feedback_executor.shutdown(timeout_sec=3.);feedback_spin.join(3.)
if spin.is_alive() or feedback_spin.is_alive():
    raise RuntimeError('回归执行器未退出，保留 ROS 句柄')
feedback_node.destroy_node();node.destroy_node();rclpy.try_shutdown()
critical_events.put(None);critical_writer.join(3.)
if critical_writer.is_alive() or critical_audit_failed.is_set():
    raise RuntimeError('故障交接审计线程未退出或发生异常')
audit.close();events.close()
