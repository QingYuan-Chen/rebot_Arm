#!/usr/bin/env python3
"""P005 固定物理指标及隔离 ROS 操作链验收.

文件职责
    检查 continuous 的跟踪和保持效果,按需验证已启动的 MuJoCo/MoveIt/
    Dashboard/键盘正式链;保存原始轨迹,配置摘要和检查结论.
对外接口
    main 接受必需的 --output;--ros 启用 ROS/HTTP 操作链,--dashboard 指定对应仿真界面.
    physics_acceptance 返回物理指标,ros_acceptance 返回各操作检查记录.
安全约束
    不启动真实控制器;--ros 会发送目标,取消并修改场景,必须使用隔离的非零 ROS 域
    和同一仿真链的 Dashboard.程序核对 MuJoCo 节点及唯一 FJT,但不替代运行环境隔离.
    不调整模型,增益,限值或 Action 容差;独立物理观察窗口不延长 MoveIt 执行预算.
    输出目录应按本次验收独立分配,同名报告会覆盖;软件结果不构成实机验收.
文件结构
    JSON 输出 → 固定物理验收 → ROS 服务/Action/网页/终端验收 → 配置摘要和 CLI.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.request
import urllib.parse
from urllib.error import HTTPError


def write_json(path, value):
    """写入一次完整报告,禁止非有限 JSON;调用者负责选择本次独立产物路径."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def physics_acceptance(output):
    """固定初始姿态和控制配置,检查连续采样的绝对跟踪,到位和保持门限."""
    from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
    from rebotarm_motion.trajectory_preparation import point_to_point
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.diagnostics.trajectory_trace import TrajectoryTrace
    start = (0., -1., -1., 0., .3, 0.)
    target = (.02, *start[1:])
    curve = build_curve(JOINT_NAMES, point_to_point(start, target, .4))
    report = {}
    # 独立建模并先保持 2 s,使参考开始时的物理状态一致.
    with RebotArmMujoco() as sim:
        sim.reset_joint_positions(start)
        sim.command_joint_positions(start)
        sim.step(round(2./sim.timestep))
        origin = sim.get_state().simulation_time
        trace = TrajectoryTrace(output/'continuous.jsonl')
        maximum = 0.
        stable_since = None
        settled = None
        for _ in range(round(5.4/sim.timestep)):
            elapsed = sim.get_state().simulation_time-origin
            command = curve.sample(elapsed)[0]
            sim.command_joint_positions(command)
            state = sim.step()
            elapsed = state.simulation_time-origin
            reference = curve.sample(elapsed)[0]
            if elapsed <= .4+1e-9:
                maximum = max(maximum, max(abs(a-b) for a,b in zip(reference,state.joint_positions[:6])))
            if elapsed >= .4:
                # 到达与连续 0.5 s 静止由独立物理观察判断,不读取 Action 的成功标记.
                quiet = (max(abs(a-b) for a,b in zip(target,state.joint_positions[:6])) <= .001
                    and max(abs(v) for v in state.joint_velocities[:6]) <= .01)
                stable_since = stable_since if quiet else None
                if quiet and stable_since is None:
                    stable_since = elapsed
                if stable_since is not None and elapsed-stable_since >= .5 and settled is None:
                    settled = elapsed-.4
            trace.record(state, sim.get_control_status(), dict(curve=curve, start=origin))
        before = sim.get_control_status()
        before_q = sim.get_state().joint_positions[:6]
        # 同时观察切换瞬间支撑力矩与随后 1 s 漂移,不能只看最终位置是否恢复.
        sim.set_mode('hold')
        after = sim.get_control_status()
        torque_jump = max(abs(a-b) for a,b in zip(before.applied_torques,after.applied_torques))
        drift = 0.
        for _ in range(round(1./sim.timestep)):
            state = sim.step()
            drift = max(drift, max(abs(a-b) for a,b in zip(before_q,state.joint_positions[:6])))
            trace.record(state, sim.get_control_status(), None)
        trace.close()
        report['continuous'] = dict(max_tracking_error=maximum, settle_observation=settled,
            hold_drift=drift, switch_torque_jump=torque_jump)
    assert report['continuous']['max_tracking_error'] <= .005, report
    assert report['continuous']['settle_observation'] is not None and report['continuous']['settle_observation'] <= 5., report
    assert report['continuous']['switch_torque_jump'] <= .01, report
    write_json(output/'physics_acceptance.json', report)
    return report


def ros_acceptance(output, dashboard, teach_record=None):
    """驱动已存在的隔离仿真图;等待,Action 终态和实际反馈分别检查,失败直接抛错."""
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('use an isolated nonzero ROS_DOMAIN_ID for simulation acceptance')
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene, GetStateValidity
    from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, AttachedCollisionObject
    from moveit_msgs.action import ExecuteTrajectory
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from rebotarm_msgs.srv import PrepareArmTrajectory, RetimeArmTrajectory, GetArmPlanningState, SetGripper, SetMode
    from rebotarm_motion.trajectory_messages import point_to_point_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = Node('p005_simulation_acceptance')
    latest = {}
    samples = []
    node.create_subscription(JointState, '/rebotarm/joint_states',
        lambda m: (latest.update(joint=m, joint_received_at=time.monotonic()), samples.append(m)), 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name in ('trajectory_execution_mode','trajectory_scene_revision','trajectory_capabilities'):
        node.create_subscription(String, '/rebotarm/'+name,
            lambda m, key=name: latest.update({key:m.data}), qos)
    def wait(predicate, timeout=8.):
        """等待期间持续 spin 接收反馈;超时表示证据不足,不能替代为成功."""
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        if not predicate():
            raise TimeoutError('acceptance wait timed out')
    def pause(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
    def call(service, name, request, timeout=8.):
        client = node.create_client(service,name)
        assert client.wait_for_service(timeout_sec=3),name
        future = client.call_async(request)
        wait(future.done, timeout)
        result = future.result()
        node.destroy_client(client)
        return result
    arm = ActionClient(node,FollowJointTrajectory,'/rebotarm/follow_joint_trajectory')
    execute = ActionClient(node,ExecuteTrajectory,'/execute_trajectory')
    names = tuple(f'joint{i}' for i in range(1,7))
    def make(target, duration=.4, grade=2):
        current = dict(zip(latest['joint'].name, latest['joint'].position))
        trajectory = point_to_point_message(names, tuple(current[n] for n in names), target,
            duration, trajectory_factory=JointTrajectory,point_factory=JointTrajectoryPoint)
        for p in trajectory.points:
            if grade < 2:p.accelerations=[]
            if grade < 1:p.velocities=[]
        goal=FollowJointTrajectory.Goal();goal.trajectory=trajectory
        return goal
    feedbacks={}
    def send(client, goal):
        assert client.wait_for_server(timeout_sec=3)
        events=[]
        future=client.send_goal_async(goal,feedback_callback=events.append);wait(future.done)
        handle=future.result()
        assert handle.accepted,'goal rejected'
        feedbacks[id(handle)]=events
        return handle
    def finish(handle, success=True):
        """同时检查 ROS Goal 终态和 FJT 错误码;MoveIt 自身的错误码由对应调用点核对."""
        future=handle.get_result_async();wait(future.done,12.)
        response=future.result()
        if success:assert response.status==4 and response.result.error_code==0,str(response)
        return response
    def post(path,payload):
        request=urllib.request.Request(dashboard+path,data=json.dumps(payload).encode(),
            headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=30) as response:return json.load(response)
        except HTTPError as exc:
            if exc.code != 400:
                raise
            return json.load(exc)
    def status():
        with urllib.request.urlopen(dashboard+'/api/status',timeout=3) as response:return json.load(response)
    results={}
    try:
        wait(lambda:len(samples)>30 and 'trajectory_scene_revision' in latest)
        assert latest['trajectory_execution_mode']=='continuous'
        nodes=node.get_node_names()
        assert 'rebotarm_mujoco_node' in nodes and 'reBotArmController' not in nodes,nodes
        from rclpy.action import get_action_server_names_and_types_by_node
        owners = [name for node_name, namespace in node.get_node_names_and_namespaces()
            for name, _ in get_action_server_names_and_types_by_node(node, node_name, namespace)
            if name == '/rebotarm/follow_joint_trajectory']
        assert len(owners) == 1, owners
        results['graph_nodes']=nodes
        # 先验证只读起点与最终计划绑定,再使用现有网页/MoveIt/示教入口.
        from rebotarm_trajectory.planning import decode_state, decode_plan
        from rebotarm_trajectory.polynomial import TrajectoryPoint,curve_from_payload
        def numeric(message):
            return tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                p.positions,p.velocities,p.accelerations) for p in message.points)
        snapshot=call(GetArmPlanningState,'/rebotarm/get_arm_planning_state',GetArmPlanningState.Request())
        assert snapshot.success,snapshot.message
        state=decode_state(snapshot.state_json)
        target=list(state['hold']);target[0]+=.005
        request=RetimeArmTrajectory.Request(trajectory=make(target,1.).trajectory,
            velocity_scaling=1.,acceleration_scaling=1.)
        request.trajectory.points[0].positions[0]+=.003
        original=message_to_ordereddict(request.trajectory)
        final=call(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory',request)
        assert final.success and final.executable,final.message
        binding=decode_plan(final.trajectory.header.frame_id,names,numeric(final.trajectory),require_execution=True)
        assert list(final.trajectory.points[0].positions)==state['hold']
        request.planning_purpose=request.REFERENCE
        reference=call(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory',request)
        assert reference.success and not reference.executable,reference.message
        assert list(reference.trajectory.points[0].positions)==list(request.trajectory.points[0].positions)
        rejected=arm.send_goal_async(FollowJointTrajectory.Goal(trajectory=reference.trajectory));wait(rejected.done)
        assert not rejected.result().accepted,'reference curve executed through compatibility FJT'
        stop=call(Trigger,'/rebotarm/trajectory_stop',Trigger.Request());assert stop.success,stop.message
        rejected=arm.send_goal_async(FollowJointTrajectory.Goal(trajectory=final.trajectory));wait(rejected.done)
        assert not rejected.result().accepted,'idle Stop did not invalidate planning binding'
        request.planning_purpose=request.FINAL_EXECUTION
        final=call(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory',request)
        assert final.success and final.executable,final.message
        binding=decode_plan(final.trajectory.header.frame_id,names,numeric(final.trajectory),require_execution=True)
        frozen=binding['state']
        # 服务发现可能耗时,先建立连接再取得本次准备请求的反馈时间戳.
        prepare_client=node.create_client(PrepareArmTrajectory,'/rebotarm/prepare_arm_trajectory')
        assert prepare_client.wait_for_service(timeout_sec=3)
        pause(.05)
        check=PrepareArmTrajectory.Request(trajectory=final.trajectory, session=frozen['session'],
            generation=frozen['generation'],hold_positions=frozen['hold'],
            feedback_positions=list(latest['joint'].position[:6]),feedback_observed_at=latest['joint_received_at'],
            enabled=True,gripper_positions=list(latest['joint'].position[-2:]),
            scene_revision=latest['trajectory_scene_revision'],deadline_monotonic=time.monotonic()+2.,
            capability_json=latest['trajectory_capabilities'])
        future=prepare_client.call_async(check)
        wait(future.done)
        checked=future.result()
        node.destroy_client(prepare_client)
        assert checked.success,checked.message
        prepared=json.loads(checked.prepared_json)
        curve=curve_from_payload(prepared['curve'])
        assert curve.points==numeric(final.trajectory),'preparation changed bound final curve'
        finish(send(arm,FollowJointTrajectory.Goal(trajectory=final.trajectory)))
        results['planning_binding']=dict(input=original,snapshot=state,binding=binding,
            final=message_to_ordereddict(final.trajectory),prepared=prepared,
            reference_preserved=True,idle_stop_invalidated=True)
        write_json(output/'planning_binding.json',results['planning_binding'])
        pause(1.)
        if teach_record is not None:
            # 复用网页正式入口;只复制输入,绝不在用户录制文件旁生成或覆盖派生文件.
            raw=output/'teach_input.jsonl'
            original=teach_record.read_bytes()
            raw.write_bytes(original)
            payload=dict(record_path=str(raw.resolve()))
            with urllib.request.urlopen(dashboard+'/api/teach_trajectory?'+urllib.parse.urlencode(
                    dict(path=str(raw.resolve()),max_points=500)),timeout=30) as response:
                preview=json.load(response)
            write_json(output/'teach_web_preview.json',preview)
            assert preview['accepted'],preview
            assert preview['prepared_replay']['curve_analysis']['degree']==3,preview
            assert preview['collision_precheck']['state']=='pass',preview
            trace=[]
            from rcl_interfaces.msg import Log
            admissions=[]
            node.create_subscription(Log,'/rosout',
                lambda m: admissions.append(m.msg) if m.msg.startswith('prepared curve accepted degree=') else None,10)
            node.create_subscription(FollowJointTrajectory.Impl.FeedbackMessage,
                '/rebotarm/follow_joint_trajectory/_action/feedback',
                lambda m:trace.append(message_to_ordereddict(m.feedback)),10)
            terminals=('done','failed','aborted','rejected','blocked','safety_stop','canceled')
            for cancel in (True,False):
                label='cancel' if cancel else 'complete'
                dry=post('/api/teach_dry_run',payload)
                write_json(output/f'teach_{label}_dry_run.json',dry)
                assert dry['accepted'],dry
                assert dry['trajectory_preparation']['state']=='pass',dry
                reference=dry['prepared_replay']
                assert reference['input_grade']==reference['reference_grade']=='q/v',reference
                assert reference['velocity_source']=='derived_filtered_q_t',reference
                assert reference['final_curve']['degree']==3,reference['final_curve']
                assert reference['final_curve']['continuity']['guaranteed_order']==1,reference['final_curve']
                assert reference['final_curve']['continuity']['max_boundary_gaps']['velocity']<1e-9
                assert reference['alignment_conversion']['reference_grade']=='q/v',reference
                from rebotarm_teach.teach_record_io import load_teach_samples
                derived=load_teach_samples(dry['prepared_record_path'])
                assert all(s.format_version==5 and s.velocities and not s.accelerations for s in derived)
                check=dry['collision_precheck']
                assert check['state']=='pass' and check['checked_samples']==check['requested_samples'],check
                replay=post('/api/teach_replay_execute',payload)
                write_json(output/f'teach_{label}_execute.json',replay)
                assert replay['accepted'],replay
                wait(lambda:status()['teleop'].get('replay',{}).get('state')=='replaying',8.)
                if cancel:
                    pause(3.)
                    before_stop=status()['teleop'].get('replay',{})
                    assert before_stop.get('state')=='replaying',before_stop
                    stop=post('/api/teach_replay_stop',{})
                    write_json(output/'teach_cancel_request.json',stop)
                deadline=time.monotonic()+float(replay['estimated_duration_sec'])*1.2+8.
                while time.monotonic()<deadline:
                    pause(.1)
                    replay=status()['teleop'].get('replay',{})
                    if replay.get('state') in terminals:
                        break
                results['teach_'+label]=replay
                write_json(output/'teach_ros_acceptance.json',results)
                assert replay.get('state')==('canceled' if cancel else 'done'),replay
                pause(1.)
            assert raw.read_bytes()==original and teach_record.read_bytes()==original,'raw record changed'
            results['raw_sha256']=hashlib.sha256(original).hexdigest()
            assert len(admissions)==2 and all('degree=3 ' in msg for msg in admissions),admissions
            results['backend_curve_admissions']=admissions
            write_json(output/'teach_feedback.json',trace)
            write_json(output/'teach_ros_acceptance.json',results)
            return results
        # 同时验证标准 byte 诊断经过网页订阅后仍能正确显示"再次可执行".
        wait(lambda:status()['teleop'].get('trajectory_admission',{}).get('ready') is True)
        results['web_admission_ready']=status()['teleop']['trajectory_admission']
        start=list(latest['joint'].position[:6]);target=start.copy();target[0]+=.02
        handle=send(arm,make(target))
        # 后端在准备期间已占用资源,夹爪请求不能改变固定状态.
        req=SetGripper.Request();req.position=.01;req.max_effort=1.
        assert not call(SetGripper,'/rebotarm/gripper/set',req).success
        finish(handle)
        pause(1.)
        assert max(abs(a-b) for a,b in zip(target,latest['joint'].position[:6]))<=.001
        results['direct_step']='pass'
        # q-only 和 q/v 保留明确的兼容等级.
        for grade in (0,1):
            target=list(latest['joint'].position[:6]);target[0]+=.005
            finish(send(arm,make(target,.8,grade)));pause(.5)
        results['compatibility']='C0/C1 pass'
        # 实际 MoveIt 服务规划,选一条不修改时间,不放宽限值的有效轨迹.
        planned=None
        # 改变规划请求取得合格样本,不修改返回轨迹;每次实际执行的成功/拒绝都留证据.
        for scaling in (.03,.01,.003):
            req=GetMotionPlan.Request();plan=req.motion_plan_request
            plan.group_name='arm';plan.pipeline_id='ompl';plan.num_planning_attempts=1
            plan.allowed_planning_time=3.;plan.max_velocity_scaling_factor=.1
            plan.max_acceleration_scaling_factor=scaling
            plan.start_state.joint_state=latest['joint'];plan.start_state.is_diff=False
            plan.goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                joint_name=name,position=float(q+(.03 if index==0 else 0)),
                tolerance_above=.0001,tolerance_below=.0001,weight=1.)
                for index,(name,q) in enumerate(zip(names,latest['joint'].position[:6]))])]
            response=call(GetMotionPlan,'/plan_kinematic_path',req)
            assert response.motion_plan_response.error_code.val==1,str(response)
            candidate=response.motion_plan_response.trajectory
            write_json(output/f'moveit_plan_{scaling}.json',message_to_ordereddict(candidate))
            # 直接执行原规划,失败也必须保留日志和明确结果,不自动重定时.
            goal=ExecuteTrajectory.Goal();goal.trajectory=candidate
            response=finish(send(execute,goal),success=False)
            results[f'moveit_{scaling}']=message_to_ordereddict(response)
            pause(.6)
            if response.status==4 and response.result.error_code.val==1:
                planned=candidate
                break
        assert planned is not None,results
        results['moveit_execution']='pass'
        for group in ('gripper',):
            pause(.8)
            current=dict(zip(latest['joint'].name,latest['joint'].position))
            selected=('left_finger_joint',)
            req=GetMotionPlan.Request();plan=req.motion_plan_request
            plan.group_name=group;plan.pipeline_id='ompl';plan.num_planning_attempts=1
            plan.allowed_planning_time=3.;plan.max_velocity_scaling_factor=.1
            plan.max_acceleration_scaling_factor=.1
            plan.start_state.joint_state=latest['joint'];plan.start_state.is_diff=False
            plan.goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                joint_name=name,position=float(current[name]+(.0004 if name=='left_finger_joint' else .01 if name=='joint1' else 0.)),
                tolerance_above=1e-6,tolerance_below=1e-6,weight=1.) for name in selected])]
            response=call(GetMotionPlan,'/plan_kinematic_path',req)
            assert response.motion_plan_response.error_code.val==1,str(response)
            trajectory=response.motion_plan_response.trajectory
            assert len(trajectory.joint_trajectory.points)>1,'small finger motion collapsed to a zero-duration plan'
            write_json(output/f'moveit_{group}.json',message_to_ordereddict(trajectory))
            goal=ExecuteTrajectory.Goal(trajectory=trajectory)
            response=finish(send(execute,goal),success=False)
            assert response.status==4 and response.result.error_code.val==1,str(response)
            results[f'moveit_{group}']=message_to_ordereddict(response)
        write_json(output/'ros_acceptance.json',results)
        pause(1.)
        # HTTP 请求穿过网页正式路由与适配器.
        assert post('/api/keyboard_enable',{})['accepted']
        first=post('/api/keyboard_key',dict(confirm='KEYBOARD_TELEOP',key='q'))
        second=post('/api/keyboard_key',dict(confirm='KEYBOARD_TELEOP',key='q'))
        assert first['accepted'] and not second['accepted'],(first,second)
        pause(1.2)
        results['keyboard_status']=status()
        assert results['keyboard_status']['teleop']['status']['state']=='done',results['keyboard_status']
        current=dict(zip(names,latest['joint'].position[:6]));current['joint1']+=.02
        assert post('/api/execute_preview',dict(confirm='EXECUTE',joint_positions=current,duration=1.))['accepted']
        pause(2.)
        results['web_status']=status()
        assert results['web_status']['teleop']['web_execute']['state']=='done',results['web_status']
        pause(.8)
        current=dict(zip(names,latest['joint'].position[:6]))
        current['joint1']+=.02
        width=dict(zip(latest['joint'].name,latest['joint'].position))['left_finger_joint']*2.
        joint_plan=post('/api/preview_coordinated',dict(joint_positions=current,duration=1.2,
            gripper_position=width+.001,arm_keyframes=[dict(time=.6,positions=list(current.values())),dict(time=1.2,positions=list(current.values()))]))
        assert joint_plan['accepted'],joint_plan
        assert len(joint_plan['joint_names'])==6 and len(joint_plan['keyframes'])==3,joint_plan
        write_json(output/'web_coordinated_plan.json',joint_plan)
        submitted=post('/api/execute_preview',dict(confirm='EXECUTE',plan_id=joint_plan['plan_id']))
        assert submitted['accepted'],submitted
        pause(joint_plan['duration']+1.)
        results['web_coordinated']=status()['teleop']['web_execute']
        assert results['web_coordinated']['state']=='done',results['web_coordinated']
        assert results['web_coordinated']['arm_state']=='succeeded',results['web_coordinated']
        assert results['web_coordinated']['gripper_state']=='succeeded',results['web_coordinated']
        # 所有手指 FJT 都必须直接拒绝,不能静默投影.
        for extra in (['left_finger_joint'],[*names,'left_finger_joint'],[*names,'right_finger_joint']):
            invalid=make(list(latest['joint'].position[:6]),1.)
            invalid.trajectory.joint_names=extra
            future=arm.send_goal_async(invalid);wait(future.done)
            assert not future.result().accepted,extra
        results['finger_fjt_rejected']=True
        # 网页组合取消并等待资源释放,再执行仅关节,旧任务不得覆盖后继.
        from rebotarm_msgs.action import ExecuteTeleopMotion
        teleop=ActionClient(node,ExecuteTeleopMotion,'/rebotarm/execute_teleop_motion')
        target=list(latest['joint'].position[:6]);target[0]+=.01
        combined=ExecuteTeleopMotion.Goal(trajectory=make(target,2.).trajectory,
            move_gripper=True,gripper_position=.01,gripper_max_effort=1.)
        handle=send(teleop,combined)
        wait(lambda:bool(feedbacks[id(handle)]),1.)
        future=handle.cancel_goal_async();wait(future.done)
        canceled=finish(handle,success=False);assert canceled.status==5,str(canceled)
        assert canceled.result.arm_state in ('canceled','failed'),str(canceled)
        assert canceled.result.gripper_state in ('failed','not_started'),str(canceled)
        pause(.8)
        target=list(latest['joint'].position[:6]);target[0]+=.005
        done=finish(send(teleop,ExecuteTeleopMotion.Goal(trajectory=make(target,1.).trajectory)))
        assert done.result.arm_state=='succeeded' and done.result.gripper_state=='not_requested',str(done)
        results['web_cancel_and_next']=message_to_ordereddict(done)
        # 新格式保留夹爪数据供查看,正式回放只派生六轴,不修改原始记录.
        pause(.8)
        current=dict(zip(latest['joint'].name,latest['joint'].position))
        raw=output/'coordinated_teach.jsonl'
        raw.write_text(''.join(json.dumps(dict(stamp=i*.1,joint_names=names,
            positions=[current[n]+(i*.0002 if n=='joint1' else 0.) for n in names],
            velocities=[],efforts=[],motor_status=dict.fromkeys(names,1),arm_state='GRAVITY_COMP',
            format_version=2,gripper_width_m=current['left_finger_joint']*2.+i*.00001,
            gripper_source_stamp=i*.1,gripper_valid=True))+'\n' for i in range(11)))
        original=raw.read_bytes()
        teach_payload=dict(record_path=str(raw.resolve()))
        dry=post('/api/teach_dry_run',teach_payload)
        write_json(output/'teach_dry_run.json',dry)
        assert dry['accepted'],dry
        replay=post('/api/teach_replay_execute',teach_payload)
        assert replay['accepted'],replay
        deadline=time.monotonic()+30.
        while time.monotonic()<deadline:
            pause(.1)
            replay=status()['teleop'].get('replay',{})
            if replay.get('state') in ('done','failed','aborted','rejected','blocked'):
                break
        results['teach_coordinated']=replay
        assert replay['state']=='done',replay
        assert raw.read_bytes()==original,'raw teach recording was overwritten'
        # 显式取消并等新帧静止确认,之后继续接纳.
        target=list(latest['joint'].position[:6]);target[0]+=.03
        handle=send(arm,make(target,2.))
        wait(lambda:bool(feedbacks[id(handle)]),.65)
        future=handle.cancel_goal_async();wait(future.done)
        canceled=finish(handle,success=False)
        assert canceled.status==5,str(canceled)
        pause(.5)
        results['cancel']='pass'
        # 场景变化在执行中触发停止保持;移除后必须能继续接纳.
        handle=send(arm,make(target,2.))
        wait(lambda:bool(feedbacks[id(handle)]),.65)
        pause(.15)
        revision=latest['trajectory_scene_revision']
        obj=CollisionObject();obj.id='p005_far_obstacle';obj.header.frame_id='base_link'
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.1,.1,.1])]
        pose=Pose();pose.position.x=3.;pose.orientation.w=1.;obj.primitive_poses=[pose]
        obj.operation=CollisionObject.ADD
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.world.collision_objects=[obj]
        assert call(ApplyPlanningScene,'/apply_planning_scene',req).success
        wait(lambda:latest['trajectory_scene_revision']!=revision)
        stopped=finish(handle,success=False);assert stopped.status!=4,str(stopped)
        obj.operation=CollisionObject.REMOVE
        req.scene.world.collision_objects=[obj]
        assert call(ApplyPlanningScene,'/apply_planning_scene',req).success
        pause(.8)
        results['scene_stop']='pass'
        # 真正终端入口经 PTY 读键;第二键必须在首步结束前明确丢弃.
        import pty
        import signal
        import subprocess
        terminal_events=[]
        subscription=node.create_subscription(String,'/rebotarm/teleop/status',
            lambda m:terminal_events.append(json.loads(m.data)),10)
        master,slave=pty.openpty()
        terminal_log=(output/'terminal_keyboard.log').open('w')
        process=subprocess.Popen(['ros2','run','rebotarm_teleop','TeleopKeyboardNode'],
            stdin=slave,stdout=terminal_log,stderr=subprocess.STDOUT,start_new_session=True)
        os.close(slave)
        try:
            wait(lambda:'teleop_keyboard_node' in node.get_node_names())
            pause(2.)
            before=latest['joint'].position[0]
            os.write(master,b'1');pause(.1);os.write(master,b'1')
            wait(lambda:any(e['state']=='done' for e in terminal_events),3.)
            assert any(e['state']=='rejected' and 'busy' in e['message'] for e in terminal_events),terminal_events
            pause(.5)
            assert abs(latest['joint'].position[0]-before-.02)<=.001
            results['terminal_keyboard']=terminal_events
        finally:
            write_json(output/'terminal_keyboard_events.json',terminal_events)
            os.killpg(process.pid,signal.SIGINT)
            process.wait(timeout=5.)
            os.close(master);terminal_log.close()
            node.destroy_subscription(subscription)
        # 真实准备服务拒绝原始 jerk 样本,不能用起点不匹配冒充超限拒绝.
        fixture=json.loads((Path(__file__).resolve().parents[1]/'tests/fixtures/p005_moveit_trajectory.json').read_text())
        req=PrepareArmTrajectory.Request()
        req.trajectory.joint_names=fixture['joint_names']
        for row in fixture['points']:
            point=JointTrajectoryPoint()
            point.positions=row['positions'];point.velocities=row['velocities'];point.accelerations=row['accelerations']
            stamp=round(row['time_from_start']*1e9)
            point.time_from_start.sec,point.time_from_start.nanosec=divmod(stamp,1_000_000_000)
            req.trajectory.points.append(point)
        req.session=req.generation=1
        req.hold_positions=req.feedback_positions=fixture['points'][0]['positions']
        req.feedback_observed_at=time.monotonic();req.stationary=req.enabled=True
        req.gripper_positions=[0.,0.];req.scene_revision=latest['trajectory_scene_revision']
        req.capability_json=latest['trajectory_capabilities']
        req.deadline_monotonic=time.monotonic()+2.
        rejection=call(PrepareArmTrajectory,'/rebotarm/prepare_arm_trajectory',req)
        assert not rejection.success and 'jerk' in rejection.message,rejection.message
        results['jerk_fixture_rejection']=rejection.message
        # 同一真实场景内分别验证环境和附着物几何的拒绝覆盖.
        for attached in (False,True):
            revision=latest['trajectory_scene_revision']
            obj=CollisionObject(id='p005_blocker');obj.header.frame_id='end_link' if attached else 'base_link'
            obj.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[2.,2.,2.])]
            pose=Pose();pose.orientation.w=1.;obj.primitive_poses=[pose];obj.operation=CollisionObject.ADD
            req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True
            if attached:
                req.scene.robot_state.attached_collision_objects=[AttachedCollisionObject(
                    link_name='end_link',object=obj,touch_links=['end_link'])]
            else:req.scene.world.collision_objects=[obj]
            assert call(ApplyPlanningScene,'/apply_planning_scene',req).success
            wait(lambda:latest['trajectory_scene_revision']!=revision);pause(.3)
            target=list(latest['joint'].position[:6]);target[0]+=.005
            rejection=finish(send(arm,make(target,1.)),success=False)
            assert rejection.status!=4 and 'collision' in rejection.result.error_string,str(rejection)
            results['attached_collision' if attached else 'world_collision']='rejected'
            obj.operation=CollisionObject.REMOVE
            if attached:
                req.scene.robot_state.attached_collision_objects=[AttachedCollisionObject(
                    link_name='end_link',object=obj)]
            else:
                req.scene.world.collision_objects=[obj]
            assert call(ApplyPlanningScene,'/apply_planning_scene',req).success
            if attached:
                # MoveIt 分离附着物时会放回 world;本测试还需显式清除该几何.
                cleanup=ApplyPlanningScene.Request();cleanup.scene.is_diff=True
                cleanup.scene.world.collision_objects=[obj]
                assert call(ApplyPlanningScene,'/apply_planning_scene',cleanup).success
            pause(.8)
        # 静止 position -> hold 仍由正式服务切换.
        req=SetMode.Request();req.mode='hold'
        assert call(SetMode,'/rebotarm/sim/set_mode',req).success
        pause(1.)
        write_json(output/'ros_acceptance.json',results)
        return results
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    """先记录本次安装资源摘要,再执行物理检查;只有显式 --ros 才访问已有仿真图."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--dashboard',default='http://127.0.0.1:18088')
    parser.add_argument('--teach-record',type=Path,help='仅验收指定记录的真实对齐、完整碰撞预检、取消和回放')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    from ament_index_python.packages import get_package_share_directory
    simulation=Path(get_package_share_directory('rebotarm_simulation'))
    moveit=Path(get_package_share_directory('rebotarm_moveit_config'))
    frozen=[*sorted((simulation/'models').rglob('*')), *sorted((simulation/'config').glob('*.yaml')),
        moveit/'config/joint_limits.yaml']
    write_json(args.output/'frozen_files.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in frozen if p.is_file()})
    if args.teach_record and not args.ros:
        parser.error('--teach-record requires --ros and an isolated simulation graph')
    result={} if args.teach_record else dict(physics=physics_acceptance(args.output))
    if args.ros:result['ros']=ros_acceptance(args.output,args.dashboard,args.teach_record)
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
