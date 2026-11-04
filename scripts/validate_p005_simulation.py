#!/usr/bin/env python3
"""P005 冻结物理对照及隔离 ROS 操作链验收。

文件职责
    比较旧阶梯与 prepared_v1 的跟踪和保持效果，按需验证已启动的 MuJoCo/MoveIt/
    Dashboard/键盘正式链；保存原始轨迹、配置摘要和检查结论。
对外接口
    main 接受必需的 --output；--ros 启用 ROS/HTTP 操作链，--dashboard 指定对应仿真界面。
    physics_acceptance 返回物理指标，ros_acceptance 返回各操作检查记录。
安全约束
    不启动真实控制器；--ros 会发送目标、取消并修改场景，必须使用隔离的非零 ROS 域
    和同一仿真链的 Dashboard。程序核对 MuJoCo 节点及唯一 FJT，但不替代运行环境隔离。
    不调整模型、增益、限值或 Action 容差；独立物理观察窗口不延长 MoveIt 执行预算。
    输出目录应按本次验收独立分配，同名报告会覆盖；软件结果不构成实机验收。
文件结构
    JSON 输出 → 固定物理对照 → ROS 服务/Action/网页/终端验收 → 配置摘要和 CLI。
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.request
from urllib.error import HTTPError


def write_json(path, value):
    """写入一次完整报告，禁止非有限 JSON；调用者负责选择本次独立产物路径。"""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def physics_acceptance(output):
    """两次独立仿真使用同一初始姿态和控制配置，只改变目标采样方式。"""
    from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
    from rebotarm_motion.trajectory_preparation import point_to_point
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.diagnostics.trajectory_trace import TrajectoryTrace
    start = (0., -1., -1., 0., .3, 0.)
    target = (.02, *start[1:])
    curve = build_curve(JOINT_NAMES, point_to_point(start, target, .4))
    report = {}
    for mode in ('legacy_step', 'prepared_v1'):
        # 每组重新建模并先保持 2 s，防止上一组的积分、速度或接触状态影响对照。
        with RebotArmMujoco() as sim:
            sim.reset_joint_positions(start)
            sim.command_joint_positions(start)
            sim.step(round(2./sim.timestep))
            origin = sim.get_state().simulation_time
            trace = TrajectoryTrace(output/f'{mode}.jsonl')
            maximum = 0.
            stable_since = None
            settled = None
            for _ in range(round(5.4/sim.timestep)):
                elapsed = sim.get_state().simulation_time-origin
                command = curve.sample(elapsed)[0] if mode == 'prepared_v1' else start if elapsed < .4 else target
                sim.command_joint_positions(command)
                state = sim.step()
                elapsed = state.simulation_time-origin
                reference = curve.sample(elapsed)[0]
                if elapsed <= .4+1e-9:
                    maximum = max(maximum, max(abs(a-b) for a,b in zip(reference,state.joint_positions[:6])))
                if elapsed >= .4:
                    # 到达与连续 0.5 s 静止由独立物理观察判断，不读取 Action 的成功标记。
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
            # 同时观察切换瞬间支撑力矩与随后 1 s 漂移，不能只看最终位置是否恢复。
            sim.set_mode('hold')
            after = sim.get_control_status()
            torque_jump = max(abs(a-b) for a,b in zip(before.applied_torques,after.applied_torques))
            drift = 0.
            for _ in range(round(1./sim.timestep)):
                state = sim.step()
                drift = max(drift, max(abs(a-b) for a,b in zip(before_q,state.joint_positions[:6])))
                trace.record(state, sim.get_control_status(), None)
            trace.close()
            report[mode] = dict(max_tracking_error=maximum, settle_observation=settled,
                hold_drift=drift, switch_torque_jump=torque_jump)
    assert report['prepared_v1']['max_tracking_error'] <= .005, report
    assert report['prepared_v1']['max_tracking_error'] < report['legacy_step']['max_tracking_error'], report
    assert report['prepared_v1']['settle_observation'] is not None and report['prepared_v1']['settle_observation'] <= 5., report
    assert report['prepared_v1']['switch_torque_jump'] <= .01, report
    write_json(output/'physics_acceptance.json', report)
    return report


def ros_acceptance(output, dashboard):
    """驱动已存在的隔离仿真图；等待、Action 终态和实际反馈分别检查，失败直接抛错。"""
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
    from rebotarm_msgs.srv import PrepareArmTrajectory, RetimeArmTrajectory, SetGripper, SetMode
    from rebotarm_motion.trajectory_messages import point_to_point_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = Node('p005_simulation_acceptance')
    latest = {}
    samples = []
    node.create_subscription(JointState, '/rebotarm/joint_states',
        lambda m: (latest.update(joint=m), samples.append(m)), 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name in ('trajectory_execution_mode','trajectory_scene_revision','trajectory_capabilities'):
        node.create_subscription(String, '/rebotarm/'+name,
            lambda m, key=name: latest.update({key:m.data}), qos)
    def wait(predicate, timeout=8.):
        """等待期间持续 spin 接收反馈；超时表示证据不足，不能替代为成功。"""
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
        """同时检查 ROS Goal 终态和 FJT 错误码；MoveIt 自身的错误码由对应调用点核对。"""
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
        assert latest['trajectory_execution_mode']=='prepared_v1'
        nodes=node.get_node_names()
        assert 'rebotarm_mujoco_node' in nodes and 'reBotArmController' not in nodes,nodes
        from rclpy.action import get_action_server_names_and_types_by_node
        owners = [name for node_name, namespace in node.get_node_names_and_namespaces()
            for name, _ in get_action_server_names_and_types_by_node(node, node_name, namespace)
            if name == '/rebotarm/follow_joint_trajectory']
        assert len(owners) == 1, owners
        results['graph_nodes']=nodes
        # 同时验证标准 byte 诊断经过网页订阅后仍能正确显示“再次可执行”。
        wait(lambda:status()['teleop'].get('trajectory_admission',{}).get('ready') is True)
        results['web_admission_ready']=status()['teleop']['trajectory_admission']
        start=list(latest['joint'].position[:6]);target=start.copy();target[0]+=.02
        handle=send(arm,make(target))
        # 后端在准备期间已占用资源，夹爪请求不能改变固定状态。
        req=SetGripper.Request();req.position=.01;req.max_effort=1.
        assert not call(SetGripper,'/rebotarm/gripper/set',req).success
        finish(handle)
        pause(1.)
        assert max(abs(a-b) for a,b in zip(target,latest['joint'].position[:6]))<=.001
        results['direct_step']='pass'
        # q-only 和 q/v 保留明确的兼容等级。
        for grade in (0,1):
            target=list(latest['joint'].position[:6]);target[0]+=.005
            finish(send(arm,make(target,.8,grade)));pause(.5)
        results['compatibility']='C0/C1 pass'
        # 实际 MoveIt 服务规划，选一条不修改时间、不放宽限值的有效轨迹。
        planned=None
        # 改变规划请求取得合格样本，不修改返回轨迹；每次实际执行的成功/拒绝都留证据。
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
            # 直接执行原规划，失败也必须保留日志和明确结果，不自动重定时。
            goal=ExecuteTrajectory.Goal();goal.trajectory=candidate
            response=finish(send(execute,goal),success=False)
            results[f'moveit_{scaling}']=message_to_ordereddict(response)
            pause(.6)
            if response.status==4 and response.result.error_code.val==1:
                planned=candidate
                break
        assert planned is not None,results
        results['moveit_execution']='pass'
        for group in ('arm_with_gripper','gripper'):
            pause(.8)
            current=dict(zip(latest['joint'].name,latest['joint'].position))
            selected=(*names,'left_finger_joint') if group=='arm_with_gripper' else ('left_finger_joint',)
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
        # HTTP 请求穿过网页正式路由与适配器。
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
            gripper_position=width+.001,gripper_keyframes=[dict(time=.3,width=width),
                dict(time=.9,width=width+.001),dict(time=1.2,width=width+.001)]))
        assert joint_plan['accepted'],joint_plan
        assert len(joint_plan['joint_names'])==7 and len(joint_plan['keyframes'])==4,joint_plan
        write_json(output/'web_coordinated_plan.json',joint_plan)
        submitted=post('/api/execute_preview',dict(confirm='EXECUTE',plan_id=joint_plan['plan_id']))
        assert submitted['accepted'],submitted
        pause(joint_plan['duration']+1.)
        results['web_coordinated']=status()['teleop']['web_execute']
        assert results['web_coordinated']['state']=='done',results['web_coordinated']
        # 新格式原始记录仍是六轴加独立源时刻；经过现有网页 dry-run/execute 派生七轴。
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
        # 复用视觉序列和服务网关；只给已知可达的接触前目标，不启动相机或抓取续动。
        from rebotarm_vision.visual_grasp_sequence import (
            PoseTarget,VisualGraspSequenceConfig,build_visual_grasp_sequence)
        from rebotarm_vision.visual_grasp_service_gateway import VisualGraspServiceGateway
        from rebotarm_vision.visual_grasp_state import VisualGraspState
        from rebotarm_motion.pose_kinematics import PoseKinematics
        from rebotarm_msgs.srv import ExecutePose,PublishTrajectoryPreview
        from types import SimpleNamespace
        import pinocchio as pin
        pause(.8)
        current=dict(zip(latest['joint'].name,latest['joint'].position))
        model=PoseKinematics()
        def pose_for(delta):
            q=[current[n]+(delta if n=='joint1' else 0.) for n in names]
            pose=model.pose(q)
            return PoseTarget(tuple(pose.translation),tuple(pin.Quaternion(pose.rotation).coeffs()))
        width=current['left_finger_joint']*2.+.0004
        stages=build_visual_grasp_sequence(pose_for(.003),pose_for(.006),
            VisualGraspSequenceConfig(coordinated_position_only=True,open_position_m=width))
        config=dict(trajectory_execution_mode='prepared_v1',execution_mode='plan_only',
            target_frame='base_link',acceleration_scaling=.1,move_velocity_scaling=.1,
            approach_velocity_scaling=.1,retreat_velocity_scaling=.1)
        def visual_call(key,request,timeout):
            service,path=(ExecutePose,'execute_pose') if key=='execute_pose' else (PublishTrajectoryPreview,'publish_trajectory_preview')
            return call(service,'/rebotarm/motion_execution/'+path,request,timeout),''
        visual_state=VisualGraspState()
        gateway=VisualGraspServiceGateway(state=visual_state,config=config,
            io_gateway=SimpleNamespace(call=visual_call))
        for stage in stages:
            ok,message=gateway._send_execute_pose(stage,execute=False)
            assert ok,message
        assert len(visual_state.preview_trajectories)==2 and not visual_state.preview_gripper_events
        assert all(len(t.joint_names)==7 for t in visual_state.preview_trajectories)
        ok,message=gateway._publish_preview_sequence();assert ok,message
        gateway._execution_mode='execute'
        for stage in stages:
            pause(.8)
            ok,message=gateway._send_execute_pose(stage,execute=True)
            assert ok,message
        results['visual_precontact']=dict(stages=[s.name for s in stages],message=message,
            preview_tracks=[message_to_ordereddict(t) for t in visual_state.preview_trajectories])
        # 障碍位于左指外侧扫掠体：起点无碰撞、张开后碰撞，固定手指检查会漏掉此反例。
        import struct
        from ament_index_python.packages import get_package_share_directory
        from rebotarm_motion.coordinated_trajectory import generate_schedule,curve_message
        pause(.8)
        current=dict(zip(latest['joint'].name,latest['joint'].position))
        mesh=Path(get_package_share_directory('rebotarm_moveit_config'))/'meshes/left_finger.stl'
        data=mesh.read_bytes();count=struct.unpack_from('<I',data,80)[0]
        faces=[[struct.unpack_from('<3f',data,84+50*i+12+12*j) for j in range(3)] for i in range(count)]
        face=max(faces,key=lambda vertices:sum(v[1] for v in vertices)/3.)
        center=[sum(v[i] for v in face)/3. for i in range(3)]
        center[1]+=current['left_finger_joint']+.002
        obj=CollisionObject(id='p005_finger_sweep');obj.header.frame_id='end_link'
        obj.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.0002])]
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=center;pose.orientation.w=1.
        obj.primitive_poses=[pose];obj.operation=CollisionObject.ADD
        scene=ApplyPlanningScene.Request();scene.scene.is_diff=True;scene.scene.world.collision_objects=[obj]
        revision=latest['trajectory_scene_revision']
        assert call(ApplyPlanningScene,'/apply_planning_scene',scene).success
        wait(lambda:latest['trajectory_scene_revision']!=revision)
        try:
            validity=GetStateValidity.Request();validity.robot_state.joint_state=latest['joint']
            validity.robot_state.is_diff=False
            initial_valid=call(GetStateValidity,'/check_state_validity',validity)
            assert initial_valid.valid,initial_valid
            positions=list(validity.robot_state.joint_state.position)
            for index,name in enumerate(validity.robot_state.joint_state.name):
                if name=='left_finger_joint':positions[index]=current[name]+.002
                if name=='right_finger_joint':positions[index]=-current['left_finger_joint']-.002
            validity.robot_state.joint_state.position=positions
            swept=call(GetStateValidity,'/check_state_validity',validity)
            assert not swept.valid,'moving-finger obstacle fixture did not intersect its sweep'
            start=tuple(current[n] for n in names);width=current['left_finger_joint']*2.
            curve=generate_schedule(start,width,start,width+.008,1.)
            query=RetimeArmTrajectory.Request(trajectory=curve_message(curve,
                trajectory_factory=JointTrajectory,point_factory=JointTrajectoryPoint),
                velocity_scaling=1.,acceleration_scaling=1.,
                capability_revision=json.loads(latest['trajectory_capabilities'])['revision'])
            planned=call(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory',query)
            assert planned.success,planned.message
            rejected=finish(send(arm,FollowJointTrajectory.Goal(trajectory=planned.trajectory)),success=False)
            assert rejected.status!=4 and 'collision' in rejected.result.error_string,str(rejected)
            results['moving_finger_collision']=message_to_ordereddict(rejected)
        finally:
            obj.operation=CollisionObject.REMOVE
            scene.scene.world.collision_objects=[obj]
            assert call(ApplyPlanningScene,'/apply_planning_scene',scene).success
            pause(.8)
        # 显式取消并等新帧静止确认，之后继续接纳。
        target=list(latest['joint'].position[:6]);target[0]+=.03
        handle=send(arm,make(target,2.))
        wait(lambda:bool(feedbacks[id(handle)]),.65)
        future=handle.cancel_goal_async();wait(future.done)
        canceled=finish(handle,success=False)
        assert canceled.status==5,str(canceled)
        pause(.5)
        results['cancel']='pass'
        # 场景变化在执行中触发停止保持；移除后必须能继续接纳。
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
        # 真正终端入口经 PTY 读键；第二键必须在首步结束前明确丢弃。
        import pty
        import signal
        import subprocess
        terminal_events=[]
        subscription=node.create_subscription(String,'/rebotarm/teleop/status',
            lambda m:terminal_events.append(json.loads(m.data)),10)
        master,slave=pty.openpty()
        terminal_log=(output/'terminal_keyboard.log').open('w')
        process=subprocess.Popen(['ros2','run','rebotarm_teleop','TeleopKeyboardNode',
            '--ros-args','-p','trajectory_execution_mode:=prepared_v1'],
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
        # 真实准备服务拒绝原始 jerk 样本，不能用起点不匹配冒充超限拒绝。
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
        req.deadline_monotonic=time.monotonic()+2.
        rejection=call(PrepareArmTrajectory,'/rebotarm/prepare_arm_trajectory',req)
        assert not rejection.success and 'jerk' in rejection.message,rejection.message
        results['jerk_fixture_rejection']=rejection.message
        # 同一真实场景内分别验证环境和附着物几何的拒绝覆盖。
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
                # MoveIt 分离附着物时会放回 world；本测试还需显式清除该几何。
                cleanup=ApplyPlanningScene.Request();cleanup.scene.is_diff=True
                cleanup.scene.world.collision_objects=[obj]
                assert call(ApplyPlanningScene,'/apply_planning_scene',cleanup).success
            pause(.8)
        # 静止 position -> hold 仍由正式服务切换。
        req=SetMode.Request();req.mode='hold'
        assert call(SetMode,'/rebotarm/sim/set_mode',req).success
        pause(1.)
        write_json(output/'ros_acceptance.json',results)
        return results
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    """先记录本次安装资源摘要，再执行物理检查；只有显式 --ros 才访问已有仿真图。"""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--ros',action='store_true')
    parser.add_argument('--dashboard',default='http://127.0.0.1:18088')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    from ament_index_python.packages import get_package_share_directory
    simulation=Path(get_package_share_directory('rebotarm_simulation'))
    moveit=Path(get_package_share_directory('rebotarm_moveit_config'))
    frozen=[*sorted((simulation/'models').rglob('*')), *sorted((simulation/'config').glob('*.yaml')),
        moveit/'config/joint_limits.yaml']
    write_json(args.output/'frozen_files.json',{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in frozen if p.is_file()})
    result=dict(physics=physics_acceptance(args.output))
    if args.ros:result['ros']=ros_acceptance(args.output,args.dashboard)
    print(json.dumps(result.get('physics'),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
