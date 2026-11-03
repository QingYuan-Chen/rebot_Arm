#!/usr/bin/env python3
"""P005 冻结物理对照及隔离 ROS 全链验收；永不启动真实控制器。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.request
from urllib.error import HTTPError


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def physics_acceptance(output):
    from rebotarm_motion.polynomial_trajectory import JOINT_NAMES, build_curve
    from rebotarm_motion.trajectory_preparation import point_to_point
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco
    from rebotarm_simulation.diagnostics.trajectory_trace import TrajectoryTrace
    start = (0., -1., -1., 0., .3, 0.)
    target = (.02, *start[1:])
    curve = build_curve(JOINT_NAMES, point_to_point(start, target, .4))
    report = {}
    for mode in ('legacy_step', 'prepared_v1'):
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
    assert report['prepared_v1']['hold_drift'] <= 3*25/65535, report
    assert report['prepared_v1']['switch_torque_jump'] <= .01, report
    write_json(output/'physics_acceptance.json', report)
    return report


def ros_acceptance(output, dashboard):
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
    from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
    from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, AttachedCollisionObject
    from moveit_msgs.action import ExecuteTrajectory
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from rebotarm_msgs.srv import PrepareArmTrajectory, SetGripper, SetMode
    from rebotarm_motion.trajectory_messages import point_to_point_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init()
    node = Node('p005_simulation_acceptance')
    latest = {}
    samples = []
    node.create_subscription(JointState, '/rebotarm/joint_states',
        lambda m: (latest.update(joint=m), samples.append(m)), 10)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for name in ('trajectory_execution_mode','trajectory_scene_revision'):
        node.create_subscription(String, '/rebotarm/'+name,
            lambda m, key=name: latest.update({key:m.data}), qos)
    def wait(predicate, timeout=8.):
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
        future=handle.get_result_async();wait(future.done,12.)
        response=future.result()
        if success:assert response.status==4 and response.result.error_code==0,str(response)
        return response
    def post(path,payload):
        request=urllib.request.Request(dashboard+path,data=json.dumps(payload).encode(),
            headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=3) as response:return json.load(response)
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
