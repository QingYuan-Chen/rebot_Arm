"""隔离域中的批次 6 ROS 验证入口：只能装配假句柄，禁止打开真实通道。

需先在同一空闲 ROS_DOMAIN_ID 启动 moveit_demo（禁用 RViz 和假关节源）。
本进程使用安装区控制器和 motion，SDK 导入被拦截；MoveIt 服务端是真实
move_group。所有使能、轨迹和失能仅作用于本进程创建的内存假电机。
"""
import importlib.abc
import json
import os
from pathlib import Path
import sys
import threading
import time

if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '0':
    raise RuntimeError('explicit isolated ROS domain required')
class BlockSDK(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] == 'reBotArm_control_py':
            raise AssertionError('SDK import attempted')
sys.meta_path.insert(0, BlockSDK())
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from test_controller_local_runtime import HandleController
from rebotarmcontroller.hardware.sdk_driver import SdkMotorDriver

buses = []
class FakeBus(HandleController):
    def add_damiao_motor(self, number, feedback, model):
        motor = super().add_damiao_motor(number, feedback, model)
        motor.state.pos = -.017453292519943295 if number == 3 else -4. if number == 7 else 0.
        if number == 7:
            original = motor.send_mit
            def send(*args):
                original(*args)
                if args[2] > 0:
                    motor.state.pos = args[0]
            motor.send_mit = send
        return motor

def fake_only(channel, baud):
    bus = FakeBus()
    buses.append(bus)
    return bus
SdkMotorDriver._open_controller = staticmethod(fake_only)

import rclpy
from rclpy.action import ActionClient, get_action_server_names_and_types_by_node
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan
from rebotarm_msgs.action import MoveToPose, ExecuteTeleopMotion
from rebotarm_msgs.srv import MoveToPoseIK
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rebotarmcontroller.rebotarm_controller import reBotArmController
from rebotarm_motion.pose_compat_node import PoseCompatNode
from rebotarm_motion.mujoco_moveit_acceptance import _build_motion_plan_request


def wait(future, timeout=15.):
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    if not future.done():
        raise TimeoutError('ROS future timeout')
    return future.result()

rclpy.init()
controller = reBotArmController()
compat = PoseCompatNode()
client = Node('batch6_offline_probe')
executor = MultiThreadedExecutor(num_threads=8)
for node in (controller, compat, client):
    executor.add_node(node)
worker = threading.Thread(target=executor.spin, daemon=True)
worker.start()
report = dict(domain=os.environ['ROS_DOMAIN_ID'], backend='in-memory fake motor handles', sdk_blocked=True)
try:
    report['trajectory_mode'] = controller.get_parameter('trajectory_execution_mode').value
    def trigger(name):
        connection = client.create_client(Trigger, '/rebotarm/'+name)
        assert connection.wait_for_service(timeout_sec=5.)
        result = wait(connection.call_async(Trigger.Request()))
        assert result.success, result.message
        return result.message
    def ready():
        if controller.get_parameter('trajectory_execution_mode').value != 'continuous':
            return
        deadline = time.monotonic()+5.
        while not controller.hardware.arm.admission_snapshot()['ready'] and time.monotonic()<deadline:
            time.sleep(.02)
        assert controller.hardware.arm.admission_snapshot()['ready'], controller.hardware.arm.admission_snapshot()
    fjt = ActionClient(client, FollowJointTrajectory, '/rebotarm/follow_joint_trajectory')
    assert fjt.wait_for_server(timeout_sec=10.)
    time.sleep(1.)
    owners = [(n, ns) for n, ns in client.get_node_names_and_namespaces()
              for name, _ in get_action_server_names_and_types_by_node(client, n, ns)
              if name == '/rebotarm/follow_joint_trajectory']
    assert owners == [('reBotArmController', '/')], owners
    report['fjt_owners'] = owners
    start = controller.hardware.feedback.get_joint_state()[0]
    def goal(target, duration=.2):
        value = FollowJointTrajectory.Goal()
        value.trajectory.joint_names = controller.hardware.runtime.joint_names
        for q, t in ((start,0.), (target,duration)):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start = Duration(sec=int(t), nanosec=round((t-int(t))*1e9))
            value.trajectory.points.append(p)
        return value
    assert not wait(fjt.send_goal_async(goal(start))).accepted
    report['default_disabled_reject'] = True
    if controller.get_parameter('trajectory_execution_mode').value == 'continuous':
        from rebotarm_msgs.srv import PrepareArmTrajectory,GetArmPlanningState,RetimeArmTrajectory
        preparation = client.create_client(PrepareArmTrajectory,'/rebotarm/prepare_arm_trajectory')
        assert preparation.wait_for_service(timeout_sec=20.)
        state_client=client.create_client(GetArmPlanningState,'/rebotarm/get_arm_planning_state')
        assert state_client.wait_for_service(timeout_sec=5.)
        snapshot=wait(state_client.call_async(GetArmPlanningState.Request()))
        assert snapshot.success,snapshot.message
        state=json.loads(snapshot.state_json)
        assert not state['executable'] and state['reason']=='disabled'
        service=client.create_client(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory')
        assert service.wait_for_service(timeout_sec=5.)
        query=RetimeArmTrajectory.Request(trajectory=goal(start).trajectory,
            velocity_scaling=1.,acceleration_scaling=1.)
        preview=wait(service.call_async(query))
        assert preview.success and not preview.executable,preview.message
        report['disabled_preview']=True
    report['enable'] = trigger('enable')
    ready()
    if controller.get_parameter('trajectory_execution_mode').value == 'continuous':
        assert not wait(fjt.send_goal_async(FollowJointTrajectory.Goal(trajectory=preview.trajectory))).accepted
        report['preview_rejected_after_enable']=True
    target = start.copy(); target[0] += .03
    handle = wait(fjt.send_goal_async(goal(target)))
    assert handle.accepted
    completed = wait(handle.get_result_async())
    assert completed.status == 4 and completed.result.error_code == 0, completed
    report['fjt_execute'] = [completed.status, completed.result.error_code]
    ready()
    start = controller.hardware.feedback.get_joint_state()[0]
    target = start.copy(); target[0] += .02
    handle = wait(fjt.send_goal_async(goal(target, 2.)))
    assert handle.accepted
    canceled = wait(handle.cancel_goal_async())
    assert canceled.goals_canceling
    completed = wait(handle.get_result_async())
    assert completed.status == 5, completed
    report['fjt_cancel'] = completed.result.error_string
    assert controller.hardware.lifecycle.enabled
    ready()
    planner = client.create_client(GetMotionPlan, '/plan_kinematic_path')
    assert planner.wait_for_service(timeout_sec=15.)
    start = controller.hardware.feedback.get_joint_state()[0]
    target = start.copy(); target[0] += .02
    request = _build_motion_plan_request(service_type=GetMotionPlan,
        joint_names=controller.hardware.runtime.joint_names, target_positions=target,
        group_name='arm', start_positions=start)
    response = wait(planner.call_async(request))
    assert response.motion_plan_response.error_code.val == 1, response
    # 必须通过 move_group 的 ExecuteTrajectory，再由控制器管理器转 FJT。
    execute = ActionClient(client, ExecuteTrajectory, '/execute_trajectory')
    assert execute.wait_for_server(timeout_sec=10.)
    value = ExecuteTrajectory.Goal()
    value.trajectory = response.motion_plan_response.trajectory
    handle = wait(execute.send_goal_async(value))
    assert handle.accepted
    completed = wait(handle.get_result_async())
    assert completed.status == 4 and completed.result.error_code.val == 1, completed
    report['moveit_execute'] = [completed.status, completed.result.error_code.val]
    report['planned_points'] = len(value.trajectory.joint_trajectory.points)
    ready()
    if controller.get_parameter('trajectory_execution_mode').value == 'continuous':
        from rebotarm_msgs.srv import RetimeArmTrajectory
        from rebotarm_motion.coordinated_trajectory import generate_schedule, curve_message
        from trajectory_msgs.msg import JointTrajectory
        time.sleep(.5)
        current = tuple(controller.hardware.feedback.get_joint_state()[0])
        width = controller.hardware.gripper.gripper_position_m()
        curve = generate_schedule(current,(current[0]+.01,*current[1:]),1.)
        service = client.create_client(RetimeArmTrajectory,'/rebotarm/retime_arm_trajectory')
        assert service.wait_for_service(timeout_sec=3.)
        query = RetimeArmTrajectory.Request(trajectory=curve_message(curve,
            trajectory_factory=JointTrajectory,point_factory=JointTrajectoryPoint),
            velocity_scaling=1.,acceleration_scaling=1.,
            capability_revision=controller.hardware.arm.trajectory_capability('continuous')['revision'])
        plan = wait(service.call_async(query)); assert plan.success, plan.message
        teleop = ActionClient(client,ExecuteTeleopMotion,'/rebotarm/execute_teleop_motion')
        assert teleop.wait_for_server(timeout_sec=5.)
        coordinated = ExecuteTeleopMotion.Goal(trajectory=plan.trajectory,move_gripper=True,
            gripper_position=width+.001,gripper_max_effort=1.)
        handle = wait(teleop.send_goal_async(coordinated)); assert handle.accepted
        assert not wait(teleop.send_goal_async(coordinated)).accepted, 'busy goal queued'
        done = wait(handle.get_result_async())
        assert done.status == 4 and done.result.error_code == 0, done
        assert done.result.arm_state == done.result.gripper_state == 'succeeded', done
        assert controller.hardware.lifecycle.enabled
        report['coordinated_execute'] = [done.status,done.result.error_code]
        # 真实 ROS Action 链覆盖残差取消与组合→六轴交接；只改内存假电机反馈。
        motor = buses[0].motors[0]
        original_send = motor.send_pos_vel
        residuals = []
        try:
            for residual in (.003, .01):
                def offset_send(position, velocity):
                    original_send(position, velocity)
                    motor.state.pos = position+residual
                motor.send_pos_vel = offset_send
                time.sleep(.3)
                ready()
                current = tuple(controller.hardware.feedback.get_joint_state()[0])
                width = controller.hardware.gripper.gripper_position_m()
                curve = generate_schedule(current,(current[0]+.01,*current[1:]),1.)
                query.trajectory = curve_message(curve,trajectory_factory=JointTrajectory,
                    point_factory=JointTrajectoryPoint)
                plan = wait(service.call_async(query)); assert plan.success, plan.message
                handle = wait(teleop.send_goal_async(ExecuteTeleopMotion.Goal(trajectory=plan.trajectory,move_gripper=True,gripper_position=width+.0002,gripper_max_effort=1.)))
                assert handle.accepted
                deadline = time.monotonic()+2.
                while controller.hardware.lifecycle.execution != 'TRAJ_RUNNING' and time.monotonic()<deadline:
                    time.sleep(.005)
                assert controller.hardware.lifecycle.execution == 'TRAJ_RUNNING'
                cancel_response = wait(handle.cancel_goal_async())
                done = wait(handle.get_result_async())
                assert cancel_response.goals_canceling, (residual, cancel_response, done)
                assert done.status == 5, done
                # 小开口可能在取消前已经独立到位；已完成侧保留成功，整体仍为取消。
                assert done.result.arm_state == 'canceled' and done.result.gripper_state in ('failed','succeeded'), done
                assert not controller.hardware.runtime.arm_resource_reserved
                assert controller.hardware.arm._stop_wait is None
                ready()
                start = controller.hardware.feedback.get_joint_state()[0]
                target = start.copy();target[0] += .01
                handle = wait(fjt.send_goal_async(goal(target,1.)))
                assert handle.accepted
                done = wait(handle.get_result_async())
                assert done.status == 4 and done.result.error_code == 0, done
                ready()
                assert not controller.hardware.runtime.arm_resource_reserved
                residuals.append(dict(residual=residual, coordinated_cancel=5, next_arm_execution=4))
        finally:
            motor.send_pos_vel = original_send
        report['residual_cancel_and_next_goal'] = residuals
    time.sleep(.5)
    ready()
    ik = client.create_client(MoveToPoseIK, '/rebotarm/move_to_pose_ik')
    assert ik.wait_for_service(timeout_sec=5.)
    request = MoveToPoseIK.Request()
    request.target_pose = controller.hardware.arm.current_pose()
    response = wait(ik.call_async(request), 35.)
    assert response.success, response.message
    report['ik_acceptance'] = response.message
    deadline = time.monotonic()+10.
    while compat.active is not None and time.monotonic()<deadline:
        time.sleep(.02)
    assert compat.active is None
    ready()
    pose = ActionClient(client, MoveToPose, '/rebotarm/move_to_pose')
    assert pose.wait_for_server(timeout_sec=5.)
    value = MoveToPose.Goal()
    value.target_pose, value.duration = controller.hardware.arm.current_pose(), .2
    handle = wait(pose.send_goal_async(value))
    assert handle.accepted
    completed = wait(handle.get_result_async(), 35.)
    assert completed.status == 4 and completed.result.success, completed
    report['pose_action'] = completed.result.message
    report['disable'] = trigger('disable')
    assert all(m.state.status_code == 0 for m in buses[0].motors)
    report['ok'] = True
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
finally:
    # 这里仅有内存假电机，不连接真实端口；显式恢复失能再释放本轮节点。
    if controller.hardware.lifecycle.enabled:
        controller.hardware.lifecycle.disable()
    controller.hardware.lifecycle.shutdown()
    executor.shutdown(timeout_sec=5.)
    for node in (client, compat, controller):
        node.destroy_node()
    rclpy.try_shutdown()
