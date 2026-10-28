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
from rebotarmcontroller.sdk_driver import SdkMotorDriver

buses = []
class FakeBus(HandleController):
    def add_damiao_motor(self, number, feedback, model):
        motor = super().add_damiao_motor(number, feedback, model)
        motor.state.pos = -.017453292519943295 if number == 3 else -4. if number == 7 else 0.
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
from rebotarm_msgs.action import MoveToPose
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
    def trigger(name):
        connection = client.create_client(Trigger, '/rebotarm/'+name)
        assert connection.wait_for_service(timeout_sec=5.)
        result = wait(connection.call_async(Trigger.Request()))
        assert result.success, result.message
        return result.message
    fjt = ActionClient(client, FollowJointTrajectory, '/rebotarm/follow_joint_trajectory')
    assert fjt.wait_for_server(timeout_sec=10.)
    time.sleep(1.)
    owners = [(n, ns) for n, ns in client.get_node_names_and_namespaces()
              for name, _ in get_action_server_names_and_types_by_node(client, n, ns)
              if name == '/rebotarm/follow_joint_trajectory']
    assert owners == [('reBotArmController', '/')], owners
    report['fjt_owners'] = owners
    start = controller.hardware.get_joint_state()[0]
    def goal(target, duration=.2):
        value = FollowJointTrajectory.Goal()
        value.trajectory.joint_names = controller.hardware.joint_names
        for q, t in ((start,0.), (target,duration)):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start = Duration(sec=int(t), nanosec=round((t-int(t))*1e9))
            value.trajectory.points.append(p)
        return value
    assert not wait(fjt.send_goal_async(goal(start))).accepted
    report['default_disabled_reject'] = True
    report['enable'] = trigger('enable')
    target = start.copy(); target[0] += .03
    handle = wait(fjt.send_goal_async(goal(target)))
    assert handle.accepted
    completed = wait(handle.get_result_async())
    assert completed.status == 4 and completed.result.error_code == 0, completed
    report['fjt_execute'] = [completed.status, completed.result.error_code]
    start = controller.hardware.get_joint_state()[0]
    target = start.copy(); target[0] += .02
    handle = wait(fjt.send_goal_async(goal(target, 2.)))
    assert handle.accepted
    canceled = wait(handle.cancel_goal_async())
    assert canceled.goals_canceling
    completed = wait(handle.get_result_async())
    assert completed.status == 5, completed
    report['fjt_cancel'] = completed.result.error_string
    assert controller.hardware.enabled
    planner = client.create_client(GetMotionPlan, '/plan_kinematic_path')
    assert planner.wait_for_service(timeout_sec=15.)
    start = controller.hardware.get_joint_state()[0]
    target = start.copy(); target[0] += .02
    request = _build_motion_plan_request(service_type=GetMotionPlan,
        joint_names=controller.hardware.joint_names, target_positions=target,
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
    time.sleep(.5)
    ik = client.create_client(MoveToPoseIK, '/rebotarm/move_to_pose_ik')
    assert ik.wait_for_service(timeout_sec=5.)
    request = MoveToPoseIK.Request()
    request.target_pose = controller.hardware.current_pose()
    response = wait(ik.call_async(request), 35.)
    assert response.success, response.message
    report['ik_acceptance'] = response.message
    deadline = time.monotonic()+10.
    while compat.active is not None and time.monotonic()<deadline:
        time.sleep(.02)
    assert compat.active is None
    pose = ActionClient(client, MoveToPose, '/rebotarm/move_to_pose')
    assert pose.wait_for_server(timeout_sec=5.)
    value = MoveToPose.Goal()
    value.target_pose, value.duration = controller.hardware.current_pose(), .2
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
    if controller.hardware.enabled:
        controller.hardware.disable()
    controller.hardware.shutdown()
    executor.shutdown(timeout_sec=5.)
    for node in (client, compat, controller):
        node.destroy_node()
    rclpy.try_shutdown()
