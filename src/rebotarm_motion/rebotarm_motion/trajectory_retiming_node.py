"""规划阶段纯计算重定时的 ROS 服务。

文件职责：读取后端只读能力，将具名点约束交给共享运动算法并编码最终纳秒轨迹。
对外接口：/<namespace>/retime_arm_trajectory；仅返回计划，不提交执行请求。
安全约束：只读后端规划快照，不预约资源或发出设备命令；无碰撞许可，固定时长超限拒绝。
    能力缺失、过期或计算超过两秒均失败；执行端仍须重新核验场景与设备上下文。
文件结构：消息/数值转换 → 重定时回调 → 能力订阅及节点生命周期。
"""

from pathlib import Path
from dataclasses import replace
import time
import math
import json
from .polynomial_trajectory import TrajectoryPoint, build_curve
from .coordinated_trajectory import retime_curve, effective_limits
from rebotarm_trajectory.joint_contract import canonical_input
from rebotarm_trajectory.capability import decode_capability
from rebotarm_trajectory.planning import decode_state, plan_binding, decode_plan


def retime_message(request, capability, config_directory, planning_state=None):
    """数值入口不创建节点；六轴执行曲线和单左指规划预算分别核定，拒绝七轴。"""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    capability = decode_capability(capability)
    if request.capability_revision and request.capability_revision != capability['revision']:
        raise ValueError('planning capability changed')
    trajectory = request.trajectory
    if any(p.effort for p in trajectory.points):
        raise ValueError('FJT effort is not a torque-cap override')
    points = tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
        p.positions,p.velocities,p.accelerations) for p in trajectory.points)
    if tuple(trajectory.joint_names) == ('left_finger_joint',):
        # 仅为 MoveIt 独立夹爪规划核定预算；执行映射为 GripperCommand，绝不送入 FJT。
        names = ('left_finger_joint',)
    else:
        names, points = canonical_input(trajectory.joint_names,points)
    if points[0].time != 0.:
        raise ValueError('planning requires an explicit initial point at time zero')
    build_curve(names, points)  # 替换起点前完整验证原始维度、等级、有限性与时间。
    low, high, limits = effective_limits(config_directory,names,capability)
    # 必须在起点替换前拒绝原始越界节点，不能以保持值覆盖来修复非法输入。
    if any(q < lo-1e-9 or q > hi+1e-9 for p in points for q,lo,hi in zip(p.positions,low,high)):
        raise ValueError('original planning position outside limits')
    purpose = 'reference' if getattr(request, 'planning_purpose', 0) == 1 else 'execution'
    if getattr(request, 'planning_purpose', 0) not in (0, 1):
        raise ValueError('unknown planning purpose')
    if names == ('left_finger_joint',):
        purpose = 'gripper'
    elif purpose == 'execution':
        if planning_state is None:
            raise ValueError('backend planning state required')
        planning_state = decode_state(planning_state)
        if planning_state['capability_revision'] != capability['revision']:
            raise ValueError('planning capability changed')
        if planning_state['executable']:
            # 起点变更仅发生于规划阶段；重定时、纳秒复核和后续碰撞均针对这条曲线。
            points = (replace(points[0], positions=tuple(planning_state['hold'])),)+points[1:]
        else:
            purpose = 'preview'
    speed = float(getattr(request,'max_arm_speed_rad_s',0.))
    if not math.isfinite(speed) or speed < 0:
        raise ValueError('nonnegative finite arm speed cap required')
    if speed:
        limits = {n:replace(limit,max_velocity=min(speed,limit.max_velocity)) if n.startswith('joint') else limit
            for n,limit in limits.items()}
    curve, scale = retime_curve(names,points,low,high,limits,
        velocity_scaling=request.velocity_scaling, acceleration_scaling=request.acceleration_scaling,
        fixed=request.fixed_duration)
    result = JointTrajectory()
    result.joint_names = list(curve.joint_names)
    for p in curve.points:
        point = JointTrajectoryPoint(positions=list(p.positions),velocities=list(p.velocities),
            accelerations=list(p.accelerations))
        ns = round(p.time*1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns,1_000_000_000)
        result.points.append(point)
    encoded = tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
        p.positions,p.velocities,p.accelerations) for p in result.points)
    # 编码后复核并绑定最终消息，摘要不能基于编码前的浮点时间。
    final = build_curve(names, encoded)
    from .trajectory_preparation import check_curve
    if check_curve(final,low,high,limits)['state'] != 'pass':
        raise ValueError('encoded final planning curve exceeds limits')
    result.header.frame_id = plan_binding(capability['revision'],names,encoded,
        state=planning_state if purpose in ('execution','preview') else None, purpose=purpose)
    return result, scale


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from ament_index_python.packages import get_package_share_directory
    from rebotarm_msgs.srv import RetimeArmTrajectory, GetArmPlanningState
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    rclpy.init(args=args)
    node = Node('trajectory_retiming')
    namespace = node.declare_parameter('arm_namespace','rebotarm').value.strip('/')
    # 默认关闭；仿真验收显式指定，用于对照原输入、后端快照与准确输出，页面不刷数值日志。
    trace_path = node.declare_parameter('planning_trace_path','').value
    directory = Path(get_package_share_directory('rebotarm_moveit_config'))/'config'
    snapshot = [None, 0.]
    state_client = node.create_client(GetArmPlanningState,f'/{namespace}/get_arm_planning_state',
        callback_group=ReentrantCallbackGroup())
    def receive(message):
        try:
            snapshot[:] = [decode_capability(message.data), time.monotonic()]
        except (ValueError,KeyError,TypeError) as exc:
            snapshot[:] = [None, 0.]
            node.get_logger().error(f'invalid trajectory capability: {exc}')
    node.create_subscription(String,f'/{namespace}/trajectory_capabilities', receive,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def retime(request,response):
        started = time.monotonic()
        try:
            capability, observed = snapshot
            if capability is None or started-observed > 1.:
                raise RuntimeError('fresh trajectory capability unavailable')
            state = None
            if (request.planning_purpose == 0
                    and tuple(request.trajectory.joint_names) != ('left_finger_joint',)):
                deadline = started+2.
                if not state_client.wait_for_service(timeout_sec=max(0.,deadline-time.monotonic())):
                    raise RuntimeError('backend planning state service unavailable')
                future = state_client.call_async(GetArmPlanningState.Request())
                while not future.done():
                    if time.monotonic() >= deadline:
                        future.cancel()
                        raise TimeoutError('planning state exceeded total 2 s deadline')
                    time.sleep(.002)
                result = future.result()
                if not result.success:
                    raise ValueError(result.message)
                state = decode_state(result.state_json)
            response.trajectory,response.time_scale = retime_message(request,capability,directory,state)
            final_points = tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                p.positions,p.velocities,p.accelerations) for p in response.trajectory.points)
            response.executable = decode_plan(response.trajectory.header.frame_id,
                response.trajectory.joint_names, final_points)['purpose'] in ('execution','gripper')
            if time.monotonic()-started > 2.:
                raise TimeoutError('trajectory retiming deadline exceeded')
            response.success,response.message = True,'planned; collision preparation still required'
            if not response.executable:
                response.message = 'reference/disabled preview only; enable and re-plan before execution'
            if trace_path:
                from rosidl_runtime_py.convert import message_to_ordereddict
                with Path(trace_path).open('a',encoding='utf-8') as stream:
                    stream.write(json.dumps(dict(input=message_to_ordereddict(request.trajectory),
                        snapshot=state,final=message_to_ordereddict(response.trajectory),
                        time_scale=response.time_scale,executable=response.executable),allow_nan=False)+'\n')
            source = request.trajectory.points[0]
            final = response.trajectory.points
            node.get_logger().info('execution plan purpose=%s start=%s hold=%s correction=%s first_dt=%.9f scale=%.6f T=%.6f' % (
                request.planning_purpose,list(source.positions),state['hold'] if state else None,
                [b-a for a,b in zip(source.positions,final[0].positions)],
                (final[1].time_from_start.sec+final[1].time_from_start.nanosec*1e-9) if len(final)>1 else 0.,
                response.time_scale,final[-1].time_from_start.sec+final[-1].time_from_start.nanosec*1e-9))
        except Exception as exc:
            response.success,response.message = False,str(exc)
        return response
    node.create_service(RetimeArmTrajectory,f'/{namespace}/retime_arm_trajectory',retime)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
