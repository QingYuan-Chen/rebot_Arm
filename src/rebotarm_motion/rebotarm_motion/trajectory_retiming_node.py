"""规划阶段纯计算重定时的 ROS 服务。

文件职责：读取后端只读能力，将具名点约束交给共享运动算法并编码最终纳秒轨迹。
对外接口：/<namespace>/retime_arm_trajectory；仅返回计划，不提交执行请求。
安全约束：无 MoveItPy、SDK、设备请求或碰撞许可；固定时长超限拒绝。
    能力缺失、过期或计算超过两秒均失败；执行端仍须重新核验场景与设备上下文。
文件结构：消息/数值转换 → 重定时回调 → 能力订阅及节点生命周期。
"""

from pathlib import Path
from dataclasses import replace
import time
import math
from .polynomial_trajectory import TrajectoryPoint
from .coordinated_trajectory import retime_curve, effective_limits
from rebotarm_trajectory.joint_contract import canonical_input
from rebotarm_trajectory.capability import decode_capability, binding


def retime_message(request, capability, config_directory):
    """数值入口不创建节点；右指核验后省去从动行，保留单夹爪请求维度。"""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    capability = decode_capability(capability)
    if request.capability_revision and request.capability_revision != capability['revision']:
        raise ValueError('planning capability changed')
    trajectory = request.trajectory
    if any(p.effort for p in trajectory.points):
        raise ValueError('FJT effort is not a torque-cap override')
    points = tuple(TrajectoryPoint(p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
        p.positions,p.velocities,p.accelerations) for p in trajectory.points)
    names, points = canonical_input(trajectory.joint_names,points,(0.,)*6)
    if not any(name.startswith('joint') for name in trajectory.joint_names):
        names = ('left_finger_joint',)
        points = tuple(TrajectoryPoint(p.time,p.positions[-1:],p.velocities[-1:],p.accelerations[-1:]) for p in points)
    if points[0].time != 0.:
        raise ValueError('planning requires an explicit initial point at time zero')
    low, high, limits = effective_limits(config_directory,names,capability)
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
    result.header.frame_id = binding(capability)
    result.joint_names = list(curve.joint_names)
    for p in curve.points:
        point = JointTrajectoryPoint(positions=list(p.positions),velocities=list(p.velocities),
            accelerations=list(p.accelerations))
        ns = round(p.time*1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns,1_000_000_000)
        result.points.append(point)
    return result, scale


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from ament_index_python.packages import get_package_share_directory
    from rebotarm_msgs.srv import RetimeArmTrajectory
    from std_msgs.msg import String
    rclpy.init(args=args)
    node = Node('trajectory_retiming')
    namespace = node.declare_parameter('arm_namespace','rebotarm').value.strip('/')
    directory = Path(get_package_share_directory('rebotarm_moveit_config'))/'config'
    snapshot = [None, 0.]
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
            response.trajectory,response.time_scale = retime_message(request,capability,directory)
            if time.monotonic()-started > 2.:
                raise TimeoutError('trajectory retiming deadline exceeded')
            response.success,response.message = True,'planned; collision preparation still required'
        except Exception as exc:
            response.success,response.message = False,str(exc)
        return response
    node.create_service(RetimeArmTrajectory,f'/{namespace}/retime_arm_trajectory',retime)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
