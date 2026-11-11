"""网页六轴预览与独立夹爪目标的一次性执行令牌适配。

文件职责：校验操作员关键点，调用运动层生成及重定时，保存可预览的准确最终消息。
对外接口：CoordinatedWebPlans.preview/take；take 只交接原消息，不发送或重建轨迹。
安全约束：无设备访问；能力变化拒绝；固定时长不自动延长。
    夹爪只保存单次开合目标与独立接口力矩，不插值、不参与机械臂重定时。
    执行准入与闭合模型碰撞检查仍由正式后端负责。
文件结构：计划缓存 → 六轴预览及夹爪目标 → 单次交接与能力复核。
"""
from dataclasses import replace
import math
import threading
import uuid
from .web_execute import validate_web_execute_request
from rebotarm_motion.coordinated_trajectory import generate_schedule, curve_message


class CoordinatedWebPlans:
    def __init__(self, *, capability, retime, joint_names, joint_limits, trajectory_factory, point_factory):
        self.capability,self.retime = capability,retime
        self.names,self.limits = tuple(joint_names),joint_limits
        self.trajectory_factory,self.point_factory = trajectory_factory,point_factory
        self._plans = {}
        self._lock = threading.Lock()

    def preview(self, payload, current, *, max_delta, min_duration, max_duration, max_speed):
        """预览不获得执行资源；新关键点必须逐点满足现有位移/位置门限。"""
        capability = self.capability()
        if capability['mode'] != 'continuous':
            raise ValueError('coordinated motion requires explicit continuous mode')
        decision = validate_web_execute_request(dict(payload,confirm='EXECUTE'), joint_names=self.names,
            current_positions=current,joint_limits=self.limits,max_delta_rad=max_delta,
            min_duration=min_duration,max_duration=max_duration)
        if not decision.accepted:
            raise ValueError(decision.message)
        target_width = float(payload.get('gripper_position',0.))
        effort = float(payload.get('max_effort',0.))
        if not math.isfinite(target_width) or not 0 <= target_width <= .09 or not math.isfinite(effort) or effort < 0:
            raise ValueError('finite gripper total opening in [0,0.09] m required')
        if payload.get('gripper_keyframes'):
            raise ValueError('gripper keyframes are unsupported; use one independent opening target')
        for item in payload.get('arm_keyframes',[]) or []:
            check = validate_web_execute_request(dict(payload,confirm='EXECUTE',
                joint_positions=dict(zip(self.names,item['positions']))),joint_names=self.names,
                current_positions=current,joint_limits=self.limits,max_delta_rad=max_delta,
                min_duration=min_duration,max_duration=max_duration)
            if not check.accepted:
                raise ValueError(f'arm keyframe: {check.message}')
        start = tuple(current[n] for n in self.names)
        curve = generate_schedule(start,decision.positions,decision.duration,
            arm_keyframes=payload.get('arm_keyframes'))
        message = curve_message(curve,trajectory_factory=self.trajectory_factory,point_factory=self.point_factory)
        message = self.retime(message,capability, bool(payload.get('fixed_duration',False)),max_speed)
        # 显示和执行均用服务返回后的六轴纳秒点，夹爪预览只表示独立目标。
        from rebotarm_motion.polynomial_trajectory import build_curve, TrajectoryPoint
        final = build_curve(message.joint_names,tuple(TrajectoryPoint(
            p.time_from_start.sec+p.time_from_start.nanosec*1e-9,p.positions,p.velocities,p.accelerations)
            for p in message.points))
        from rebotarm_trajectory.planning import decode_plan
        plan = decode_plan(message.header.frame_id, message.joint_names, final.points,
            capability_revision=capability['revision'])
        if plan is None:
            raise ValueError('final planning binding missing')
        token = uuid.uuid4().hex
        decision = replace(decision,duration=final.duration, positions=final.points[-1].positions)
        with self._lock:
            if len(self._plans) >= 32:
                self._plans.pop(next(iter(self._plans)))
            self._plans[token] = (capability['revision'],message,decision,
                (target_width, min(effort, capability['gripper_torque_cap_nm'])) if 'gripper_position' in payload else ())
        count = min(3000,max(2,math.ceil(final.duration/.05)))
        return dict(accepted=True,executable=plan['purpose']=='execution',plan_id=token,duration=final.duration,
            torque_cap_nm=capability['gripper_torque_cap_nm'],joint_names=list(final.joint_names),
            keyframes=[dict(time=p.time,positions=p.positions,velocities=p.velocities,accelerations=p.accelerations)
                for p in final.points],
            preview=[dict(time=final.duration*i/count,positions=final.sample(final.duration*i/count)[0]) for i in range(count+1)],
            gripper_position=target_width, collision_policy='web_closed',
            message=('仅预览：使能后须重新预览。' if plan['purpose']=='preview' else '')+
                '按闭合夹爪模型检查；夹爪独立开合，不保证与机械臂同时到达')

    def take(self, token):
        """单次交接原计划；准备只复核，不能修改已绑定的节点与时间。"""
        capability = self.capability()
        with self._lock:
            # 先消费令牌再复核：并发提交只交接一次，复核失败后须重新预览取得新令牌。
            plan = self._plans.pop(str(token),None)
        if plan is None:
            raise ValueError('coordinated preview missing; preview again')
        revision,message,decision,gripper = plan
        if revision != capability['revision']:
            raise ValueError('coordinated planning capability changed; preview again')
        from rebotarm_trajectory.planning import decode_plan
        from rebotarm_trajectory.polynomial import TrajectoryPoint
        decode_plan(message.header.frame_id, message.joint_names, tuple(TrajectoryPoint(
            p.time_from_start.sec+p.time_from_start.nanosec*1e-9,p.positions,p.velocities,p.accelerations)
            for p in message.points), capability_revision=revision, require_execution=True)
        return message,decision,gripper
