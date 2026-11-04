"""网页联合计划与一次性执行令牌适配。

文件职责：校验操作员关键点，调用运动层生成及重定时，保存可预览的准确最终消息。
对外接口：CoordinatedWebPlans.preview/take；take 只交接原消息，不发送或重建轨迹。
安全约束：无设备访问；能力变化拒绝；固定时长不自动延长。
    独立夹爪的逐请求力矩不能进入协同路径。执行准入与碰撞仍由正式后端负责。
文件结构：计划缓存 → 联合预览 → 单次交接与上下文复核。
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
        if capability['mode'] != 'prepared_v1':
            raise ValueError('coordinated motion requires explicit prepared_v1 mode')
        if 'max_effort' in payload or 'torque_cap' in payload:
            raise ValueError('coordinated torque cap is fixed by startup configuration')
        decision = validate_web_execute_request(dict(payload,confirm='EXECUTE'), joint_names=self.names,
            current_positions=current,joint_limits=self.limits,max_delta_rad=max_delta,
            min_duration=min_duration,max_duration=max_duration)
        if not decision.accepted:
            raise ValueError(decision.message)
        width = float(current['gripper'])
        target_width = float(payload['gripper_position'])
        if not all(math.isfinite(q) and 0 <= q <= .09 for q in (width,target_width)):
            raise ValueError('finite gripper total opening in [0,0.09] m required')
        for item in payload.get('arm_keyframes',[]) or []:
            check = validate_web_execute_request(dict(payload,confirm='EXECUTE',
                joint_positions=dict(zip(self.names,item['positions']))),joint_names=self.names,
                current_positions=current,joint_limits=self.limits,max_delta_rad=max_delta,
                min_duration=min_duration,max_duration=max_duration)
            if not check.accepted:
                raise ValueError(f'arm keyframe: {check.message}')
        start = tuple(current[n] for n in self.names)
        curve = generate_schedule(start,width,decision.positions,target_width,decision.duration,
            arm_keyframes=payload.get('arm_keyframes'),gripper_keyframes=payload.get('gripper_keyframes'))
        message = curve_message(curve,trajectory_factory=self.trajectory_factory,point_factory=self.point_factory)
        message = self.retime(message,capability, bool(payload.get('fixed_duration',False)),max_speed)
        # 显示和执行均用服务返回后的纳秒点，不拿原始时长或独立动画冒充联合计划。
        from rebotarm_motion.polynomial_trajectory import build_curve, TrajectoryPoint
        final = build_curve(message.joint_names,tuple(TrajectoryPoint(
            p.time_from_start.sec+p.time_from_start.nanosec*1e-9,p.positions,p.velocities,p.accelerations)
            for p in message.points))
        token = uuid.uuid4().hex
        decision = replace(decision,duration=final.duration, positions=final.points[-1].positions[:6])
        with self._lock:
            if len(self._plans) >= 32:
                self._plans.pop(next(iter(self._plans)))
            self._plans[token] = (capability['revision'],message,decision)
        count = min(3000,max(2,math.ceil(final.duration/.05)))
        return dict(accepted=True,plan_id=token,duration=final.duration,
            torque_cap_nm=capability['gripper_torque_cap_nm'],joint_names=list(final.joint_names),
            keyframes=[dict(time=p.time,positions=p.positions,velocities=p.velocities,accelerations=p.accelerations)
                for p in final.points],
            preview=[dict(time=final.duration*i/count,positions=final.sample(final.duration*i/count)[0]) for i in range(count+1)],
            message='联合计划已生成；执行前仍须通过碰撞准备与后端准入')

    def take(self, token, current):
        """单次交接原计划；准备服务按预约状态规范化起点并检查实际执行曲线。"""
        capability = self.capability()
        with self._lock:
            plan = self._plans.pop(str(token),None)
        if plan is None:
            raise ValueError('coordinated preview missing; preview again')
        revision,message,decision = plan
        if revision != capability['revision']:
            raise ValueError('coordinated planning capability changed; preview again')
        return message,decision
