"""规划快照及不可变执行计划的纯数值协议。

文件职责：绑定后端会话、六轴代次、已发送保持目标和准确纳秒轨迹。
对外接口：planning_state、plan_binding、decode_plan、require_plan_context。
安全约束：不读取 ROS、设备或时钟；绑定不是执行许可，后端仍需复核反馈和碰撞。
    实际位置残差不参与绑定判定；参考轨迹、失能预览及旧能力绑定不能冒充执行计划。
文件结构：快照校验 → 轨迹摘要 → 版本化编码/解码 → 执行上下文比对。
"""

import hashlib
import json

from .polynomial import canonical_names, ordered_points, vector6

PREFIX = 'rebotarm-plan:v1:'


def planning_state(capability_revision, session, generation, hold, *, executable, reason=''):
    """复制设备线程或仿真锁内取得的快照；失能可预览，其他不可用状态须明确失败。"""
    if (not isinstance(capability_revision, str) or len(capability_revision) != 64
            or any(c not in '0123456789abcdef' for c in capability_revision)
            or type(session) is not int or session < 0
            or type(generation) is not int or generation < 0 or type(executable) is not bool):
        raise ValueError('invalid planning state identity')
    positions = list(vector6(hold)) if hold is not None and len(hold) else []
    if executable and not positions:
        raise ValueError('acknowledged planning hold required')
    if not executable and reason != 'disabled':
        raise ValueError('only explicitly disabled state permits preview-only planning')
    return dict(version=1, capability_revision=capability_revision, session=session,
        generation=generation, hold=positions, executable=executable, reason=str(reason))


def decode_state(value):
    """拒绝未知或不完整快照，避免把服务失败当作失能预览。"""
    data = json.loads(value) if isinstance(value, str) else value
    try:
        expected = planning_state(data['capability_revision'], data['session'], data['generation'],
            data['hold'], executable=data['executable'], reason=data['reason'])
    except (KeyError, TypeError) as exc:
        raise ValueError('invalid planning state schema') from exc
    if data != expected:
        raise ValueError('planning state schema mismatch')
    return expected


def trajectory_digest(names, points):
    """摘要以规范关节顺序和整数纳秒编码，保留所有 q/v/a 约束及输入等级。"""
    values = ordered_points(names, points)
    data = [canonical_names(names), [[round(p.time*1e9), p.positions, p.velocities, p.accelerations]
                                    for p in values]]
    return hashlib.sha256(json.dumps(data, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def plan_binding(capability_revision, names, points, *, state=None, purpose='execution'):
    """执行计划必须有保持快照；参考段和失能预览显式标记且不能下发六轴执行。"""
    if purpose not in ('execution', 'reference', 'preview', 'gripper'):
        raise ValueError('unknown planning purpose')
    if (not isinstance(capability_revision,str) or len(capability_revision)!=64
            or any(c not in '0123456789abcdef' for c in capability_revision)):
        raise ValueError('sha256 planning capability revision required')
    state = decode_state(state) if state is not None else None
    if purpose == 'execution' and (state is None or not state['executable']):
        raise ValueError('executable planning state required')
    if state is not None and state['capability_revision'] != capability_revision:
        raise ValueError('planning capability changed')
    data = dict(version=1, purpose=purpose, capability_revision=capability_revision,
        state=state, curve_digest=trajectory_digest(names, points))
    return PREFIX+json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False)


def decode_plan(header, names, points, *, capability_revision=None, require_execution=False):
    """空或普通 frame_id 沿用旧 FJT；已声明的旧/未知协议不得静默降级。"""
    if not header.startswith(PREFIX):
        if header.startswith('rebotarm-'):
            raise ValueError('old or unknown planning binding; update client and re-plan')
        return None
    try:
        data = json.loads(header[len(PREFIX):])
        expected = plan_binding(data['capability_revision'], names, points,
            state=data['state'], purpose=data['purpose'])
    except (KeyError, TypeError, json.JSONDecodeError) as exc:
        raise ValueError('invalid planning binding') from exc
    if expected != header:
        raise ValueError('planned trajectory changed; re-plan')
    if capability_revision is not None and data['capability_revision'] != capability_revision:
        raise ValueError('planning capability changed; re-plan')
    if require_execution and data['purpose'] != 'execution':
        raise ValueError('reference or disabled preview is not executable; enable and re-plan')
    return data


def require_plan_context(plan, *, capability_revision, session, generation, hold):
    """比较任务身份和命令保持值；不比较实际位置，不设置位置偏差门限。"""
    if plan is None:
        return
    state = plan['state']
    if plan['purpose'] != 'execution' or state is None:
        raise ValueError('planning start context is preview/reference; re-plan')
    changes = [key for key,expected,current in (
        ('capability',plan['capability_revision'],capability_revision),
        ('session',state['session'],session), ('generation',state['generation'],generation),
        ('hold',tuple(state['hold']),tuple(hold))) if expected != current]
    if changes:
        raise ValueError('planning start context changed: '+', '.join(changes)+'; re-plan')
