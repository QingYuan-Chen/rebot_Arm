#!/usr/bin/env python3
"""只读回放旧缓存，比较量化导数诊断及静止窗口；不导入 ROS/SDK、不访问设备。

旧会话未记录守卫实际采样和每次发送目标，本工具不将缓存重采样冒充原守卫。
输出原文件 SHA256、去重规则、策略和已知位置偏移，保留历史验收结论。
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/rebotarmcontroller'), str(ROOT/'src/rebotarm_motion')]
from rebotarmcontroller.stationary_evidence import StationaryEvidence, POSITION_SPAN
from rebotarm_motion.trajectory_runtime_limits import JointRuntimeLimit, TrajectoryRuntimeLimitGuard


def replay(session):
    path = Path(session)
    baseline = json.loads((path/'baseline.json').read_text())
    anchor = [baseline[f'joint{k}'] for k in range(1, 7)]
    evidence = StationaryEvidence()
    evidence.reset(anchor)
    limits = {f'joint{k}': JointRuntimeLimit(max_velocity=1., max_jerk=20.) for k in range(1, 7)}
    guard = TrajectoryRuntimeLimitGuard(limits, derivative_policy='diagnostic')
    last_identity = None
    last_elapsed = None
    reasons, derivatives = Counter(), Counter()
    snapshots = distinct = confirmed = 0
    maxima = [0.]*6
    speed_events = []
    for line in (path/'cache.jsonl').open():
        row = json.loads(line)
        if row.get('event') != 'snapshot':
            continue
        snapshots += 1
        identity = tuple(row['sequences'])
        if identity == last_identity:
            continue
        last_identity = identity
        distinct += 1
        stamp = row['mono']-row['age']
        reason = evidence.observe(row['q'], row['v'], stamp, row['mono'], sequences=identity)
        reasons[reason] += 1
        if reason not in ('duplicate', 'invalid_or_stale', 'source_regressed'):
            if last_elapsed is not None and stamp <= last_elapsed:
                guard.reset()
            guard.observe(joint_names=list(limits), velocities=row['v'],
                          efforts=row['torques'], elapsed=stamp)
            last_elapsed = stamp
            derivatives.update(value.kind for value in guard.derivative_diagnostics)
        confirmed += bool(evidence.confirmed())
        for j in range(6):
            maxima[j] = max(maxima[j], abs(row['q'][j]-anchor[j]))
            if abs(row['v'][j]) > .07:
                speed_events.append(dict(joint=j+1, sequence=identity[j], observed_at=stamp,
                                         velocity=row['v'][j], position=row['q'][j]))
    files = {name: hashlib.sha256((path/name).read_bytes()).hexdigest()
             for name in ('baseline.json', 'cache.jsonl', 'received.jsonl')}
    return dict(session=path.name, source_sha256=files, snapshots=snapshots,
                distinct_sequence_tuples=distinct, evidence_reasons=dict(reasons),
                confirmed_observations=confirmed, max_anchor_displacement_rad=maxima,
                baseline_span_violating_joints=[j+1 for j, value in enumerate(maxima) if value > POSITION_SPAN],
                velocity_events=speed_events, derivative_diagnostics=dict(derivatives),
                limitation='缓存观测回放，不是原守卫采样重建；无逐周期指令/原始字节，不重新评定硬件验收')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sessions', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = dict(schema_version=1, policy=dict(arm_window_sec=.2, motion_window_sec=.3,
                 field_window_sec=1., position_span_rad=POSITION_SPAN, max_age_sec=.15,
                 velocity_limits_rad_s=dict(arm=.05, motion=.03, field=.07)),
                 sessions=[replay(path) for path in args.sessions])
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    for item in report['sessions']:
        print(item['session'], item['distinct_sequence_tuples'], item['evidence_reasons'],
              'max anchor displacement', item['max_anchor_displacement_rad'])


if __name__ == '__main__':
    main()
