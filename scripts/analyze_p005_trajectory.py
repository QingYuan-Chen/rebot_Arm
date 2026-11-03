#!/usr/bin/env python3
"""P005 纯离线 JSON 分析入口; 不创建 ROS 客户端, 不具备运动发送或串口能力."""

import argparse
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rebotarm_motion'))
from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
from rebotarm_motion.trajectory_preparation import (
    PreparationContext, analyze_trajectory, load_reference_limits,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='包含 joint_names、points 及可选 context 的 JSON')
    parser.add_argument('--output', type=Path, required=True, help='新报告文件, 不覆盖既有结果')
    parser.add_argument('--now', type=float, default=0., help='与 context.observed_at 同源的离线时间')
    args = parser.parse_args()
    started = time.monotonic()
    def reject_constant(value):
        raise ValueError(f'nonfinite JSON constant: {value}')
    try:
        source = json.loads(args.input.read_text(), parse_constant=reject_constant)
        points = tuple(TrajectoryPoint(p['time_from_start'], p['positions'],
            p.get('velocities', ()), p.get('accelerations', ())) for p in source['points'])
        context = PreparationContext(**source['context']) if 'context' in source else None
        lower, upper, limits = load_reference_limits(ROOT / 'src/rebotarm_moveit_config/config')
        _, report = analyze_trajectory(source['joint_names'], points, lower=lower, upper=upper,
            limits=limits, context=context, now=args.now)
        report['input'] = source
    except (ValueError, KeyError, TypeError, OverflowError) as exc:
        report = dict(state='rejected', reason=str(exc), scope='offline_reference_only')
    report['preparation_seconds'] = time.monotonic() - started
    report['collision_note'] = '此工具不连接碰撞服务; 未检查不能作为可执行结论'
    with args.output.open('x') as target:
        json.dump(report, target, ensure_ascii=False, indent=2, allow_nan=False)
        target.write('\n')
    print(json.dumps(dict(state=report['state'], output=str(args.output)), ensure_ascii=False))
    return 2 if report['state'] == 'rejected' else 0


if __name__ == '__main__':
    raise SystemExit(main())
