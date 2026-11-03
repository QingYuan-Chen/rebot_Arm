#!/usr/bin/env python3
"""P005 轨迹 JSON 的纯离线分析入口。

文件职责
    从输入点和可选保持上下文重建参考曲线，报告限值、连续性和旧阶梯对照。
对外接口
    main 处理 input、必需的 --output 及 --now；输出 JSON 报告和简短终端结果。
    被拒绝时退出 2，其余分析结果退出 0；unknown 的零退出码不表示可执行。
安全约束
    不创建 ROS 客户端、不发送运动、不访问串口，也不提供碰撞检查器。
    --now 必须与输入上下文时钟同源；报告以独占创建模式写入，不覆盖既有文件。
文件结构
    源码数值包定位 → 输入解析 → 准备分析 → 不覆盖的报告输出 → CLI 入口。
"""

import argparse
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rebotarm_motion'))
sys.path.insert(0, str(ROOT / 'src/rebotarm_trajectory'))
from rebotarm_motion.polynomial_trajectory import TrajectoryPoint
from rebotarm_motion.trajectory_preparation import (
    PreparationContext, analyze_trajectory, load_reference_limits,
)


def main():
    """完成一次离线报告；本机计时仅统计耗时，不能替代输入反馈所使用的时间轴。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='包含 joint_names、points 及可选 context 的 JSON')
    parser.add_argument('--output', type=Path, required=True, help='新报告文件, 不覆盖既有结果')
    parser.add_argument('--now', type=float, default=0., help='与 context.observed_at 同源的离线时间')
    args = parser.parse_args()
    started = time.monotonic()
    def reject_constant(value):
        """拒绝 JSON 扩展中的 NaN/Infinity，避免在后续数值报告中传播非有限值。"""
        raise ValueError(f'nonfinite JSON constant: {value}')
    try:
        source = json.loads(args.input.read_text(), parse_constant=reject_constant)
        points = tuple(TrajectoryPoint(p['time_from_start'], p['positions'],
            p.get('velocities', ()), p.get('accelerations', ())) for p in source['points'])
        context = PreparationContext(**source['context']) if 'context' in source else None
        lower, upper, limits = load_reference_limits(ROOT / 'src/rebotarm_moveit_config/config')
        # 此工具没有场景快照，主动省略 collision_check；数值合格也不能颁发执行候选。
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
