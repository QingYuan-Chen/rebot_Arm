"""手眼残差分析的命令行入口(控制台脚本 ``rebotarm_handeye_residual``).

从 JSON 文件读入多样本手眼数据集(含 ``end_to_camera`` 与 ``samples``),调用
:func:`handeye_residual.analyze_handeye_residual` 做残差与姿态多样性判定,把报告写到
``--output`` 指定路径并同时打印到标准输出.所有门限都可用命令行参数覆盖,默认值面向
"精细复标"场景.

输入输出都是 UTF-8 JSON;写文件时使用 ``indent=2`` 且 ``allow_nan=False``,一旦出现
NaN/Inf 会直接报错,而不是写出非法 JSON 让下游解析失败.参数名与报告键名(``--input``,
``--min-samples``,``position_residual`` 等)是对外接口,脚本,测试与证据文件都依赖,
不要改名.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .handeye_residual import analyze_handeye_residual


def main() -> None:
    """解析参数 → 读入数据集 → 分析 → 写文件并打印报告.

    ``--input`` 与 ``--output`` 必填.其余为门限参数,语义与
    ``analyze_handeye_residual`` 的同名关键字参数一一对应,其中"跨度"类是下限,
    "残差"类是上限:``--min-samples`` 最少样本数,``--min-translation-span-m`` 末端
    平移跨度下限(m),``--min-rotation-span-deg`` 末端姿态跨度下限(度),
    ``--max-position-rms-m`` 与 ``--max-position-residual-m`` 位置 RMS / 最大残差上限
    (m),``--max-rotation-rms-deg`` 与 ``--max-rotation-residual-deg`` 姿态 RMS / 最大
    残差上限(度).数据集非法或门限取值不合理时,由被调用函数抛 ``ValueError``.

    副作用:创建 ``--output`` 的父目录,并把报告写入该文件.返回值只是打印用的报告,
    调用方(如流水线脚本)应读文件而不是解析标准输出.
    """
    parser = argparse.ArgumentParser(description="Analyze multi-pose hand-eye residual samples")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    # 以下门限与 analyze_handeye_residual 的关键字参数一一对应:跨度类为下限,残差类为
    # 上限;平移类单位为 m,姿态类单位为 deg
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--min-translation-span-m", type=float, default=0.05)
    parser.add_argument("--min-rotation-span-deg", type=float, default=20.0)
    parser.add_argument("--max-position-rms-m", type=float, default=0.005)
    parser.add_argument("--max-position-residual-m", type=float, default=0.010)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=1.5)
    parser.add_argument("--max-rotation-residual-deg", type=float, default=3.0)
    args = parser.parse_args()
    payload = json.loads(args.input.read_text(encoding="utf-8"))
    report = analyze_handeye_residual(
        payload,
        min_samples=args.min_samples,
        min_end_translation_span_m=args.min_translation_span_m,
        min_end_rotation_span_deg=args.min_rotation_span_deg,
        max_position_rms_m=args.max_position_rms_m,
        max_position_residual_m=args.max_position_residual_m,
        max_rotation_rms_deg=args.max_rotation_rms_deg,
        max_rotation_residual_deg=args.max_rotation_residual_deg,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
