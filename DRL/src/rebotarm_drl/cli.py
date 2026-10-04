"""Explicit simulation workflows and generic deployment tools."""
import argparse
import json
from pathlib import Path

from .runtime import ROOT, configure_runtime


def read_json(path):
    return json.loads(Path(path).read_text())


def main(argv=None):
    parser = argparse.ArgumentParser(description="MJLab Reach-and-Hold、模型导出、评测与回放")
    subs = parser.add_subparsers(dest="command", required=True)
    p = subs.add_parser("doctor", help="检查依赖、模型与可选 GPU 算术")
    p.add_argument("--gpu", action="store_true")
    p = subs.add_parser("export", help="导出显式提供的策略与接口契约")
    p.add_argument("--format", choices=["torchscript", "rsl"], required=True)
    p.add_argument("--source", required=True)
    p.add_argument("--agent-config")
    p.add_argument("--contract", required=True)
    p.add_argument("--output", required=True)
    p = subs.add_parser("verify-export")
    p.add_argument("--output", required=True)
    p.add_argument("--contract", required=True)
    p = subs.add_parser("compare", help="比较相同协议下已有的成对评测报告")
    p.add_argument("--candidate", required=True)
    p.add_argument("--baseline", required=True)
    p = subs.add_parser("replay", help="运动学回放已有 NPZ 记录")
    p.add_argument("--model", required=True)
    p.add_argument("--recording", required=True)
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--headless", action="store_true")
    p = subs.add_parser("bundle")
    p.add_argument("--output", required=True)
    p.add_argument("--with-wheels", action="store_true")
    p = subs.add_parser("verify-bundle")
    p.add_argument("archive")
    p = subs.add_parser("train", help="仅远端：训练 Reach-and-Hold")
    p.add_argument("--confirm-remote-training", action="store_true")
    p.add_argument("--num-envs", type=int, default=256)
    p.add_argument("--iterations", type=int, default=2000)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--resume")
    p = subs.add_parser("reach-smoke", help="任务物理步进与策略前向检查，不训练")
    p.add_argument("--steps", type=int, default=25)
    p.add_argument("--num-envs", type=int, default=2)
    p.add_argument("--device", default="cuda:0")
    p = subs.add_parser("reach-eval", help="固定16组未见目标；默认评测IK基线")
    p.add_argument("--policy", help="含manifest.json及policy.onnx的导出目录；省略则评测IK基线")
    p.add_argument("--output", required=True)
    p.add_argument("--device", default="cuda:0")
    p = subs.add_parser("reach-play", help="加载远端检查点做交互式仿真回放")
    p.add_argument("--checkpoint", required=True)
    p.add_argument("--viewer", choices=["native", "viser"], default="native")
    p.add_argument("--device", default="cuda:0")
    args = parser.parse_args(argv)
    configure_runtime()
    if args.command == "doctor":
        from .doctor import doctor
        report = doctor(gpu=args.gpu)
    elif args.command == "export":
        from .export import export_torchscript, export_rsl_checkpoint
        if args.format == "rsl":
            if not args.agent_config:
                parser.error("RSL 导出必须提供 --agent-config")
            report = export_rsl_checkpoint(args.source, args.agent_config, read_json(args.contract), args.output)
        else:
            report = export_torchscript(args.source, read_json(args.contract), args.output)
    elif args.command == "verify-export":
        from .export import verify_export
        report = verify_export(args.output, read_json(args.contract))
    elif args.command == "compare":
        from .analysis import compare_reports
        report = compare_reports(read_json(args.candidate), read_json(args.baseline))
    elif args.command == "replay":
        from .replay import replay
        report = replay(args.model, args.recording, speed=args.speed, headless=args.headless)
    elif args.command in {"train", "reach-smoke", "reach-play"}:
        from .reach_workflow import train, smoke, play
        if args.command == "train":
            report = train(confirmed=args.confirm_remote_training, num_envs=args.num_envs,
                iterations=args.iterations, seed=args.seed, device=args.device, resume=args.resume)
        elif args.command == "reach-smoke":
            report = smoke(steps=args.steps, num_envs=args.num_envs, device=args.device)
        else:
            report = play(args.checkpoint, viewer=args.viewer, device=args.device)
    elif args.command == "reach-eval":
        from .reach_eval import evaluate
        report = evaluate(args.output, policy=args.policy, device=args.device)
    else:
        from .bundle import build_bundle, verify_bundle
        report = (build_bundle(ROOT, args.output, with_wheels=args.with_wheels)
                  if args.command == "bundle" else verify_bundle(args.archive))
        report = {"verified_files": len(report["files"]), "with_wheels": report["with_wheels"]}
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
