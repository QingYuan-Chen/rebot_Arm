# 离线物理环境（MuJoCo）健康自检工具。
#
# 职责与位置：
#     本模块属于仿真包的“无头物理检查”能力，不依赖任何真实电机 SDK，也不打开硬件通道。
#     它被离线/无头环境自检（命令行入口 mujoco_health）以及启动前的环境校验流程使用，
#     用于回答一个问题：当前环境能否加载项目自带的物理模型、推进物理步进，并做离屏渲染。
#
# 对外接口：
#     - probe_renderer：在子进程中做一次离屏渲染探测；
#     - collect_health：汇总一次完整的健康快照（可注入仿真后端与渲染检查器以便测试）；
#     - ModelHealthReport / check_model_health：基础物理健康报告；
#     - 命令行：--model、--skip-renderer、--renderer-timeout，
#       结果以 JSON 打到 stdout，退出码 0 表示健康、1 表示不健康（供 shell/CI 判定）。
#
# 安全与可靠性约束：
#     1. 渲染探测必须放在子进程里执行。GL 驱动在无显示环境下可能直接 abort 整个进程，
#        子进程隔离后主进程仍能拿到结构化失败信息，而不是被一起杀死。
#     2. 所有数值（关节位置/速度、执行器力、仿真时间）都必须是有限值；出现 NaN/Inf 即判定不健康。
#
# 基础有限性检查和增强版渲染/模型数量检查统一由本模块提供。
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable

from rebotarm_simulation.core.resource_paths import package_resource
from rebotarm_simulation.diagnostics.physics_probe import PhysicsProbe, positive_steps


def _default_robot_path() -> Path:
    """返回当前自动生成的 canonical 机器人 MJCF。"""
    return package_resource("rebotarm_simulation", "models/rebotarm/robot.xml")


# 当前 MuJoCo 运行时场景的健康门槛：6 个机械臂关节和 2 个夹爪关节；
# 执行器数量由传入的 canonical 模型实际报告。
EXPECTED_JOINT_COUNT = 8
EXPECTED_ACTUATOR_COUNT = 8
# 子进程渲染探测的默认超时（秒）。无显示驱动缺失时可能长时间阻塞，故必须有上限。
DEFAULT_RENDERER_TIMEOUT = 30.0


@dataclass(frozen=True)
class ModelHealthReport:
    """基础模型健康报告，已与增强版健康检查统一维护。"""

    model_path: str
    model_loaded: bool
    physics_step_finite: bool
    joint_count: int
    actuator_count: int
    simulation_time: float
    ok: bool


def check_model_health(
    model_path: Path | None = None,
    *,
    steps: int = 1,
) -> ModelHealthReport:
    """加载模型并推进物理步，返回基础有限性健康报告。"""
    steps = positive_steps(steps)
    probe = PhysicsProbe.load(_default_robot_path() if model_path is None else model_path)
    finite = probe.advance(steps)
    path, model, data = probe.path, probe.model, probe.data
    return ModelHealthReport(
        model_path=str(path),
        model_loaded=True,
        physics_step_finite=finite,
        joint_count=int(model.njnt),
        actuator_count=int(model.nu),
        simulation_time=float(data.time),
        ok=finite,
    )


def _positive_finite(value: str) -> float:
    """命令行参数类型：只接受正的有限浮点数（用于超时秒数）。

    显式拒绝 NaN/Inf/0/负数，避免把无限等待或非物理的超时值传进子进程调用。
    """
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def _version() -> str:
    """返回当前环境中的 MuJoCo 版本号；缺失时抛出 RuntimeError 提示安装依赖。"""
    try:
        import mujoco
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            "MuJoCo is required; install requirements-mujoco.txt in the active environment"
        ) from exc
    return str(mujoco.__version__)


# 子进程探测脚本：在独立进程中加载模型并做一次离屏渲染，把结果以紧凑 JSON 写到 stdout。
# 之所以整段脚本作为字符串常量下发，是为了让子进程只依赖标准库 + MuJoCo/numpy，不引入本项目其它模块。
#
# 脚本内部要点（注意：下面这些说明不能写进字符串里，否则会改变该常量的内容）：
#   * 渲染尺寸固定 64x64——这是"能证明渲染链路可用"的最小尺寸，避免在无显示环境下浪费显存与时间；
#   * 优先使用名为 overview 的相机；模型里没有该相机时（ValueError）退回自由视角，
#     因为这里只验证渲染链路本身，不校验取景；
#   * 对输出逐项校验形状与非有限值，防止驱动"假成功"（返回空图或 NaN 图像）；
#   * backend 字段回填当前生效的渲染后端环境变量，便于定位 GL 选择问题。
_RENDERER_PROBE = """\
import json
import mujoco
import numpy as np
import os
import sys

model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=64, width=64)
try:
    try:
        renderer.update_scene(data, camera='overview')
    except ValueError:
        renderer.update_scene(data)
    rgb = renderer.render()
    if tuple(rgb.shape) != (64, 64, 3):
        raise RuntimeError(f'unexpected RGB shape: {rgb.shape}')
    if not bool(np.isfinite(rgb).all()):
        raise RuntimeError('RGB output contains non-finite values')
finally:
    renderer.close()
print(json.dumps({
    'ok': True,
    'shape': [64, 64, 3],
    'backend': os.environ.get('MUJOCO_GL', 'default'),
}, separators=(',', ':')))
"""


def _text(value) -> str:
    """把子进程输出统一成去首尾空白的字符串。

    ``None``（未捕获输出）返回空串；``bytes``（未以文本模式捕获）按 UTF-8 容错解码，
    避免解码失败直接打断健康检查；其余类型直接 ``str()`` 转换。
    """
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode(errors="replace").strip()
    return str(value).strip()


def probe_renderer(
    model_path,
    *,
    backend: str | None = None,
    timeout: float = DEFAULT_RENDERER_TIMEOUT,
    runner=subprocess.run,
    command=None,
) -> dict[str, object]:
    """在子进程中探测离屏渲染能力，使原生驱动崩溃不会带走本进程。

    参数：
        model_path：待加载的 MJCF 模型/场景路径（透传给子进程脚本的 ``argv[1]``）。
        backend：若不为 ``None``，作为 ``MUJOCO_GL`` 注入子进程环境，用于强制选择渲染后端。
        timeout：子进程超时秒数，超时返回 ``timed_out=True`` 而不是抛异常。
        runner：子进程执行器，默认 ``subprocess.run``，测试可注入伪造实现。
        command：完整的探测命令前缀（默认用当前解释器执行内置探测脚本），测试可覆盖。

    返回：字典字段含义——
        ``available``：子进程退出码为 0 且 stdout 是合法的成功 JSON 才为 ``True``；
        ``timed_out``：是否因超时被终止；
        ``returncode`` / ``signal``：退出码；被信号杀死时 Python 给出负返回值，
        此处把 ``signal`` 还原为正的信号编号，``returncode`` 保留原值；
        ``stdout`` / ``stderr``：子进程文本输出（已 strip）；
        ``error``：失败原因（超时、信号、非零退出、成功但输出不合法时为对应文案，成功时为 ``None``）；
        ``details``：解析后的探测 JSON（含 ``shape`` 与 ``backend``），失败时为 ``None``。
    """
    probe_command = list(command or (sys.executable, "-c", _RENDERER_PROBE))
    probe_command.append(str(model_path))
    child_environment = os.environ.copy()
    if backend is not None:
        child_environment["MUJOCO_GL"] = backend
    try:
        completed = runner(
            probe_command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
            env=child_environment,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "available": False,
            "timed_out": True,
            "returncode": None,
            "signal": None,
            "stdout": _text(exc.output),
            "stderr": _text(exc.stderr),
            "error": f"renderer probe timed out after {timeout:g}s",
        }
    returncode = int(completed.returncode)
    # Python 对“被信号杀死”的子进程返回负的 returncode；取反还原成信号编号便于日志定位
    signal = -returncode if returncode < 0 else None
    details = None
    if returncode == 0:
        # 退出码为 0 还不够：驱动可能“假成功”，必须校验 JSON 结构、图像形状与后端字段
        try:
            details = json.loads(_text(completed.stdout))
            valid = (
                isinstance(details, dict)
                and details.get("ok") is True
                and details.get("shape") == [64, 64, 3]
                and isinstance(details.get("backend"), str)
            )
        except (json.JSONDecodeError, TypeError):
            valid = False
        error = None if valid else "renderer probe returned invalid success output"
    elif signal is not None:
        error = f"renderer probe terminated by signal {signal}"
    else:
        error = f"renderer probe failed with return code {returncode}"
    return {
        "available": returncode == 0 and error is None,
        "timed_out": False,
        "returncode": returncode,
        "signal": signal,
        "stdout": _text(completed.stdout),
        "stderr": _text(completed.stderr),
        "error": error,
        "details": details,
    }


def collect_health(
    model_path=None,
    *,
    renderer_check: Callable[[object], tuple[bool, str | None]] | None = None,
    renderer_timeout: float = DEFAULT_RENDERER_TIMEOUT,
    sim_factory=None,
    mujoco_version: str | None = None,
) -> dict[str, object]:
    """采集一次完整的环境健康快照。

    流程：构造仿真后端 → 读取推进前状态 → 推进一步物理步进 → 校验数值有限性、
    仿真时间是否前进、关节/执行器数量是否为期望值 → 运行渲染探测 → 汇总为字典。
    无论成功与否都会在 ``finally`` 中关闭仿真后端，避免泄漏模型句柄。

    参数：
        model_path：模型/场景路径；``None`` 表示使用仿真后端自带的默认场景。
        renderer_check：可注入的渲染检查器。返回字典时直接作为探测结果使用；
            返回 ``(available, error)`` 元组时会包装成与 ``probe_renderer`` 同构的字典。
            为 ``None`` 时走真实的子进程渲染探测。
        renderer_timeout：渲染探测超时秒数。
        sim_factory：仿真后端工厂，默认延迟导入同包默认物理仿真实现（避免顶层硬依赖）。
        mujoco_version：覆盖上报的 MuJoCo 版本号；``None`` 时尝试从仿真后端读取。

    返回的字典既包含原始计数，也包含期望计数，便于比较；``renderer_available`` /
    ``renderer_error`` 是给调用方直接判断用的摘要字段，``renderer_probe`` 保留完整明细。
    """
    if sim_factory is None:
        # 延迟导入：健康检查在没有物理仿真依赖的纯离线环境下也应能导入本模块
        from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

        sim_factory = RebotArmMujoco
    sim = sim_factory(model_path)
    try:
        before = sim.get_state()
        after = sim.step()
        # 把状态里所有需要“有限性”判定的数值展平成一个序列，统一做 isfinite 检查
        numeric_state = (
            *after.joint_positions,
            *after.joint_velocities,
            *after.actuator_forces,
            after.simulation_time,
        )
        joints = len(after.joint_names)
        actuators = len(after.actuator_forces)
        if renderer_check is None:
            renderer_probe = probe_renderer(sim.model_path, timeout=renderer_timeout)
        else:
            checked = renderer_check(sim)
            if isinstance(checked, dict):
                renderer_probe = checked
            else:
                renderer_available, renderer_error = checked
                renderer_probe = {
                    "available": bool(renderer_available),
                    "timed_out": False,
                    "returncode": None,
                    "signal": None,
                    "stdout": "",
                    "stderr": "",
                    "error": renderer_error,
                }
        result = {
            # 整体判定：数值全有限 + 物理时间确实前进 + 关节/执行器数量符合 canonical 模型
            "ok": bool(
                all(math.isfinite(float(value)) for value in numeric_state)
                and after.simulation_time > before.simulation_time
                and joints == EXPECTED_JOINT_COUNT
                and actuators == EXPECTED_ACTUATOR_COUNT
            ),
            "python_version": platform.python_version(),
            "mujoco_version": (
                mujoco_version
                if mujoco_version is not None
                # 未显式传入版本号时尝试从仿真后端的 MuJoCo 模块读取；
                # 注入的测试替身没有该属性，则回退为 "injected-simulation" 标记
                else str(getattr(getattr(sim, "_mj", None), "__version__", "injected-simulation"))
            ),
            "model_path": str(sim.model_path),
            "model_loaded": True,
            "physics_step_finite": all(math.isfinite(float(value)) for value in numeric_state),
            "simulation_time": float(after.simulation_time),
            "joint_count": joints,
            "expected_joint_count": EXPECTED_JOINT_COUNT,
            "actuator_count": actuators,
            "expected_actuator_count": EXPECTED_ACTUATOR_COUNT,
            "headless": True,
            "renderer_available": bool(renderer_probe["available"]),
            "renderer_error": renderer_probe["error"],
            "renderer_probe": renderer_probe,
        }
        return result
    finally:
        sim.close()


def build_parser() -> argparse.ArgumentParser:
    """构造健康检查命令行解析器。

    ``--model``：MJCF 场景路径，缺省时由仿真后端选择包内自带场景；
    ``--skip-renderer``：只做物理检查、跳过离屏渲染（无显示环境下的降级开关）；
    ``--renderer-timeout``：渲染子进程超时秒数（正有限浮点数，默认 30 秒）。
    三处 help 文案是对外 CLI 接口，保持英文原样。
    """
    parser = argparse.ArgumentParser(description="Check the reBotArm MuJoCo environment")
    parser.add_argument("--model", help="path to an MJCF scene (defaults to packaged scene.xml)")
    parser.add_argument("--skip-renderer", action="store_true", help="skip the offscreen renderer probe")
    parser.add_argument(
        "--renderer-timeout",
        type=_positive_finite,
        default=DEFAULT_RENDERER_TIMEOUT,
        help=f"renderer child-process timeout in seconds (default: {DEFAULT_RENDERER_TIMEOUT:g})",
    )
    return parser


def main(
    argv=None,
    *,
    sim_factory=None,
    renderer_check=None,
    stdout=None,
    stderr=None,
) -> int:
    """健康检查命令行入口。

    参数：
        argv：命令行参数（``None`` 表示取 ``sys.argv``），便于测试注入。
        sim_factory / renderer_check：透传给 ``collect_health`` 的依赖注入点。
        stdout / stderr：输出流，缺省为标准流，便于测试捕获。

    返回：退出码——0 表示健康；1 表示不健康或采集过程抛出异常。
    采集异常时仍会向 stdout 输出一份 ``ok=False`` 的 JSON（含异常类型与消息），
    保证调用方始终能拿到结构化结论。
    """
    stdout = sys.stdout if stdout is None else stdout
    stderr = sys.stderr if stderr is None else stderr
    args = build_parser().parse_args(argv)
    # --skip-renderer 用一个恒返回 (False, 原因) 的替身顶替渲染检查，
    # 使渲染能力标记为不可用但整体流程仍继续（无显示环境下属于预期结果）
    check = (
        (lambda _sim: (False, "renderer check skipped"))
        if args.skip_renderer
        else renderer_check
    )
    try:
        result = collect_health(
            args.model,
            renderer_check=check,
            renderer_timeout=args.renderer_timeout,
            sim_factory=sim_factory,
        )
    except Exception as exc:
        result = {
            "ok": False,
            "python_version": platform.python_version(),
            "model_path": args.model,
            "model_loaded": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(result, ensure_ascii=False), file=stdout)
        print(f"MuJoCo health check failed: {exc}", file=stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), file=stdout)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
