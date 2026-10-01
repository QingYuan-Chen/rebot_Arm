"""无头物理健康检查：确认仿真模型能加载、能步进、且数值不发散。

职责与位置
    本模块是仿真包的底层健康检查实现，不依赖 ROS 运行时、不开窗口、不接触任何真实
    电机 SDK 或硬件通道。它只回答三个问题：模型能否加载？物理能否推进？推进后状态
    是否仍是有限值？同包的主健康检查命令行入口在其之上补齐模型档位与离屏渲染探测。

状态判定
    ``ok`` 取"步进后全部状态有限"：``qpos``（广义坐标）、``qvel``（广义速度）、
    ``actuator_force``（执行器力）与 ``time`` 中任何一项出现 NaN/Inf，都说明模型参数、
    接触或积分器已经发散，该模型不能用于离线仿真结论。

模型资源约定
    包自带源 XML 里的网格名是相对的，直接加载会找不到 STL；因此当传入的就是该默认
    模型时，先走同包模型档位构建流程把资源路径补成绝对路径，再交给物理引擎。其它
    路径按原样加载。

对外接口
    ``check_model_health``  返回结构化报告；
    命令行 ``--model``（默认包自带带夹爪模型）与 ``--steps``（默认 1 步），
    报告以 JSON 打印到 stdout，退出码 0 表示健康、1 表示不健康，便于 shell/CI 判定。
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from .mujoco_model_profile import DEFAULT_GRIPPER_XML, build_physics_profile_tree


@dataclass(frozen=True)
class ModelHealthReport:
    """一次健康检查的结构化结果，会被直接序列化为对外证据。"""

    model_path: str  # 实际加载的模型绝对路径
    model_loaded: bool  # 模型是否构造成功（能走到上报即为 True）
    physics_step_finite: bool  # 步进后状态量是否全部有限（无 NaN/Inf）
    joint_count: int  # 模型关节数：6 个臂旋转关节 + 2 个手指滑移关节
    actuator_count: int  # 模型执行器数：6 个臂电机 + 1 个夹爪执行器
    simulation_time: float  # 步进结束时的仿真时间（s），等于 步数 × 模型 timestep
    ok: bool  # 总体判定，目前等价于 physics_step_finite


def _load_mujoco() -> tuple[Any, Any]:
    """延迟导入物理引擎与数值库，缺依赖时转成带安装指引的错误。

    返回 ``(mujoco, numpy)``。刻意不在模块顶层导入：这样没有安装 MuJoCo 的环境
    仍然可以导入本模块（例如只校验命令行参数或做纯逻辑测试）。
    """
    try:
        import mujoco
        import numpy as np
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "MuJoCo health check requires the pinned MuJoCo environment; "
            "install docs/setup/dependencies/mujoco.md first"
        ) from exc
    return mujoco, np


def check_model_health(model_path: Path = DEFAULT_GRIPPER_XML, *, steps: int = 1) -> ModelHealthReport:
    """加载模型并推进 ``steps`` 个物理步，返回健康报告。

    参数：
        model_path: 模型 XML 路径；默认是包自带的带夹爪机器人模型。
        steps: 物理步数，必须是正整数。

    返回：
        ``ModelHealthReport``。``ok`` 为 False 表示物理状态已发散（出现非有限值）。

    异常：
        ``ValueError``  步数不是正整数时抛出；
        ``RuntimeError`` 缺少物理引擎依赖时抛出。
    """
    # 显式拒绝 bool：True/False 也是 int 的子类，但作为步数几乎一定是调用方笔误。
    if isinstance(steps, bool) or int(steps) < 1:
        raise ValueError("steps must be a positive integer")

    mujoco, np = _load_mujoco()
    path = Path(model_path).expanduser().resolve()
    if path == DEFAULT_GRIPPER_XML.resolve():
        # 包自带源 XML 使用相对网格名；加载前先按当前命令行使用的方式补齐为
        # 绝对资源路径的模型档位，否则物理引擎找不到 STL 网格。
        tree = build_physics_profile_tree(path)
        model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding="unicode"))
    else:
        model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    for _ in range(int(steps)):
        mujoco.mj_step(model, data)
    # 四个状态量全部有限才算健康：任一项发散（NaN/Inf）都说明该模型不可用于离线结论。
    finite = bool(
        np.isfinite(data.qpos).all()
        and np.isfinite(data.qvel).all()
        and np.isfinite(data.actuator_force).all()
        and np.isfinite(data.time)
    )
    return ModelHealthReport(
        model_path=str(path),
        model_loaded=True,
        physics_step_finite=finite,
        joint_count=int(model.njnt),
        actuator_count=int(model.nu),
        simulation_time=float(data.time),
        ok=finite,
    )


def main(argv: list[str] | None = None) -> int:
    """命令行入口：跑一次健康检查并把报告以 JSON 打到 stdout。

    返回 0 表示健康、1 表示不健康，调用方（shell/CI）据此判定。
    """
    parser = argparse.ArgumentParser(description="Run a headless MuJoCo model health check")
    parser.add_argument("--model", type=Path, default=DEFAULT_GRIPPER_XML)
    parser.add_argument("--steps", type=int, default=1)
    args = parser.parse_args(argv)
    report = check_model_health(args.model, steps=args.steps)
    print(json.dumps(asdict(report), ensure_ascii=False))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
