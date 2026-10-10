"""离线 MuJoCo 物理体检套件：不依赖 ROS，直接加载 MJCF 模型跑正动力学。

职责与位置
    本模块属于仿真层的底层"体检"工具：给定一份 MJCF 模型文件，加载后推进若干物理
    步，把模型规模、数值有限性、接触数与步进结果整理成不可变的结果对象。上层
    （健康检查、模型配置校验、成对轨迹对照脚本、pytest）只读这些结果做判定，
    本模块自身不做阈值决策、不下发任何真实硬件指令。

四类检查
    1. :func:`run_smoke`           冒烟：只看模型能否加载并稳定步进（数值不发散）；
    2. :func:`run_step_response`   单关节阶跃响应：正式位置控制器 + 力矩执行器下的跟踪质量；
    3. :func:`run_step_response_suite` 把第 2 项按关节批量执行并汇总最坏值；
    4. :func:`run_grasp_benchmark` 抓取场景：接触与抬升判定（委托抓取质量模块）。

依赖与约束
    物理引擎与数值库在共享 PhysicsProbe 内延迟导入，因此缺少该可选依赖时本模块
    仍可被导入（调用方需自行跳过）；抓取质量判定复用同目录的质量模块；结果类型全部
    ``frozen=True``，保证证据一旦生成就不会被后续代码改写。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time

from rebotarm_simulation.diagnostics.mujoco_grasp_quality import evaluate_grasp_quality
from rebotarm_simulation.diagnostics.physics_probe import PhysicsProbe


@dataclass(frozen=True)
class SmokeResult:
    """一次冒烟检查的输出。

    字段：
    - ``xml_path``：被检查 MJCF 文件的绝对路径（已 resolve）；
    - ``nq``：广义坐标维数（关节位置自由度，含夹爪与被抓物体）；
    - ``nv``：广义速度维数（自由度，等于 nq 减去被约束的坐标）；
    - ``nu``：执行器个数；
    - ``finite``：步进结束后 qpos/qvel 是否全部为有限数（NaN/Inf 说明数值发散）；
    - ``contacts``：步进结束瞬间的接触点数量；
    - ``sim_time``：步进结束时的仿真时间，单位 s。
    """

    xml_path: Path
    nq: int
    nv: int
    nu: int
    finite: bool
    contacts: int
    sim_time: float


@dataclass(frozen=True)
class StepResponseResult:
    """单个关节阶跃响应的输出。

    字段：
    - ``joint``：被测试的关节名（如 joint2）；
    - ``target``：阶跃目标角，单位 rad；
    - ``final_position``：仿真结束时的实际关节角，单位 rad；
    - ``final_abs_error``：结束时刻的绝对跟踪误差，单位 rad；
    - ``max_abs_error``：整个过程中出现的最大绝对跟踪误差，单位 rad；
    - ``rms_error``：全过程跟踪误差的均方根，单位 rad（既能反映偏差大小，
      又不会像最大值那样被单个毛刺完全主导）；
    - ``max_abs_velocity``：过程中关节角速度绝对值的峰值，单位 rad/s；
    - ``max_abs_actuator_force``：过程中执行器广义力的绝对值峰值，单位 N·m，
      用于确认限力没有失效；
    - ``sim_time``：仿真结束时间，单位 s。
    """

    joint: str
    target: float
    final_position: float
    final_abs_error: float
    max_abs_error: float
    rms_error: float
    max_abs_velocity: float
    max_abs_actuator_force: float
    sim_time: float


@dataclass(frozen=True)
class StepResponseSuiteResult:
    """多关节阶跃响应套件的汇总输出。

    字段：
    - ``xml_path``：被测模型的绝对路径；
    - ``results``：各关节单独的响应结果，顺序与调用方给出的目标字典一致；
    - 其余 ``max_*`` 字段：对全部子结果取最大值，即"最坏关节"的指标，
      供上层只用一个数就能与限值比较；目标字典为空时取 0.0。
    """

    xml_path: Path
    results: list[StepResponseResult]
    max_final_abs_error: float
    max_abs_error: float
    max_rms_error: float
    max_abs_velocity: float
    max_abs_actuator_force: float


@dataclass(frozen=True)
class GraspBenchmarkResult:
    """One explicit simulated trial; contacts refer only to the named fingers/target.

    Success requires bilateral contact at the final sample and target lift.
    This is a trial metric, not a complete grasp planner or stability certificate.
    """
    xml_path: Path
    target_body: str
    finite: bool
    initial_object_height_m: float
    object_height_m: float
    max_contacts: int
    final_contacts: int
    contact_detected: bool
    lift_detected: bool
    grasp_success: bool
    grasp_status: str
    sim_time: float


def run_smoke(xml_path: Path, *, seconds: float = 3.0) -> SmokeResult:
    """对 MJCF 模型做冒烟步进：加载 -> 推进 -> 汇报模型规模与数值稳定性。

    不做任何控制输入，只让模型在自带初始状态下自由步进，因此适合快速回答
    "这份模型文件是否可用、有没有明显数值发散"。

    参数：
    - ``xml_path``：MJCF 文件路径，内部会 resolve 成绝对路径；
    - ``seconds``：仿真时长，单位 s，默认 3.0。

    返回 :class:`SmokeResult`。若模型文件缺失或解析失败，底层引擎会抛出异常，
    本函数不吞异常、也不做降级处理。
    """
    seconds = _positive_seconds(seconds)
    probe = PhysicsProbe.load(xml_path)
    model, data = probe.model, probe.data
    steps = max(1, int(seconds / float(model.opt.timestep)))
    finite = probe.advance(steps)
    return SmokeResult(
        xml_path=xml_path.resolve(),
        nq=int(model.nq),
        nv=int(model.nv),
        nu=int(model.nu),
        finite=finite,
        contacts=int(data.ncon),
        sim_time=float(data.time),
    )


def run_step_response(
    xml_path: Path,
    *,
    joint: str = "joint2",
    target: float = -0.6,
    seconds: float = 3.0,
) -> StepResponseResult:
    """Measure the canonical position controller, with torque actuators underneath."""
    from rebotarm_simulation.core.mujoco_sim import ARM_JOINT_NAMES, RebotArmMujoco

    seconds = _positive_seconds(seconds)
    if joint not in ARM_JOINT_NAMES:
        raise ValueError(f"unknown arm joint: {joint}")
    index = ARM_JOINT_NAMES.index(joint)
    with RebotArmMujoco(xml_path) as sim:
        targets = list(sim.get_state().joint_positions[:6])
        targets[index] = float(target)
        applied = sim.set_joint_position_targets(targets)
        if applied[index] != float(target):
            raise ValueError("step target exceeds joint position limits")
        errors = []
        max_velocity = max_force = 0.0
        for _ in range(max(1, math.ceil(seconds / sim.timestep))):
            state = sim.step()
            errors.append(float(target) - state.joint_positions[index])
            max_velocity = max(max_velocity, abs(state.joint_velocities[index]))
            max_force = max(max_force, abs(state.actuator_forces[index]))
        return StepResponseResult(
            joint=joint, target=float(target),
            final_position=state.joint_positions[index],
            final_abs_error=abs(errors[-1]), max_abs_error=max(map(abs, errors)),
            rms_error=math.sqrt(sum(e * e for e in errors) / len(errors)),
            max_abs_velocity=max_velocity, max_abs_actuator_force=max_force,
            sim_time=state.simulation_time,
        )


def run_step_response_suite(
    xml_path: Path,
    *,
    targets: dict[str, float],
    seconds: float = 3.0,
) -> StepResponseSuiteResult:
    """按关节批量执行阶跃响应，并汇总各指标的"最坏值"。

    参数：
    - ``xml_path``：MJCF 文件路径；
    - ``targets``：关节名 -> 目标角（rad）的映射，遍历顺序（即字典插入顺序）就是
      执行顺序，也是结果列表顺序；空字典会得到空结果列表；
    - ``seconds``：每个关节各自的仿真时长，单位 s，默认 3.0。

    每个关节都从同一份模型文件重新加载并重置关键帧，因此各关节互不影响。返回
    :class:`StepResponseSuiteResult`，其中 ``max_*`` 字段用 ``default=0.0`` 覆盖
    空输入，保证不会对空序列调用 max()。
    """
    results = [
        run_step_response(xml_path, joint=joint, target=target, seconds=seconds)
        for joint, target in targets.items()
    ]
    return StepResponseSuiteResult(
        xml_path=xml_path,
        results=results,
        max_final_abs_error=max((result.final_abs_error for result in results), default=0.0),
        max_abs_error=max((result.max_abs_error for result in results), default=0.0),
        max_rms_error=max((result.rms_error for result in results), default=0.0),
        max_abs_velocity=max((result.max_abs_velocity for result in results), default=0.0),
        max_abs_actuator_force=max((result.max_abs_actuator_force for result in results), default=0.0),
    )


def run_grasp_benchmark(
    xml_path: Path, *, target_body: str, gripper_bodies: tuple[str, str],
    command, seconds: float = 5.0,
) -> GraspBenchmarkResult:
    """Execute command(sim, elapsed_seconds) and evaluate a named grasp trial.

    The caller supplies the trajectory/closure procedure. Missing objects or fingers
    fail before stepping. Floor, table, arm and unrelated object contacts are excluded.
    A historical touch followed by release cannot count as final grasp success.
    """
    from rebotarm_simulation.core.mujoco_sim import RebotArmMujoco

    seconds = _positive_seconds(seconds)
    if not callable(command):
        raise TypeError("command must be callable(sim, elapsed_seconds)")
    if len(gripper_bodies) != 2 or len(set(gripper_bodies)) != 2 or target_body in gripper_bodies:
        raise ValueError("two distinct gripper bodies separate from target are required")
    with RebotArmMujoco(xml_path) as sim:
        initial = sim.get_state()
        if target_body not in initial.object_poses:
            raise ValueError(f"target must be a free scene object: {target_body}")
        for name in gripper_bodies:
            if not sim.has_body(name):
                raise ValueError(f"missing gripper body: {name}")
        initial_z = initial.object_poses[target_body][2]
        max_contacts = 0
        for _ in range(max(1, math.ceil(seconds / sim.timestep))):
            command(sim, sim.get_state().simulation_time)
            state = sim.step()
            counts = target_gripper_contacts(sim.get_contacts(), target_body, gripper_bodies)
            max_contacts = max(max_contacts, sum(counts.values()))
        final_z = state.object_poses[target_body][2]
        quality = evaluate_grasp_quality(
            gripper_contacts=counts, initial_object_height_m=initial_z,
            final_object_height_m=final_z,
        )
        return GraspBenchmarkResult(
            xml_path=xml_path.resolve(), target_body=target_body, finite=True,
            initial_object_height_m=initial_z, object_height_m=final_z,
            max_contacts=max_contacts, final_contacts=sum(counts.values()),
            contact_detected=quality.contact_detected, lift_detected=quality.lift_detected,
            grasp_success=quality.success, grasp_status=quality.status,
            sim_time=state.simulation_time,
        )


def target_gripper_contacts(contacts, target_body, gripper_bodies):
    """Count only target-to-finger contacts, regardless of contact ordering."""
    counts = dict.fromkeys(gripper_bodies, 0)
    for contact in contacts:
        for finger in counts:
            if {contact.body1, contact.body2} == {target_body, finger}:
                counts[finger] += 1
    return counts


def _positive_seconds(seconds):
    seconds = float(seconds)
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be finite and positive")
    return seconds


def benchmark_wall_time(command) -> tuple[float, object]:
    """执行无参可调用对象并返回 (真实耗时秒数, 其返回值)。

    用单调时钟测量墙钟时间，专门用来把"仿真时长"与"实际算力开销"区分开；
    被调用对象的异常照常向上抛出，不做捕获。
    """
    start = time.monotonic()
    result = command()
    return time.monotonic() - start, result
