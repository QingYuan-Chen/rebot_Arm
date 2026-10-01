"""视觉抓取动作序列的唯一构造入口（纯计算，不下发任何指令）。

职责与位置
----------
把"预抓取位姿 + 抓取位姿 + 夹爪策略"翻译成执行器可逐阶段下发的动作阶段列表，
阶段顺序固定为：可选的初始开爪 -> 移动到预抓取位 -> 接近抓取点 -> 合爪
-> 可选的沿接近路径反向撤退 -> 可选的回安全位。

数据流与安全边界
----------------
- 位姿单位统一为米与四元数（顺序 xyzw），坐标系由调用方决定：执行器先把候选位姿
  变换到目标坐标系、叠加 TCP 偏置与基座偏移后再传入本模块，所以这里只做几何推
  导，不接触 TF，也不做碰撞检查。
- 本模块不发布话题、不调用服务，只返回 :class:`VisualGraspStage` 列表；真正的
  规划、碰撞检查、执行使能与急停门控都在执行器与控制器侧。
- 出错方式为抛异常，绝不静默降级：抓取点低于安全高度、夹爪策略判定物体过宽、
  撤退轴为零向量都会让整条序列构造失败，宁可放弃本次抓取。
"""

from __future__ import annotations

from dataclasses import dataclass

from .policies.gripper_policy import GripperCommand
from .policies.retreat_policy import RetreatPolicyConfig, build_retreat_pose


@dataclass(frozen=True)
class PoseTarget:
    """末端 TCP 目标位姿。

    position 为 (x, y, z)，单位 m；orientation 为四元数 (x, y, z, w)，单位无。
    本仓库约定四元数按 xyzw 存放（与几何消息字段顺序一致），使用前应由调用方
    保证已归一化。该类型同时被候选目标策略、可行性与伺服策略等模块共用。
    """

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class VisualGraspSequenceConfig:
    """序列构造配置。

    全部长度单位 m，力/力矩单位为控制器约定的归一化量纲（本仓库夹爪取 0~0.6）。
    字段语义与联动关系：

    - ``open_before_approach``：是否在接近前先插一条开爪阶段。合爪前若夹爪仍处于
      上次的闭合状态，接近时可能刮碰目标或桌面。
    - ``open_position_m`` / ``close_position_m``：固定开爪、合爪开口，单位 m；仅当
      ``auto_gripper_width`` 关闭或未测得有效目标宽度时生效。
    - ``close_max_effort``：合爪阶段最大夹持力（归一化），越大夹得越紧也越容易压坏
      目标，且更容易触发夹爪堵转/过载报错。
    - ``min_grasp_z_m``：抓取点最低允许高度，是安全门；低于该值说明目标太贴桌面或
      在桌面以下，直接拒绝而不是继续规划。
    - ``auto_gripper_width`` 及以下 6 个字段：按测量到的夹爪目标宽度自动推导开合
      宽度。``detected_jaw_width_m`` 为检测/候选给出的目标宽度；开爪 = 检测宽度 +
      ``open_clearance_m``，合爪 = 检测宽度 - ``close_margin_m``，两者分别被
      ``min/max_open_position_m``、``min/max_close_position_m`` 夹到安全行程内。
    - ``gripper_command``：夹爪策略预计算好的命令；非空时直接采用它的开合宽度与
      夹持力（并复用其允许/拒绝判定），此时上面那组自动宽度字段不再参与计算。
    - ``retreat_policy``：启用后从抓取点沿 ``grasp -> pregrasp`` 方向退出；不使用
      固定基座方向，也不先做独立垂直抬升。
    - ``include_safe_home``：是否在序列末尾追加回安全位阶段（设备侧服务按自身
      记录的基准位姿执行，本模块不携带目标点）。
    """

    open_before_approach: bool = False
    open_position_m: float = 0.09
    close_position_m: float = 0.025
    close_max_effort: float = 0.4
    min_grasp_z_m: float = 0.0
    auto_gripper_width: bool = False
    detected_jaw_width_m: float = 0.0
    open_clearance_m: float = 0.0
    close_margin_m: float = 0.012
    min_open_position_m: float = 0.035
    max_open_position_m: float = 0.09
    min_close_position_m: float = 0.006
    max_close_position_m: float = 0.08
    gripper_command: GripperCommand | None = None
    retreat_policy: RetreatPolicyConfig = RetreatPolicyConfig()
    include_safe_home: bool = False


@dataclass(frozen=True)
class VisualGraspStage:
    """动作序列中的一个阶段，供执行器逐条下发。

    ``kind`` 决定执行器走哪条分支："move" 走位姿执行服务，"gripper" 走夹爪设置/
    夹取服务，"safe_home" 调用设备侧回安全位服务。因此 ``pose`` 只在 "move" 阶段
    有值，``gripper_position_m``/``gripper_max_effort`` 只在 "gripper" 阶段有值。

    ``detected_jaw_width_m`` 是从候选继承下来的观测宽度，仅作诊断与后续阶段校验
    用，不直接作为夹爪指令；``gripper_position_m`` 才是真正下发的位置。
    """

    name: str
    kind: str
    pose: PoseTarget | None = None
    gripper_position_m: float | None = None
    gripper_max_effort: float | None = None
    detected_jaw_width_m: float = 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def resolve_gripper_widths(config: VisualGraspSequenceConfig) -> tuple[float, float]:
    """推导实际的（开爪宽度, 合爪宽度），单位 m。

    未启用自动宽度、或检测宽度非正时，直接返回配置里的固定开合宽度。启用后以检测
    宽度为基准，开爪留 ``open_clearance_m`` 余量、合爪留 ``close_margin_m`` 余量，
    再分别夹到各自的行程上下限；最后保证合爪宽度不超过开爪宽度，避免出现"先合后
    开"的异常序列。
    """
    if not config.auto_gripper_width or float(config.detected_jaw_width_m) <= 0.0:
        return float(config.open_position_m), float(config.close_position_m)
    detected_width = max(float(config.detected_jaw_width_m), 0.0)
    open_width = _clamp(
        detected_width + float(config.open_clearance_m),
        float(config.min_open_position_m),
        float(config.max_open_position_m),
    )
    close_width = _clamp(
        detected_width - float(config.close_margin_m),
        float(config.min_close_position_m),
        float(config.max_close_position_m),
    )
    close_width = min(close_width, open_width)
    return open_width, close_width


def build_visual_grasp_sequence(
    pregrasp: PoseTarget,
    grasp: PoseTarget,
    config: VisualGraspSequenceConfig,
) -> list[VisualGraspStage]:
    """构造完整的视觉抓取阶段列表（纯函数，只做几何与安全检查）。

    阶段顺序：可选的初始开爪 -> move_to_pregrasp -> approach_grasp ->
    close_gripper -> 可选的 safe_retreat -> 可选的 safe_home。

    异常：抓取点 Z 低于 ``min_grasp_z_m`` 时抛 ``ValueError``（安全门）；夹爪策略
    已判定不允许抓取时抛 ``ValueError``（物体过宽等）；撤退轴为零向量时由撤退策略
    抛出。返回值可直接交给执行器逐阶段执行，本函数不产生任何副作用。
    """
    if grasp.position[2] < config.min_grasp_z_m:
        raise ValueError(
            f"grasp z={grasp.position[2]:.3f} is below minimum "
            f"{config.min_grasp_z_m:.3f}"
        )

    # 夹爪策略给出的命令优先：它已包含过宽拒绝判定与宽度上下限钳制。
    if config.gripper_command is not None:
        if not config.gripper_command.allowed:
            raise ValueError(f"gripper policy rejected grasp: {config.gripper_command.reason}")
        open_width = float(config.gripper_command.open_width_m)
        close_width = float(config.gripper_command.close_width_m)
        max_effort = float(config.gripper_command.max_effort)
    else:
        open_width, close_width = resolve_gripper_widths(config)
        max_effort = float(config.close_max_effort)
    stages: list[VisualGraspStage] = []
    if config.open_before_approach:
        stages.append(
            VisualGraspStage(
                name="open_gripper",
                kind="gripper",
                gripper_position_m=open_width,
                gripper_max_effort=max_effort,
                detected_jaw_width_m=float(config.detected_jaw_width_m),
            )
        )
    # 主体阶段始终存在，顺序不可调换：先到位、再接近、再合爪。
    stages.extend(
        [
            VisualGraspStage(
                name="move_to_pregrasp",
                kind="move",
                pose=pregrasp,
            ),
            VisualGraspStage(
                name="approach_grasp",
                kind="move",
                pose=grasp,
            ),
            VisualGraspStage(
                name="close_gripper",
                kind="gripper",
                gripper_position_m=close_width,
                gripper_max_effort=max_effort,
                detected_jaw_width_m=float(config.detected_jaw_width_m),
            ),
        ]
    )
    if config.retreat_policy.enabled:
        stages.append(
            VisualGraspStage(
                name="safe_retreat",
                kind="move",
                pose=build_retreat_pose(grasp, pregrasp, config.retreat_policy),
            )
        )
    if config.include_safe_home:
        stages.append(VisualGraspStage(name="safe_home", kind="safe_home"))
    return stages
