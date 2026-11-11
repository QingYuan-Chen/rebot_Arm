"""TCP(工具中心点)标定的数值核心.

本模块只做纯计算:不依赖 ROS 运行时,不订阅话题,也不读写文件.上层节点负责采集
样本,拿到基座坐标系下的末端位姿与参考点并落盘,本模块负责把样本解算成 TCP 偏置,
并给出可判定的通过/失败门限.

提供两种互相独立的标定模型:

1. **已知参考点模型**(:func:`analyze_tcp_samples`)
   参考点在基座坐标系中的位置由外部给定(人为冻结的物理点,或由视觉标记测得的
   点).对每个样本按 ``offset = R_end^T @ (p_reference - p_end)`` 求出 TCP 在
   **末端连杆坐标系**下的偏置,再对多样本取平均.
2. **经典四点/枢轴模型**(:func:`analyze_tcp_pivot_samples`)
   TCP 偏置与枢轴点在基座坐标系中的位置**同时未知**,用线性最小二乘联合求解
   ``p_end + R_end @ tcp_offset = pivot``,不依赖手眼标定,也不依赖视觉参考点.

坐标系与单位约定:位置一律使用米(m),姿态使用 ``xyzw`` 顺序的四元数;旋转矩阵为
3x3,正交且行列式 +1 的右手系矩阵.解算结果只是**候选值**,是否部署由门限
(``passed``)决定,本模块不会自动写入任何配置.

边界保护:四元数范数小于 ``1e-12`` 视为非法(无法归一化);样本不足时姿态跨度,
轴标准差等统计量退化为 0,让对应门限判失败,而不是抛异常或给出看似合理的假解.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
import math

import numpy as np


# 三维向量类型别名:位置与偏置统一用"3 元浮点元组"表示,单位 m
Vector3 = tuple[float, float, float]


def _vector3(values: Sequence[float], name: str) -> Vector3:
    """把输入规范成 3 元浮点元组,并做长度与有限性校验.

    作为所有外部输入的公共入口校验:长度必须恰好为 3,且不能出现 NaN/Inf,否则后续
    旋转运算与最小二乘会被污染成无意义的解.失败时抛 ``ValueError``,异常文本里带上
    ``name``,便于定位是哪个字段出错.
    """
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


def quaternion_to_rotation_matrix(values: Sequence[float]) -> np.ndarray:
    """四元数 ``(x, y, z, w)`` → 3x3 旋转矩阵(Hamilton 约定,右手系).

    先归一化以容忍 TF 或标定文件中的非单位四元数;范数 <= ``1e-12`` 时无法归一化
    (零四元数或数值退化),直接抛 ``ValueError``,避免产生除零得到的全 NaN 矩阵.
    """
    if len(values) != 4:
        raise ValueError("orientation must contain exactly 4 xyzw values")
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("orientation quaternion must be finite and non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # 标准四元数→旋转矩阵展开(归一化后 x/y/z/w 的二次型)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def estimate_sample_offset(
    *,
    end_link_position: Sequence[float],
    end_link_orientation_xyzw: Sequence[float],
    tcp_reference_position: Sequence[float],
) -> Vector3:
    """单样本求 TCP 在末端连杆坐标系下的偏置(单位 m).

    ``end_link_position`` 与 ``tcp_reference_position`` 都是**基座坐标系**下的位置.
    模型为 ``p_reference = p_end + R_end @ offset``,因此
    ``offset = R_end^T @ (p_reference - p_end)``;用转置代替显式求逆,既省算力也避免
    数值求逆带来的误差.参考点在采样期间必须保持不动,否则该式不成立.
    """
    end_position = np.asarray(_vector3(end_link_position, "end_link_position"))
    reference_position = np.asarray(_vector3(tcp_reference_position, "tcp_reference_position"))
    rotation = quaternion_to_rotation_matrix(end_link_orientation_xyzw)
    offset = rotation.T @ (reference_position - end_position)
    return tuple(float(value) for value in offset)  # type: ignore[return-value]


def average_offsets(offsets: Iterable[Sequence[float]]) -> Vector3:
    """对多样本 TCP 偏置按轴求算术平均(单位 m),作为候选 TCP 偏置.

    空输入直接抛 ``ValueError``:零样本时均值没有定义,静默返回 0 会掩盖"一个样本都
    没采到"这种错误状态.传入的偏置必须已经统一到末端连杆坐标系(即各自乘过
    ``R_end^T``),否则在基座坐标系下直接平均没有物理意义.
    """
    samples = np.asarray([_vector3(offset, "offset") for offset in offsets], dtype=np.float64)
    if samples.size == 0:
        raise ValueError("at least one offset sample is required")
    result = np.mean(samples, axis=0)
    return tuple(float(value) for value in result)  # type: ignore[return-value]


def _rotation_distance_deg(left: np.ndarray, right: np.ndarray) -> float:
    """两个旋转矩阵之间的测地线夹角(单位:度).

    由迹公式 ``cos(theta) = (trace(L^T R) - 1) / 2`` 求得;``clip`` 到 [-1, 1] 是浮点
    保护(trace 略微越界会让 ``acos`` 返回 NaN).本模块用它衡量样本姿态的多样性:
    跨度太小说明姿态几乎没变,TCP 偏置与参考点误差无法分辨.
    """
    cosine = float(np.clip((np.trace(left.T @ right) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def analyze_tcp_samples(
    samples: Sequence[Mapping[str, Sequence[float]]],
    *,
    minimum_samples: int = 5,
    minimum_rotation_span_deg: float = 20.0,
    maximum_rms_residual_m: float = 0.003,
    maximum_residual_m: float = 0.006,
    maximum_axis_std_m: float = 0.003,
) -> dict[str, object]:
    """已知参考点模型的多样本 TCP 标定解算与门限判定.

    每个样本是映射,需含 ``end_link_position``(m,基座系),
    ``end_link_orientation_xyzw`` 与 ``tcp_reference_position``(m,基座系,采样期间
    冻结不动的参考点);缺字段时抛 ``ValueError`` 并在消息里给出样本序号.

    算法:逐样本求末端坐标系下的偏置 → 取平均作为候选 TCP 偏置 → 用候选值**正向
    重算**参考点,残差向量 ``p_end + R_end @ tcp_offset - p_reference`` 的范数用于评估
    一致性与重复性(正向重算而不是复用求解式,才能真实反映平均带来的误差).

    门限(全部满足 ``passed`` 才为 True):
    - ``minimum_samples``:最少样本数,默认 5,样本太少时均值不可信;
    - ``minimum_rotation_span_deg``:样本间最大姿态夹角下限,默认 20 度,姿态跨度
      不足时偏置与参考点误差不可分辨;
    - ``maximum_rms_residual_m``:残差 RMS 上限,默认 0.003 m;
    - ``maximum_residual_m``:单样本最大残差上限,默认 0.006 m,对应抓取 TCP 的
      典型允许偏差量级;
    - ``maximum_axis_std_m``:各轴偏置标准差上限,默认 0.003 m,用于发现某个方向
      上的系统性误差.

    返回的报告字典键名(``tcp_offset_xyz``/``rms_residual_m``/``gates``/``passed``
    等)是上层节点与工具读取的对外接口,不要改动.
    """
    _validate_limits(locals())
    if not samples:
        raise ValueError("at least one TCP sample is required")

    offsets: list[np.ndarray] = []
    rotations: list[np.ndarray] = []
    end_positions: list[np.ndarray] = []
    references: list[np.ndarray] = []
    for index, sample in enumerate(samples):
        try:
            end_position = np.asarray(_vector3(sample["end_link_position"], "end_link_position"))
            reference = np.asarray(_vector3(sample["tcp_reference_position"], "tcp_reference_position"))
            rotation = quaternion_to_rotation_matrix(sample["end_link_orientation_xyzw"])
        except KeyError as exc:
            raise ValueError(f"sample {index} missing field: {exc.args[0]}") from exc
        offsets.append(rotation.T @ (reference - end_position))
        rotations.append(rotation)
        end_positions.append(end_position)
        references.append(reference)

    offset_array = np.stack(offsets)
    estimate = np.mean(offset_array, axis=0)
    # 用平均偏置正向重算参考点,残差反映"单一 TCP 偏置能否同时解释所有样本"
    residual_vectors = np.stack(
        [end + rotation @ estimate - reference for end, rotation, reference in zip(end_positions, rotations, references)]
    )
    residual_norms = np.linalg.norm(residual_vectors, axis=1)
    rms = float(np.sqrt(np.mean(np.square(residual_norms))))
    maximum = float(np.max(residual_norms))
    axis_std = np.std(offset_array, axis=0)
    # 姿态多样性指标:所有样本两两之间的最大夹角(度);不足 2 个样本时为 0
    rotation_span = max(
        (_rotation_distance_deg(left, right) for i, left in enumerate(rotations) for right in rotations[i + 1 :]),
        default=0.0,
    )
    gates = {
        "sample_count": len(samples) >= int(minimum_samples),
        "rotation_span": rotation_span >= float(minimum_rotation_span_deg),
        "rms_residual": rms <= float(maximum_rms_residual_m),
        "maximum_residual": maximum <= float(maximum_residual_m),
        "axis_std": float(np.max(axis_std)) <= float(maximum_axis_std_m),
    }
    return {
        "sample_count": len(samples),
        "tcp_offset_xyz": estimate.tolist(),
        "per_sample_offset_xyz": offset_array.tolist(),
        "offset_axis_std_m": axis_std.tolist(),
        "reference_residual_vectors_m": residual_vectors.tolist(),
        "reference_residual_norms_m": residual_norms.tolist(),
        "rms_residual_m": rms,
        "maximum_residual_m": maximum,
        "rotation_span_deg": rotation_span,
        "limits": {
            "minimum_samples": int(minimum_samples),
            "minimum_rotation_span_deg": float(minimum_rotation_span_deg),
            "maximum_rms_residual_m": float(maximum_rms_residual_m),
            "maximum_residual_m": float(maximum_residual_m),
            "maximum_axis_std_m": float(maximum_axis_std_m),
        },
        "gates": gates,
        "passed": all(gates.values()),
    }


def analyze_tcp_pivot_samples(
    samples: Sequence[Mapping[str, Sequence[float]]],
    *,
    minimum_samples: int = 5,
    minimum_rotation_span_deg: float = 30.0,
    maximum_condition_number: float = 100.0,
    maximum_rms_residual_m: float = 0.003,
    maximum_residual_m: float = 0.006,
) -> dict[str, object]:
    """求解 ``p_end + R_end * tcp_offset = fixed_pivot``.

    TCP 在末端坐标系下的偏置与枢轴点在**基座坐标系**下的位置都未知,这正是经典的
    枢轴/四点 TCP 标定模型:让工具尖端始终顶住同一个物理点,摆出多个姿态,即可联立
    求解.它不依赖手眼标定,也不需要视觉测量出的参考点,因此不受相机外参误差影响.

    线性化:每个样本给出 3 个方程 ``[R_end | -I] @ [tcp_offset; pivot] = -p_end``,
    把 N 个样本纵向堆叠后用最小二乘求解 6 个未知量.可辨识的前提是姿态足够分散
    (设计矩阵满秩且条件数良好),所以门限里包含秩与条件数检查.

    门限(全部满足 ``passed`` 才为 True):
    - ``minimum_samples``:最少样本数,默认 5;
    - ``minimum_rotation_span_deg``:样本间最大姿态夹角下限,默认 30 度(比已知参考点
      模型更严,因为这里要同时辨识位置与姿态两项未知量);
    - ``maximum_condition_number``:设计矩阵条件数上限,默认 100,条件数过大说明解对
      测量噪声极其敏感;
    - ``maximum_rms_residual_m`` / ``maximum_residual_m``:枢轴点残差的 RMS 与最大
      值上限,默认 0.003 m / 0.006 m.

    返回报告中的 ``method`` 固定为 ``classic_tcp_pivot_unknown_reference``,供上层区分
    结果来自哪种模型.
    """

    _validate_limits(locals())
    if not samples:
        raise ValueError("at least one TCP pivot sample is required")
    rotations: list[np.ndarray] = []
    positions: list[np.ndarray] = []
    for index, sample in enumerate(samples):
        try:
            positions.append(
                np.asarray(_vector3(sample["end_link_position"], "end_link_position"))
            )
            rotations.append(
                quaternion_to_rotation_matrix(sample["end_link_orientation_xyzw"])
            )
        except KeyError as exc:
            raise ValueError(f"sample {index} missing field: {exc.args[0]}") from exc

    identity = np.eye(3, dtype=np.float64)
    # 设计矩阵 A = 纵向堆叠的 [R_end | -I](每个样本 3 行),未知量 x = [tcp_offset; pivot]
    matrix = np.vstack(
        [np.hstack((rotation, -identity)) for rotation in rotations]
    )
    # 右端项 b = -p_end(把 p_end 移到等号右侧)
    target = np.concatenate([-position for position in positions])
    # rcond=None 时以"机器精度 × max(行数, 列数)"作为小奇异值的截断阈值;rank 反映可辨识
    # 的自由度(满秩应为 6)
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, target, rcond=None)
    tcp_offset = solution[:3]
    pivot_position = solution[3:]
    residual_vectors = np.stack(
        [
            position + rotation @ tcp_offset - pivot_position
            for position, rotation in zip(positions, rotations)
        ]
    )
    residual_norms = np.linalg.norm(residual_vectors, axis=1)
    rms = float(np.sqrt(np.mean(np.square(residual_norms))))
    maximum = float(np.max(residual_norms))
    condition = (
        float(singular_values[0] / singular_values[-1])
        if len(singular_values) == 6 and singular_values[-1] > 1e-12
        else float("inf")
    )
    # 姿态多样性指标:所有样本两两之间的最大夹角(度)
    rotation_span = max(
        (
            _rotation_distance_deg(left, right)
            for index, left in enumerate(rotations)
            for right in rotations[index + 1 :]
        ),
        default=0.0,
    )
    gates = {
        "sample_count": len(samples) >= int(minimum_samples),
        "full_rank": int(rank) == 6,
        "condition_number": condition <= float(maximum_condition_number),
        "rotation_span": rotation_span >= float(minimum_rotation_span_deg),
        "rms_residual": rms <= float(maximum_rms_residual_m),
        "maximum_residual": maximum <= float(maximum_residual_m),
    }
    return {
        "method": "classic_tcp_pivot_unknown_reference",
        "sample_count": len(samples),
        "tcp_offset_xyz": tcp_offset.tolist(),
        "pivot_position_base_xyz": pivot_position.tolist(),
        "matrix_rank": int(rank),
        "condition_number": condition if math.isfinite(condition) else None,
        "singular_values": singular_values.tolist(),
        "rotation_span_deg": rotation_span,
        "pivot_residual_vectors_m": residual_vectors.tolist(),
        "pivot_residual_norms_m": residual_norms.tolist(),
        "rms_residual_m": rms,
        "maximum_residual_m": maximum,
        "limits": {
            "minimum_samples": int(minimum_samples),
            "minimum_rotation_span_deg": float(minimum_rotation_span_deg),
            "maximum_condition_number": float(maximum_condition_number),
            "maximum_rms_residual_m": float(maximum_rms_residual_m),
            "maximum_residual_m": float(maximum_residual_m),
        },
        "gates": gates,
        "passed": all(gates.values()),
    }
def format_tcp_offset_yaml(offset: Sequence[float]) -> str:
    """把 TCP 偏置格式化成可直接粘贴进 YAML 的单行片段(单位 m,6 位小数).

    键名 ``tcp_offset_xyz`` 与输出格式是给标定结果文件与上层解析用的对外约定,不要
    改动键名或小数位数.
    """
    ox, oy, oz = _vector3(offset, "offset")
    return f"tcp_offset_xyz: [{ox:.6f}, {oy:.6f}, {oz:.6f}]"


def _validate_limits(values):
    for name, value in values.items():
        if name.startswith(("minimum_", "maximum_")):
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f"{name} must be finite and positive")
    count = values["minimum_samples"]
    if int(count) != count or count < 3:
        raise ValueError("minimum_samples must be an integer >= 3")
