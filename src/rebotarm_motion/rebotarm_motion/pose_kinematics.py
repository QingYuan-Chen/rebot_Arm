"""规范六轴模型及位姿纯计算, 供预览和旧位姿协议共用.

职责与位置
    规范六轴模型的 FK/IK 纯计算, 供预览与位姿兼容入口共用.

对外接口
    model_resource 定位模型; vector6 / pose_target 转换数值; PoseKinematics 返回 FK 或 IKResult.

安全与并发约束
    只锁定模型中的双手指关节; 每次求解创建独立 Data. 失败结果可含最佳角度, 但 success=False 时不得执
    行.
    IK 收敛仅表示模型误差达标, 不证明路径连续, 无碰撞或可在硬件上安全执行.

统一约定
    joint1..joint6 为 rad, 位置 m, RPY rad; SE(3) 目标相对模型基座.

文件结构(按定义顺序)
    model_resource: 定位规范配置包资源: 优先安装索引, 再回退源码布局, 缺失时拒绝.
    vector6: 将输入校验为六个有限浮点值并返回数组副本; 本函数不检查关节限位.
    pose_target: 把 xyz(m)和固定轴 RPY(rad)六元组转换为相对模型基座的 SE(3).
    IKResult: IK 的收敛标志, 最佳关节解, SE(3) 残差范数与迭代记录; 失败角度不得下发.
    PoseKinematics: 规范六轴模型的 FK 与有界重试 IK, 求解过程中不共享可变 Data.
"""

from dataclasses import dataclass
from pathlib import Path
import numpy as np
import pinocchio as pin

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def model_resource(relative):
    """定位规范配置包资源: 优先安装索引, 再回退源码布局, 缺失时拒绝."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('rebotarm_moveit_config')) / relative
        if path.is_file():
            return path
    except (ImportError, LookupError):
        pass
    path = Path(__file__).resolve().parents[2] / 'rebotarm_moveit_config' / relative
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path


def vector6(values):
    """将输入校验为六个有限浮点值并返回数组副本; 本函数不检查关节限位."""
    q = np.asarray(values, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('six finite joint/pose values required')
    return q.copy()


def pose_target(values):
    """把 xyz(m)和固定轴 RPY(rad)六元组转换为相对模型基座的 SE(3)."""
    values = vector6(values)
    return pin.SE3(pin.rpy.rpyToMatrix(*values[3:]), values[:3])


@dataclass(frozen=True)
class IKResult:
    """IK 的收敛标志, 最佳关节解, SE(3) 残差范数与迭代记录; 失败角度不得下发."""

    success: bool
    q: np.ndarray
    error: float
    iterations: int


class PoseKinematics:
    """规范六轴模型的 FK 与有界重试 IK, 求解过程中不共享可变 Data."""

    def __init__(self, path=None, end_frame='end_link', *, model=None):
        """加载或接收规范模型, 仅约化双手指并核对六轴顺序, 自由度和末端帧."""
        if model is None:
            full = pin.buildModelFromUrdf(str(path or model_resource('config/rebotarm.urdf')))
            extras = [name for name in list(full.names)[1:] if name not in JOINT_NAMES]
            if set(extras) - {'left_finger_joint', 'right_finger_joint'}:
                raise ValueError('unexpected non-arm joints')
            model = pin.buildReducedModel(full, [full.getJointId(n) for n in extras],
                                          pin.neutral(full)) if extras else full
        if model.nq != 6 or model.nv != 6 or tuple(model.names)[1:] != JOINT_NAMES:
            raise ValueError('canonical joint1..joint6 model required')
        if not model.existFrame(end_frame):
            raise ValueError(f'missing frame {end_frame}')
        self.model, self.frame_id = model, model.getFrameId(end_frame)
        self.lower = model.lowerPositionLimit.copy()
        self.upper = model.upperPositionLimit.copy()

    def joints(self, values):
        """检查六轴有限值与规范位置上下限, 返回可独立使用的数组."""
        q = vector6(values)
        if (q < self.lower).any() or (q > self.upper).any():
            raise ValueError('joint positions outside canonical limits')
        return q

    def pose(self, values):
        """创建本次 Data, 用合法六轴关节角计算并复制末端 SE(3)."""
        data = self.model.createData()
        pin.framesForwardKinematics(self.model, data, self.joints(values))
        return data.oMf[self.frame_id].copy()

    def solve(self, target, seed, *, retries=0, rng=None, max_iter=200,
              tolerance=1e-4, step_size=.5, damping=1e-6):
        """以 seed 为起点求解目标 SE(3), 返回收敛结果或最佳失败残差.

        先校验刚体变换与算法参数, 再使用局部误差, 阻尼伪逆及回溯下降.
        每步投影到规范限位; retries 允许有限次随机种子, 默认随机源固定以便复测.
        不检查碰撞, 也不保证不同种子解之间的路径连续性; 调用方必须检查 success.
        """
        if (not np.isfinite(target.homogeneous).all()
                or not np.allclose(target.rotation.T @ target.rotation, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(target.rotation), 1.)):
            raise ValueError('target must be a finite rigid transform')
        if (not np.isfinite([tolerance, step_size, damping]).all()
                or min(tolerance, step_size, damping) <= 0 or max_iter < 1 or retries < 0):
            raise ValueError('invalid IK parameters')
        q_seed = self.joints(seed)
        random = np.random.default_rng(0) if rng is None else rng
        best = IKResult(False, q_seed, float('inf'), 0)
        for attempt in range(retries + 1):
            q = q_seed.copy() if attempt == 0 else random.uniform(self.lower, self.upper)
            data = self.model.createData()
            for iteration in range(max_iter):
                pin.framesForwardKinematics(self.model, data, q)
                delta = data.oMf[self.frame_id].actInv(target)
                error = pin.log6(delta).vector
                norm = float(np.linalg.norm(error))
                if norm < best.error:
                    best = IKResult(norm <= tolerance, q.copy(), norm, iteration+1)
                if norm <= tolerance:
                    return best
                jacobian = pin.computeFrameJacobian(self.model, data, q, self.frame_id, pin.LOCAL)
                # d log(T(q)^-1 Td) / dq; 负号由右侧下降方向抵消.
                jacobian = -pin.Jlog6(delta.inverse()) @ jacobian
                increment = -jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping*np.eye(6), error)
                accepted = False
                # 回溯只接受残差下降的候选, 避免投影到关节边界后仍沿错误方向迭代.
                for scale in (step_size, step_size/2, step_size/4, step_size/8, step_size/16):
                    candidate = np.clip(pin.integrate(self.model, q, increment*scale),
                                        self.lower, self.upper)
                    pin.framesForwardKinematics(self.model, data, candidate)
                    candidate_error = np.linalg.norm(pin.log6(data.oMf[self.frame_id].actInv(target)).vector)
                    if candidate_error < norm-1e-12:
                        q, accepted = candidate, True
                        break
                if not accepted:
                    break
        return best
