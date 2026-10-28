"""规范六轴模型及位姿纯计算，供预览和旧位姿协议共用。

职责：从配置包加载规范 URDF、锁定手指，提供 FK、阻尼最小二乘 IK 和
有限次数的种子重试。对外接口为 ``PoseKinematics.pose/solve`` 及 ``pose_target``。
不读取 SDK 模型、访问电机或创建线程。每次求解独立创建 Pinocchio Data，
目标必须是有限的 SE(3)，结果必须通过回代误差与规范限位；收敛不代表无碰撞。

算法为本地独立实现：局部 SE(3) 误差、阻尼伪逆、限位投影和回溯下降。
随机重试源可注入；默认固定种子便于回归。不可达时返回失败和最佳残差，
绝不把未收敛角度作为执行目标。仅承接项目已使用能力，不构建通用 SDK。
"""
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import pinocchio as pin

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def model_resource(relative):
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
    q = np.asarray(values, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('six finite joint/pose values required')
    return q.copy()


def pose_target(values):
    values = vector6(values)
    return pin.SE3(pin.rpy.rpyToMatrix(*values[3:]), values[:3])


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: np.ndarray
    error: float
    iterations: int


class PoseKinematics:
    def __init__(self, path=None, end_frame='end_link', *, model=None):
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
        q = vector6(values)
        if (q < self.lower).any() or (q > self.upper).any():
            raise ValueError('joint positions outside canonical limits')
        return q

    def pose(self, values):
        data = self.model.createData()
        pin.framesForwardKinematics(self.model, data, self.joints(values))
        return data.oMf[self.frame_id].copy()

    def solve(self, target, seed, *, retries=0, rng=None, max_iter=200,
              tolerance=1e-4, step_size=.5, damping=1e-6):
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
                # d log(T(q)^-1 Td) / dq；负号由右侧下降方向抵消。
                jacobian = -pin.Jlog6(delta.inverse()) @ jacobian
                increment = -jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + damping*np.eye(6), error)
                accepted = False
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
