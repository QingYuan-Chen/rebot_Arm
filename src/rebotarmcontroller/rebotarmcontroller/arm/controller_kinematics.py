"""提供控制器反馈位姿所需的正运动学适配.

职责与位置
    控制器状态发布需要的纯数值 FK 适配, 使用已校验的规范六轴模型.

对外接口
    ArmKinematics.pose 接收关节角, 返回 end_link 在模型基座中的位置与旋转矩阵副本.

安全与并发约束
    每次计算创建独立 Data; 不负责 IK, 路径或碰撞, 不根据 FK 结果许可运动.

文件结构(按定义顺序)
    ArmKinematics: 规范六轴模型的 FK 适配, 只返回数值副本, 不提供 IK 或运动发送.
"""

import numpy as np
from ..gravity.gravity_control import JOINT_NAMES, vector


class ArmKinematics:
    """规范六轴模型的 FK 适配, 只返回数值副本, 不提供 IK 或运动发送."""

    def __init__(self, model):
        """校验模型维数, 关节固定顺序及 end_link, 保存只用于 FK 的模型引用."""
        if tuple(model.names)[1:]!=JOINT_NAMES or model.nq!=6 or model.nv!=6:
            raise ValueError('kinematics requires the canonical six-axis reduced model')
        self.model = model
        self.frame_id = model.getFrameId('end_link')
        if self.frame_id >= model.nframes: raise ValueError('end_link is missing')

    def pose(self,q):
        """按规范关节顺序计算 ``end_link`` 位姿并复制返回值."""
        import pinocchio as pin
        data = self.model.createData()
        pin.forwardKinematics(self.model,data,vector(q,6,'FK joints'))
        pin.updateFramePlacements(self.model,data)
        transform = data.oMf[self.frame_id]
        return np.array(transform.translation,copy=True),np.array(transform.rotation,copy=True)
