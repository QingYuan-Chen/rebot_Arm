"""提供控制器反馈位姿所需的正运动学适配。

``ArmKinematics(model)`` 检查约化模型的六轴名称、维度及末端帧；
``pose(q)`` 接收六个关节值，返回末端平移向量和旋转矩阵的数值副本。
计算依赖 Pinocchio 与传入的规范模型，不访问电机、ROS 消息或 SDK 运动线程。

此处只计算正运动学，不负责逆运动学、路径生成或碰撞校验。每次计算创建
独立的数据对象，返回数组不暴露模型内部的可变缓存。
"""
import numpy as np
from .gravity_control import JOINT_NAMES, vector

class ArmKinematics:
    def __init__(self, model):
        if tuple(model.names)[1:]!=JOINT_NAMES or model.nq!=6 or model.nv!=6:
            raise ValueError('kinematics requires the canonical six-axis reduced model')
        self.model = model
        self.frame_id = model.getFrameId('end_link')
        if self.frame_id >= model.nframes: raise ValueError('end_link is missing')

    def pose(self,q):
        """按规范关节顺序计算 ``end_link`` 位姿并复制返回值。"""
        import pinocchio as pin
        data = self.model.createData()
        pin.forwardKinematics(self.model,data,vector(q,6,'FK joints'))
        pin.updateFramePlacements(self.model,data)
        transform = data.oMf[self.frame_id]
        return np.array(transform.translation,copy=True),np.array(transform.rotation,copy=True)
