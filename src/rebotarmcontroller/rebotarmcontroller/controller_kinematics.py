"""规范六轴模型的正运动学适配，只返回数值。"""
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
        import pinocchio as pin
        data = self.model.createData()
        pin.forwardKinematics(self.model,data,vector(q,6,'FK joints'))
        pin.updateFramePlacements(self.model,data)
        transform = data.oMf[self.frame_id]
        return np.array(transform.translation,copy=True),np.array(transform.rotation,copy=True)
