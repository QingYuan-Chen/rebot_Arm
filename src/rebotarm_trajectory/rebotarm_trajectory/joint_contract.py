"""公共 FJT 的六轴纯数值边界契约。

文件职责：校验完整机械臂关节集合并按规范顺序排列轨迹点。
对外接口：canonical_input 返回六轴关节名和原时间、原导数字段的有序点。
安全约束：拒绝夹爪、任意缺轴和额外关节，不静默投影、不补造导数。
文件结构：机构集合检查 → 共享点校验与排序。
"""

from .polynomial import JOINT_NAMES, ordered_points


def canonical_input(names, points, hold=None):
    """hold 保留原调用签名；不能用保持值补齐非法的缺轴请求。"""
    names, points = tuple(names), tuple(points)
    if len(names) != 6 or set(names) != set(JOINT_NAMES):
        raise ValueError('FJT requires exactly joint1..joint6; use the independent gripper interface')
    return JOINT_NAMES, ordered_points(names, points)
