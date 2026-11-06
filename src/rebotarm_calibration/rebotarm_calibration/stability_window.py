"""同期机器人与标记位姿的连续稳定窗口，以及保留同期配对的代表帧选择。"""
import numpy as np
from .handeye_solver import rotation_angle_deg


class StabilityWindow:
    def __init__(self, duration, max_gap, translation, rotation):
        self.duration, self.max_gap = duration, max_gap
        self.translation, self.rotation = translation, rotation
        self.values = []
        self.markers = []

    def reset(self):
        self.values.clear()
        self.markers.clear()

    def _distance(self, first, second):
        return (np.linalg.norm(first[:3, 3] - second[:3, 3]) / self.translation +
                rotation_angle_deg(first[:3, :3].T @ second[:3, :3]) / self.rotation)

    def _moved(self, first, second):
        return (np.linalg.norm(first[:3, 3] - second[:3, 3]) > self.translation or
                rotation_angle_deg(first[:3, :3].T @ second[:3, :3]) > self.rotation)

    def add(self, stamp, pose, marker_pose=None):
        if self.values:
            delta = (stamp - self.values[-1][0]) / 1e9
            if delta <= 0 or delta > self.max_gap:
                self.reset()
        if any(self._moved(pose, p) for _, p in self.values):
            self.reset()
        if self.markers and ((marker_pose is None) != (self.markers[0] is None)):
            raise ValueError('stability window observation mode changed')
        if marker_pose is not None and any(self._moved(marker_pose, p) for p in self.markers):
            self.reset()
        self.values.append((stamp, pose.copy()))
        self.markers.append(None if marker_pose is None else marker_pose.copy())
        return len(self.values) >= 3 and (stamp - self.values[0][0]) / 1e9 >= self.duration

    def representative_index(self):
        """选总位姿距离最小的实测帧，机器人和视觉始终使用同一时刻的配对。"""
        if not self.values:
            raise ValueError('empty stability window')
        def cost(index):
            score = sum(self._distance(self.values[index][1], p) for _, p in self.values)
            if self.markers[index] is not None:
                score += sum(self._distance(self.markers[index], p) for p in self.markers)
            return score
        return min(range(len(self.values)), key=cost)
