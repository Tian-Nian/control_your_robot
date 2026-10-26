"""Explicit Piper model adapter; does not control a runtime or physical robot."""
import hashlib
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from robot.harness.executor import _vector


class PiperKinematics:
    def __init__(self):
        from robot.controller.PiperSim_controller import PiperSimController
        self.model = PiperSimController("harness_kinematics", spawn=False)
        self.model.set_up(tool_in_urdf=True)
        self.calibration_id = "piper:" + hashlib.sha256(
            Path(self.model.urdf_path).read_bytes()).hexdigest()[:16]

    def pose(self, joint, point="flange"):
        if point not in ("flange", "tool"):
            raise ValueError("point must be flange or tool")
        self.model.q[:6] = _vector(joint, 6)
        state = self.model.get_state()
        pose = state["eef"].copy()
        if point == "tool":
            pose[:3] = self.model.tool_point()
        return pose.tolist()

    def solve(self, joint, pose, *, frame="base", point="flange"):
        if frame != "base" or point not in ("flange", "tool"):
            raise ValueError("only base-frame flange/tool poses are supported")
        pose = _vector(pose, 7)
        if abs(np.linalg.norm(pose[3:]) - 1) > 1e-5:
            raise ValueError("quaternion must be unit wxyz")
        rotation = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
        translation = pose[:3].copy()
        if point == "tool":
            translation -= rotation @ self.model.tool_offset
        self.model.q[:6] = _vector(joint, 6)
        return self.model._ik(rotation, translation, commit=False).tolist()

    def error(self, joint, desired, point):
        actual = np.asarray(self.pose(joint, point))
        desired = np.asarray(desired)
        translation = float(np.linalg.norm(actual[:3] - desired[:3]))
        a = Rotation.from_quat(actual[[4, 5, 6, 3]])
        b = Rotation.from_quat(desired[[4, 5, 6, 3]])
        return translation, float((a.inv() * b).magnitude())
