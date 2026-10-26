"""Simulation-ground-truth validators, independent of model/physics packages.

These are task evidence, not visual perception. Three bowls have local +Y up.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def _touches(contact, first, second):
    a, b = contact["geom1"], contact["geom2"]
    return (a.startswith(first) and b.startswith(second)) or (b.startswith(first) and a.startswith(second))


def grasp_evidence(state, bowl, initial_z, minimum_lift=.03):
    objects = state["objects"]
    item = objects[bowl]
    contacts = [c for c in state["contacts"] if c["normal_force_n"] > .01]
    left = any(_touches(c, "robot_link7", bowl + "_collision_") for c in contacts)
    right = any(_touches(c, "robot_link8", bowl + "_collision_") for c in contacts)
    rise = float(item["pose"][2] - initial_z)
    return {"bilateral_contact": left and right, "lift_m": rise,
            "grasped_and_lifted": bool(left and right and rise >= minimum_lift),
            "source": "simulation_ground_truth"}


class StackVerifier:
    """Require sustained released, upright, aligned, supported, stationary bowls."""
    def __init__(self, stable_s=1.0):
        self.stable_s = float(stable_s)
        if not np.isfinite(self.stable_s) or self.stable_s <= 0:
            raise ValueError("stable_s must be finite and positive")
        self.since = self.last = None

    def update(self, state):
        now = float(state["sim_time"])
        objects = state["objects"]
        order = sorted(objects, key=lambda key: objects[key]["pose"][2])
        valid = len(order) == 3
        positions = np.array([objects[k]["pose"][:3] for k in order])
        if valid:
            valid = bool(np.max(np.linalg.norm(positions[:, :2] - positions[0, :2], axis=1)) < .02
                         and np.all(np.diff(positions[:, 2]) > .01)
                         and np.all(np.diff(positions[:, 2]) < .07))
        for key in order:
            p = np.asarray(objects[key]["pose"])
            velocity = np.asarray(objects[key]["velocity"])
            rotation = Rotation.from_quat(p[[4, 5, 6, 3]]).as_matrix()
            valid = bool(valid and rotation[2, 1] > .95 and np.linalg.norm(velocity[3:]) < .01
                         and np.linalg.norm(velocity[:3]) < .1)
        contacts = [c for c in state["contacts"] if c["normal_force_n"] > .01]
        if len(order) == 3:
            valid = bool(valid and any(_touches(c, "table", order[0] + "_collision_") for c in contacts)
                         and all(any(_touches(c, lower + "_collision_", upper + "_collision_") for c in contacts)
                                 for lower, upper in zip(order, order[1:])))
        valid = bool(valid and not any(c["geom1"].startswith("robot_") or c["geom2"].startswith("robot_") for c in contacts)
                     and float(np.asarray(state["gripper"]).reshape(-1)[0]) > .8)
        if not np.isfinite(now) or (self.last is not None and (now < self.last or now - self.last > .2)):
            self.since = None
        if not valid:
            self.since = None
        elif self.since is None:
            self.since = now
        self.last = now
        return {"stacked": bool(valid and self.since is not None and now - self.since >= self.stable_s),
                "conditions_met": valid, "stable_s": 0. if self.since is None else now - self.since,
                "order_bottom_to_top": order, "source": "simulation_ground_truth"}
