"""Opt-in contact world for harness experiments; imports MuJoCo only at setup.

One runtime component owns physics, arm, objects and timestamped cameras.
No dependency installation, downloads, asset conversion or model API calls.
"""
from pathlib import Path
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from robot.controller.arm_controller import ArmController

READY_JOINT = np.array([0., 1.3, -1., 0., .5, 0.])


def default_scene():
    for root in Path(__file__).resolve().parents:
        candidate = root / "assets/sim/piper_mujoco/scene.xml"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("Prepare optional assets with examples/assets/prepare_piper_mujoco.py --robotwin-assets PATH")


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"expected {size} finite values")
    return result


def pose(position, matrix):
    q = Rotation.from_matrix(np.asarray(matrix).reshape(3, 3)).as_quat()
    return np.r_[position, q[3], q[:3]]


class PiperMuJoCoController(ArmController):
    supported_modes = ("position",)

    def __init__(self, name="left_arm", scene_path=None, cameras=True, camera_hz=10,
                 width=320, height=240, spawn=False, record_path=None):
        super().__init__()
        self.name = name
        self.DoFs = 6
        self.scene_path = scene_path
        self.cameras_enabled = bool(cameras)
        self.camera_hz = float(camera_hz)
        self.width, self.height = int(width), int(height)
        if not math.isfinite(self.camera_hz) or not 0 < self.camera_hz <= 50:
            raise ValueError("camera_hz must be in (0, 50]")
        if not 16 <= self.width <= 640 or not 16 <= self.height <= 480:
            raise ValueError("camera size must be 16..640 by 16..480")
        self.spawn, self.record_path = bool(spawn), record_path
        self.model = self.data = self.renderer = self.rr = None
        self.fault = ""
        self._cameras = {}
        self._next_camera = 0.
        self._remainder = 0.
        self._frame_id = 0
        self._closed = False
        self._home = READY_JOINT.copy()

    def set_up(self, control_mode="position"):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError('Optional backend unavailable. Explicitly select the harness-mujoco extra, or install requirements/harness-mujoco.txt in an isolated environment.') from exc
        self.mj = mujoco
        self.set_control_mode(control_mode)
        path = Path(self.scene_path).resolve() if self.scene_path else default_scene()
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self._joint_ids = np.array([self.model.joint(f"joint{i}").id for i in range(1, 9)])
        self._qadr = self.model.jnt_qposadr[self._joint_ids]
        self._vadr = self.model.jnt_dofadr[self._joint_ids]
        self._lower, self._upper = self.model.jnt_range[self._joint_ids, 0], self.model.jnt_range[self._joint_ids, 1]
        self._act = np.array([self.model.actuator(f"joint{i}").id for i in range(1, 9)])
        self._flange = self.model.site("flange").id
        self._tool = self.model.site("tool").id
        self.data.qpos[self._qadr[:6]] = READY_JOINT
        self.data.qpos[self._qadr[6:]] = .035
        self.data.ctrl[self._act] = self.data.qpos[self._qadr]
        self._goal = self.data.ctrl[self._act].copy()
        mujoco.mj_forward(self.model, self.data)
        if self.cameras_enabled:
            self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
            self._capture()
        if self.spawn or self.record_path:
            self._start_rerun()
            self._log_rerun()

    def get_state(self):
        if self.data is None:
            raise RuntimeError("device not set up")
        objects = {}
        for i in range(3):
            body = self.model.body(f"bowl_{i}").id
            velocity = np.zeros(6)
            self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, body, velocity, 0)
            objects[f"bowl_{i}"] = {"pose": np.r_[self.data.xpos[body], self.data.xquat[body]].copy(),
                                     "velocity": velocity, "source": "simulation_ground_truth"}
        contacts = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            force = np.zeros(6)
            self.mj.mj_contactForce(self.model, self.data, i, force)
            contacts.append({"geom1": self.model.geom(c.geom1).name, "geom2": self.model.geom(c.geom2).name,
                             "position": c.pos.copy(), "normal_force_n": float(force[0])})
        return {"joint": self.data.qpos[self._qadr[:6]].copy(),
                "velocity": self.data.qvel[self._vadr[:6]].copy(),
                "eef": pose(self.data.site_xpos[self._flange], self.data.site_xmat[self._flange]),
                "gripper": np.array([np.clip(np.mean(self.data.qpos[self._qadr[6:]] / self._upper[6:]), 0., 1.)]),
                "gripper_joint_m": self.data.qpos[self._qadr[6:]].copy(),
                "control_mode": self.control_mode, "servo_fault": self.fault,
                "objects": objects, "contacts": contacts, "cameras": self._cameras,
                "sim_time": float(self.data.time)}

    def get_information(self):
        return self.get_state()

    def get(self):
        # Avoid formatting multi-megabyte camera arrays in the base debug logger.
        state = self.get_state()
        state["timestamp"] = time.time_ns()
        return {key: state[key] for key in self.collect_info}

    def set_joint(self, joint):
        q = vector(joint, 6)
        if np.any(q < self._lower[:6]) or np.any(q > self._upper[:6]):
            raise ValueError("joint target outside limits")
        self._goal[:6] = q

    def set_gripper(self, gripper):
        value = np.asarray(gripper, dtype=float).reshape(-1)
        if value.size != 1 or not np.isfinite(value).all() or not 0 <= value[0] <= 1:
            raise ValueError("gripper must be a scalar in [0,1]")
        self._goal[6:] = value[0] * self._upper[6:]

    def apply_reset(self, start_state):
        """Set an arm-only home goal; physics continues in servo_step, objects remain."""
        self.set_joint(READY_JOINT if start_state is None else start_state)

    def solve_ik(self, target, point="flange"):
        target = vector(target, 7)
        if abs(np.linalg.norm(target[3:]) - 1) > 1e-5 or point not in ("flange", "tool"):
            raise ValueError("unit wxyz and flange/tool point required")
        desired = Rotation.from_quat(target[[4, 5, 6, 3]]).as_matrix()
        scratch = self.mj.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        site = self._tool if point == "tool" else self._flange
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        for _ in range(150):
            self.mj.mj_forward(self.model, scratch)
            current = scratch.site_xmat[site].reshape(3, 3)
            error = np.r_[target[:3] - scratch.site_xpos[site], Rotation.from_matrix(desired @ current.T).as_rotvec()]
            if np.linalg.norm(error[:3]) < 2e-4 and np.linalg.norm(error[3:]) < .002:
                return scratch.qpos[self._qadr[:6]].copy()
            self.mj.mj_jacSite(self.model, scratch, jp, jr, site)
            jac = np.vstack([jp, jr])[:, self._vadr[:6]]
            step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), error)
            scratch.qpos[self._qadr[:6]] = np.clip(scratch.qpos[self._qadr[:6]] + np.clip(step, -.1, .1), self._lower[:6], self._upper[:6])
        raise ValueError("IK did not converge; goal not sent")

    def set_position(self, position):
        self.set_joint(self.solve_ik(position))

    def servo_step(self, dt):
        if self._closed or self.fault:
            return
        dt = float(dt)
        if not math.isfinite(dt) or not 0 < dt <= .2:
            raise ValueError("simulation advance must be in (0, .2] seconds")
        self._remainder += dt
        step = self.model.opt.timestep
        while self._remainder >= step - 1e-12:
            delta = self._goal - self.data.ctrl[self._act]
            speed = np.array([1.] * 6 + [.08, .08])
            self.data.ctrl[self._act] += np.clip(delta, -speed * step, speed * step)
            # Compensate arm gravity; contacts and actuator limits remain physical.
            self.data.qfrc_applied[:] = 0
            self.data.qfrc_applied[self._vadr[:6]] = self.data.qfrc_bias[self._vadr[:6]]
            self.mj.mj_step(self.model, self.data)
            self._remainder -= step
            if not np.isfinite(self.data.qpos).all() or np.max(np.abs(self.data.qvel[self._vadr[:6]])) > 5:
                self.fault = "nonfinite state or arm velocity exceeded 5 rad/s"
                break
        self.mj.mj_forward(self.model, self.data)
        if self.data.time + 1e-9 >= self._next_camera:
            if self.renderer is not None:
                self._capture()
            if self.rr is not None:
                self._log_rerun()
            self._next_camera = self.data.time + 1 / self.camera_hz

    def _capture(self):
        self._frame_id += 1
        timestamp = time.time_ns()
        for name in ("head", "wrist"):
            cid = self.model.camera(name).id
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data, camera=name)
            rgb = self.renderer.render().copy()
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render().copy()
            self.renderer.disable_depth_rendering()
            fy = .5 * self.height / math.tan(math.radians(self.model.cam_fovy[cid]) / 2)
            camera_rotation = self.data.cam_xmat[cid].reshape(3, 3) @ np.diag([1, -1, -1])
            self._cameras[name] = {"rgb": rgb, "depth_m": depth, "frame_id": self._frame_id,
                "timestamp_ns": timestamp, "sim_time": float(self.data.time), "optical_axes": "RDF",
                "intrinsics": np.array([[fy, 0, self.width / 2], [0, fy, self.height / 2], [0, 0, 1]]),
                "world_from_camera": pose(self.data.cam_xpos[cid], camera_rotation)}

    def _start_rerun(self, port=9876):
        import rerun as rr
        import rerun.blueprint as rrb
        rr.init("piper_mujoco_harness", spawn=False)
        if self.spawn:
            # Use the viewer bundled with this SDK, not a different Conda PATH version.
            import sys
            import rerun_cli
            cli_root = Path(rerun_cli.__file__).parent
            executable = cli_root / "Rerun.app/Contents/MacOS/Rerun" if sys.platform == "darwin" else cli_root / "rerun"
            if not executable.is_file():
                executable = cli_root / ("rerun.exe" if sys.platform == "win32" else "rerun")
            rr.spawn(port=port, memory_limit="1GiB", executable_path=str(executable))
        if self.record_path:
            rr.save(str(self.record_path))
        self.rr = rr
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        for gid in range(self.model.ngeom):
            if self.model.geom_group[gid] == 3:
                continue
            path = f"world/geoms/{self.model.geom(gid).name}"
            mesh = int(self.model.geom_dataid[gid])
            color = self.model.geom_rgba[gid]
            if self.model.geom_type[gid] == self.mj.mjtGeom.mjGEOM_MESH:
                va, vn = self.model.mesh_vertadr[mesh], self.model.mesh_vertnum[mesh]
                fa, fn = self.model.mesh_faceadr[mesh], self.model.mesh_facenum[mesh]
                rr.log(path, rr.Mesh3D(vertex_positions=self.model.mesh_vert[va:va+vn],
                    triangle_indices=self.model.mesh_face[fa:fa+fn], albedo_factor=color), static=True)
            elif self.model.geom_type[gid] == self.mj.mjtGeom.mjGEOM_BOX:
                rr.log(path, rr.Boxes3D(half_sizes=self.model.geom_size[gid], colors=color), static=True)
        rr.send_blueprint(rrb.Vertical(rrb.Spatial3DView(origin="world"), rrb.Horizontal(
            rrb.Spatial2DView(origin="world/cameras/head"), rrb.Spatial2DView(origin="world/cameras/wrist"))))

    def _log_rerun(self):
        rr = self.rr
        rr.set_time("sim_time", duration=float(self.data.time))
        for gid in range(self.model.ngeom):
            if self.model.geom_group[gid] == 3:
                continue
            rr.log(f"world/geoms/{self.model.geom(gid).name}", rr.Transform3D(
                translation=self.data.geom_xpos[gid], mat3x3=self.data.geom_xmat[gid].reshape(3, 3)))
        for name, camera in self._cameras.items():
            path = f"world/cameras/{name}"
            p = camera["world_from_camera"]
            rr.log(path, rr.Transform3D(translation=p[:3], quaternion=rr.Quaternion(xyzw=p[[4, 5, 6, 3]])))
            rr.log(path, rr.Pinhole(image_from_camera=camera["intrinsics"], resolution=[self.width, self.height]))
            rr.log(path + "/rgb", rr.Image(camera["rgb"]))
            rr.log(path + "/depth", rr.DepthImage(camera["depth_m"], meter=1))

    def stop(self):
        if self.data is not None:
            self.data.ctrl[self._act] = self.data.qpos[self._qadr]
        self._closed = True
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.rr is not None:
            self.rr.disconnect()
