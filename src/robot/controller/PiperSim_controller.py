"""单臂 Piper 仿真控制器。

模型来自仓库里的 assets/sim/piper/piper_single.urdf（RoboTwin 的 Piper，只留一条臂和夹爪）。
运动学和重力用 Pinocchio，画面用 Rerun。`move` 直接改关节；`step_torque` 按 URDF 惯量积分，用来看力矩命令会把臂带到哪里。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot.controller.arm_controller import ArmController
from robot.utils.base.data_handler import debug_print

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINTS = ("joint7", "joint8")
READY_JOINT = np.array([0.0, 1.3, -1.0, 0.0, 0.5, 0.0])


class SimProtection(RuntimeError):
    """仿真速度保护。真机上对应的动作要靠力矩限幅和急停停下来。"""


def default_urdf() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "assets" / "sim" / "piper" / "piper_single.urdf"
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("找不到 assets/sim/piper/piper_single.urdf")


class PiperSimController(ArmController):
    def __init__(self, name, urdf_path=None, spawn=False, record_path=None, INFO="DEBUG"):
        super().__init__()
        self.name = name
        self.controller_type = "user_controller"
        self.controller = None
        self.INFO = INFO
        self.DoFs = 6
        self.urdf_path = str(Path(urdf_path) if urdf_path else default_urdf())
        self.spawn = bool(spawn)
        self.record_path = None if record_path in (None, "") else str(record_path)
        self.model = None
        self.data = None
        self.q = None
        self.qd = None
        self.fault = ""
        self.sim_time = 0.0
        self.tool_offset = np.zeros(3)
        self._lower = None
        self._upper = None
        self._flange_id = None
        self._rr = None
        self._tree = None
        self._tick = 0
        self._last_result = {}

    def set_up(self, input=None):
        import pinocchio as pin

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self._lower = np.array(self.model.lowerPositionLimit, dtype=float)
        self._upper = np.array(self.model.upperPositionLimit, dtype=float)
        self.q = pin.neutral(self.model)
        self.q[:6] = np.clip(READY_JOINT, self._lower[:6], self._upper[:6])
        self.qd = np.zeros(self.model.nv)
        self.fault = ""
        self.sim_time = 0.0
        self._flange_id = self.model.getFrameId("flange")
        self._refresh()
        flange = self.data.oMf[self._flange_id]
        tool = self.data.oMf[self.model.getFrameId("tool")]
        self.tool_offset = flange.rotation.T @ (tool.translation - flange.translation)
        self._start_viewer()
        self._publish()
        debug_print(self.name, f"piper sim ready urdf={self.urdf_path}", self.INFO)
        if self.record_path:
            debug_print(self.name, f"rerun 录到 {self.record_path}", self.INFO)

    def get_state(self):
        self._require_model()
        opening = float(self.q[6] / max(self._upper[6], 1e-9))
        return {
            "joint": self.q[:6].copy(),
            "eef": self._pose_vector(self._flange_id),
            "gripper": np.array([opening]),
            "velocity": self.qd[:6].copy(),
        }

    def set_joint(self, joint):
        self._require_model()
        target = np.asarray(joint, dtype=float).reshape(-1)
        if target.size != self.DoFs:
            debug_print(self.name, f"set_joint() input size should be {self.DoFs}", "ERROR")
            return
        self.q[:6] = np.clip(target, self._lower[:6], self._upper[:6])
        self.qd[:] = 0.0
        self.fault = ""
        self._publish()

    def set_position(self, position):
        self._require_model()
        position = np.asarray(position, dtype=float).reshape(-1)
        if position.size == 7:
            rotation = _rotation_from_wxyz(position[3:])
        elif position.size == 6:
            rotation = _rotation_from_rpy(position[3:])
        else:
            debug_print(self.name, "set_position input size should be 6 -> EULER or 7 -> QUATERNION", "ERROR")
            return
        self._ik(rotation, position[:3])

    def set_gripper(self, gripper):
        self._require_model()
        value = float(np.asarray(gripper, dtype=float).reshape(-1)[0])
        if not 0.0 <= value <= 1.0:
            debug_print(self.name, f"gripper better be 0~1, but get number {value}", "WARNING")
        opening = float(np.clip(value, 0.0, 1.0) * self._upper[6])
        self.q[6] = opening
        self.q[7] = opening
        self._publish()

    def flange_pose(self):
        self._require_model()
        self._refresh()
        return _matrix(self.data.oMf[self._flange_id])

    def tool_point(self):
        pose = self.flange_pose()
        return pose[:3, 3] + pose[:3, :3] @ self.tool_offset

    def set_tool_offset(self, offset):
        """工具原点在法兰系下的平移，单位米。只影响计算，不改夹爪网格。"""
        self.tool_offset = np.asarray(offset, dtype=float).reshape(3)

    def link_inertias(self):
        """URDF 写进模型的质量和质心。质心在该连杆的关节坐标系，单位米。

        重力方向不在 URDF 里，这里固定成基座 z 轴向上的 [0, 0, -9.81]。
        """
        self._require_model()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        rows = []
        for index, name in enumerate(self.model.names):
            inertia = self.model.inertias[index]
            link = "base_link" if index == 0 else str(name).replace("joint", "link")
            rows.append(
                {
                    "link": link,
                    "mass_kg": float(inertia.mass),
                    "com_m": np.array(inertia.lever, dtype=float).reshape(3).copy(),
                }
            )
        return rows

    def gravity_compensation(self):
        """抵消重力所需的电机力矩，单位 N·m。

        静止、速度和加速度都为 0 时，逆动力学 τ = g(q)。g(q) 用 URDF 的质量、质心和惯量。
        返回手臂 6 个关节，不含夹爪。
        """
        import pinocchio as pin

        self._require_model()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        torque = pin.rnea(
            self.model,
            self.data,
            self.q,
            np.zeros(self.model.nv),
            np.zeros(self.model.nv),
        )
        return np.array(torque[:6], dtype=float)

    def joint_impedance_torque(self, q_des, stiffness, damping, qd_des=None):
        """τ = K (q_d - q) + D (qd_d - qd) + g(q)。g(q) 来自 URDF。"""
        q_des = np.asarray(q_des, dtype=float).reshape(6)
        stiffness = np.asarray(stiffness, dtype=float).reshape(6)
        damping = np.asarray(damping, dtype=float).reshape(6)
        qd_des = np.zeros(6) if qd_des is None else np.asarray(qd_des, dtype=float).reshape(6)
        spring = stiffness * (q_des - self.q[:6])
        viscous = damping * (qd_des - self.qd[:6])
        gravity = self.gravity_compensation()
        return {
            "spring": spring,
            "damping": viscous,
            "gravity": gravity,
            "torque": spring + viscous + gravity,
        }

    def cartesian_impedance_torque(self, position_des, stiffness_p, rotation_des=None, stiffness_r=None):
        """法兰误差写成力，再经 URDF 的雅可比变到关节，并加上 g(q)。

        f = K_p (p_d - p)，n = K_r log(R_d R^T)。τ = J^T [f; n] + g(q)。
        """
        import pinocchio as pin

        position_des = np.asarray(position_des, dtype=float).reshape(3)
        stiffness_p = np.asarray(stiffness_p, dtype=float).reshape(3)
        pose = self.flange_pose()
        force = stiffness_p * (position_des - pose[:3, 3])
        if rotation_des is None:
            moment = np.zeros(3)
        else:
            gain = stiffness_p if stiffness_r is None else np.asarray(stiffness_r, dtype=float).reshape(3)
            moment = gain * pin.log3(np.asarray(rotation_des, dtype=float).reshape(3, 3) @ pose[:3, :3].T)
        wrench = np.concatenate([force, moment])
        spring = self.flange_jacobian().T @ wrench
        gravity = self.gravity_compensation()
        return {
            "wrench": wrench,
            "spring": spring,
            "gravity": gravity,
            "torque": spring + gravity,
        }

    def mit_torque(self, q_des, kp, kd, qd_des=None, tau_ff=None):
        """τ = kp (q_d - q) + kd (qd_d - qd) + τ_ff。不传 τ_ff 时用 URDF 的 g(q)。"""
        q_des = np.asarray(q_des, dtype=float).reshape(6)
        kp = np.asarray(kp, dtype=float).reshape(6)
        kd = np.asarray(kd, dtype=float).reshape(6)
        qd_des = np.zeros(6) if qd_des is None else np.asarray(qd_des, dtype=float).reshape(6)
        feedforward = self.gravity_compensation() if tau_ff is None else np.asarray(tau_ff, dtype=float).reshape(6)
        spring = kp * (q_des - self.q[:6])
        viscous = kd * (qd_des - self.qd[:6])
        return {
            "spring": spring,
            "damping": viscous,
            "feedforward": feedforward,
            "torque": spring + viscous + feedforward,
        }

    def flange_jacobian(self):
        """法兰在世界系下的 6×6 雅可比，前三行线速度，后三行角速度。"""
        import pinocchio as pin

        self._require_model()
        self._refresh()
        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            self.q,
            self._flange_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        return np.array(jacobian[:, :6], dtype=float)

    def move_tool_delta(self, delta_in_tool):
        """p' = p + R Δp，Δp 写在法兰系，姿态保持不变。"""
        pose = self.flange_pose()
        delta = np.asarray(delta_in_tool, dtype=float).reshape(3)
        self._ik(pose[:3, :3], pose[:3, 3] + pose[:3, :3] @ delta)
        return self.flange_pose()

    def touch_fixed_point(self, point, rotation):
        """工具原点放到 point，法兰姿态为 rotation（3×3）。"""
        rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        point = np.asarray(point, dtype=float).reshape(3)
        self._ik(rotation, point - rotation @ self.tool_offset)
        return self.tool_point()

    def step_torque(self, torque, dt, torque_limit=None, speed_limit=3.0):
        """用 Pinocchio 正动力学走一步。torque 是 6 个手臂关节的电机力矩。

        speed_limit 用 URDF 里的关节速度上限（rad/s）。超过以后速度清零并停止，避免仿真数值发散。
        真机没有这一步的话，同样的力矩会一直加速。
        """
        import pinocchio as pin

        self._require_model()
        if self.fault:
            raise SimProtection(self.fault)
        commanded = np.asarray(torque, dtype=float).reshape(-1)
        if commanded.size != self.DoFs:
            raise ValueError(f"torque 需要 {self.DoFs} 个数")
        full = np.zeros(self.model.nv)
        full[:6] = commanded
        if torque_limit is not None:
            limit = float(torque_limit)
            full[:6] = np.clip(full[:6], -limit, limit)
        # 力矩在这一拍里保持不变。积分用 1 ms，避免腕关节惯量很小时显式欧拉发散。
        # 这不等价于驱动器内部的电流环，只是让 200 Hz 的命令还能数值上站住。
        remaining = float(dt)
        while remaining > 1e-12:
            step = min(0.001, remaining)
            acceleration = np.array(pin.aba(self.model, self.data, self.q, self.qd, full), dtype=float)
            self.qd = self.qd + acceleration * step
            peak = float(np.max(np.abs(self.qd[:6])))
            if speed_limit is not None and peak > float(speed_limit):
                self.qd[:] = 0.0
                self.fault = f"关节速度达到 {peak:.2f} rad/s，超过保护阈值 {speed_limit}，仿真已停止积分"
                self.sim_time += float(dt) - remaining
                self._publish()
                raise SimProtection(self.fault)
            self.q = pin.integrate(self.model, self.q, self.qd * step)
            self._clip_joints()
            remaining -= step
        self.sim_time += float(dt)
        self._publish()
        return self.q[:6].copy()

    def _ik(self, rotation, translation, iterations=40):
        import pinocchio as pin

        target = np.asarray(translation, dtype=float).reshape(3)
        desired = np.asarray(rotation, dtype=float).reshape(3, 3)
        for _ in range(iterations):
            self._refresh()
            current = self.data.oMf[self._flange_id]
            error_position = target - current.translation
            error_rotation = pin.log3(desired @ current.rotation.T)
            error = np.concatenate([error_position, error_rotation])
            if float(np.linalg.norm(error)) < 1e-4:
                self.qd[:] = 0.0
                self.fault = ""
                self._publish()
                return
            jacobian = pin.computeFrameJacobian(
                self.model,
                self.data,
                self.q,
                self._flange_id,
                pin.LOCAL_WORLD_ALIGNED,
            )[:, :6]
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-4 * np.eye(6), error)
            self.q[:6] = self.q[:6] + step
            self._clip_joints()
        self._refresh()
        current = self.data.oMf[self._flange_id]
        position_error = float(np.linalg.norm(target - current.translation))
        rotation_error = float(np.linalg.norm(pin.log3(desired @ current.rotation.T)))
        raise RuntimeError(
            f"{self.name}: 逆解没收敛，位置残差 {position_error:.4f} m，姿态残差 {rotation_error:.4f} rad"
        )

    def _clip_joints(self):
        clipped = np.clip(self.q, self._lower, self._upper)
        hit = np.abs(clipped - self.q) > 1e-12
        self.qd[hit] = 0.0
        self.q = clipped

    def _refresh(self):
        import pinocchio as pin

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

    def _pose_vector(self, frame_id):
        self._refresh()
        placement = self.data.oMf[frame_id]
        return np.concatenate([placement.translation, _wxyz(placement.rotation)])

    def _publish(self):
        self._refresh()
        self._tick += 1
        if self._rr is None:
            return
        self._rr.set_time("sim_time", duration=self.sim_time)
        self._rr.set_time("step", sequence=self._tick)
        for index, name in enumerate(ARM_JOINTS + GRIPPER_JOINTS):
            joint = self._tree.get_joint_by_name(name)
            self._rr.log(f"tf/{self.name}/{name}", joint.compute_transform(float(self.q[index])))

    def _start_viewer(self):
        import rerun as rr

        rr.init(f"piper_sim_{self.name}", spawn=False)
        if self.record_path:
            Path(self.record_path).parent.mkdir(parents=True, exist_ok=True)
            rr.save(self.record_path)
        self._tree = rr.urdf.UrdfTree.from_file_path(self.urdf_path, frame_prefix=self.name)
        self._tree.log_urdf_to_recording()
        if self.spawn:
            self._open_window(rr)
        self._rr = rr

    def _open_window(self, rr) -> None:
        import os
        import shutil
        import sys

        _attach_desktop()
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            debug_print(self.name, "当前没有显示器，Rerun 窗口没有打开。在图形终端里重新 cyr start", "WARNING")
            return
        viewer = Path(sys.executable).with_name("rerun")
        if not viewer.is_file():
            found = shutil.which("rerun")
            viewer = Path(found) if found else None
        if viewer is None:
            debug_print(self.name, "找不到 rerun 可执行文件，窗口没有打开", "WARNING")
            return
        try:
            rr.spawn(executable_path=str(viewer))
        except Exception as exc:
            debug_print(self.name, f"Rerun 窗口没有打开: {exc}", "WARNING")

    def get_information(self):
        info = super().get_information()
        if self.collect_info and "result" in self.collect_info:
            info["result"] = self._last_result
        return info

    def handle_runtime_op(self, data: dict) -> None:
        """cyr 转发过来的计算和运动。结果放进下一次 get() 的 result。

        逆解失败只记在 result 里。这条仿真用来讲解，一次没收敛不应该把窗口锁死。
        """
        try:
            self._dispatch_runtime_op(data)
        except RuntimeError as exc:
            self._last_result = {"kind": str(data.get("op") or ""), "error": str(exc)}

    def _dispatch_runtime_op(self, data: dict) -> None:
        op = str(data.get("op") or "")
        if op == "pose":
            self._remember_pose()
            return
        if op == "gravity":
            gravity = self.gravity_compensation()
            self._last_result = {
                "kind": "gravity",
                "links": [
                    {"link": row["link"], "mass_kg": row["mass_kg"], "com_m": _tolist(row["com_m"])}
                    for row in self.link_inertias()
                ],
                "torque": _tolist(gravity),
                "missing": _tolist(-gravity),
            }
            return
        if op == "impedance":
            current = self.q[:6]
            q_des = np.asarray(data.get("q_des", current), dtype=float).reshape(6)
            stiffness = np.asarray(data.get("stiffness", [16, 16, 16, 8, 8, 8]), dtype=float).reshape(6)
            damping = np.asarray(data.get("damping", [1.8] * 6), dtype=float).reshape(6)
            parts = self.joint_impedance_torque(q_des, stiffness, damping)
            self._last_result = {"kind": "impedance", **{key: _tolist(value) for key, value in parts.items()}}
            return
        if op == "cartesian":
            position = np.asarray(data["position_des"], dtype=float).reshape(3)
            stiffness = np.asarray(data.get("stiffness_p", [100.0, 100.0, 100.0]), dtype=float).reshape(3)
            parts = self.cartesian_impedance_torque(position, stiffness)
            self._last_result = {"kind": "cartesian", **{key: _tolist(value) for key, value in parts.items()}}
            return
        if op == "mit":
            q_des = np.asarray(data.get("q_des", np.zeros(6)), dtype=float).reshape(6)
            kp = np.asarray(data.get("kp", np.zeros(6)), dtype=float).reshape(6)
            kd = np.asarray(data.get("kd", np.zeros(6)), dtype=float).reshape(6)
            parts = self.mit_torque(q_des, kp, kd)
            self._last_result = {"kind": "mit", **{key: _tolist(value) for key, value in parts.items()}}
            return
        if op == "tool_delta":
            before = self.flange_pose()
            delta = np.asarray(data["delta"], dtype=float).reshape(3)
            after = self.move_tool_delta(delta)
            expected = before[:3, 3] + before[:3, :3] @ delta
            self._last_result = {
                "kind": "tool_delta",
                "before": _tolist(before[:3, 3]),
                "after": _tolist(after[:3, 3]),
                "expected": _tolist(expected),
            }
            return
        if op == "touch":
            point = np.asarray(data["point"], dtype=float).reshape(3)
            rotation = np.asarray(data["rotation"], dtype=float).reshape(3, 3)
            self.touch_fixed_point(point, rotation)
            self._remember_pose()
            self._last_result["kind"] = "touch"
            return
        raise ValueError(f"未知操作 {op}")

    def _remember_pose(self) -> None:
        pose = self.flange_pose()
        self._last_result = {
            "kind": "pose",
            "joint": _tolist(self.q[:6]),
            "flange_translation": _tolist(pose[:3, 3]),
            "flange_rotation": _tolist(pose[:3, :3]),
            "tool_point": _tolist(self.tool_point()),
            "tool_offset": _tolist(self.tool_offset),
        }

    def _require_model(self):
        if self.model is None:
            raise RuntimeError(f"{self.name}: 先 set_up")


def _attach_desktop() -> None:
    """当前 shell 没有显示器时，接到已经登录的桌面。不用事先 export。"""
    import os
    from pathlib import Path

    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
    runtime_path = Path(runtime)
    if not runtime_path.is_dir():
        return
    os.environ.setdefault("XDG_RUNTIME_DIR", runtime)
    if (runtime_path / "wayland-0").exists():
        os.environ["WAYLAND_DISPLAY"] = "wayland-0"
    if (Path("/tmp/.X11-unix") / "X0").exists():
        os.environ.setdefault("DISPLAY", ":0")
    if os.environ.get("DISPLAY") and not os.environ.get("XAUTHORITY"):
        matches = sorted(runtime_path.glob(".mutter-Xwaylandauth.*"))
        if matches:
            os.environ["XAUTHORITY"] = str(matches[-1])


def _tolist(value) -> list:
    array = np.asarray(value, dtype=float)
    return array.tolist()


def _matrix(placement) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = placement.rotation
    pose[:3, 3] = placement.translation
    return pose


def _wxyz(rotation) -> np.ndarray:
    import pinocchio as pin

    quaternion = pin.Quaternion(np.asarray(rotation, dtype=float))
    xyzw = np.array(quaternion.coeffs(), dtype=float).reshape(4)
    return np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])


def _rotation_from_wxyz(quaternion) -> np.ndarray:
    import pinocchio as pin

    quaternion = np.asarray(quaternion, dtype=float).reshape(4)
    quaternion = quaternion / np.linalg.norm(quaternion)
    # Pinocchio 的四元数构造顺序是 w, x, y, z。
    return pin.Quaternion(
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    ).matrix()


def _rotation_from_rpy(rpy) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler("xyz", np.asarray(rpy, dtype=float).reshape(3)).as_matrix()
