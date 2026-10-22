"""关节 MIT 的重力和阻抗。只算指令，不积分。

能发关节 MIT 的控制器继承 `MitServo`。力矩是 URDF 的 g(q) 加上法兰负载，真机再乘 `torque_scale`。
弹簧和阻尼留在 kp、kd 里，不叠进前馈。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot.controller.payload_gravity import payload_torque
from robot.controller.piper_gravity_config import compute_piper_teleop_torque_terms

DEFAULT_MIT_KD = (0.0, 0.0, 0.0, 0.01, 0.008, 0.006)


class ArmServoLaw:
    """一份 URDF 上的补偿力矩，以及要下发给驱动的 MIT 五项。"""

    def __init__(
        self,
        urdf_path,
        *,
        dofs: int = 6,
        torque_scale=None,
        torque_limit: float = 0.0,
        mit_kd=None,
        joint_K=None,
        joint_D=None,
        apply_scale: bool = True,
    ):
        import pinocchio as pin

        self.urdf_path = str(urdf_path)
        self.dofs = int(dofs)
        self.apply_scale = bool(apply_scale)
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        self._flange_id = _flange_frame(self.model)
        self.torque_scale = np.ones(self.dofs) if torque_scale is None else _column(torque_scale, self.dofs, "torque_scale")
        self.torque_limit = float(torque_limit)
        self.mit_kd = _column(DEFAULT_MIT_KD if mit_kd is None else mit_kd, self.dofs, "mit_kd")
        self.joint_K = None if joint_K is None else _column(joint_K, self.dofs, "joint_K")
        self.joint_D = None if joint_D is None else _column(joint_D, self.dofs, "joint_D")
        self.mass = 0.0
        self.com = np.zeros(3)

    def set_payload(self, mass, com) -> None:
        self.mass = float(mass)
        self.com = np.asarray(com, dtype=float).reshape(3).copy()

    def set_gains(self, joint_K, joint_D) -> None:
        self.joint_K = _column(joint_K, self.dofs, "joint_K")
        self.joint_D = _column(joint_D, self.dofs, "joint_D")

    def require_gains(self) -> None:
        if self.joint_K is None or self.joint_D is None:
            raise ValueError("阻抗需要配置里的 joint_K 和 joint_D")

    def link_gravity(self, q) -> np.ndarray:
        import pinocchio as pin

        configuration = self._sync(q)
        torque = pin.rnea(
            self.model,
            self.data,
            configuration,
            np.zeros(self.model.nv),
            np.zeros(self.model.nv),
        )
        return np.array(torque[: self.dofs], dtype=float)

    def compensation(self, q) -> np.ndarray:
        """电机前馈。仿真不乘 scale。"""
        configuration = self._sync(q)
        raw = self.link_gravity(configuration) + self._payload(configuration, self.mass, self.com)
        if not self.apply_scale:
            return raw
        limited = compute_piper_teleop_torque_terms(
            raw,
            _Scale(self.torque_scale, self.torque_limit),
        )
        return np.array(limited["gravity_ff"], dtype=float)

    def gravity_terms(self, q) -> dict:
        joint = _column(q, self.dofs, "joint")
        return {
            "q_des": joint.copy(),
            "qd_des": np.zeros(self.dofs),
            "kp": np.zeros(self.dofs),
            "kd": self.mit_kd.copy(),
            "tau_ff": self.compensation(joint),
        }

    def impedance_terms(self, q, q_des) -> dict:
        self.require_gains()
        joint = _column(q, self.dofs, "joint")
        return {
            "q_des": _column(q_des, self.dofs, "q_des"),
            "qd_des": np.zeros(self.dofs),
            "kp": self.joint_K.copy(),
            "kd": self.joint_D.copy(),
            "tau_ff": self.compensation(joint),
        }

    def hold(self, q, measured) -> dict:
        """测到的力矩换回 URDF 的单位，再和连杆 g(q) 一起交给辨识。"""
        configuration = self._sync(q)
        link = self.link_gravity(configuration)
        torque = _column(measured, self.dofs, "torque")
        if self.apply_scale:
            scale = np.where(np.abs(self.torque_scale) < 1e-9, 1.0, self.torque_scale)
            torque = torque / scale
        pose = self._pose(configuration)
        jacobian = self._jacobian(configuration)
        return {
            "kind": "hold_torque",
            "joint": np.array(configuration[: self.dofs], dtype=float).tolist(),
            "torque": torque.tolist(),
            "link_gravity": link.tolist(),
            "jacobian": jacobian.reshape(-1).tolist(),
            "rotation": pose[:3, :3].reshape(-1).tolist(),
        }

    def _payload(self, configuration, mass, com) -> np.ndarray:
        if abs(float(mass)) < 1e-12:
            return np.zeros(self.dofs)
        pose = self._pose(configuration)
        return payload_torque(self._jacobian(configuration), pose[:3, :3], mass, com)

    def _jacobian(self, configuration):
        import pinocchio as pin

        jacobian = pin.computeFrameJacobian(
            self.model,
            self.data,
            configuration,
            self._flange_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        return np.array(jacobian[:, : self.dofs], dtype=float)

    def _pose(self, configuration) -> np.ndarray:
        import pinocchio as pin

        pin.forwardKinematics(self.model, self.data, configuration)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self._flange_id]
        pose = np.eye(4)
        pose[:3, :3] = np.array(placement.rotation, dtype=float)
        pose[:3, 3] = np.array(placement.translation, dtype=float)
        return pose

    def _sync(self, q) -> np.ndarray:
        values = np.asarray(q, dtype=float).reshape(-1)
        configuration = np.zeros(self.model.nq)
        count = min(values.size, self.model.nq)
        configuration[:count] = values[:count]
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])
        return configuration


class _Scale:
    def __init__(self, torque_scale, torque_limit):
        self.torque_scale = np.asarray(torque_scale, dtype=float)
        self.torque_limit = float(torque_limit)


class MitServo:
    """子类实现 `_send_mit_refresh`。这个方法不能改写对外的 `control_mode`。"""

    def bind_mit_servo(self, law: ArmServoLaw) -> None:
        self._mit_servo = law
        self._servo_q_des = None

    def mit_servo_ready(self) -> bool:
        return getattr(self, "_mit_servo", None) is not None

    def require_mit_servo(self) -> ArmServoLaw:
        law = getattr(self, "_mit_servo", None)
        if law is None:
            raise ValueError(f"{getattr(self, 'name', 'arm')} 没有 URDF，不能做重力补偿或阻抗")
        return law

    def note_joint_target(self, joint) -> None:
        self._servo_q_des = np.asarray(joint, dtype=float).reshape(-1).copy()

    def set_identified_payload(self, mass, com) -> None:
        self.require_mit_servo().set_payload(mass, com)

    def servo_terms(self, joint=None) -> dict | None:
        law = getattr(self, "_mit_servo", None)
        mode = getattr(self, "control_mode", "position")
        if law is None or mode not in ("gravity_compensation", "impedance"):
            return None
        if joint is None:
            joint = self.get_state()["joint"]
        q = np.asarray(joint, dtype=float).reshape(-1)
        if mode == "impedance":
            target = getattr(self, "_servo_q_des", None)
            if target is None:
                target = q
            return law.impedance_terms(q, target)
        return law.gravity_terms(q)

    def servo_step(self, dt: float) -> None:
        del dt
        terms = self.servo_terms()
        if terms is None:
            return
        self._send_mit_refresh(terms)

    def dispatch_servo_op(self, data: dict) -> bool:
        op = str(data.get("op") or "")
        if op == "set_payload":
            self.set_identified_payload(data["mass"], data["com"])
            law = self.require_mit_servo()
            self._last_result = {"kind": "set_payload", "mass": law.mass, "com": law.com.tolist()}
            return True
        if op == "hold_torque":
            law = self.require_mit_servo()
            state = self.get_state()
            measured = state.get("force")
            if measured is None:
                raise RuntimeError(f"{self.name} 这一拍没有关节力矩")
            self._last_result = law.hold(state["joint"], measured)
            return True
        if op == "impedance":
            law = self.require_mit_servo()
            if "stiffness" in data or "damping" in data:
                current_k = law.joint_K if law.joint_K is not None else np.zeros(law.dofs)
                current_d = law.joint_D if law.joint_D is not None else np.zeros(law.dofs)
                law.set_gains(data.get("stiffness", current_k), data.get("damping", current_d))
            if "q_des" in data:
                self.note_joint_target(data["q_des"])
            if self.control_mode == "impedance":
                terms = law.impedance_terms(self.get_state()["joint"], self._servo_q_des)
            else:
                terms = None
            self._last_result = {
                "kind": "impedance",
                "kp": None if law.joint_K is None else law.joint_K.tolist(),
                "kd": None if law.joint_D is None else law.joint_D.tolist(),
                "tau_ff": None if terms is None else terms["tau_ff"].tolist(),
            }
            return True
        return False

    def _send_mit_refresh(self, terms: dict) -> None:
        raise NotImplementedError(f"{getattr(self, 'name', 'arm')} 没有 MIT 下发")


def _flange_frame(model) -> int:
    if model.existFrame("flange"):
        return int(model.getFrameId("flange"))
    if model.existFrame("link6"):
        return int(model.getFrameId("link6"))
    raise ValueError(f"URDF 里没有 flange 或 link6：{model.name}")


def _column(value, dofs: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size == 1:
        return np.repeat(float(array[0]), dofs)
    if array.size != dofs:
        raise ValueError(f"{name} 需要 {dofs} 个数，得到 {array.size}")
    return array.copy()


def law_from_piper_profile(urdf_path, config, *, joint_K=None, joint_D=None, apply_scale: bool = True) -> ArmServoLaw:
    return ArmServoLaw(
        Path(urdf_path),
        torque_scale=config.torque_scale,
        torque_limit=config.torque_limit,
        mit_kd=config.mit_kd,
        joint_K=joint_K,
        joint_D=joint_D,
        apply_scale=apply_scale,
    )
