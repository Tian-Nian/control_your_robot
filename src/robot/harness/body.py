"""只读身体模型。限位和 home 来自描述文件，位姿和间隙来自 URDF。"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

_MESH_PREFIX = "package://Marvin M6-S-CCS-696-V4.0_Base_and_Stand_Asm urdf/meshes/"


@dataclass(frozen=True)
class ArmSpec:
    name: str
    joints: tuple[str, ...]
    lower: np.ndarray
    upper: np.ndarray
    home: np.ndarray
    base_frame: str
    flange_frame: str
    palm_frame: str
    unified_from_base: np.ndarray


class BodyModel:
    """一台机器人的关节限位、home、法兰、掌心和自碰撞间隙。"""

    _loaded: dict[str, "BodyModel"] = {}

    def __init__(self, name: str, arms: dict[str, ArmSpec], urdf_path: str = "", collision: tuple[tuple[str, str], ...] = ()):
        self.name = name
        self.arms = arms
        self.urdf_path = urdf_path
        self.collision = collision
        self._kin = None

    @classmethod
    def load(cls, path: str | Path) -> BodyModel:
        config_path = Path(path).resolve()
        cached = cls._loaded.get(str(config_path))
        if cached is not None:
            return cached
        with config_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
        if not isinstance(raw, dict):
            raise ValueError(f"{config_path}: 身体描述必须是映射")
        urdf = _resolve(config_path.parent, str(raw.get("urdf") or ""))
        limits = _urdf_limits(urdf) if urdf else {}
        arms = {}
        for name, item in (raw.get("arms") or {}).items():
            if not isinstance(item, dict):
                raise ValueError(f"{config_path}: arms.{name} 必须是映射")
            joints = tuple(item.get("joints") or [])
            if not joints:
                raise ValueError(f"{config_path}: arms.{name} 缺少 joints")
            lower, upper = _limits_for(joints, item, limits)
            home = np.asarray(item.get("home") or [], dtype=float).reshape(-1)
            if home.size != len(joints):
                raise ValueError(f"{config_path}: arms.{name} home 长度应为 {len(joints)}")
            unified = np.asarray(item.get("unified_from_base"), dtype=float).reshape(3, 3)
            arms[str(name)] = ArmSpec(
                name=str(name),
                joints=joints,
                lower=lower,
                upper=upper,
                home=home,
                base_frame=str(item.get("base_frame") or ""),
                flange_frame=str(item.get("flange_frame") or ""),
                palm_frame=str(item.get("palm_frame") or ""),
                unified_from_base=unified,
            )
        if not arms:
            raise ValueError(f"{config_path}: 至少要有一条臂")
        pairs = tuple(tuple(pair) for pair in (raw.get("collision") or []))
        model = cls(str(raw.get("name") or config_path.stem), arms, urdf, pairs)
        cls._loaded[str(config_path)] = model
        return model

    def arm_names(self) -> list[str]:
        return list(self.arms)

    def limits(self, arm: str) -> tuple[np.ndarray, np.ndarray]:
        spec = self._arm(arm)
        return spec.lower.copy(), spec.upper.copy()

    def home(self, arm: str) -> np.ndarray:
        return self._arm(arm).home.copy()

    def flange_pose(self, arm: str, joints) -> np.ndarray:
        return self._kinematics().pose(self._arm(arm), joints, palm=False)

    def palm_pose(self, arm: str, joints) -> np.ndarray:
        return self._kinematics().pose(self._arm(arm), joints, palm=True)

    def flange_to_palm(self, arm: str) -> np.ndarray:
        """掌心相对法兰的 xyz（米），在法兰坐标系下。"""
        return self._kinematics().flange_to_palm(self._arm(arm))

    def clearance(self, arm: str, joints) -> float:
        spec = self._arm(arm)
        others = {name: self.arms[name].home for name in self.arms if name != arm}
        return self._kinematics().clearance(spec, joints, others)

    def ik(self, arm: str, joints, eef, *, max_error_m: float = 0.008) -> np.ndarray:
        from robot.harness.gate import CommandRejected

        spec = self._arm(arm)
        try:
            target = np.asarray(eef, dtype=float).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("末端目标必须是数字") from exc
        if target.size < 3:
            raise CommandRejected("末端目标至少要有 xyz")
        solved = self._kinematics().ik(spec, joints, target[:3])
        reached = self.flange_pose(arm, solved)
        error = float(np.linalg.norm(reached[:3] - target[:3]))
        if error > float(max_error_m):
            raise CommandRejected(f"FK 误差 {error * 1000:.1f} mm")
        return solved

    def _arm(self, arm: str) -> ArmSpec:
        try:
            return self.arms[arm]
        except KeyError as exc:
            raise KeyError(f"没有这条臂 {arm!r}") from exc

    def _kinematics(self):
        if self._kin is None:
            self._kin = _PinocchioBody(self)
        return self._kin


def _resolve(base: Path, value: str) -> str:
    if not value:
        return ""
    path = Path(value)
    if not path.is_absolute():
        path = (base / path).resolve()
    return str(path)


def _limits_for(joints: tuple[str, ...], item: dict, parsed: dict) -> tuple[np.ndarray, np.ndarray]:
    if "lower" in item and "upper" in item:
        lower = np.asarray(item["lower"], dtype=float).reshape(-1)
        upper = np.asarray(item["upper"], dtype=float).reshape(-1)
    else:
        lower = np.array([parsed[name][0] for name in joints], dtype=float)
        upper = np.array([parsed[name][1] for name in joints], dtype=float)
    if lower.size != len(joints) or upper.size != len(joints):
        raise ValueError(f"限位长度应为 {len(joints)}")
    return lower, upper


def _urdf_limits(path: str) -> dict[str, tuple[float, float]]:
    root = ET.parse(path).getroot()
    limits = {}
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        limit = joint.find("limit")
        if not name or limit is None or joint.get("type") != "revolute":
            continue
        limits[name] = (float(limit.get("lower")), float(limit.get("upper")))
    return limits


class _PinocchioBody:
    def __init__(self, body: BodyModel):
        if not body.urdf_path:
            raise RuntimeError(f"{body.name} 没有 URDF，不能算位姿")
        import pinocchio as pin

        self._pin = pin
        prepared = _rewrite_meshes(body.urdf_path)
        self.model = pin.buildModelFromUrdf(prepared)
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(self.model, prepared, pin.GeometryType.COLLISION)
        self._pairs = []
        names = [self.geom.geometryObjects[index].name for index in range(self.geom.ngeoms)]
        for first, second in body.collision:
            left = [index for index, name in enumerate(names) if first in name]
            right = [index for index, name in enumerate(names) if second in name]
            for a in left:
                for b in right:
                    if a == b:
                        continue
                    self.geom.addCollisionPair(pin.CollisionPair(a, b))
                    self._pairs.append((a, b))
        self.gdata = self.geom.createData()
        self._index = {}
        for arm in body.arms.values():
            self._index[arm.name] = np.array([self._joint_q(name) for name in arm.joints], dtype=int)
        self._palm_offset = {}

    def _joint_q(self, name: str) -> int:
        joint_id = self.model.getJointId(name)
        if joint_id >= self.model.njoints:
            raise KeyError(f"URDF 里没有关节 {name}")
        return int(self.model.joints[joint_id].idx_q)

    def _pack(self, spec: ArmSpec, joints, others: dict[str, np.ndarray] | None = None) -> np.ndarray:
        configuration = self._pin.neutral(self.model)
        configuration[self._index[spec.name]] = np.asarray(joints, dtype=float).reshape(-1)
        for name, value in (others or {}).items():
            configuration[self._index[name]] = np.asarray(value, dtype=float).reshape(-1)
        return configuration

    def pose(self, spec: ArmSpec, joints, *, palm: bool) -> np.ndarray:
        configuration = self._pack(spec, joints)
        self._pin.forwardKinematics(self.model, self.data, configuration)
        self._pin.updateFramePlacements(self.model, self.data)
        frame_name = spec.palm_frame if palm else spec.flange_frame
        frame = self.data.oMf[self.model.getFrameId(frame_name)]
        base = self.data.oMf[self.model.getFrameId(spec.base_frame)]
        rotation = spec.unified_from_base @ (base.rotation.T @ frame.rotation)
        translation = spec.unified_from_base @ (base.rotation.T @ (frame.translation - base.translation))
        quat = Rotation.from_matrix(rotation).as_quat()
        return np.array(
            [translation[0], translation[1], translation[2], quat[3], quat[0], quat[1], quat[2]],
            dtype=float,
        )

    def flange_to_palm(self, spec: ArmSpec) -> np.ndarray:
        cached = self._palm_offset.get(spec.name)
        if cached is not None:
            return cached.copy()
        configuration = self._pack(spec, spec.home)
        self._pin.forwardKinematics(self.model, self.data, configuration)
        self._pin.updateFramePlacements(self.model, self.data)
        flange = self.data.oMf[self.model.getFrameId(spec.flange_frame)]
        palm = self.data.oMf[self.model.getFrameId(spec.palm_frame)]
        offset = flange.rotation.T @ (palm.translation - flange.translation)
        self._palm_offset[spec.name] = np.asarray(offset, dtype=float)
        return self._palm_offset[spec.name].copy()

    def clearance(self, spec: ArmSpec, joints, others: dict[str, np.ndarray]) -> float:
        if not self._pairs:
            return float("inf")
        configuration = self._pack(spec, joints, others)
        self._pin.forwardKinematics(self.model, self.data, configuration)
        self._pin.updateGeometryPlacements(self.model, self.data, self.geom, self.gdata)
        self._pin.computeDistances(self.geom, self.gdata)
        return float(min(self.gdata.distanceResults[index].min_distance for index in range(len(self._pairs))))

    def ik(self, spec: ArmSpec, joints, target_xyz: np.ndarray) -> np.ndarray:
        pin = self._pin
        current = np.asarray(joints, dtype=float).reshape(-1).copy()
        low, high = spec.lower, spec.upper
        indices = self._index[spec.name]
        weight = np.ones(current.size, dtype=float)
        weight[0] = 20.0
        winverse = np.diag(1.0 / weight)
        for _ in range(25):
            configuration = self._pack(spec, current)
            pin.forwardKinematics(self.model, self.data, configuration)
            pin.computeJointJacobians(self.model, self.data, configuration)
            pin.updateFramePlacements(self.model, self.data)
            frame_id = self.model.getFrameId(spec.flange_frame)
            base_id = self.model.getFrameId(spec.base_frame)
            frame = self.data.oMf[frame_id]
            base = self.data.oMf[base_id]
            local = base.rotation.T @ (frame.translation - base.translation)
            unified = spec.unified_from_base @ local
            error = np.asarray(target_xyz, dtype=float).reshape(3) - unified
            if float(np.linalg.norm(error)) < 1e-3:
                break
            jacobian = pin.getFrameJacobian(self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
            linear = spec.unified_from_base @ (base.rotation.T @ jacobian[:, indices])
            weighted = linear @ winverse
            gram = weighted @ weighted.T + 1e-4 * np.eye(3)
            delta = winverse @ weighted.T @ np.linalg.solve(gram, error)
            delta = np.clip(delta, -0.15, 0.15)
            current = np.clip(current + delta, low, high)
        return current


def _rewrite_meshes(urdf_path: str) -> str:
    source = Path(urdf_path)
    text = source.read_text(encoding="utf-8")
    mesh_dir = source.parent.parent / "meshes"
    if _MESH_PREFIX in text and mesh_dir.is_dir():
        text = text.replace(_MESH_PREFIX, mesh_dir.as_posix().rstrip("/") + "/")
    destination = Path("/tmp") / f"cyr_{source.stem}.urdf"
    destination.write_text(text, encoding="utf-8")
    return str(destination)
