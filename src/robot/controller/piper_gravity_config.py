from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

import numpy as np


PIPER_JOINT_COUNT = 6
PIPER_GRAVITY_MIT_KD_ENV = "XONE_PIPER_GRAVITY_MIT_KD"
PIPER_GRAVITY_TORQUE_LIMIT_ENV = "XONE_PIPER_GRAVITY_TORQUE_LIMIT"
PIPER_GRAVITY_TORQUE_SCALE_ENV = "XONE_PIPER_GRAVITY_TORQUE_SCALE"


def normalize_piper_joint_vector(value, name, expected_len=PIPER_JOINT_COUNT):
    vector = np.asarray(value, dtype=float).ravel()
    if vector.size == 1:
        return np.repeat(vector[0], expected_len)
    if vector.size == expected_len:
        return vector
    if vector.size < expected_len:
        return np.pad(vector, (0, expected_len - vector.size), mode="edge")
    raise ValueError(f"{name} must contain {expected_len} values or one scalar, got {vector.size}.")


def parse_piper_joint_vector(value, default, name, expected_len=PIPER_JOINT_COUNT):
    if value is None:
        return normalize_piper_joint_vector(default, name, expected_len)
    if isinstance(value, str):
        if value == "":
            return normalize_piper_joint_vector(default, name, expected_len)
        value = [float(item.strip()) for item in value.split(",") if item.strip()]
    return normalize_piper_joint_vector(value, name, expected_len)


def normalize_piper_arm_type_name(arm_type):
    normalized = str(getattr(arm_type, "value", arm_type)).strip().lower()
    normalized = normalized.split(".")[-1]
    if normalized == "piper_x":
        return "piper_x"
    if normalized == "piper":
        return "piper"
    raise ValueError(f"Unsupported arm type: {arm_type}")


def infer_piper_urdf_profile(arm_type, urdf_path=None):
    arm_type_name = normalize_piper_arm_type_name(arm_type)
    if urdf_path is None:
        return f"{arm_type_name}_default"

    filename = Path(str(urdf_path)).name.lower()
    path_text = str(urdf_path).lower()
    has_teach = "teach" in filename or "teach" in path_text

    if arm_type_name == "piper_x":
        return "piper_x_teach_gripper" if has_teach else "piper_x_standard"
    return "piper_teach_gripper" if has_teach else "piper_standard"


@dataclass
class PiperTeleopGravityConfig:
    arm_type_name: str
    urdf_profile: str
    mit_kd: np.ndarray
    torque_scale: np.ndarray
    torque_limit: float = 0.0

    # 这里统一设置 硬件/URDF profile -> MIT 力矩通道修正 的现场经验。
    DEFAULT_MIT_KD: ClassVar[tuple[float, ...]] = (0.0, 0.0, 0.0, 0.01, 0.008, 0.006)
    TORQUE_SCALE_BY_PROFILE: ClassVar[dict[str, tuple[float, ...]]] = {
        # 普通 Piper 现场普通夹爪验证保留 joint5 经验幅值修正。
        "piper_default": (1.0, 1.0, 1.0, 1.0, 1.25, 1.0),
        "piper_standard": (1.0, 1.0, 1.0, 1.0, 1.25, 1.0),
        "piper_teach_gripper": (1.0, 1.0, 1.0, 1.0, 1.25, 1.0),
        # 2026-05-03 can0 PiperX 实机脉冲诊断显示 joint4/joint5 的 MIT 力矩正方向与 q 正方向相反。
        "piper_x_default": (1.0, 1.0, 1.1, -1.0, -1.0, 1.0),
        "piper_x_standard": (1.0, 1.0, 1.0, -1.0, -1.0, 1.0),
        # PiperX teach-gripper 实机调试额外保留 joint3 幅值修正，避免污染普通 PiperX profile。
        "piper_x_teach_gripper": (1.0, 1.0, 1.1, -1.0, -1.0, 1.0),
    }

    @classmethod
    def for_arm_urdf(
        cls,
        arm_type,
        urdf_path=None,
        *,
        mit_kd_override=None,
        torque_limit=None,
        torque_scale_override=None,
        env=None,
    ):
        env = {} if env is None else env
        arm_type_name = normalize_piper_arm_type_name(arm_type)
        urdf_profile = infer_piper_urdf_profile(arm_type_name, urdf_path)
        default_torque_scale = cls.TORQUE_SCALE_BY_PROFILE[urdf_profile]

        mit_kd = parse_piper_joint_vector(
            mit_kd_override if mit_kd_override is not None else env.get(PIPER_GRAVITY_MIT_KD_ENV),
            cls.DEFAULT_MIT_KD,
            PIPER_GRAVITY_MIT_KD_ENV,
        )
        scale_override = torque_scale_override if torque_scale_override is not None else env.get(PIPER_GRAVITY_TORQUE_SCALE_ENV)
        torque_scale = parse_piper_joint_vector(
            scale_override,
            default_torque_scale,
            PIPER_GRAVITY_TORQUE_SCALE_ENV,
        )
        if torque_limit is None:
            torque_limit = env.get(PIPER_GRAVITY_TORQUE_LIMIT_ENV, 0.0)

        return cls(
            arm_type_name=arm_type_name,
            urdf_profile=urdf_profile,
            mit_kd=mit_kd,
            torque_scale=torque_scale,
            torque_limit=float(torque_limit),
        )

    def with_overrides(self, *, mit_kd=None, torque_limit=None, torque_scale=None):
        return PiperTeleopGravityConfig(
            arm_type_name=self.arm_type_name,
            urdf_profile=self.urdf_profile,
            mit_kd=parse_piper_joint_vector(mit_kd, self.mit_kd, "mit_kd") if mit_kd is not None else self.mit_kd.copy(),
            torque_scale=parse_piper_joint_vector(torque_scale, self.torque_scale, "torque_scale") if torque_scale is not None else self.torque_scale.copy(),
            torque_limit=float(self.torque_limit if torque_limit is None else torque_limit),
        )


def compute_piper_teleop_torque_terms(gravity_torque, config):
    gravity_raw = np.asarray(gravity_torque, dtype=float)
    torque_scale = normalize_piper_joint_vector(config.torque_scale, "teleop_torque_scale", len(gravity_raw))
    gravity_ff = gravity_raw * torque_scale
    torque_limit = float(config.torque_limit)
    if torque_limit > 0:
        gravity_ff = np.clip(gravity_ff, -torque_limit, torque_limit)
    saturation = np.abs(gravity_raw * torque_scale) > torque_limit if torque_limit > 0 else np.zeros_like(gravity_raw, dtype=bool)

    return {
        "gravity_raw": gravity_raw,
        "gravity_ff": gravity_ff,
        "torque_scale": torque_scale,
        "torque_limit": torque_limit,
        "saturation": saturation,
    }
