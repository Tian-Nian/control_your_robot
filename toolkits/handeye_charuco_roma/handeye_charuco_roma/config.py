"""YAML for the roma capture front-end. Board spec matches handeye_charuco."""

from __future__ import annotations

import os
from copy import deepcopy
from pathlib import Path
from typing import Optional

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "configs" / "default.yml"
NIANTIAN_ROOT = PACKAGE_ROOT.parent
DEFAULT_ROMA_ROOT = NIANTIAN_ROOT / "roma"
DEFAULT_SDK_ROOT = NIANTIAN_ROOT / "X-Robot-Pipeline_new" / "third_party" / "TJ_FX_ROBOT_CONTRL_SDK"

SETUPS = ("eye_in_hand", "eye_to_hand")
WRIST_ROLE = {"left": "left_wrist", "right": "right_wrist"}
ARM_DEVICE = {"left": "left_arm", "right": "right_arm"}
CAM_DEVICE = {
    "head": "cam_head",
    "left_wrist": "cam_left",
    "right_wrist": "cam_right",
}


def _deep_update(base: dict, overlay: dict) -> dict:
    out = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_update(out[key], value)
        else:
            out[key] = deepcopy(value)
    return out


def load_yaml(path: Path | str) -> dict:
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a mapping")
    return data


def load_config(path: Optional[Path | str] = None, overlay: Optional[dict] = None) -> dict:
    cfg = load_yaml(path or DEFAULT_CONFIG_PATH)
    if overlay:
        cfg = _deep_update(cfg, overlay)
    return cfg


def resolve_setup(cfg: dict, setup: Optional[str] = None) -> str:
    value = (setup or cfg.get("setup") or "eye_to_hand").replace("-", "_")
    if value not in SETUPS:
        raise ValueError(f"setup must be one of {SETUPS}, got {value!r}")
    return value


def resolve_camera_role(cfg: dict, setup: Optional[str] = None, arm_side: Optional[str] = None) -> str:
    setup = resolve_setup(cfg, setup)
    side = arm_side or cfg.get("arm_side") or "right"
    role = cfg.get("camera_role") or "auto"
    if role not in (None, "", "auto"):
        return str(role)
    if setup == "eye_to_hand":
        return "head"
    if side not in WRIST_ROLE:
        raise ValueError(f"arm_side must be left/right, got {side!r}")
    return WRIST_ROLE[side]


def resolve_arm_device(cfg: dict, arm_side: Optional[str] = None) -> str:
    if cfg.get("arm_device"):
        return str(cfg["arm_device"])
    side = arm_side or cfg.get("arm_side") or "right"
    if side not in ARM_DEVICE:
        raise ValueError(f"arm_side must be left/right, got {side!r}")
    return ARM_DEVICE[side]


def resolve_camera_device(cfg: dict, camera_role: Optional[str] = None) -> str:
    raw = cfg.get("camera_device")
    if raw not in (None, "", "auto"):
        return str(raw)
    role = camera_role or resolve_camera_role(cfg)
    if role not in CAM_DEVICE:
        raise ValueError(f"unknown camera_role {role!r}")
    return CAM_DEVICE[role]


def resolve_roma_root(cfg: dict, *, explicit: Optional[Path | str] = None) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    raw = cfg.get("roma_root")
    if raw:
        p = Path(raw).expanduser()
        if not p.is_absolute():
            p = (PACKAGE_ROOT / p).resolve()
        return p
    return DEFAULT_ROMA_ROOT.resolve()


def resolve_sdk_root(cfg: dict, *, explicit: Optional[Path | str] = None) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    raw = cfg.get("sdk_root")
    if raw:
        p = Path(raw).expanduser()
        if not p.is_absolute():
            p = (PACKAGE_ROOT / p).resolve()
        return p
    candidates = []
    env = os.environ.get("MARVIN_SDK_ROOT")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.append(DEFAULT_SDK_ROOT)
    candidates.append(Path("/home/user/kehe/TJ_FX_ROBOT_CONTRL_SDK-master"))
    for p in candidates:
        if (p / "SDK_PYTHON" / "libKine.so").is_file():
            return p.resolve()
    return DEFAULT_SDK_ROOT.resolve()


def resolve_kine_cfg(cfg: dict, sdk_root: Optional[Path] = None) -> Path:
    raw = cfg.get("kine_cfg")
    if raw:
        p = Path(raw).expanduser()
        if not p.is_absolute():
            p = (PACKAGE_ROOT / p).resolve()
        return p
    root = sdk_root or resolve_sdk_root(cfg)
    return (root / "CommonConfig" / "ccs_m6_40.MvKDCfg").resolve()
