"""Marvin flange FK without opening the arm UDP socket.

Uses ``libKine.so`` + ``ccs_m6_40.MvKDCfg`` the same way Pipeline
``MarvinArmController.fk_xyzquat`` does, then maps native millimetres into the
unified flange TCP (X forward, Y left, Z up). Do not ``Connect`` / ``OnLinkTo``.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from handeye_charuco import se3

from .config import DEFAULT_SDK_ROOT

# Same matrices as X-Robot-Pipeline_new MarvinArmController.
_R_UNIFIED_TO_NATIVE = {
    "left": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]],
        dtype=np.float64,
    ),
    "right": np.array(
        [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    ),
}


def _ensure_sdk_on_path(sdk_root: Path) -> Path:
    py = sdk_root / "SDK_PYTHON"
    if not py.is_dir():
        raise FileNotFoundError(f"Marvin SDK_PYTHON not found: {py}")
    so = py / "libKine.so"
    if not so.is_file():
        raise FileNotFoundError(f"libKine.so not found: {so}")
    py_s = str(py)
    if py_s not in sys.path:
        sys.path.insert(0, py_s)
    ld = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [p for p in ld.split(os.pathsep) if p]
    if py_s not in parts:
        os.environ["LD_LIBRARY_PATH"] = py_s + (os.pathsep + ld if ld else "")
    return py


def native_mm_to_unified(T_mm, *, side: str) -> np.ndarray:
    if side not in _R_UNIFIED_TO_NATIVE:
        raise ValueError(f"side must be left/right, got {side!r}")
    r_n2u = _R_UNIFIED_TO_NATIVE[side].T
    T_n = np.asarray(T_mm, dtype=np.float64).reshape(4, 4).copy()
    T_n[:3, 3] /= 1000.0
    T_u = np.eye(4, dtype=np.float64)
    T_u[:3, :3] = r_n2u @ T_n[:3, :3]
    T_u[:3, 3] = r_n2u @ T_n[:3, 3]
    return T_u


class MarvinOfflineFk:
    """One kinematics instance per arm side. No robot IP."""

    def __init__(
        self,
        *,
        side: str = "right",
        sdk_root: Optional[Path | str] = None,
        kine_cfg: Optional[Path | str] = None,
    ):
        side = str(side)
        if side not in _R_UNIFIED_TO_NATIVE:
            raise ValueError(f"side must be left/right, got {side!r}")
        self.side = side
        self.arm_type = 0 if side == "left" else 1
        self.sdk_root = Path(sdk_root).expanduser().resolve() if sdk_root else DEFAULT_SDK_ROOT.resolve()
        if kine_cfg:
            self.kine_cfg = Path(kine_cfg).expanduser().resolve()
        else:
            self.kine_cfg = (self.sdk_root / "CommonConfig" / "ccs_m6_40.MvKDCfg").resolve()
        self._kk = None

    def _ensure(self):
        if self._kk is not None:
            return self._kk
        if not self.kine_cfg.is_file():
            raise FileNotFoundError(f"kinematics config not found: {self.kine_cfg}")
        _ensure_sdk_on_path(self.sdk_root)
        from fx_kine import Marvin_Kine

        kk = Marvin_Kine()
        kk.log_switch(0)
        ini = kk.load_config(arm_type=self.arm_type, config_path=str(self.kine_cfg))
        if not ini:
            raise RuntimeError(f"load_config failed for {self.kine_cfg}")
        ok = kk.initial_kine(
            robot_type=ini["TYPE"][0],
            dh=ini["DH"][0],
            pnva=ini["PNVA"][0],
            j67=ini["BD"][0],
        )
        if not ok:
            raise RuntimeError("initial_kine failed")
        self._kk = kk
        return kk

    def fk_T(self, joint_rad) -> np.ndarray:
        q = np.asarray(joint_rad, dtype=np.float64).reshape(-1)
        if q.size < 7:
            raise ValueError(f"expected 7 joints, got {q.size}")
        kk = self._ensure()
        T_mm = kk.fk(joints=np.degrees(q[:7]).tolist())
        if not T_mm:
            raise RuntimeError("Marvin_Kine.fk failed")
        return native_mm_to_unified(T_mm, side=self.side)

    def fk_eef_wxyz(self, joint_rad) -> np.ndarray:
        return se3.to_xyz_wxyz(self.fk_T(joint_rad))


class FakeFk:
    """Deterministic FK for tests. ``q[0]`` yaws a fixed reach pose."""

    def __init__(self, *, side: str = "right"):
        self.side = side

    def fk_T(self, joint_rad) -> np.ndarray:
        q = np.asarray(joint_rad, dtype=np.float64).reshape(-1)
        yaw = float(q[0]) if q.size else 0.0
        return se3.from_xyz_rpy([0.30, -0.17, -0.28], [0.0, 0.0, yaw])

    def fk_eef_wxyz(self, joint_rad) -> np.ndarray:
        return se3.to_xyz_wxyz(self.fk_T(joint_rad))
