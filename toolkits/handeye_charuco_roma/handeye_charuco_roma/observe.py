"""roma Observe helpers. The core package never imports bus_py; this file does."""

from __future__ import annotations

from typing import Callable, Optional, Protocol

import numpy as np

from handeye_charuco.ingest import rgb_from_roma_image_sample


class ObserveLike(Protocol):
    names: list[str]
    hardware_phase: str

    def wait_ready(self, timeout_s: float) -> None: ...
    def getobs(self, names: Optional[list[str]] = None) -> dict: ...
    def get(self, name: str) -> dict: ...
    def status(self, name: str) -> dict: ...


def rgb_from_sample(sample: dict) -> np.ndarray:
    return rgb_from_roma_image_sample(sample)


def connect_observe(*, timeout_s: float = 15.0):
    try:
        from bus_py import Observe
    except ImportError as exc:
        raise ImportError(
            "无法导入 bus_py。请用装过 roma bus-py 的解释器（conda roma），"
            "且 hw-supervisor 已起、花名册已写。"
        ) from exc
    obs = Observe.connect()
    obs.wait_ready(float(timeout_s))
    return obs


def read_joint(obs: ObserveLike, arm: str) -> np.ndarray:
    sample = obs.get(arm)
    q = np.asarray(sample.get("q") if isinstance(sample, dict) else sample, dtype=np.float64).reshape(-1)
    if q.size < 7:
        raise RuntimeError(f"{arm} joint size {q.size}, need 7")
    return q[:7]


def grab_pair(obs: ObserveLike, *, arm: str, cam: str) -> tuple[np.ndarray, np.ndarray, dict, dict]:
    """One getobs for camera RGB + arm joints (same bus tick window)."""
    pack = obs.getobs([arm, cam])
    if cam not in pack:
        raise RuntimeError(f"no sample for {cam}")
    if arm not in pack:
        raise RuntimeError(f"no sample for {arm}")
    rgb = rgb_from_sample(pack[cam])
    q = np.asarray(pack[arm]["q"], dtype=np.float64).reshape(-1)
    return rgb, q, pack[cam], pack[arm]


class FakeObserve:
    """In-process Observe stand-in. Tests never import bus_py."""

    def __init__(
        self,
        *,
        arm: str = "right_arm",
        cam: str = "cam_head",
        q: Optional[np.ndarray] = None,
        rgb_fn: Optional[Callable[[], np.ndarray]] = None,
    ):
        self.arm = arm
        self.cam = cam
        self.names = [arm, cam]
        self.hardware_phase = "ready"
        self.q = np.zeros(7, dtype=np.float64) if q is None else np.asarray(q, dtype=np.float64)
        self._rgb_fn = rgb_fn or (lambda: np.full((48, 64, 3), 80, dtype=np.uint8))

    def wait_ready(self, timeout_s: float) -> None:
        del timeout_s

    def get(self, name: str) -> dict:
        if name == self.cam:
            rgb = np.asarray(self._rgb_fn())
            h, w = rgb.shape[:2]
            return {
                "kind": "image",
                "seq": 1,
                "width": int(w),
                "height": int(h),
                "channels": 3,
                "format": 1,
                "array": rgb,
            }
        if name == self.arm:
            return {"kind": "joint", "seq": 1, "mode": "idle", "q": self.q.tolist(), "dq": [0.0] * 7}
        raise RuntimeError(f"unknown device {name}")

    def getobs(self, names: Optional[list[str]] = None) -> dict:
        use = names or self.names
        return {n: self.get(n) for n in use}

    def status(self, name: str) -> dict:
        if name not in self.names:
            raise RuntimeError(f"unknown device {name}")
        return {"phase": "ready", "mode": "idle", "health": 1, "pid": 0, "owner": ""}
