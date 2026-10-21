"""记录一条 episode 的观测和实际下发的动作。图像只留形状。"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np


class Episode:
    def __init__(self):
        self.path: Path | None = None
        self.started_ns = 0
        self.steps: list[dict] = []
        self.active = False

    def start(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        self.path = target
        self.started_ns = time.time_ns()
        self.steps = []
        self.active = True

    def record(self, observation: dict, action) -> None:
        if not self.active:
            raise RuntimeError("episode 还没开始")
        self.steps.append(
            {
                "t_ns": time.time_ns(),
                "skew_ms": observation.get("skew_ms"),
                "within_skew": observation.get("within_skew"),
                "state": _state_view(observation),
                "action": _jsonable(action),
            }
        )

    def finish(self, success: bool, reason: str = "") -> Path:
        if not self.active or self.path is None:
            raise RuntimeError("episode 还没开始")
        payload = {
            "started_ns": self.started_ns,
            "finished_ns": time.time_ns(),
            "success": bool(success),
            "reason": str(reason),
            "steps": self.steps,
        }
        self.path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        self.active = False
        return self.path


def _state_view(observation: dict) -> dict:
    components = {}
    for name, state in (observation.get("components") or {}).items():
        if not state or not isinstance(state.get("data"), dict):
            components[name] = None
            continue
        components[name] = {key: _jsonable(value) for key, value in state["data"].items()}
    view = {"components": components}
    for key in ("palm", "flange"):
        if key in observation:
            view[key] = {name: _jsonable(value) for name, value in observation[key].items()}
    return view


def _jsonable(value):
    if isinstance(value, np.ndarray):
        if value.ndim >= 2 or value.size > 32:
            return {
                "kind": "ndarray",
                "shape": [int(dim) for dim in value.shape],
                "dtype": str(value.dtype),
            }
        return [_number(item) for item in value.reshape(-1).tolist()]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (np.floating, float)):
        return float(value)
    if isinstance(value, (np.integer, int)) and not isinstance(value, bool):
        return int(value)
    if isinstance(value, (bytes, bytearray)):
        return {"kind": "bytes", "nbytes": len(value)}
    return value


def _number(value):
    if isinstance(value, (float, np.floating)):
        return float(value)
    if isinstance(value, (int, np.integer)) and not isinstance(value, bool):
        return int(value)
    return value
