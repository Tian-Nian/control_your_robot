"""Teach-time joint list. Images / FK are taken later during replay."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


class JointLog:
    def __init__(self, path: Path, *, arm: str, q: list[np.ndarray] | None = None):
        self.path = Path(path)
        self.arm = str(arm)
        self.q: list[np.ndarray] = [] if q is None else [np.asarray(x, dtype=np.float64).reshape(-1)[:7] for x in q]

    @classmethod
    def load_or_new(cls, path: Path | str, *, arm: str) -> "JointLog":
        path = Path(path)
        if not path.is_file():
            return cls(path, arm=arm)
        data = json.loads(path.read_text(encoding="utf-8"))
        rows = data.get("q") or []
        return cls(path, arm=str(data.get("arm") or arm), q=[np.asarray(r, dtype=np.float64) for r in rows])

    def append(self, q) -> int:
        self.q.append(np.asarray(q, dtype=np.float64).reshape(-1)[:7].copy())
        return len(self.q) - 1

    def save(self) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"arm": self.arm, "q": [row.tolist() for row in self.q]}
        self.path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return self.path
