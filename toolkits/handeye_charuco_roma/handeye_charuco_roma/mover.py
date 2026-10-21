"""Go to a joint waypoint. Replay claims the arm; teach never does."""

from __future__ import annotations

import time
from typing import Protocol

import numpy as np

from .observe import ObserveLike, read_joint


class ArmMover(Protocol):
    def goto(self, q, *, max_deg_s: float = 20.0) -> np.ndarray: ...
    def close(self) -> None: ...


class FakeMover:
    """Tests: just write q onto FakeObserve."""

    def __init__(self, obs):
        self.obs = obs

    def goto(self, q, *, max_deg_s: float = 20.0) -> np.ndarray:
        del max_deg_s
        arr = np.asarray(q, dtype=np.float64).reshape(-1)[:7]
        self.obs.q = arr
        return arr

    def close(self) -> None:
        return None


class RomaSessionMover:
    def __init__(self, obs: ObserveLike, arm: str, *, owner: str = "handeye_replay"):
        from bus_py import Session

        self.obs = obs
        self.arm = arm
        self.sess = Session.connect(owner=owner)
        self.sess.wait_ready(15.0)
        self.sess.claim(arm)
        self.sess.change_mode(arm, "position")

    def goto(self, q, *, max_deg_s: float = 20.0) -> np.ndarray:
        tgt = np.asarray(q, dtype=np.float64).reshape(-1)[:7]
        cur = read_joint(self.obs, self.arm)
        max_deg = float(np.max(np.abs(np.degrees(tgt - cur))))
        dur = max(1.0, max_deg / max(float(max_deg_s), 1.0))
        t0 = time.monotonic()
        dt = 0.02
        while True:
            u = min(1.0, (time.monotonic() - t0) / dur)
            s = 0.5 - 0.5 * np.cos(np.pi * u)
            now = cur * (1.0 - s) + tgt * s
            self.sess.move(self.arm, [float(x) for x in now])
            self.sess.heartbeat()
            if u >= 1.0:
                break
            time.sleep(dt)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            self.sess.move(self.arm, [float(x) for x in tgt])
            self.sess.heartbeat()
            got = read_joint(self.obs, self.arm)
            if float(np.max(np.abs(np.degrees(got - tgt)))) < 1.5:
                return got
            time.sleep(dt)
        return read_joint(self.obs, self.arm)

    def close(self) -> None:
        try:
            self.sess.release(self.arm)
        except Exception:
            pass
