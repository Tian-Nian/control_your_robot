"""长线控制的租约。心跳和目标分开，主臂停着也续租。"""

from __future__ import annotations

import threading


class ControlLease:
    def __init__(self, session, role: str, lease_s: float = 2.0):
        self.session = session
        self.role = role
        self.lease_s = float(lease_s)
        self.token = session.acquire_control(role, self.lease_s)
        self.alive = True
        self._stop = threading.Event()
        self._mu = threading.Lock()
        self._thread = threading.Thread(target=self._beat, name=f"cyr-{role}-lease", daemon=True)
        self._thread.start()

    def _beat(self) -> None:
        pause = max(self.lease_s / 3.0, 0.05)
        while not self._stop.wait(pause):
            with self._mu:
                if self._stop.is_set():
                    return
                try:
                    ok = self.session.renew_control(self.token, self.lease_s)
                except Exception:
                    self.alive = False
                    return
            if not ok:
                self.alive = False
                return

    def close(self) -> None:
        self._stop.set()
        with self._mu:
            self.session.release_control(self.token)
            self.alive = False
        self._thread.join(timeout=self.lease_s + 1.0)
