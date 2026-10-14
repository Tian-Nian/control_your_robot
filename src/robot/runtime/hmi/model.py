"""界面数据：组件列表、选中项、关闭确认。"""

from __future__ import annotations

from dataclasses import dataclass, field

from robot.runtime.present import detail_lines


@dataclass
class ComponentStat:
    hz: float = 0.0
    age_ms: float | None = None
    state_seq: int = 0
    drops: int = 0
    error: str = ""
    alive: bool = False
    lines: list[str] = field(default_factory=list)


class HmiModel:
    def __init__(self, names: list[str] | None = None):
        self.names = list(names or [])
        self.stats: dict[str, ComponentStat] = {}
        self.selected = 0
        self.connected = False
        self.pid = 0
        self.shm_name = ""
        self.confirming_stop = False
        self.pending_action = ""
        self.notice = ""

    def set_names(self, names: list[str]) -> None:
        if list(names) == self.names:
            return
        self.names = list(names)
        self.selected = 0

    def set_link(self, connected: bool, pid: int, shm_name: str) -> None:
        if shm_name and shm_name != self.shm_name:
            self.stats.clear()
            self.names = []
            self.selected = 0
            self.shm_name = shm_name
        self.connected = connected
        if connected:
            self.pid = pid
            self.notice = ""

    def update_component(self, name: str, status: dict | None, state: dict | None, now_ns: int) -> None:
        stat = self.stats.setdefault(name, ComponentStat())
        if status:
            stat.hz = float(status.get("hz") or 0.0)
            stat.state_seq = int(status.get("state_seq") or 0)
            stat.drops = int(status.get("drops") or 0)
            stat.error = str(status.get("error") or "")
            stat.alive = bool(status.get("alive"))
        if state is not None:
            timestamp_ns = int(state.get("timestamp_ns") or 0)
            if timestamp_ns:
                stat.age_ms = max(0.0, (now_ns - timestamp_ns) / 1e6)
            stat.lines = detail_lines(state.get("data"))
            stat.state_seq = int(state.get("seq") or stat.state_seq)

    def visible_names(self) -> list[str]:
        if self.names and self.selected >= len(self.names):
            self.selected = len(self.names) - 1
        if not self.names:
            self.selected = 0
        return list(self.names)

    def selected_name(self) -> str | None:
        rows = self.visible_names()
        if not rows:
            return None
        return rows[self.selected]

    def selected_stat(self) -> ComponentStat | None:
        name = self.selected_name()
        if name is None:
            return None
        return self.stats.get(name, ComponentStat())

    def counts(self) -> tuple[int, int]:
        alive = 0
        errors = 0
        for name in self.names:
            stat = self.stats.get(name)
            if stat is None:
                continue
            if stat.alive:
                alive += 1
            if stat.error:
                errors += 1
        return alive, errors

    def take_action(self) -> str:
        action = self.pending_action
        self.pending_action = ""
        return action

    def apply_key(self, key: str) -> bool:
        if self.confirming_stop:
            return self._confirm_stop(key)
        if key == "quit":
            return False
        if key == "stop":
            self.confirming_stop = True
            return True
        if key in ("up", "k"):
            self._move(-1)
            return True
        if key in ("down", "j"):
            self._move(1)
            return True
        return True

    def _move(self, delta: int) -> None:
        rows = self.visible_names()
        if not rows:
            return
        self.selected = min(max(0, self.selected + delta), len(rows) - 1)

    def _confirm_stop(self, key: str) -> bool:
        if key in ("yes", "enter"):
            self.confirming_stop = False
            self.pending_action = "stop"
            return True
        if key in ("no", "escape", "quit"):
            self.confirming_stop = False
        return True
