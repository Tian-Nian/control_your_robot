"""按组件名读写最新观测和命令。"""

from __future__ import annotations

from robot.runtime.transport import Bus


class RuntimeClient:
    def __init__(self, shm_name: str):
        self._bus = Bus.attach(shm_name)

    def read_state(self, name: str) -> dict | None:
        return self._bus.read_state(name)

    def write_cmd(self, name: str, data: dict, is_delta: bool = False) -> int:
        return self._bus.write_cmd(name, data, is_delta=is_delta)

    def read_status(self, name: str) -> dict | None:
        return self._bus.read_status(name)

    def read_safety(self) -> dict:
        return self._bus.read_safety()

    def component_names(self) -> list[str]:
        return self._bus.component_names()

    def close(self) -> None:
        self._bus.close()
