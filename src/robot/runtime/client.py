"""按组件名读写最新观测和命令。"""

from __future__ import annotations

from robot.runtime.transport import Bus


class RuntimeClient:
    def __init__(self, shm_name: str):
        self._bus = Bus.attach(shm_name)

    @property
    def version(self) -> int:
        return self._bus.version

    def read_state(self, name: str) -> dict | None:
        return self._bus.read_state(name)

    def write_cmd(self, name: str, data: dict, is_delta: bool = False, token: int = 0) -> int:
        return self._bus.write_cmd(name, data, is_delta=is_delta, token=token)

    def write_stream(self, name: str, data: dict, token: int = 0) -> int:
        return self._bus.write_stream(name, data, token=token)

    def read_control(self) -> dict:
        return self._bus.read_control()

    def acquire_control(self, role: str, lease_s: float = 2.0) -> int:
        return self._bus.acquire_control(role, lease_s)

    def renew_control(self, token: int, lease_s: float = 2.0) -> bool:
        return self._bus.renew_control(token, lease_s)

    def release_control(self, token: int) -> bool:
        return self._bus.release_control(token)

    def read_status(self, name: str) -> dict | None:
        return self._bus.read_status(name)

    def read_safety(self) -> dict:
        return self._bus.read_safety()

    def component_names(self) -> list[str]:
        return self._bus.component_names()

    def close(self) -> None:
        self._bus.close()
