"""给 agent 用的服务会话。附着已启动的运行时，不进入设备循环。"""

from __future__ import annotations

import time

from robot.runtime.client import RuntimeClient
from robot.runtime.config import RuntimeConfig, load_config
from robot.runtime.discover import live_records
from robot.runtime.present import summarize_value
from robot.runtime.transport import SafetyStopError


class Session:
    def __init__(self, client: RuntimeClient, config: RuntimeConfig):
        self._client = client
        self._config = config

    @classmethod
    def attach(cls) -> Session:
        records = live_records()
        if not records:
            raise RuntimeError("未运行")
        record = records[0]
        config = load_config(record["config_path"])
        client = RuntimeClient(str(record["shm_name"]))
        return cls(client, config)

    def close(self) -> None:
        self._client.close()

    def describe(self) -> list[dict]:
        return [
            {
                "name": item.name,
                "publish": list(item.publish),
                "rate_hz": item.rate_hz,
                "accepts_commands": _accepts_commands(item.class_path),
            }
            for item in self._config.components
        ]

    def snapshot(self) -> dict:
        now_ns = time.time_ns()
        safety = self._read_safety()
        components = {}
        for item in self._config.components:
            components[item.name] = _component_view(self._client.read_state(item.name), now_ns)
        return {"safety": safety, "components": components}

    def call(self, name: str, data: dict, is_delta: bool = False, timeout: float = 2.0) -> dict:
        spec = self._spec(name)
        if spec is None:
            return _result(False, None, f"没有组件 {name}", None)
        if not spec["accepts_commands"]:
            return _result(False, None, "组件不接受命令", self._state(name))
        try:
            seq = self._client.write_cmd(name, data, is_delta=is_delta)
        except SafetyStopError as exc:
            return _result(False, None, str(exc), self._state(name))
        except KeyError as exc:
            return _result(False, None, str(exc), None)
        return self._wait(name, seq, timeout)

    def reset(self, name: str, start_state=None, timeout: float = 2.0) -> dict:
        payload = {"op": "reset"}
        if start_state is not None:
            payload["start_state"] = start_state
        return self.call(name, payload, timeout=timeout)

    def _wait(self, name: str, seq: int, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            safety = self._read_safety()
            if safety["latched"]:
                return _result(False, seq, safety["reason"] or "安全保护已触发", self._state(name))
            try:
                status = self._client.read_status(name)
            except FileNotFoundError:
                return _result(False, seq, "共享内存不存在", None)
            if status is not None and int(status.get("cmd_read_seq") or 0) >= seq:
                error = str(status.get("error") or "")
                return _result(True, seq, error, self._state(name))
            time.sleep(0.01)
        return _result(False, seq, "超时", self._state(name))

    def _spec(self, name: str) -> dict | None:
        for item in self.describe():
            if item["name"] == name:
                return item
        return None

    def _state(self, name: str) -> dict | None:
        try:
            return _component_view(self._client.read_state(name), time.time_ns())
        except FileNotFoundError:
            return None

    def _read_safety(self) -> dict:
        try:
            safety = self._client.read_safety()
        except FileNotFoundError:
            return {"latched": True, "reason": "共享内存不存在"}
        return {"latched": bool(safety.get("latched")), "reason": str(safety.get("reason") or "")}


def _accepts_commands(class_path: str) -> bool:
    return ".sensor." not in f".{class_path}."


def _component_view(state: dict | None, now_ns: int) -> dict:
    if state is None:
        return {"seq": 0, "age_ms": None, "data": None}
    timestamp_ns = int(state.get("timestamp_ns") or 0)
    age_ms = None if timestamp_ns <= 0 else max(0.0, (now_ns - timestamp_ns) / 1e6)
    return {
        "seq": int(state.get("seq") or 0),
        "age_ms": age_ms,
        "data": summarize_value(state.get("data")),
    }


def _result(accepted: bool, seq: int | None, error: str, state: dict | None) -> dict:
    return {"accepted": accepted, "seq": seq, "error": error, "state": state}
