"""给 agent 用的服务会话。附着已启动的运行时，不进入设备循环。"""

from __future__ import annotations

import logging
import time

from robot.runtime.client import RuntimeClient
from robot.runtime.config import RuntimeConfig, load_config
from robot.runtime.discover import live_records
from robot.runtime.present import summarize_value
from robot.runtime.transport import ControlBusyError, SafetyStopError


class Session:
    def __init__(self, client: RuntimeClient, config: RuntimeConfig):
        self._client = client
        self._config = config
        self._token = 0

    @classmethod
    def attach(cls, name: str | None = None) -> Session:
        records = live_records()
        if not records:
            raise RuntimeError("未运行")
        record = _select_record(records, name)
        config = load_config(record["config_path"])
        client = RuntimeClient(str(record["shm_name"]))
        return cls(client, config)

    def close(self) -> None:
        self._client.close()

    def read_raw(self, name: str) -> dict | None:
        """最新原始观测。图像保留像素，给对齐观测用。"""
        return self._client.read_state(name)

    def describe(self) -> list[dict]:
        return [
            {
                "name": item.name,
                "class_path": item.class_path,
                "class_name": item.class_name,
                "setup": dict(item.setup),
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
        return {"safety": safety, "control": self._client.read_control(), "components": components}

    def acquire_control(self, role: str, lease_s: float = 2.0) -> int:
        token = self._client.acquire_control(role, lease_s)
        self._token = int(token)
        return self._token

    def renew_control(self, token: int | None = None, lease_s: float = 2.0) -> bool:
        current = self._token if token is None else int(token)
        return self._client.renew_control(current, lease_s)

    def release_control(self, token: int | None = None) -> bool:
        current = self._token if token is None else int(token)
        released = self._client.release_control(current)
        if released and self._token == current:
            self._token = 0
        return released

    def call(self, name: str, data: dict, is_delta: bool = False, timeout: float = 2.0, token: int | None = None) -> dict:
        spec = self._spec(name)
        if spec is None:
            return _result(False, None, f"没有组件 {name}", None)
        if not spec["accepts_commands"]:
            return _result(False, None, "组件不接受命令", self._state(name))
        writer = self._token if token is None else int(token)
        try:
            seq = self._client.write_cmd(name, data, is_delta=is_delta, token=writer)
        except ControlBusyError as exc:
            _warn_busy(name, exc)
            return _result(False, None, str(exc), self._state(name))
        except SafetyStopError as exc:
            return _result(False, None, str(exc), self._state(name))
        except KeyError as exc:
            return _result(False, None, str(exc), None)
        return self._wait(name, seq, timeout)

    def write_target(self, name: str, data: dict, token: int | None = None) -> dict:
        """写下最新目标。成功只表示已经盖住上一份，不表示臂已经走到。"""
        spec = self._spec(name)
        if spec is None:
            return _result(False, None, f"没有组件 {name}", None)
        if not spec["accepts_commands"]:
            return _result(False, None, "组件不接受命令", self._state(name))
        writer = self._token if token is None else int(token)
        try:
            seq = self._client.write_stream(name, data, token=writer)
        except ControlBusyError as exc:
            _warn_busy(name, exc)
            return _result(False, None, str(exc), self._state(name))
        except SafetyStopError as exc:
            return _result(False, None, str(exc), self._state(name))
        except (KeyError, ValueError) as exc:
            return _result(False, None, str(exc), self._state(name))
        return _result(True, seq, "", self._state(name))

    def reset(self, name: str, start_state=None, timeout: float = 15.0) -> dict:
        payload = {"op": "reset"}
        if start_state is not None:
            payload["start_state"] = start_state
        return self.call(name, payload, timeout=timeout)

    def _wait(self, name: str, seq: int, timeout: float) -> dict:
        deadline = time.monotonic() + timeout
        motion_deadline = None
        while True:
            now = time.monotonic()
            if motion_deadline is None and now >= deadline:
                return _result(False, seq, "超时", self._state(name))
            if motion_deadline is not None and now >= motion_deadline:
                return _result(False, seq, "跟踪超时", self._state(name))
            safety = self._read_safety()
            if safety["latched"]:
                return _result(False, seq, safety["reason"] or "安全保护已触发", self._state(name))
            try:
                status = self._client.read_status(name)
            except FileNotFoundError:
                return _result(False, seq, "共享内存不存在", None)
            if status is not None and int(status.get("cmd_read_seq") or 0) >= seq:
                if int(status.get("last_cmd_seq") or 0) == seq and status.get("last_cmd_error"):
                    return _result(False, seq, str(status["last_cmd_error"]), self._state(name))
                if status.get("motion_active") and int(status.get("motion_seq") or 0) == seq:
                    if motion_deadline is None:
                        motion_deadline = now + float(status.get("motion_timeout_s") or timeout) + 0.5
                    time.sleep(0.01)
                    continue
                error = str(status.get("error") or "")
                return _result(True, seq, error, self._state(name))
            time.sleep(0.01)

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


def _warn_busy(name: str, exc: ControlBusyError) -> None:
    logging.getLogger("cyr").warning("%s 的控制被拒绝：%s", name, exc)


def _select_record(records: list[dict], name: str | None) -> dict:
    if name is None:
        if len(records) != 1:
            listed = ", ".join(str(item.get("shm_name") or item.get("name")) for item in records)
            raise RuntimeError(f"有多台服务: {listed}")
        return records[0]
    matches = [item for item in records if name in (item.get("name"), item.get("shm_name"))]
    if not matches:
        raise RuntimeError(f"没有服务 {name}")
    if len(matches) != 1:
        raise RuntimeError(f"服务 {name} 不唯一")
    return matches[0]


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
