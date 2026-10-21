"""服务发现记录，写在用户状态目录。"""

from __future__ import annotations

import json
import os
from pathlib import Path

from robot.runtime.transport import pid_alive


def user_state_dir() -> Path:
    base = os.environ.get("XDG_STATE_HOME")
    if not base:
        base = str(Path.home() / ".local" / "state")
    return Path(base) / "cyr"


def record_path() -> Path:
    return user_state_dir() / "runtime.json"


def write_record(record: dict) -> None:
    path = record_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {key: value for key, value in record.items() if not key.startswith("_")}
    records = [item for item in _read_records(path) if item.get("shm_name") != payload.get("shm_name")]
    records.append(payload)
    _write_records(path, records)


def remove_record(path: Path | None = None, shm_name: str | None = None) -> None:
    target = path if path is not None else record_path()
    if shm_name is None:
        try:
            target.unlink()
        except FileNotFoundError:
            pass
        return
    records = [item for item in _read_records(target) if item.get("shm_name") != shm_name]
    if records:
        _write_records(target, records)
        return
    try:
        target.unlink()
    except FileNotFoundError:
        pass


def load_record(path: Path | None = None) -> dict | None:
    records = load_records(path)
    if not records:
        return None
    return records[0]


def load_records(path: Path | None = None) -> list[dict]:
    target = path if path is not None else record_path()
    return _read_records(target)


def live_records() -> list[dict]:
    return [record for record in load_records() if pid_alive(int(record["pid"]))]


def clear_dead_records() -> None:
    path = record_path()
    records = _read_records(path)
    if not records:
        return
    alive = [record for record in records if pid_alive(int(record["pid"]))]
    if len(alive) == len(records):
        return
    if alive:
        _write_records(path, alive)
        return
    remove_record(path)


def _read_records(path: Path) -> list[dict]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    if isinstance(data, dict) and isinstance(data.get("services"), list):
        items = data["services"]
    elif isinstance(data, dict) and "pid" in data:
        items = [data]
    elif isinstance(data, list):
        items = data
    else:
        return []
    records = []
    for item in items:
        if not isinstance(item, dict) or "pid" not in item:
            continue
        record = dict(item)
        record["_path"] = str(path)
        records.append(record)
    return records


def _write_records(path: Path, records: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "services": [{key: value for key, value in record.items() if not key.startswith("_")} for record in records]
    }
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.chmod(temporary, 0o644)
    temporary.replace(path)
