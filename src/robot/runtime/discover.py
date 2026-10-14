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
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    os.chmod(temporary, 0o644)
    temporary.replace(path)


def remove_record(path: Path | None = None) -> None:
    target = path if path is not None else record_path()
    try:
        target.unlink()
    except FileNotFoundError:
        pass


def load_record(path: Path | None = None) -> dict | None:
    target = path if path is not None else record_path()
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or "pid" not in data:
        return None
    data["_path"] = str(target)
    return data


def live_records() -> list[dict]:
    record = load_record()
    if record is None or not pid_alive(int(record["pid"])):
        return []
    return [record]


def clear_dead_records() -> None:
    record = load_record()
    if record is None:
        return
    if not pid_alive(int(record["pid"])):
        remove_record(Path(record["_path"]))
