"""运行时 YAML。只校验结构，不导入驱动。"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

import yaml

_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SHM = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,63}$")
_TOP_KEYS = {"name", "shm_name", "cmd_capacity", "components"}
_COMPONENT_KEYS = {
    "name",
    "class_path",
    "class_name",
    "args",
    "setup",
    "publish",
    "rate_hz",
    "max_bytes",
    "body",
    "gate",
    "motion",
}


@dataclass(frozen=True)
class ComponentConfig:
    name: str
    class_path: str
    class_name: str
    args: dict
    setup: dict
    publish: tuple[str, ...]
    rate_hz: float
    max_bytes: int
    body: str = ""
    gate: dict | None = None
    motion: dict | None = None

    def as_spec(self) -> dict:
        return {
            "name": self.name,
            "class_path": self.class_path,
            "class_name": self.class_name,
            "args": dict(self.args),
            "setup": dict(self.setup),
            "publish": list(self.publish),
            "rate_hz": self.rate_hz,
            "max_bytes": self.max_bytes,
            "body": self.body,
            "gate": dict(self.gate or {}),
            "motion": dict(self.motion or {}),
        }


@dataclass(frozen=True)
class RuntimeConfig:
    name: str
    shm_name: str
    cmd_capacity: int
    components: tuple[ComponentConfig, ...]
    path: Path


def load_config(path: str | Path) -> RuntimeConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, dict):
        raise ValueError(f"{config_path}: document must be a mapping")
    unknown = set(raw) - _TOP_KEYS
    if unknown:
        raise ValueError(f"{config_path}: unknown keys {sorted(unknown)}")
    name = _require_str(raw, "name")
    shm_name = _require_str(raw, "shm_name")
    if not _SHM.match(shm_name):
        raise ValueError(f"{config_path}: shm_name {shm_name!r} must match {_SHM.pattern}")
    cmd_capacity = int(raw.get("cmd_capacity", 8))
    if cmd_capacity < 1:
        raise ValueError(f"{config_path}: cmd_capacity must be positive")
    items = raw.get("components")
    if not isinstance(items, list) or not items:
        raise ValueError(f"{config_path}: components must be a non-empty list")
    components = tuple(_component(config_path, item) for item in items)
    names = [item.name for item in components]
    if len(names) != len(set(names)):
        raise ValueError(f"{config_path}: duplicate component names")
    return RuntimeConfig(
        name=name,
        shm_name=shm_name,
        cmd_capacity=cmd_capacity,
        components=components,
        path=config_path.resolve(),
    )


def _component(path: Path, raw: object) -> ComponentConfig:
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: each component must be a mapping")
    unknown = set(raw) - _COMPONENT_KEYS
    if unknown:
        raise ValueError(f"{path}: unknown component keys {sorted(unknown)}")
    name = _require_str(raw, "name")
    if not _NAME.match(name):
        raise ValueError(f"{path}: component name {name!r} must match {_NAME.pattern}")
    publish = raw.get("publish")
    if not isinstance(publish, list) or not publish or not all(isinstance(item, str) and item for item in publish):
        raise ValueError(f"{path}: {name}: publish must be a list of field names")
    rate_hz = float(raw.get("rate_hz", 0))
    max_bytes = int(raw.get("max_bytes", 0))
    if rate_hz <= 0:
        raise ValueError(f"{path}: {name}: rate_hz must be positive")
    if max_bytes < 64:
        raise ValueError(f"{path}: {name}: max_bytes must be at least 64")
    body = raw.get("body") or ""
    if not isinstance(body, str):
        raise ValueError(f"{path}: {name}: body must be a path")
    body_path = ""
    if body.strip():
        candidate = Path(body.strip())
        if not candidate.is_absolute():
            candidate = (path.parent / candidate).resolve()
        body_path = str(candidate)
    return ComponentConfig(
        name=name,
        class_path=_require_str(raw, "class_path"),
        class_name=_require_str(raw, "class_name"),
        args=_mapping(raw.get("args"), f"{name}.args"),
        setup=_mapping(raw.get("setup"), f"{name}.setup"),
        publish=tuple(publish),
        rate_hz=rate_hz,
        max_bytes=max_bytes,
        body=body_path,
        gate=_mapping(raw.get("gate"), f"{name}.gate"),
        motion=_mapping(raw.get("motion"), f"{name}.motion"),
    )


def _require_str(raw: dict, key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value.strip()


def _mapping(value: object, label: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return dict(value)
