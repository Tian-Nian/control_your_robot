"""原始 HDF5。group 是设备名，dataset 是字段，和 CollectAny.write 一样。"""

from __future__ import annotations

import json
import re
from pathlib import Path

import h5py
import numpy as np


class Hdf5Episode:
    def __init__(self, save_dir, task: str, save_freq: float):
        self.save_dir = Path(save_dir)
        self.task = str(task)
        self.save_freq = float(save_freq)
        self._frames: list[dict] = []
        self._skew: list[float] = []

    def record(self, devices: dict, skew_ms: float) -> None:
        if not isinstance(devices, dict) or not devices:
            raise ValueError("这一帧没有设备")
        self._frames.append({name: _compress(fields) for name, fields in devices.items()})
        self._skew.append(float(skew_ms))

    def finish(self) -> Path:
        if not self._frames:
            raise RuntimeError("没有可写的帧")
        folder = self.save_dir / self.task
        folder.mkdir(parents=True, exist_ok=True)
        episode_id = _next_episode_id(folder)
        path = folder / f"{episode_id}.hdf5"
        mapping = {}
        for frame in self._frames:
            for name, fields in frame.items():
                mapping.setdefault(name, set()).update(fields)
        with h5py.File(path, "w") as handle:
            handle.attrs["skew_ms_max"] = float(max(self._skew))
            handle.create_dataset("skew_ms", data=np.asarray(self._skew, dtype=np.float64))
            for name, fields in mapping.items():
                group = handle.create_group(name)
                for field in sorted(fields):
                    _write_field(group, field, [_field(frame, name, field) for frame in self._frames])
        _write_config(folder, self.task, self.save_dir, self.save_freq, mapping)
        self._frames = []
        self._skew = []
        return path


def collect_until(session, task: str, save_dir, save_freq: float, stop) -> Path:
    """按 save_freq 读最新帧。不写命令，采集进行中不复位。"""
    period = 1.0 / float(save_freq)
    episode = Hdf5Episode(save_dir, task, save_freq)
    while not stop.is_set():
        devices = {}
        stamps = []
        for item in session.describe():
            state = session.read_raw(item["name"])
            if state is None or not isinstance(state.get("data"), dict):
                continue
            fields = {}
            for key, value in state["data"].items():
                if key == "timestamp":
                    continue
                fields[key] = value
            fields["timestamp"] = int(state["timestamp_ns"])
            devices[item["name"]] = fields
            stamps.append(int(state["timestamp_ns"]))
        if devices:
            skew = 0.0 if len(stamps) < 2 else (max(stamps) - min(stamps)) / 1e6
            episode.record(devices, skew)
        if stop.wait(period):
            break
    return episode.finish()


def _field(frame: dict, name: str, field: str):
    fields = frame.get(name) or {}
    if field not in fields:
        return None
    return fields[field]


def _write_field(group, name: str, values: list) -> None:
    present = [item for item in values if item is not None]
    if not present:
        return
    if all(isinstance(item, (bytes, bytearray)) for item in present):
        dataset = group.create_dataset(name, shape=(len(present),), dtype=h5py.vlen_dtype(np.dtype("uint8")))
        for index, item in enumerate(present):
            dataset[index] = np.frombuffer(bytes(item), dtype=np.uint8).copy()
        return
    group.create_dataset(name, data=np.asarray(present))


def _compress(fields: dict) -> dict:
    stored = {}
    for key, value in fields.items():
        if key in ("color", "image") and not isinstance(value, (bytes, bytearray)):
            array = np.asarray(value)
            if array.ndim >= 2 and array.dtype == np.uint8:
                stored[key] = _jpeg(array)
                continue
        stored[key] = value
    return stored


def _jpeg(image: np.ndarray) -> bytes:
    import cv2

    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("图像不能收成 JPEG")
    return encoded.tobytes()


def _write_config(folder: Path, task: str, save_dir, save_freq: float, mapping: dict) -> None:
    path = folder / "config.json"
    payload = {}
    if path.is_file():
        payload = json.loads(path.read_text(encoding="utf-8"))
    payload.update(
        {
            "task_name": task,
            "save_path": str(save_dir),
            "save_freq": float(save_freq),
            "save_format": "hdf5",
        }
    )
    for name, fields in mapping.items():
        payload[name] = sorted(fields)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=4), encoding="utf-8")


def _next_episode_id(folder: Path) -> int:
    highest = -1
    for path in folder.glob("*.hdf5"):
        match = re.fullmatch(r"(\d+)", path.stem)
        if match:
            highest = max(highest, int(match.group(1)))
    return highest + 1
