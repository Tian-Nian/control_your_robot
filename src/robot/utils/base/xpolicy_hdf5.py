"""Helpers for XPolicyLab / xspark v1.0 trajectory HDF5 metadata."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import h5py
import numpy as np

XPOLICY_DATA_FORMAT_VERSION = "v1.0"
REQUIRED_XSPARK_GROUPS = ("vision", "state", "action")


def normalize_instruction(text: str) -> str:
    text = str(text).strip()
    if not text:
        raise ValueError("instruction must be a non-empty string")
    return text


def normalize_instructions_list(value: str | list[str]) -> list[str]:
    if isinstance(value, str):
        text = value.strip()
        return [text] if text else []
    items = [str(item).strip() for item in value if str(item).strip()]
    if not items:
        raise ValueError("instructions must contain at least one non-empty string")
    return items


def _decode_hdf5_text(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if isinstance(value, np.ndarray):
        if value.dtype.kind in {"S", "U", "O"}:
            if value.shape == ():
                return _decode_hdf5_text(value.item())
            if value.size == 1:
                return _decode_hdf5_text(value.item())
            return ""
        return str(value)
    if isinstance(value, str):
        return value
    return str(value)


def read_instruction_from_hdf5(hdf5_path: str | Path) -> str:
    """Read the primary language instruction from an xspark episode file."""
    with h5py.File(hdf5_path, "r") as f:
        if "instruction" in f:
            text = _decode_hdf5_text(f["instruction"][()])
            if text.strip():
                return text.strip()

        if "instructions" in f:
            raw = f["instructions"][()]
            text = _decode_hdf5_text(raw)
            if text.startswith("["):
                try:
                    parsed = json.loads(text)
                    if isinstance(parsed, list) and parsed:
                        return str(parsed[0]).strip()
                except json.JSONDecodeError:
                    pass
            if isinstance(raw, np.ndarray) and raw.dtype.kind in {"S", "U", "O"} and raw.size > 0:
                first = _decode_hdf5_text(raw.reshape(-1)[0])
                if first.strip():
                    return first.strip()
            if text.strip():
                return text.strip()

    return ""


def extract_instruction_from_payload(payload: Any) -> str | None:
    if isinstance(payload, str) and payload.strip():
        return payload.strip()
    if not isinstance(payload, dict):
        return None
    for key in ("instruction", "instructions", "task_instruction"):
        value = payload.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
        if isinstance(value, list) and value:
            first = value[0]
            if isinstance(first, str) and first.strip():
                return first.strip()
    return None


def load_instruction_from_sources(
    *,
    repo_root: Path | None = None,
    hdf5_path: Path | None = None,
    task_name: str | None = None,
    task_info_dir: Path | None = None,
    explicit_instruction: str | None = None,
    instruction_map: dict[str, str] | None = None,
) -> str | None:
    """Resolve language instruction from CLI, sidecars, maps, or task_info JSON."""
    if explicit_instruction is not None and str(explicit_instruction).strip():
        return str(explicit_instruction).strip()

    if hdf5_path is not None and instruction_map:
        stem = hdf5_path.stem
        if stem in instruction_map and instruction_map[stem].strip():
            return instruction_map[stem].strip()
        if hdf5_path.name in instruction_map and instruction_map[hdf5_path.name].strip():
            return instruction_map[hdf5_path.name].strip()

    if hdf5_path is not None:
        for candidate in (
            hdf5_path.with_suffix(".json"),
            hdf5_path.parent / "info.json",
        ):
            if not candidate.is_file():
                continue
            try:
                with open(candidate, "r", encoding="utf-8") as f:
                    payload = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue
            instruction = extract_instruction_from_payload(payload)
            if instruction:
                return instruction

    if task_name:
        root = repo_root or Path.cwd()
        task_info_candidates = []
        if task_info_dir is not None:
            task_info_candidates.append(task_info_dir / f"{task_name}.json")
        task_info_candidates.extend(
            [
                root / "task_info" / f"{task_name}.json",
                root / "task_instructions" / f"{task_name}.json",
            ]
        )
        for candidate in task_info_candidates:
            if not candidate.is_file():
                continue
            try:
                with open(candidate, "r", encoding="utf-8") as f:
                    payload = json.load(f)
            except (OSError, json.JSONDecodeError):
                continue
            instruction = extract_instruction_from_payload(payload)
            if instruction:
                return instruction

    return None


def validate_xspark_episode(hdf5_path: str | Path, *, require_instruction: bool = False) -> list[str]:
    """Return a list of validation errors; empty list means the file looks compliant."""
    errors: list[str] = []
    path = Path(hdf5_path)
    if not path.is_file():
        return [f"file not found: {path}"]

    with h5py.File(path, "r") as f:
        for group_name in REQUIRED_XSPARK_GROUPS:
            if group_name not in f:
                errors.append(f"missing top-level group: {group_name}")

        if require_instruction and not read_instruction_from_hdf5(path):
            errors.append("missing non-empty top-level instruction")

        if "data_format_version" not in f:
            errors.append("missing data_format_version")
        elif _decode_hdf5_text(f["data_format_version"][()]) != XPOLICY_DATA_FORMAT_VERSION:
            errors.append("unexpected data_format_version")

        if "additional_info" not in f or "frequency" not in f["additional_info"]:
            errors.append("missing additional_info/frequency")

        for cam in ("cam_head", "cam_left_wrist", "cam_right_wrist"):
            cam_path = f"vision/{cam}"
            if cam_path not in f:
                errors.append(f"missing camera group: {cam_path}")
                continue
            cam_group = f["vision"][cam]
            if "colors" not in cam_group:
                errors.append(f"missing dataset: vision/{cam}/colors")
            if "shape" not in cam_group:
                errors.append(f"missing dataset: vision/{cam}/shape")

        for key in (
            "state/left_arm_joint_states",
            "state/left_ee_joint_states",
            "state/right_arm_joint_states",
            "state/right_ee_joint_states",
            "state/left_ee_poses",
            "state/right_ee_poses",
            "action/left_arm_joint_states",
            "action/left_ee_joint_states",
            "action/right_arm_joint_states",
            "action/right_ee_joint_states",
        ):
            if key not in f:
                errors.append(f"missing dataset: {key}")

    return errors


def write_xpolicy_episode_metadata(
    hdf5_path: str | Path,
    *,
    instruction: str | None = None,
    frequency: int = 25,
    data_format_version: str = XPOLICY_DATA_FORMAT_VERSION,
    overwrite: bool = True,
) -> None:
    """Write XPolicyLab trajectory metadata onto an existing xspark episode HDF5."""
    string_dtype = h5py.string_dtype(encoding="utf-8")
    instruction_text = str(instruction).strip() if instruction is not None else ""

    with h5py.File(hdf5_path, "a") as f:
        for group_name in REQUIRED_XSPARK_GROUPS:
            if group_name not in f:
                raise ValueError(f"{hdf5_path} is missing required group: {group_name}")

        def _replace_dataset(name: str, data, **kwargs) -> None:
            if name in f:
                if not overwrite:
                    return
                del f[name]
            f.create_dataset(name, data=data, **kwargs)

        if instruction_text:
            normalized = normalize_instruction(instruction_text)
            instructions = normalize_instructions_list(normalized)
            _replace_dataset("instruction", normalized)
            _replace_dataset(
                "instructions",
                np.asarray(instructions, dtype=object),
                dtype=string_dtype,
            )

        _replace_dataset("data_format_version", data_format_version)

        if "additional_info" in f:
            add_info = f["additional_info"]
        else:
            add_info = f.create_group("additional_info")

        if "frequency" in add_info:
            if overwrite:
                del add_info["frequency"]
            else:
                return
        add_info.create_dataset("frequency", data=int(frequency))


def write_xpolicy_episode_metadata_to_file(
    h5_file: h5py.File,
    *,
    instruction: str | None = None,
    frequency: int = 25,
    data_format_version: str = XPOLICY_DATA_FORMAT_VERSION,
) -> None:
    """Write metadata into an open HDF5 file before it is closed."""
    string_dtype = h5py.string_dtype(encoding="utf-8")
    instruction_text = str(instruction).strip() if instruction is not None else ""

    h5_file.create_dataset("data_format_version", data=data_format_version, dtype=string_dtype)
    add_info = h5_file.create_group("additional_info")
    add_info.create_dataset("frequency", data=int(frequency))

    if not instruction_text:
        return

    normalized = normalize_instruction(instruction_text)
    instructions = normalize_instructions_list(normalized)
    h5_file.create_dataset("instruction", data=normalized, dtype=string_dtype)
    h5_file.create_dataset(
        "instructions",
        data=np.asarray(instructions, dtype=object),
        dtype=string_dtype,
    )
