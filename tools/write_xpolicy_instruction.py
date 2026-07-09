#!/usr/bin/env python3
"""Write or update XPolicyLab trajectory metadata on xspark episode HDF5 files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "src"))

from robot.utils.base.data_handler import debug_print, get_files
from robot.utils.base.xpolicy_hdf5 import (
    extract_instruction_from_payload,
    load_instruction_from_sources,
    validate_xspark_episode,
    write_xpolicy_episode_metadata,
)


def _load_instruction_map(path: Path) -> dict[str, str]:
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)

    if isinstance(payload, str):
        raise ValueError("instruction map JSON must be an object, not a bare string")
    if not isinstance(payload, dict):
        raise ValueError("instruction map JSON must be an object")

    mapping: dict[str, str] = {}
    for key, value in payload.items():
        if isinstance(value, str) and value.strip():
            mapping[str(key)] = value.strip()
            continue
        instruction = extract_instruction_from_payload(value)
        if instruction:
            mapping[str(key)] = instruction
    return mapping


def _collect_hdf5_paths(data_path: Path, recursive: bool) -> list[str]:
    if recursive:
        return sorted(str(p) for p in data_path.rglob("*.hdf5"))
    return sorted(get_files(str(data_path), "*.hdf5"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Write XPolicyLab instruction/metadata onto xspark episode HDF5 files.",
    )
    parser.add_argument(
        "data_path",
        type=str,
        help="Directory containing episode_*.hdf5 files (usually .../data/).",
    )
    parser.add_argument(
        "--task-name",
        type=str,
        default=None,
        help="Task name used to resolve task_info/{task_name}.json.",
    )
    parser.add_argument(
        "--instruction",
        type=str,
        default=None,
        help="Single instruction applied to every episode unless overridden by sidecar/map.",
    )
    parser.add_argument(
        "--instruction-map",
        type=str,
        default=None,
        help="JSON file mapping episode stem or filename to instruction text.",
    )
    parser.add_argument(
        "--task-info-dir",
        type=str,
        default="task_info",
        help="Directory containing task-level instruction JSON files.",
    )
    parser.add_argument(
        "--frequency",
        type=int,
        default=25,
        help="Value written to additional_info/frequency.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Recursively search for *.hdf5 under data_path.",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Validate files after writing metadata (requires instruction).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned updates without modifying files.",
    )
    args = parser.parse_args()

    data_path = Path(args.data_path).expanduser().resolve()
    if not data_path.is_dir():
        raise SystemExit(f"data_path is not a directory: {data_path}")

    instruction_map: dict[str, str] = {}
    if args.instruction_map:
        instruction_map = _load_instruction_map(Path(args.instruction_map).expanduser().resolve())

    task_info_dir = Path(args.task_info_dir).expanduser().resolve()
    hdf5_paths = _collect_hdf5_paths(data_path, args.recursive)
    if not hdf5_paths:
        raise SystemExit(f"No HDF5 files found under {data_path}")

    fail_count = 0
    updated_count = 0
    skipped_count = 0
    for hdf5_path_str in hdf5_paths:
        hdf5_path = Path(hdf5_path_str)
        try:
            instruction = load_instruction_from_sources(
                repo_root=_REPO_ROOT,
                hdf5_path=hdf5_path,
                task_name=args.task_name,
                task_info_dir=task_info_dir,
                explicit_instruction=args.instruction,
                instruction_map=instruction_map,
            )
            if not instruction:
                skipped_count += 1
                debug_print(
                    "write_xpolicy_instruction",
                    f"skipped {hdf5_path.name}: no instruction found",
                    "INFO",
                )
                continue

            if args.dry_run:
                debug_print(
                    "write_xpolicy_instruction",
                    f"[dry-run] {hdf5_path.name}: instruction={instruction!r}, frequency={args.frequency}",
                    "INFO",
                )
                continue

            write_xpolicy_episode_metadata(
                hdf5_path,
                instruction=instruction,
                frequency=args.frequency,
            )
            if args.verify:
                errors = validate_xspark_episode(hdf5_path, require_instruction=True)
                if errors:
                    raise ValueError("; ".join(errors))
            updated_count += 1
            debug_print(
                "write_xpolicy_instruction",
                f"updated {hdf5_path}",
                "INFO",
            )
        except Exception as exc:
            fail_count += 1
            debug_print(
                "write_xpolicy_instruction",
                f"failed {hdf5_path}: {exc}",
                "ERROR",
            )

    if fail_count:
        debug_print(
            "write_xpolicy_instruction",
            f"Done with failures: updated={updated_count}, skipped={skipped_count}, fail={fail_count}",
            "ERROR",
        )
        return 1

    debug_print(
        "write_xpolicy_instruction",
        f"Done: updated={updated_count}, skipped={skipped_count}",
        "INFO",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
