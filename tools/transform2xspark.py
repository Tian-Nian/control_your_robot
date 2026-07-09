#!/usr/bin/env python3
"""Convert control_your_robot raw HDF5 episodes to Xspark layout via X_spark_format_pipeline."""

from __future__ import annotations

import argparse
import copy
import os
import sys
from functools import partial
from multiprocessing import Pool, cpu_count
from pathlib import Path

import yaml
from tqdm import tqdm

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "src"))
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from robot.data.collect_any import CollectAny
from robot.robot.base_robot import dict_to_list
from robot.utils.base.data_handler import debug_print, get_files, hdf5_groups_to_dict
from robot.utils.base.data_transform_pipeline import X_spark_format_pipeline
from robot.utils.base.xpolicy_hdf5 import load_instruction_from_sources


def _load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _normalize_legacy_arm_keys(frame: dict) -> dict:
    """Map legacy arm field names to the Xspark layout."""
    normalized = copy.deepcopy(frame)
    rename_map = {
        "slave_left_arm": "left_arm",
        "slave_right_arm": "right_arm",
    }
    for old_name, new_name in rename_map.items():
        if old_name in normalized and new_name not in normalized:
            normalized[new_name] = normalized.pop(old_name)

    for arm_name in ("left_arm", "right_arm"):
        arm_data = normalized.get(arm_name)
        if not isinstance(arm_data, dict):
            continue
        if "eef" not in arm_data and "qpos" in arm_data:
            arm_data["eef"] = arm_data["qpos"]
    return normalized


def _build_collect_cfg(
    *,
    save_dir: str,
    task_name: str,
    collect_type: str,
    save_freq: int,
) -> dict:
    return {
        "save_path": save_dir,
        "task_name": task_name,
        "type": collect_type,
        "save_format": "hdf5",
        "save_freq": save_freq,
    }


def process_episode(
    hdf5_path: str | Path,
    *,
    collect_cfg: dict,
    task_name: str,
    task_info_dir: Path,
    explicit_instruction: str | None,
    skip_video: bool,
) -> str | None:
    hdf5_path = Path(hdf5_path)
    prev_skip_preview = os.environ.get("XSPARK_SKIP_PREVIEW")
    if skip_video:
        os.environ["XSPARK_SKIP_PREVIEW"] = "1"
    try:
        local_cfg = copy.deepcopy(collect_cfg)
        instruction = load_instruction_from_sources(
            repo_root=_REPO_ROOT,
            hdf5_path=hdf5_path,
            task_name=task_name,
            task_info_dir=task_info_dir,
            explicit_instruction=explicit_instruction,
        )
        if instruction:
            local_cfg["xpolicy_instruction"] = instruction
        collection = CollectAny(condition=local_cfg, move_check=False, resume=False)
        collection._add_data_transform_pipeline(X_spark_format_pipeline)

        episode_frames = [
            _normalize_legacy_arm_keys(frame)
            for frame in dict_to_list(hdf5_groups_to_dict(str(hdf5_path)))
        ]
        for frame in episode_frames:
            collection.collect(frame, None)

        debug_print(
            "transform2xspark",
            f"{hdf5_path.name}: instruction={instruction!r}" if instruction else f"{hdf5_path.name}: instruction skipped",
            "INFO",
        )
        collection.write(episode_id=hdf5_path.stem)
        return None
    except Exception as exc:
        debug_print("transform2xspark", f"converting {hdf5_path} failed:\n{exc}", "ERROR")
        return str(hdf5_path)
    finally:
        if skip_video:
            if prev_skip_preview is None:
                os.environ.pop("XSPARK_SKIP_PREVIEW", None)
            else:
                os.environ["XSPARK_SKIP_PREVIEW"] = prev_skip_preview


def main() -> int:
    parser = argparse.ArgumentParser(description="Transform raw HDF5 episodes to Xspark format.")
    parser.add_argument(
        "data_path",
        type=str,
        help="Directory containing source *.hdf5 files.",
    )
    parser.add_argument(
        "--task-name",
        type=str,
        default="xspark",
        help="Task name used in output directory layout.",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="./data",
        help="Root directory for transformed output.",
    )
    parser.add_argument(
        "--collect-type",
        type=str,
        default="xspark",
        help="Collect type label stored in collect config.",
    )
    parser.add_argument(
        "--collect-config",
        type=str,
        default=None,
        help="Optional YAML whose collect: section overrides defaults.",
    )
    parser.add_argument(
        "--instruction",
        type=str,
        default=None,
        help="Explicit language instruction. If omitted, load from sidecar JSON or task_info/{task_name}.json.",
    )
    parser.add_argument(
        "--task-info-dir",
        type=str,
        default="task_info",
        help="Directory containing task-level instruction JSON files.",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=min(12, cpu_count()),
        help="Parallel worker count.",
    )
    parser.add_argument(
        "--skip-video",
        action="store_true",
        help="Skip ffmpeg preview generation (HDF5 conversion only).",
    )
    args = parser.parse_args()

    data_path = Path(args.data_path).expanduser().resolve()
    if not data_path.is_dir():
        raise SystemExit(f"data_path is not a directory: {data_path}")

    collect_cfg = _build_collect_cfg(
        save_dir=os.path.abspath(args.save_dir),
        task_name=args.task_name,
        collect_type=args.collect_type,
        save_freq=25,
    )
    if args.collect_config:
        cfg = _load_yaml(args.collect_config)
        collect_cfg.update(cfg.get("collect", cfg))

    hdf5_paths = sorted(get_files(str(data_path), "*.hdf5"))
    if not hdf5_paths:
        raise SystemExit(f"No HDF5 files found under {data_path}")

    num_workers = max(1, min(args.num_workers, len(hdf5_paths)))
    debug_print(
        "transform2xspark",
        f"Converting {len(hdf5_paths)} episodes with {num_workers} workers",
        "INFO",
    )

    task_info_dir = Path(args.task_info_dir).expanduser().resolve()

    process_func = partial(
        process_episode,
        collect_cfg=collect_cfg,
        task_name=args.task_name,
        task_info_dir=task_info_dir,
        explicit_instruction=args.instruction,
        skip_video=args.skip_video,
    )

    results: list[str | None] = []
    if num_workers == 1:
        for path in tqdm(hdf5_paths, desc="Converting episodes"):
            results.append(process_func(path))
    else:
        with Pool(num_workers) as pool:
            for res in tqdm(
                pool.imap_unordered(process_func, hdf5_paths),
                total=len(hdf5_paths),
                desc="Converting episodes",
            ):
                results.append(res)

    fail_episode_list = [res for res in results if res is not None]
    output_path = Path(collect_cfg["save_path"]) / collect_cfg["task_name"]
    fail_save_path = output_path / "fail_episodes.txt"
    fail_save_path.parent.mkdir(parents=True, exist_ok=True)
    with open(fail_save_path, "w", encoding="utf-8") as f:
        for path in fail_episode_list:
            f.write(f"{path}\n")

    success_count = len(hdf5_paths) - len(fail_episode_list)
    debug_print(
        "transform2xspark",
        f"Done: success={success_count}, fail={len(fail_episode_list)}, output={output_path}",
        "INFO" if not fail_episode_list else "ERROR",
    )
    return 0 if not fail_episode_list else 1


if __name__ == "__main__":
    raise SystemExit(main())
