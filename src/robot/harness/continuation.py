"""Load bounded, allowlisted history for a new visual run; never replay commands."""
import json
import math
from pathlib import Path

from .policy import parse_action, parse_plan, parse_verification
from .progress import EpisodeProgress


def _read_json(path, limit):
    if path.stat().st_size > limit:
        raise ValueError(f"{path.name} exceeds the continuation size limit")
    return json.loads(path.read_text(encoding="utf-8"))


def _vector(value, count):
    if (not isinstance(value, list) or len(value) != count
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        return None
    return list(value)


def load_continuation(directory):
    """Recover task/target/history from old or new layered logs, not live scene state.

    Full model messages, raw images, paths, arbitrary result fields and pending
    commands are never copied into policy input. A fresh plan and fresh executor
    hold are required before any new action. Round numbers continue; budgets reset.
    """
    directory = Path(directory).resolve()
    metadata_path = directory / "run_context.json"
    if not metadata_path.exists():
        metadata_path = directory / "evidence_notes.json"  # Older, cleanly exited runs.
    metadata = _read_json(metadata_path, 32_768)
    task = metadata.get("task")
    if (type(metadata.get("version")) is not int or metadata["version"] != 1 or not isinstance(task, str)
            or not task.strip() or len(task) > 4_096):
        raise ValueError("continuation requires a bounded version-1 task record")
    path = directory / "turns.jsonl"
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError("turns.jsonl exceeds the continuation size limit")
    progress, evidence, last_round, count = EpisodeProgress(), [], 0, 0
    notes_path = directory / "evidence_notes.json"
    if notes_path.exists():
        notes = _read_json(notes_path, 32_768)
        if notes.get("task") != task or not isinstance(notes.get("evidence"), list) or len(notes["evidence"]) > 8:
            raise ValueError("invalid historical image evidence")
        for item in notes["evidence"]:
            if (not isinstance(item, dict) or type(item.get("round")) is not int or item["round"] < 1
                    or not isinstance(item.get("stage"), str) or not 0 < len(item["stage"]) <= 200):
                raise ValueError("invalid historical evidence entry")
            check = parse_verification({key: item.get(key) for key in ("complete", "reason")})
            evidence.append({"round": item["round"], "stage": item["stage"],
                             "complete": check["complete"], "reason": check["reason"]})
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            count += 1
            if count > 10_000:
                raise ValueError("continuation supports at most 10000 source turns")
            row = json.loads(line)
            number = row.get("round")
            if type(number) is not int or number <= last_round:
                raise ValueError("source rounds must be strictly increasing positive integers")
            last_round = number
            diagnostics = {}
            stages = parse_plan({"stages": [row.get("stage")]}, task=task, diagnostics=diagnostics)
            if diagnostics["status"] != "accepted":
                raise ValueError("continuation requires layered stage logs")
            stage = stages[0]
            progress.enter_stage(stage)
            status = row.get("status")
            if status not in {"succeeded", "failed", "unknown", "cancelled", "rejected", "waited",
                              "stage_complete", "not_complete", "replan_requested", "queried"}:
                raise ValueError("unrecognized historical action status")
            record = {"status": status}
            try:
                record["action"] = parse_action(row.get("action"))
            except (TypeError, ValueError, KeyError):
                record["action"] = {"action": "invalid", "reason": "historical rejected/invalid response"}
            if isinstance(row.get("reason"), str):
                record["reason"] = row["reason"][:500]
            actual = row.get("actual_feedback")
            if isinstance(actual, dict):
                clean = {key: vector for key, size in (("joint_rad", 6), ("tool_pose", 7), ("flange_pose", 7))
                         if (vector := _vector(actual.get(key), size)) is not None}
                gripper = actual.get("gripper")
                if type(gripper) in (int, float) and math.isfinite(gripper) and 0 <= gripper <= 1:
                    clean["gripper"] = gripper
                if clean:
                    clean["source"] = "historical measured feedback; recheck current robot"
                    record["actual_feedback"] = clean
            effect = row.get("measured_effect")
            delta = _vector(effect.get("delta_base_m"), 3) if isinstance(effect, dict) else None
            if delta is not None:
                record["measured_effect"] = {"delta_base_m": delta, "source": "historical measured FK delta"}
            check = row.get("verification")
            if isinstance(check, dict):
                record["verification"] = parse_verification({key: check.get(key) for key in ("complete", "reason")})
                item = {"round": number, "stage": stage["id"],
                        "complete": check["complete"], "reason": check["reason"]}
                if item not in evidence:
                    evidence.append(item)
            progress.record(record, stage, number)
    if not count:
        raise ValueError("no completed controller turns to continue from")
    # Old measurements and image verdicts are history only. Do not restore waits,
    # failure budgets, blocked commands, images, point queries or pending paths.
    progress.replan_reason = "explicit continuation from prior logs; re-evaluate the current scene before any action"
    progress.begin_plan()
    return {"task": task, "source_directory": str(directory), "last_round": last_round,
            "progress": progress, "evidence": sorted(evidence, key=lambda item: item["round"])[-8:]}
