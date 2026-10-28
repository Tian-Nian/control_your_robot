"""Continuous visual agent via a configurable API; Enter stops, --attach keeps the service alive."""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from piper_mujoco_harness import scene_session
from robot.harness import PositionExecutor
from robot.harness.progress import EpisodeProgress
from robot.harness.continuation import load_continuation
from robot.harness.supervision import decide_next, verification_trigger
from robot.harness.visual_execution import execute_motion
from robot.harness.visual_tracking import RGBTracks, image_id
from robot.harness.visual_plans import MotionPlans, observation_id
from robot.harness.recovery import recovery_options
from robot.harness.scene_reference import load_height_reference, height_feedback
from robot.harness.policy import interpret, parse_action, parse_plan, parse_verification, wrist_tool_guide
from robot.harness.vision import (LOWER, UPPER, Stopped, build_messages, chat_action,
                                  eef_motion_path, interruptible_call, observation, one_cm_options, orientation_feedback, resolve_action, validate_action)


def load_notes(path, task):
    """Reuse structured image evidence only on explicit request for the same task."""
    if path.stat().st_size > 16_384:
        raise ValueError("notes file is too large")
    value = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(value, dict) or set(value) != {"version", "task", "evidence"}
            or type(value["version"]) is not int or value["version"] != 1 or value["task"] != task
            or not isinstance(value["evidence"], list) or len(value["evidence"]) > 8):
        raise ValueError("notes must be version 1 evidence for exactly this task")
    for item in value["evidence"]:
        if (not isinstance(item, dict) or set(item) != {"round", "stage", "complete", "reason"}
                or type(item["round"]) is not int or item["round"] < 1
                or type(item["complete"]) is not bool
                or any(not isinstance(item[k], str) or not item[k].strip() or len(item[k]) > 200
                       for k in ("stage", "reason"))):
            raise ValueError("notes must contain short image evidence records only")
    return value["evidence"]


def run_loop(session, policy, stop, output, *, task, interval=1.0, max_rounds=500, max_waits=3,
             kinematics=None, max_failures=3, legacy_actions=True, planner=None, verifier=None,
             dry_run=False, max_replans=2, stage_rounds=12, notes=None, max_wait_replans=3, resume=None, verification_interval=0, allow_depth_point=True, grounded_tools=False):
    """Callbacks receive only messages. CLI uses layered mode; legacy callers opt into old actions.

    The programmatic default preserves the existing low-level loop adapter. Pass
    legacy_actions=False for the CLI's planner/controller/verifier contract.
    At most 500 additional controller rounds by default; max_rounds=0 removes
    that cap. Explicit continuation carries history, not pending commands.
    """
    if max_replans < 0 or max_wait_replans < 0 or stage_rounds < 1 or verification_interval < 0:
        raise ValueError("replan budgets must be nonnegative and stage_rounds positive")
    if resume is not None and (legacy_actions or resume["task"] != task):
        raise ValueError("continuation requires layered mode and the original task")
    spec = next((s for s in session.describe() if s["name"] == "left_arm"), {})
    if (spec.get("class_path") != "robot.controller.PiperMuJoCo_controller"
            or spec.get("class_name") != "PiperMuJoCoController"):
        raise RuntimeError("this example requires the Piper MuJoCo simulator")
    # Session.describe intentionally hides constructor args. Read only the local
    # declared scene path; never copy the runtime config or scene file to the API.
    component = next((item for item in getattr(getattr(session, "_config", None), "components", [])
                      if item.name == "left_arm"), None)
    scene_args = component.args if component is not None else spec.get("args", {})
    height_reference = load_height_reference(scene_args.get("scene_path"))
    output.mkdir(parents=True, exist_ok=False)
    print(f"输入审计与执行日志：{output.resolve()}", flush=True)
    tracks, motion_plans = RGBTracks(), MotionPlans()
    history, previous = [], None
    previous_rgb = None
    previous_image_timestamps = None
    start_round = resume["last_round"] if resume else 0
    round_index = start_round
    stage_review_count = 0
    wait_count = 0
    wait_replan_count = 0
    wait_anchor = None
    plan_kind = "resume" if resume else "initial"
    failure_count = 0
    rejection_count = 0
    blocked_signature = None
    blocked_action = last_failed = point_query = None
    stages, stage_index, stage_turns, replan_count = None, 0, 0, 0
    needs_replan = False
    task_complete = False
    exit_reason = "user_or_external_stop"
    evidence = list(resume["evidence"]) if resume else []
    progress = resume["progress"] if resume else EpisodeProgress()
    history = progress.recent
    run_context = {"version": 1, "task": task, "start_round": start_round,
                   "continuation": {"source_round": start_round} if resume else None,
                   "verification_interval": verification_interval, "allow_depth_point": allow_depth_point, "grounded_tools": grounded_tools,
                   "limits": {"max_rounds": max_rounds, "stage_rounds": stage_rounds,
                              "max_replans": max_replans, "max_wait_replans": max_wait_replans}}
    (output / "run_context.json").write_text(json.dumps(run_context, ensure_ascii=False, indent=2), encoding="utf-8")
    if resume:
        print(f"继续上次第 {start_round} 轮后的任务：重新读取当前图像和状态，先规划；不回放旧动作。", flush=True)
    stage_start = None
    finished = threading.Event()
    holder = [None]

    def acquire(previous, *, local=None):
        # Fast callbacks may finish before the 10 Hz camera publishes again.
        # Wait only for a new frame; stale/unhealthy feedback still fails closed.
        deadline = time.monotonic() + 1.0
        while not stop.is_set():
            try:
                captured = local if local is not None else {}
                obs, stamps = observation(session, previous, local=captured)
                if grounded_tools and not legacy_actions:
                    tracks.update(captured)
                    obs["observation_id"] = observation_id(stamps)
                    obs["image_ids"] = {name: image_id(name, stamp) for name, stamp in stamps.items()}
                    obs["visual_panel"] = {"rgb_tracking": tracks.summary(), "motion_plans": motion_plans.summaries(),
                        "motion_epoch": motion_plans.epoch, "plan_requires_fresh_execution_check": True}
                return obs, stamps
            except RuntimeError as exc:
                if str(exc) != "camera did not advance" or time.monotonic() >= deadline:
                    raise
            if stop.wait(.02):
                break
        raise Stopped()

    def enrich_observation(obs, cameras):
        obs["sensor_policy"] = {"single_camera_depth_point_allowed": allow_depth_point,
                                "default_observation": "head/wrist RGB + robot proprioception"}
        obs["loop_status"] = {"consecutive_waits": wait_count, "wait_trigger": max_waits,
            "wait_replans_used": wait_replan_count, "wait_replans_remaining": max_wait_replans - wait_replan_count,
            "since_last_wait": None}
        if wait_count and wait_anchor:
            obs["loop_status"]["since_last_wait"] = {
                "identical_rgb_bytes": obs["rgb"] == wait_anchor["rgb"],
                "max_joint_change_rad": max(abs(a - b) for a, b in zip(obs["joint_rad"], wait_anchor["joint"])),
                "gripper_change": abs(obs["gripper"] - wait_anchor["gripper"])}
        if progress.recovery_context:
            obs["recovery_context"] = dict(progress.recovery_context)
        if kinematics is None:
            return
        obs["eef_fk"] = {"source": "joint feedback + Pinocchio FK", "frame": "base",
            "format": "xyz_m + quaternion_wxyz",
            **{point: kinematics.pose(obs["joint_rad"], point) for point in ("tool", "flange")}}
        obs["eef_fk"]["one_cm_translation_options"] = one_cm_options(obs["joint_rad"], kinematics)
        obs["eef_fk"]["tool_translation_options"] = one_cm_options(obs["joint_rad"], kinematics, frame="tool")
        obs["eef_fk"].update(orientation_feedback(obs["joint_rad"], kinematics))
        obs["wrist_tool_guide"] = wrist_tool_guide(obs["joint_rad"], kinematics, cameras)
        obs["scene_reference"] = height_feedback(height_reference, obs["eef_fk"])
        if wait_count or progress.recovery_context:
            obs["recovery_options"] = recovery_options(obs["joint_rad"], kinematics, obs["eef_fk"])

    def persist_turn(record, stage, number, action):
        nonlocal history
        if record["status"] == "verification_requested":
            record = {**record, "status": "not_complete", "reason": "image verification interrupted"}
        progress.record(record, stage, number)
        history = progress.recent
        log.write(json.dumps({"round": number, "stage": stage, "raw_action": action, **record}) + "\n")
        log.flush()

    def model_call(role, messages, callback):
        # Log the exact allowlisted role input before the call, including interrupted calls.
        (output / "latest_messages.json").write_text(json.dumps(messages, ensure_ascii=False), encoding="utf-8")
        entry = {"round": round_index, "role": role, "messages": messages}
        with (output / "model_calls.jsonl").open("a", encoding="utf-8") as audit:
            audit.write(json.dumps({**entry, "event": "request"}, ensure_ascii=False) + "\n")
            audit.flush()
            try:
                reply = interruptible_call(lambda: callback(messages), stop)
            except Exception as exc:
                audit.write(json.dumps({"round": round_index, "role": role, "event": "error",
                                       "error_type": type(exc).__name__}) + "\n")
                raise
            audit.write(json.dumps({"round": round_index, "role": role, "event": "response",
                                   "raw": reply}, ensure_ascii=False) + "\n")
            return reply

    def cancel_on_stop():
        while not finished.wait(.01):
            if stop.is_set() and holder[0] is not None:
                executor = holder[0]
                active = executor.get_status()["active_request"]
                if active:
                    executor.cancel(active)

    watcher = threading.Thread(target=cancel_on_stop, daemon=True)
    watcher.start()
    try:
        with (output / "turns.jsonl").open("x") as log:
            while not stop.is_set():
                # Bound executor deduplication memory for arbitrarily long sessions.
                with PositionExecutor(session, lower=LOWER, upper=UPPER, max_age_ms=500,
                                      journal=output / f"control_{round_index:06d}.jsonl") as executor:
                    holder[0] = executor
                    for _ in range(128):
                        if stop.is_set():
                            break
                        cameras = {}
                        obs, previous = acquire(previous, local=cameras)
                        if previous_rgb is not None:
                            obs["previous_rgb"] = previous_rgb
                            obs["previous_image_timestamps_ns"] = previous_image_timestamps
                        enrich_observation(obs, cameras)
                        obs["task_progress"] = progress.snapshot(stages[stage_index] if stages else None)
                        if not legacy_actions and (stages is None or needs_replan):
                            if stages is not None:
                                if plan_kind == "wait_recovery":
                                    wait_replan_count += 1
                                elif plan_kind == "stage_review":
                                    stage_review_count += 1
                                else:
                                    replan_count += 1
                            obs["loop_status"].update(wait_replans_used=wait_replan_count,
                                wait_replans_remaining=max_wait_replans - wait_replan_count)
                            if plan_kind != "initial":
                                obs["planning_context"] = {"kind": plan_kind, "periodic_reviews_used": stage_review_count}
                                if resume:
                                    obs["planning_context"]["continued_from_round"] = start_round
                            plan_diagnostics = {}
                            if dry_run:
                                stages = parse_plan(None, task=task)
                            else:
                                try:
                                    raw_plan = model_call("planner", build_messages(obs, task, [], role="planner", notes=notes),
                                                          planner or policy)
                                except Stopped:
                                    raise
                                except Exception:
                                    raw_plan = None
                                stages = parse_plan(raw_plan, task=task, diagnostics=plan_diagnostics)
                                if plan_diagnostics["status"] == "fallback":
                                    print("规划响应无效或不可用：已降级为单阶段；详情见 plans.jsonl。", flush=True)
                            stages = progress.adopt_plan(stages)
                            with (output / "plans.jsonl").open("a", encoding="utf-8") as plan_log:
                                plan_log.write(json.dumps({"round": round_index, "replan": replan_count, "wait_replan": wait_replan_count, "kind": plan_kind, "stage_review": stage_review_count,
                                    **plan_diagnostics, "stages": stages, "progress": obs["task_progress"]}, ensure_ascii=False) + "\n")
                            progress.begin_plan()
                            if progress.recovery_context:
                                progress.recovery_context["status"] = "replanned"
                            stage_start = None
                            stage_index, stage_turns, needs_replan = 0, 0, False
                            if not dry_run:
                                cameras = {}
                                obs, previous = acquire(previous, local=cameras)
                                enrich_observation(obs, cameras)
                                if previous_rgb is not None:
                                    obs["previous_rgb"] = previous_rgb
                                    obs["previous_image_timestamps_ns"] = previous_image_timestamps
                        stage = stages[stage_index] if stages else None
                        if stage_start is None:
                            progress.enter_stage(stage)
                            stage_start = (dict(obs["rgb"]), dict(obs["image_timestamps_ns"]))
                        obs["task_progress"] = progress.snapshot(stage)
                        obs.update(blocked_action=blocked_action, last_failed_action=last_failed,
                                   consecutive_failures=max(failure_count, rejection_count), point_query=point_query)
                        if obs.get("scene_reference", {}).get("available"):
                            ref = obs["scene_reference"]
                            print(f"  高度反馈：工具 Z={ref['tool_z_base_m']:.3f} m；桌面 Z={ref['table_top_z_m']:.3f} m；工具点离桌={ref['tool_height_above_table_m']:.3f} m", flush=True)
                        messages = build_messages(obs, task, history, stage=stage, notes=notes, legacy=legacy_actions)
                        previous_rgb = obs["rgb"]
                        previous_image_timestamps = dict(obs["image_timestamps_ns"])
                        round_index += 1
                        stage_turns += 1
                        print(f"第 {round_index} 轮：发送 head/wrist RGB + 关节/夹爪状态"
                              + (" + Pinocchio FK 末端位姿" if kinematics else ""), flush=True)
                        callback = (lambda _: {"action": "wait", "reason": "离线保持位置"}) if dry_run else policy
                        action = model_call("controller", messages, callback)
                        if stop.is_set():
                            break
                        pause_reason = None
                        try:
                            # Validate against both the pictured and current pose; reject drift.
                            if legacy_actions:
                                validate_action(action, obs["joint_rad"], kinematics)
                                current_joint = executor.observe()["joint"]
                                resolved, resolution = resolve_action(action, current_joint, kinematics)
                                target = validate_action(resolved, current_joint, kinematics)
                                if resolved["action"] == "move_eef" and resolution is None:
                                    resolution = {"point": resolved["point"], "target_pose": resolved["pose"],
                                                  "waypoints": eef_motion_path(resolved, current_joint, kinematics)}
                            else:
                                action = parse_action(action)
                                if action["action"] == "point" and "second" not in action and not allow_depth_point:
                                    raise ValueError("single-camera depth point is disabled; use RGB or an explicitly matched two-view query")
                                stateful = action["action"] in ("track_point", "forget_track", "plan_motion", "execute_plan")
                                if stateful:
                                    if not grounded_tools:
                                        raise ValueError("grounded tools are disabled in this session")
                                    current_joint = executor.observe()["joint"]
                                    name = action["action"]
                                    if name == "track_point":
                                        resolution = tracks.register(action["camera"], action["image_id"], action["pixel"], action["label"])
                                        target = None
                                    elif name == "forget_track":
                                        resolution = tracks.forget(action["track_id"])
                                        target = None
                                    elif name == "plan_motion":
                                        resolution = motion_plans.create(action, obs, cameras, kinematics, tracks)
                                        target = None
                                    else:
                                        # Check live pixels again after inference, before consuming the cached plan.
                                        live_obs, previous = acquire(previous)
                                        measured = executor.observe()
                                        current_joint = measured["joint"]
                                        target, resolution = motion_plans.admit(action, obs, measured, kinematics, tracks, latest_obs=live_obs)
                                    resolved = ({"action": "move_eef", "pose": resolution["target_pose"],
                                                 "frame": "base", "point": "tool"} if target else action)
                                else:
                                    # The point uses exactly the pictured snapshot, even if new frames arrived during HTTP.
                                    interpret(action, obs["joint_rad"], kinematics, cameras=cameras)
                                    current_joint = executor.observe()["joint"]
                                    if action["action"] in ("tool_move", "tool_rotate"):
                                        pe, re = kinematics.error(current_joint, kinematics.pose(obs["joint_rad"], "tool"), "tool")
                                        if pe > .001 or re > .01:
                                            raise ValueError("tool pose changed since pictured snapshot; reobserve before a tool-frame step")
                                    target, resolution = interpret(action, current_joint, kinematics, cameras=cameras)
                                    resolved = ({"action": "move_eef", "pose": resolution["target_pose"],
                                                 "frame": "base", "point": resolution["point"]}
                                                if "target_pose" in resolution else action)
                            command = {k: v for k, v in action.items() if k != "reason"}
                            if action["action"] == "execute_plan":
                                # A new opaque plan ID must not bypass the failed-command guard.
                                command = {"action": "planned_eef", "delta_xyz": resolution["resolved_delta_xyz_m"],
                                           "delta_rotvec": resolution["delta_rotvec_base_rad"]}
                            # Equivalent numeric spellings must not bypass the failed-action guard.
                            for key in ("joint", "pose", "delta_xyz", "delta_rotvec"):
                                if key in command:
                                    command[key] = [round(float(v), 9) + 0.0 for v in command[key]]
                            if action["action"] == "move_eef_delta":
                                command.setdefault("delta_rotvec", [0.0, 0.0, 0.0])
                            if "gripper" in command:
                                command["gripper"] = float(command["gripper"])
                            if action["action"] in ("move", "tool_move"):
                                command["step_cm"] = float(command.get("step_cm", 1))
                            signature = json.dumps(command, sort_keys=True)
                            if target and signature == blocked_signature:
                                raise ValueError("same action failed previously; choose a different direction/posture before retrying")
                        except ValueError as exc:
                            wait_count = 0
                            rejection_count += 1
                            if rejection_count >= max_failures:
                                pause_reason = f"连续 {max_failures} 轮动作被拒绝"
                            print(f"动作拒绝：{exc}", flush=True)
                            # Do not echo untrusted/raw model output back into the observation.
                            record = {"action": "invalid", "status": "rejected", "reason": str(exc)}
                            if isinstance(action, dict) and set(action) <= {"action", "pose", "frame", "point", "joint", "gripper", "delta_xyz", "delta_rotvec", "direction", "step_cm", "axis", "joint_index", "sign", "sense", "reason"}:
                                record["requested_action"] = action
                        else:
                            rejection_count = 0
                            if stop.is_set():
                                break
                            reason = action.get("reason", "模型未提供原因")
                            print(f"  模型说明（非验收结论）：{reason}", flush=True)
                            if target:
                                wait_count = 0
                                motion_plans.invalidate_for_motion()
                                outcome, segments = execute_motion(executor, f"turn-{round_index}", target,
                                                                   resolution, kinematics, stop)
                                record = {"action": action, "status": outcome["status"], "reason": outcome["reason"],
                                          "stop_confirmed": outcome.get("stop_confirmed", False)}
                                if segments:
                                    record["execution_segments"] = segments
                                if resolution:
                                    record["action_resolution"] = resolution
                                    if "resolved_delta_xyz_m" in resolution:
                                        print(f"  EEF 请求增量：{resolution['resolved_delta_xyz_m']} m（比例 {resolution['scale']}）", flush=True)
                                        print(f"  EEF 基座旋转增量：{resolution['delta_rotvec_base_rad']} rad", flush=True)
                                if outcome.get("stop_feedback"):
                                    actual = outcome["stop_feedback"]
                                    record["actual_feedback"] = {"joint_rad": actual["joint"], "gripper": actual["gripper"],
                                        "source": "measured joint/gripper feedback"}
                                    if kinematics:
                                        before_pose = kinematics.pose(current_joint, "tool")
                                        after_pose = kinematics.pose(actual["joint"], "tool")
                                        record["measured_effect"] = {
                                            "delta_base_m": [float(b - a) for a, b in zip(before_pose[:3], after_pose[:3])],
                                            "source": "pre/post measured joints + Pinocchio FK"}
                                        record["actual_feedback"].update(
                                            tool_pose=kinematics.pose(actual["joint"], "tool"),
                                            flange_pose=kinematics.pose(actual["joint"], "flange"),
                                            pose_source="joint feedback + Pinocchio FK")
                                if resolved["action"] == "move_eef" and outcome.get("stop_feedback"):
                                    pe, re = kinematics.error(outcome["stop_feedback"]["joint"],
                                                              resolved["pose"], resolved["point"])
                                    record["eef_verification"] = {"position_error_m": pe,
                                        "rotation_error_rad": re, "source": "joint feedback + Pinocchio FK"}
                                    if outcome["status"] == "succeeded" and (pe > .001 or re > .01):
                                        outcome["status"] = record["status"] = "failed"
                                        record["reason"] = "EEF feedback FK outside tolerance"
                                    print(f"  EEF FK 误差：{pe:.6f} m / {re:.6f} rad", flush=True)
                                print(f"  {action['action']}: {outcome['status']}", flush=True)
                                if outcome["status"] != "succeeded":
                                    failure_count += 1
                                    blocked_signature = signature
                                    blocked_action = command
                                    last_failed = {key: record[key] for key in ("action", "status", "reason", "actual_feedback", "eef_verification") if key in record}
                                    print(f"  执行原因：{record['reason']}；停止确认：{record['stop_confirmed']}", flush=True)
                                    recoverable = (outcome["status"] == "failed" and record["stop_confirmed"]
                                        and record["reason"] in ("verification timeout", "EEF feedback FK outside tolerance", "waypoint joint revalidation failed")
                                        and failure_count < max_failures and not stop.is_set())
                                    if recoverable:
                                        try:
                                            # Explicit simulator policy: verify hold/fresh feedback again,
                                            # then let the model choose a NEW action. Never retry here.
                                            executor.observe()
                                            if stop.is_set():
                                                raise Stopped()
                                            recovery = executor.recover()
                                            record["recovery"] = {"status": recovery["status"],
                                                "policy": "verified simulator hold; reobserve and replan, no automatic retry"}
                                            print("  保持与反馈已重新确认；下一轮由模型重新观察并调整姿势。", flush=True)
                                        except Stopped:
                                            record["recovery"] = {"status": "skipped", "reason": "user stop"}
                                        except Exception as exc:
                                            record["recovery"] = {"status": "failed", "reason": str(exc)}
                                            pause_reason = "恢复验证失败"
                                    elif not stop.is_set():
                                        pause_reason = (f"连续 {max_failures} 次执行失败" if failure_count >= max_failures
                                                        else "停止/反馈状态不满足恢复条件")
                                else:
                                    failure_count = 0
                                    blocked_signature = blocked_action = None
                            elif not legacy_actions and action["action"] in ("track_point", "forget_track", "plan_motion"):
                                wait_count = 0
                                record = {"action": action, "status": "queried", "tool_result": resolution}
                                print(f"  {action['action']}：只读结果已记录，未发送运动目标。", flush=True)
                            elif not legacy_actions and action["action"] == "point":
                                wait_count = 0
                                point_query = resolution
                                record = {"action": action, "status": "queried", "action_resolution": point_query}
                            elif not legacy_actions and action["action"] == "replan":
                                wait_count = 0
                                record = {"action": action, "status": "replan_requested"}
                                print("  replan：当前策略不再适用，请求带新反馈的恢复规划。", flush=True)
                            elif not legacy_actions and action["action"] == "done":
                                wait_count = 0
                                record = {"action": action, "status": "verification_requested"}
                            else:
                                wait_count += 1
                                wait_anchor = {"rgb": dict(obs["rgb"]), "joint": list(obs["joint_rad"]), "gripper": obs["gripper"]}
                                record = {"action": action, "status": "waited"}
                                print(f"  wait：保持位置，连续等待 {wait_count} 轮", flush=True)
                        check_trigger = ("controller_done" if record["status"] == "verification_requested" else
                            verification_trigger(record, stage, interval=verification_interval,
                                                 stage_turns=stage_turns, grasp_state=progress.grasp_state))
                        if check_trigger and not pause_reason and not stop.is_set() and not legacy_actions:
                            if dry_run:
                                check = {"complete": False, "reason": "dry run does not verify completion"}
                            else:
                                try:
                                    fresh, previous = acquire(previous)
                                except Exception:
                                    persist_turn(record, stage, round_index, action)
                                    raise  # Stale/unsafe feedback is not a mere negative visual verdict.
                                fresh["stage_start_rgb"], fresh["stage_start_image_timestamps_ns"] = stage_start
                                previous_rgb = fresh["rgb"]
                                previous_image_timestamps = dict(fresh["image_timestamps_ns"])
                                try:
                                    raw_check = model_call("verifier", build_messages(fresh, task, [], role="verifier", stage=stage),
                                                           verifier or policy)
                                    check = parse_verification(raw_check)
                                except Stopped:
                                    persist_turn(record, stage, round_index, action)
                                    raise
                                except Exception:
                                    check = {"complete": False, "reason": "verifier response unavailable or invalid"}
                            record.update(verification=check, verification_trigger=check_trigger,
                                          verification_image_timestamps_ns=dict(previous))
                            if check_trigger == "controller_done":
                                record["status"] = "stage_complete" if check["complete"] else "not_complete"
                            evidence.append({"round": round_index, "stage": stage["id"],
                                             "complete": check["complete"], "reason": check["reason"]})
                            evidence = evidence[-8:]
                            print(f"  图像验收（{check_trigger}）：{'通过' if check['complete'] else '未通过'}；{check['reason']}", flush=True)
                            if check["complete"]:
                                stage_index += 1
                                stage_start, stage_turns, point_query = None, 0, None
                                if stage_index == len(stages):
                                    task_complete = True
                                    stop.set()
                        persist_turn(record, stage, round_index, action)
                        if max_rounds and round_index - start_round >= max_rounds:
                            exit_reason = "max_rounds"
                            if not stop.is_set():
                                print(f"已达到最大控制轮数 {max_rounds}，结束本次运行。", flush=True)
                            stop.set()
                        if (not stop.is_set() and not legacy_actions and not dry_run
                                and not pause_reason and not task_complete):
                            decision = decide_next(record=record, observation=obs, stagnation=progress.stagnation,
                                stage_turns=stage_turns, stage_rounds=stage_rounds,
                                failures=max(failure_count, rejection_count), waits=wait_count, max_waits=max_waits,
                                replans=replan_count, max_replans=max_replans,
                                recovery_plans=wait_replan_count, max_recovery_plans=max_wait_replans)
                            with (output / "decisions.jsonl").open("a", encoding="utf-8") as decisions:
                                decisions.write(json.dumps({"round": round_index, "kind": decision.kind,
                                    "reason": decision.reason, "trigger": decision.trigger,
                                    "stage_turns": stage_turns, "stall_plans_used": replan_count,
                                    "recovery_plans_used": wait_replan_count}, ensure_ascii=False) + "\n")
                            if decision.kind == "pause":
                                pause_reason = decision.reason
                            elif decision.kind != "continue":
                                progress.replan_reason = decision.reason
                                plan_kind, needs_replan = decision.kind, True
                                if decision.kind == "wait_recovery":
                                    progress.recovery_context = {
                                        "trigger": decision.trigger, "status": "pending", "attempt": wait_replan_count + 1,
                                        "wait_streak": wait_count, "last_stage_id": stage["id"],
                                        "grasp_state": progress.grasp_state,
                                        "last_wait_feedback": obs["loop_status"]["since_last_wait"],
                                        "source": "loop/action history; not a failed-grasp or contact oracle"}
                                    wait_count = 0
                                print(f"调度 {decision.kind}：{decision.reason}", flush=True)
                        if (wait_count >= max_waits and (legacy_actions or dry_run) and not stop.is_set()):
                            pause_reason = f"连续 {max_waits} 轮 wait：兼容/离线模式已暂停模型调用"
                        if pause_reason and not stop.is_set():
                            exit_reason = pause_reason
                            suffix = "本次运行结束；可使用 --resume 日志目录继续。" if max_rounds and not legacy_actions else "按 Enter 退出。"
                            print(f"{pause_reason}，暂停运动并保留日志。{suffix}", flush=True)
                            if max_rounds:
                                stop.set()
                            else:
                                stop.wait()
                        if stop.wait(interval):
                            break
                holder[0] = None
    except Stopped:
        pass
    except Exception as exc:
        exit_reason = "error:" + type(exc).__name__
        raise
    finally:
        finished.set()
        watcher.join(timeout=1)
        (output / "run_summary.json").write_text(json.dumps({
            "task_complete": task_complete, "exit_reason": "verified_complete" if task_complete else exit_reason,
            "controller_rounds_started": round_index - start_round, "last_round": round_index,
            "stage_reviews": stage_review_count, "stall_replans": replan_count, "recovery_replans": wait_replan_count,
            "attempt_count": progress.attempt_count, "last_image_check": progress.last_check},
            ensure_ascii=False, indent=2), encoding="utf-8")
        (output / "evidence_notes.json").write_text(json.dumps(
            {"version": 1, "task": task, "evidence": evidence}, ensure_ascii=False, indent=2), encoding="utf-8")
    print("任务已由独立图像验证完成。" if task_complete else "harness 已退出；任务未确认完成。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--architecture", choices=("robowork", "layered"), default="robowork", help="Default: RoboHarness-style single agent and RGB-D tools; layered retains the previous implementation")
    parser.add_argument("--attach", action="store_true", help="Connect to cyr_piper_mujoco")
    parser.add_argument("--viewer", action="store_true", help="Open Rerun for an owned scene")
    parser.add_argument("--api-config", type=Path, default=Path.home() / ".config/control_your_robot/vision_api.json")
    parser.add_argument("--base-url", help="OpenAI-compatible API base URL")
    parser.add_argument("--model", help="Model ID; overrides private API configuration")
    parser.add_argument("--api-key-env", help="Environment variable containing the API key")
    parser.add_argument("--joint-only", action="store_true", help="Disable Pinocchio FK/IK and move_eef")
    parser.add_argument("--task", help="Task text; continuation keeps the original task")
    parser.add_argument("--resume", type=Path, help="Continue from a prior vision log directory using fresh images and a new plan; implies --attach")
    parser.add_argument("--legacy-actions", action="store_true", help="Compatibility mode: low-level actions, no planner or completion claims")
    parser.add_argument("--max-replans", type=int, default=2, help="Maximum stall-recovery plans; periodic successful-execution reviews do not use this budget")
    parser.add_argument("--stage-rounds", type=int, default=12, help="Review the plan after this many stage turns; successful execution does not exhaust recovery plans")
    parser.add_argument("--verification-interval", type=int, default=6, help="Automatic grasp/lift image check interval; close/upward probes check immediately; 0 disables")
    parser.add_argument("--no-grounded-tools", action="store_true", help="Disable RGB tracks and plan/execute tools; retain direct actions")
    parser.add_argument("--allow-depth-point", action="store_true", help="Explicitly enable single-camera point depth queries (default: RGB only)")
    parser.add_argument("--notes", type=Path, help="Explicitly reuse structured evidence_notes.json for exactly this task")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between completed rounds")
    parser.add_argument("--max-rounds", type=int, default=None,
                        help="Maximum tool rounds (robowork default: unlimited; layered: 500); 0 removes the cap; on resume counts additional rounds; Enter stops early")
    parser.add_argument("--max-waits", type=int, default=3, help="Consecutive waits that trigger a fresh recovery plan; legacy/dry-run pause")
    parser.add_argument("--max-wait-replans", type=int, default=3, help="Additional wait/explicit recovery plans, separate from --max-replans; 0 disables")
    parser.add_argument("--max-failures", type=int, default=3, help="Pause after consecutive unsuccessful executed actions")
    parser.add_argument("--dry-run", action="store_true", help="Local wait policy; NO API or autonomous motion")
    parser.add_argument("--out", type=Path, default=ROOT / "save" / f"vision_{uuid.uuid4().hex[:10]}")
    args = parser.parse_args()
    retired = {"--max-replans", "--stage-rounds", "--verification-interval", "--no-grounded-tools",
               "--allow-depth-point", "--max-waits", "--max-wait-replans", "--max-failures"}
    if args.architecture == "robowork" and any(a.split("=")[0] in retired for a in sys.argv[1:]):
        parser.error("old stage/recovery/RGB-only switches belong to --architecture layered; robowork does not use them")
    if args.max_rounds is None:
        args.max_rounds = 0 if args.architecture == "robowork" else 500
    if args.architecture == "robowork" and (args.legacy_actions or args.joint_only or args.notes):
        parser.error("robowork needs Pinocchio and its own tool protocol; legacy-actions/joint-only/notes belong to --architecture layered")
    if args.attach and args.viewer:
        parser.error("--attach uses the running service's viewer settings")
    if (not .2 <= args.interval <= 60 or args.max_rounds < 0 or args.max_waits < 1
            or args.max_failures < 1 or args.max_replans < 0 or args.max_wait_replans < 0 or args.stage_rounds < 1 or args.verification_interval < 0):
        parser.error("interval must be 0.2..60; rounds/replans nonnegative; waits/failures/stage-rounds positive")
    resume = None
    if args.resume:
        if args.legacy_actions or args.viewer:
            parser.error("--resume requires the existing service (--attach), without --legacy-actions or --viewer")
        try:
            if args.architecture == "robowork":
                from robot.harness.robowork import load_workspace_resume
                resume = load_workspace_resume(args.resume)
            else:
                resume = load_continuation(args.resume)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            parser.error(f"Cannot continue this log directory: {exc}")
        if args.task is not None and args.task != resume["task"]:
            parser.error("--resume keeps the original task; omit --task or supply exactly the same text")
        args.task = resume["task"]
        args.attach = True
    elif args.task is None:
        args.task = "夹住并抬起橙色碗，以新图像中碗随夹爪离开桌面作为完成证据。"
    notes = None
    if args.notes:
        try:
            notes = load_notes(args.notes, args.task)
        except (OSError, ValueError, TypeError):
            parser.error("notes must be a valid, bounded evidence_notes.json for exactly this task")
    config = {}
    if args.api_config.exists():
        try:
            config = json.loads(args.api_config.read_text())
            if not isinstance(config, dict) or set(config) - {"base_url", "model", "api_key", "api_key_env"}:
                raise ValueError()
        except (OSError, ValueError):
            parser.error("Cannot read API configuration; expected base_url/model/api_key/api_key_env JSON fields")
    base_url = args.base_url or config.get("base_url", "https://api.deepseek.com")
    model = args.model or config.get("model", "deepseek-flash")
    if not isinstance(base_url, str) or not isinstance(model, str) or not model.strip():
        parser.error("API base URL and model must be nonempty strings")
    from urllib.parse import urlsplit
    parsed = urlsplit(base_url)
    if parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        parser.error("API base URL must be HTTP(S), without embedded credentials, query or fragment")
    # CLI endpoint changes must not accidentally reuse a key from another server.
    changed_endpoint = args.base_url is not None and args.base_url.rstrip("/") != str(config.get("base_url", "")).rstrip("/")
    key_env = args.api_key_env or (None if changed_endpoint else config.get("api_key_env"))
    if key_env:
        key = os.environ.get(key_env, "")
    elif changed_endpoint:
        key = os.environ.get("CYR_VISION_API_KEY", "")
    else:
        key = config.get("api_key") or os.environ.get("CYR_VISION_API_KEY", "") or os.environ.get("DEEPSEEK_API_KEY", "")
    if not args.dry_run and not key:
        parser.error("Set api_key in the private API config or export the variable selected by --api-key-env")
    kinematics = None
    if not args.joint_only:
        try:
            from robot.harness.kinematics import PiperKinematics
            kinematics = PiperKinematics()
        except ImportError:
            parser.error("EEF control requires Pinocchio: python -m pip install -r requirements/harness-vision.txt; or use --joint-only")
    args.no_cameras = False
    from robot.harness.stop_signal import StopSignal, watch_terminal
    stop = StopSignal()
    # EOF still stops, but is distinct from pressing Enter in the audit.
    threading.Thread(target=watch_terminal, args=(sys.stdin, stop), daemon=True).start()
    print("按 Enter 停止；模型请求期间也可停止，迟到回复不会执行。", flush=True)
    print("DRY RUN：本地 wait 策略，不调用模型。" if args.dry_run else f"视觉模型：{model}；接口：{base_url}", flush=True)
    print("RoboHarness 工具模式：单 agent / RGB-D / 持续夹爪命令。" if args.architecture == "robowork" else
          "旧分层模式：规划 / 离散动作 / 独立图像验收。", flush=True)
    policy = (lambda messages: ({"tool": "capture_camera", "arguments": {}, "reason": "离线观察"} if args.architecture == "robowork" else {"action": "wait", "reason": "离线保持位置"})) if args.dry_run else (
        lambda messages: chat_action(messages, key, model, base_url,
                                     deepseek_options=parsed.hostname == "api.deepseek.com"))
    try:
        with scene_session(args) as session:
            if args.architecture == "robowork":
                from robot.harness.robowork import run_workspace
                run_workspace(session, policy, stop, args.out, task=args.task, kinematics=kinematics,
                              max_rounds=args.max_rounds, interval=args.interval, resume=resume)
                return
            run_loop(session, policy, stop, args.out, task=args.task,
                     interval=args.interval, max_rounds=args.max_rounds, max_waits=args.max_waits,
                     kinematics=kinematics, max_failures=args.max_failures,
                     legacy_actions=args.legacy_actions, dry_run=args.dry_run,
                     max_replans=args.max_replans, stage_rounds=args.stage_rounds, notes=notes,
                     max_wait_replans=args.max_wait_replans, resume=resume,
                     verification_interval=args.verification_interval, allow_depth_point=args.allow_depth_point,
                     grounded_tools=not args.no_grounded_tools)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
