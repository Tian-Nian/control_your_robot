"""Continuous visual agent via a configurable API; Enter stops, --attach keeps the service alive."""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from piper_mujoco_harness import scene_session
from robot.harness import PositionExecutor
from robot.harness.vision import (LOWER, UPPER, Stopped, build_messages, chat_action,
                                  interruptible_call, observation, one_cm_options, orientation_feedback, resolve_action, validate_action)


def run_loop(session, policy, stop, output, *, task, interval=1.0, max_rounds=0, max_waits=3,
             kinematics=None, max_failures=3):
    """policy receives only the serialized allowlist, never a Session/executor."""
    spec = next((s for s in session.describe() if s["name"] == "left_arm"), {})
    if (spec.get("class_path") != "robot.controller.PiperMuJoCo_controller"
            or spec.get("class_name") != "PiperMuJoCoController"):
        raise RuntimeError("this example requires the Piper MuJoCo simulator")
    output.mkdir(parents=True, exist_ok=False)
    print(f"输入审计与执行日志：{output.resolve()}", flush=True)
    history, previous = [], None
    previous_rgb = None
    round_index = 0
    wait_count = 0
    failure_count = 0
    rejection_count = 0
    blocked_signature = None
    finished = threading.Event()
    holder = [None]

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
                        obs, previous = observation(session, previous)
                        if previous_rgb is not None:
                            obs["previous_rgb"] = previous_rgb
                        if kinematics is not None:
                            obs["eef_fk"] = {"source": "joint feedback + Pinocchio FK", "frame": "base",
                                             "format": "xyz_m + quaternion_wxyz",
                                             **{p: kinematics.pose(obs["joint_rad"], p) for p in ("tool", "flange")}}
                            obs["eef_fk"]["one_cm_translation_options"] = one_cm_options(obs["joint_rad"], kinematics)
                            obs["eef_fk"].update(orientation_feedback(obs["joint_rad"], kinematics))
                        messages = build_messages(obs, task, history)
                        previous_rgb = obs["rgb"]
                        # Exact model-visible messages for auditing; no credential or raw frame.
                        (output / "latest_messages.json").write_text(
                            json.dumps(messages, ensure_ascii=False), encoding="utf-8")
                        round_index += 1
                        print(f"第 {round_index} 轮：发送 head/wrist RGB + 关节/夹爪状态"
                              + (" + Pinocchio FK 末端位姿" if kinematics else ""), flush=True)
                        action = interruptible_call(lambda: policy(messages), stop)
                        if stop.is_set():
                            break
                        pause_reason = None
                        try:
                            # Validate against both the pictured and current pose; reject drift.
                            validate_action(action, obs["joint_rad"], kinematics)
                            current_joint = executor.observe()["joint"]
                            resolved, resolution = resolve_action(action, current_joint, kinematics)
                            target = validate_action(resolved, current_joint, kinematics)
                            command = {k: v for k, v in action.items() if k != "reason"}
                            # Equivalent numeric spellings must not bypass the failed-action guard.
                            for key in ("joint", "pose", "delta_xyz", "delta_rotvec"):
                                if key in command:
                                    command[key] = [round(float(v), 9) + 0.0 for v in command[key]]
                            if action["action"] == "move_eef_delta":
                                command.setdefault("delta_rotvec", [0.0, 0.0, 0.0])
                            if "gripper" in command:
                                command["gripper"] = float(command["gripper"])
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
                            if isinstance(action, dict) and set(action) <= {"action", "pose", "frame", "point", "joint", "gripper", "delta_xyz", "delta_rotvec", "reason"}:
                                record["requested_action"] = action
                        else:
                            rejection_count = 0
                            if stop.is_set():
                                break
                            reason = action.get("reason", "模型未提供原因")
                            print(f"  模型说明（非验收结论）：{reason}", flush=True)
                            if target:
                                wait_count = 0
                                outcome = executor.execute(f"turn-{round_index}", **target)
                                record = {"action": action, "status": outcome["status"], "reason": outcome["reason"],
                                          "stop_confirmed": outcome.get("stop_confirmed", False)}
                                if resolution:
                                    record["action_resolution"] = resolution
                                    print(f"  EEF 实际增量：{resolution['resolved_delta_xyz_m']} m（比例 {resolution['scale']}）", flush=True)
                                    print(f"  EEF 基座旋转增量：{resolution['delta_rotvec_base_rad']} rad", flush=True)
                                if outcome.get("stop_feedback"):
                                    actual = outcome["stop_feedback"]
                                    record["actual_feedback"] = {"joint_rad": actual["joint"], "gripper": actual["gripper"],
                                        "source": "measured joint/gripper feedback"}
                                    if kinematics:
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
                                    print(f"  执行原因：{record['reason']}；停止确认：{record['stop_confirmed']}", flush=True)
                                    recoverable = (outcome["status"] == "failed" and record["stop_confirmed"]
                                        and record["reason"] in ("verification timeout", "EEF feedback FK outside tolerance")
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
                                    blocked_signature = None
                            else:
                                wait_count += 1
                                record = {"action": action, "status": "waited"}
                                print(f"  wait：保持位置，连续等待 {wait_count} 轮", flush=True)
                        history = (history + [record])[-8:]
                        log.write(json.dumps({"round": round_index, **record}) + "\n"); log.flush()
                        if max_rounds and round_index >= max_rounds:
                            stop.set()
                        if pause_reason and not stop.is_set():
                            print(f"{pause_reason}，暂停运动并保留日志。按 Enter 退出。", flush=True)
                            if max_rounds:
                                stop.set()
                            else:
                                stop.wait()
                        if wait_count >= max_waits and not stop.is_set():
                            print(f"连续 {max_waits} 轮 wait：已暂停模型调用，未完成任务。按 Enter 退出后调整任务或技能。", flush=True)
                            if max_rounds:
                                stop.set()  # Bounded tests must not wait for terminal input.
                            else:
                                stop.wait()
                        if stop.wait(interval):
                            break
                holder[0] = None
    except Stopped:
        pass
    finally:
        finished.set()
        watcher.join(timeout=1)
    print("harness 已退出；位置保持已确认，控制租约已释放。", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attach", action="store_true", help="Connect to cyr_piper_mujoco")
    parser.add_argument("--viewer", action="store_true", help="Open Rerun for an owned scene")
    parser.add_argument("--api-config", type=Path, default=Path.home() / ".config/control_your_robot/vision_api.json")
    parser.add_argument("--base-url", help="OpenAI-compatible API base URL")
    parser.add_argument("--model", help="Model ID; overrides private API configuration")
    parser.add_argument("--api-key-env", help="Environment variable containing the API key")
    parser.add_argument("--joint-only", action="store_true", help="Disable Pinocchio FK/IK and move_eef")
    parser.add_argument("--task", default="根据双视角RGB，尝试对齐、夹住并抬起橙色碗。先在较高处逐步调整腕部朝向，抓取时尽量保持工具轴垂直向下。当碗边缘接近腕部视角的夹爪时，就可以开始下降，不要一直精细调整位置。桌面高度约0米，碗沿约5厘米，工具点抓取参考高度约3到4厘米。当接近该高度、碗缘在腕部夹爪右侧时，尝试闭合并抬起，用前后图像验证抓取。允许末端平移、旋转和关节调整。")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between completed rounds")
    parser.add_argument("--max-rounds", type=int, default=0, help="0: until Enter; positive: bounded test")
    parser.add_argument("--max-waits", type=int, default=3, help="Pause API calls after this many consecutive waits")
    parser.add_argument("--max-failures", type=int, default=3, help="Pause after consecutive unsuccessful executed actions")
    parser.add_argument("--dry-run", action="store_true", help="Local wait policy; NO API or autonomous motion")
    parser.add_argument("--out", type=Path, default=ROOT / "save" / f"vision_{uuid.uuid4().hex[:10]}")
    args = parser.parse_args()
    if args.attach and args.viewer:
        parser.error("--attach uses the running service's viewer settings")
    if not .2 <= args.interval <= 60 or args.max_rounds < 0 or args.max_waits < 1 or args.max_failures < 1:
        parser.error("interval must be 0.2..60; max-rounds nonnegative; max-waits/max-failures positive")
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
    stop = threading.Event()
    def read_enter():
        # EOF also terminates: a closed terminal must not leave an autonomous loop.
        sys.stdin.readline()
        stop.set()
    threading.Thread(target=read_enter, daemon=True).start()
    print("按 Enter 停止。连续 wait 会暂停调用；模型请求期间也可停止，迟到回复不会执行。", flush=True)
    print("DRY RUN：本地 wait 策略，不调用模型。" if args.dry_run else f"视觉模型：{model}；接口：{base_url}", flush=True)
    print("动作：joint / gripper / wait" + (" / move_eef / move_eef_delta（Pinocchio）" if kinematics else ""), flush=True)
    policy = (lambda messages: {"action": "wait"}) if args.dry_run else (
        lambda messages: chat_action(messages, key, model, base_url,
                                     deepseek_options=parsed.hostname == "api.deepseek.com"))
    try:
        with scene_session(args) as session:
            run_loop(session, policy, stop, args.out, task=args.task,
                     interval=args.interval, max_rounds=args.max_rounds, max_waits=args.max_waits,
                     kinematics=kinematics, max_failures=args.max_failures)
    finally:
        stop.set()


if __name__ == "__main__":
    main()
