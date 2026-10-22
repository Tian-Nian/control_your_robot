"""关节阻抗能不能在当前服务上做。失败时打印问题，退出码 2。"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

MODE = "impedance"


def evaluate(components, states, service_up=True, confirm_ready=False) -> dict:
    if not service_up:
        return {
            "ok": False,
            "arm": None,
            "questions": [
                "后台没有服务。在 control_your_robot 目录执行 "
                "cyr start --config config/runtime/piper_sim_impedance.yml ，然后再跑这个检查。"
            ],
            "ask": [],
        }
    questions = []
    ask = []
    usable = []
    for item in components:
        modes, error = _modes(item)
        if error:
            if "controller" in str(item.get("class_path")):
                questions.append(f"{item.get('name')} 读不到 supported_modes：{error}")
            continue
        if modes is None:
            continue
        usable.append((item, modes))
    chosen = next(((item, modes) for item, modes in usable if MODE in modes), None)
    if chosen is None:
        if usable:
            listed = ", ".join(f"{item['name']}={list(modes)}" for item, modes in usable)
            questions.append(f"这些臂都不支持 {MODE}：{listed}。换一台支持该模式的臂。")
        elif not questions:
            questions.append("服务里没有申报 supported_modes 的机械臂。")
        return {"ok": False, "arm": None, "questions": questions, "ask": ask}
    item, modes = chosen
    name = item["name"]
    publish = list(item.get("publish") or [])
    setup = dict(item.get("setup") or {})
    if "joint" not in publish:
        questions.append(f"{name} 的 publish 里没有 joint。")
    if "control_mode" not in publish:
        questions.append(f"{name} 的 publish 里没有 control_mode。")
    if "joint_K" not in setup or "joint_D" not in setup:
        questions.append(
            f"{name} 的 setup 里要有 joint_K 和 joint_D。示例是 config/runtime/piper_sim_impedance.yml。"
        )
    state = (states or {}).get(name) or {}
    mode = state.get("control_mode")
    if mode != MODE:
        questions.append(
            f"{name} 当前模式是 {mode!r}。用 config/runtime/piper_sim_impedance.yml 启动。"
        )
    if item.get("class_name") != "PiperSimController" and not confirm_ready:
        ask.append(f"{name} 已上电、工作空间已清空吗？确认后给 check.py 加上 --confirm-ready。")
    blocking = questions + ([] if confirm_ready or not ask else ask)
    return {"ok": not blocking, "arm": name, "modes": list(modes), "questions": questions, "ask": ask}


def _modes(item):
    class_path = item.get("class_path")
    class_name = item.get("class_name")
    if not class_path or not class_name:
        return None, ""
    try:
        cls = getattr(importlib.import_module(class_path), class_name)
    except Exception as exc:
        return None, str(exc)
    modes = getattr(cls, "supported_modes", None)
    if not modes:
        return None, ""
    return tuple(modes), ""


def from_session(session, confirm_ready=False) -> dict:
    described = session.describe()
    states = {}
    for item in described:
        raw = session.read_raw(item["name"])
        data = None if raw is None else raw.get("data")
        states[item["name"]] = data if isinstance(data, dict) else {}
    report = evaluate(described, states, service_up=True, confirm_ready=confirm_ready)
    report["service"] = session._config.name
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="检查关节阻抗条件")
    parser.add_argument("--service", default=None)
    parser.add_argument("--confirm-ready", action="store_true")
    args = parser.parse_args(argv)
    from robot.runtime.session import Session

    try:
        session = Session.attach(args.service)
    except RuntimeError as exc:
        report = evaluate([], {}, service_up=False)
        report["error"] = str(exc)
    else:
        try:
            report = from_session(session, confirm_ready=args.confirm_ready)
        finally:
            session.close()
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
