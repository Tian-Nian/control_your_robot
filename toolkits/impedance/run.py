"""把第 2 轴目标加 0.1 rad，确认臂跟上而不是瞬移。"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from check import from_session  # noqa: E402


def main() -> int:
    from robot.runtime.session import Session

    try:
        session = Session.attach("piper_sim")
    except RuntimeError:
        report = {
            "ok": False,
            "questions": [
                "后台没有 piper_sim。先执行 cyr start --config config/runtime/piper_sim_impedance.yml。"
            ],
        }
        print(json.dumps(report, ensure_ascii=False))
        return 2
    try:
        report = from_session(session)
        if not report["ok"]:
            print(json.dumps(report, ensure_ascii=False))
            return 2
        arm = report["arm"]
        start = _joint(session, arm)
        goal = start.copy()
        goal[1] = float(goal[1] + 0.1)
        called = session.call(arm, {"joint": goal.tolist()}, timeout=2.0)
        time.sleep(0.05)
        early = _joint(session, arm)
        time.sleep(0.5)
        later = _joint(session, arm)
        jumped = abs(float(early[1] - goal[1])) < 1e-3
        followed = float(later[1] - start[1]) > 0.01
        payload = {
            "ok": called.get("accepted") is True and followed and not jumped,
            "arm": arm,
            "start_joint2": float(start[1]),
            "early_joint2": float(early[1]),
            "later_joint2": float(later[1]),
            "goal_joint2": float(goal[1]),
            "servo_fault": _data(session, arm).get("servo_fault", ""),
        }
        if not payload["ok"]:
            payload["questions"] = ["第 2 轴没有朝目标靠近，或一下子跳到了目标。看 control_mode 和 servo_fault。"]
        print(json.dumps(payload, ensure_ascii=False))
        return 0 if payload["ok"] else 2
    finally:
        session.close()


def _data(session, arm) -> dict:
    raw = session.read_raw(arm) or {}
    data = raw.get("data")
    return data if isinstance(data, dict) else {}


def _joint(session, arm):
    return np.asarray(_data(session, arm)["joint"], dtype=float).reshape(-1)


if __name__ == "__main__":
    raise SystemExit(main())
