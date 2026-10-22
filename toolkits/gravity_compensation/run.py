"""启动检查通过后，读 g(q)，并确认臂留在准备姿态。"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

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
                "后台没有 piper_sim。先执行 cyr start --config config/runtime/piper_sim_gravity.yml。"
            ],
        }
        print(json.dumps(report, ensure_ascii=False))
        return 2
    try:
        report = from_session(session, verify_hold=True)
        if not report["ok"]:
            print(json.dumps(report, ensure_ascii=False))
            return 2
        arm = report["arm"]
        before = _joint(session, arm)
        called = session.call(arm, {"op": "gravity"}, timeout=2.0)
        time.sleep(0.4)
        after = _joint(session, arm)
        delta = [float(after[i] - before[i]) for i in range(len(before))]
        payload = {
            "ok": max(abs(item) for item in delta) < 0.05,
            "arm": arm,
            "control_mode": _data(session, arm).get("control_mode"),
            "servo_fault": _data(session, arm).get("servo_fault", ""),
            "gravity": (called.get("state") or {}).get("data", {}).get("result"),
            "joint_delta": delta,
        }
        if not payload["ok"]:
            payload["questions"] = ["臂没有停在准备姿态。看 servo_fault，确认模式是 gravity_compensation。"]
        print(json.dumps(payload, ensure_ascii=False))
        return 0 if payload["ok"] else 2
    finally:
        session.close()


def _data(session, arm) -> dict:
    raw = session.read_raw(arm) or {}
    data = raw.get("data")
    return data if isinstance(data, dict) else {}


def _joint(session, arm):
    import numpy as np

    return np.asarray(_data(session, arm)["joint"], dtype=float).reshape(-1)


if __name__ == "__main__":
    raise SystemExit(main())
