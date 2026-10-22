"""关节主从。源组件的 joint 和 gripper 原样写成从臂的最新目标。"""

from __future__ import annotations

from robot.runtime.lease import ControlLease


def follow(session, source: str, follower: str, rate_hz: float, stop, lease_s: float = 2.0) -> None:
    period = 1.0 / float(rate_hz)
    lease = ControlLease(session, "teleop", lease_s)
    try:
        while not stop.is_set():
            if not lease.alive:
                raise RuntimeError("遥操控制锁已过期")
            state = session.read_raw(source)
            data = None if state is None else state.get("data")
            payload = {}
            if isinstance(data, dict):
                if data.get("joint") is not None:
                    payload["joint"] = _plain(data["joint"])
                if data.get("gripper") is not None:
                    payload["gripper"] = _plain(data["gripper"])
            if payload:
                result = session.write_target(follower, payload)
                if not result["accepted"]:
                    raise RuntimeError(result["error"] or "从臂没有接受目标")
            stop.wait(period)
    finally:
        lease.close()


def _plain(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value
