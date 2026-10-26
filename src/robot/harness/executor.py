"""Verified, bounded position skills. No policy inference or hardware stop guarantee."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import threading
import time
import uuid

import numpy as np

from robot.runtime.lease import ControlLease


def _vector(value, size=None):
    array = np.asarray(value, dtype=float)
    if array.ndim != 1 or not array.size or not np.isfinite(array).all():
        raise ValueError("expected a nonempty finite vector")
    if size is not None and array.size != size:
        raise ValueError(f"expected {size} values, got {array.size}")
    return array


def _positive(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


class PositionExecutor:
    """One component, absolute joint/gripper goals, fresh-feedback verification.

    Construct with explicit joint limits and an unused JSONL path. Use as a
    context manager; it owns a control lease, not the supplied Session. Only
    position mode with published joint/gripper/control_mode/servo_fault works.
    Deduplication lasts for this instance; IDs are never evicted or persisted
    across restart. cancel() is callable from another thread.
    """

    def __init__(self, session, *, lower, upper, journal, component="left_arm",
                 max_age_ms=100.0, poll_s=0.01, max_requests=1000):
        self.session = session
        self.component = component
        self.lower = _vector(lower)
        self.upper = _vector(upper, self.lower.size)
        if np.any(self.lower >= self.upper):
            raise ValueError("lower must be less than upper")
        self.max_age_ms = _positive(max_age_ms, "max_age_ms")
        self.poll_s = _positive(poll_s, "poll_s")
        if not isinstance(max_requests, int) or max_requests < 1:
            raise ValueError("max_requests must be a positive integer")
        self.max_requests = max_requests
        self.episode_id = uuid.uuid4().hex
        self._mutex = threading.RLock()
        self._active = None
        self._cancel = threading.Event()
        self._cache = {}
        self._closed = False
        self._poisoned = False
        self._stopped = False
        self.lease = None
        # Fail before acquiring control or opening a log when prerequisites fail.
        self._read()
        path = Path(journal)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._log = path.open("x", encoding="utf-8")
        try:
            self.lease = ControlLease(session, "model", 2.0)
            # A previous executor can leave the runtime stop latch engaged.
            # Verify a fresh stop, then explicitly release it on the first action.
            initial = self._stop_and_confirm()
            self._emit("opened", feedback=initial)
        except BaseException:
            if self.lease is not None:
                self.lease.close()
            self._log.close()
            raise

    def capabilities(self):
        return {"component": self.component, "joint_dof": len(self.lower),
                "mode": "position", "skills": ["move_joint_absolute", "set_gripper"],
                "lower_rad": self.lower.tolist(), "upper_rad": self.upper.tolist(),
                "gripper_range": [0.0, 1.0], "verification": "fresh position feedback",
                "dedup_scope": "executor instance", "episode_id": self.episode_id}

    def _emit(self, event, **fields):
        record = dict(episode_id=self.episode_id, request_id=self._active,
                      event=event, wall_ns=time.time_ns(), monotonic_s=time.monotonic(), **fields)
        self._log.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
        self._log.flush()

    def _read(self):
        frame = self.session.read_raw(self.component)
        if not frame or not isinstance(frame.get("data"), dict):
            raise RuntimeError("missing feedback")
        stamp = int(frame["timestamp_ns"])
        age = (time.time_ns() - stamp) / 1e6
        if stamp <= 0 or age < 0 or age > self.max_age_ms:
            raise RuntimeError("stale or future feedback")
        data = frame["data"]
        if data.get("control_mode") != "position":
            raise RuntimeError("position mode required")
        if "servo_fault" not in data or data["servo_fault"]:
            raise RuntimeError("servo fault or missing fault telemetry")
        q = _vector(data.get("joint"), len(self.lower))
        grip = _vector(np.asarray(data["gripper"]).reshape(-1), 1)[0].item()
        if not np.isfinite(grip) or not 0 <= grip <= 1:
            raise RuntimeError("invalid gripper feedback")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise RuntimeError("feedback outside configured limits")
        return {"seq": int(frame["seq"]), "timestamp_ns": stamp,
                "joint": q.tolist(), "gripper": grip}

    def _write(self, payload, valid_for_s=None):
        if not self.lease.alive:
            raise RuntimeError("control lease lost")
        result = self.session.write_target(self.component, payload, valid_for_s=valid_for_s)
        self._emit("write", target=payload, receipt={k: result.get(k) for k in ("accepted", "seq", "error")})
        if not result.get("accepted") or result.get("error"):
            raise RuntimeError(result.get("error") or "target rejected")

    def _hold(self, feedback):
        self._write({"joint": feedback["joint"], "gripper": feedback["gripper"]})

    def execute(self, request_id, *, joint=None, gripper=None, timeout_s=3.0,
                tolerance_rad=0.005, gripper_tolerance=0.01, dwell_s=0.2):
        """Return structured terminal result; invalid inputs raise before writing.

        Identical completed requests return a deep copy of the cached result.
        Conflicting IDs, concurrent calls and exhausted history are rejected.
        """
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("nonempty request_id required")
        timeout_s = _positive(timeout_s, "timeout_s")
        tolerance_rad = _positive(tolerance_rad, "tolerance_rad")
        gripper_tolerance = _positive(gripper_tolerance, "gripper_tolerance")
        dwell_s = _positive(dwell_s, "dwell_s")
        if dwell_s >= timeout_s:
            raise ValueError("dwell_s must be less than timeout_s")
        target = {}
        if joint is not None:
            q = _vector(joint, len(self.lower))
            if np.any(q < self.lower) or np.any(q > self.upper):
                raise ValueError("joint target outside limits")
            target["joint"] = q.tolist()
        if gripper is not None:
            g = float(gripper)
            if not np.isfinite(g) or not 0 <= g <= 1:
                raise ValueError("gripper must be finite in [0, 1]")
            target["gripper"] = g
        if not target:
            raise ValueError("joint or gripper target required")
        fingerprint = json.dumps([target, timeout_s, tolerance_rad, gripper_tolerance, dwell_s], sort_keys=True)
        with self._mutex:
            if self._closed:
                raise RuntimeError("executor closed")
            if request_id in self._cache:
                old, result = self._cache[request_id]
                if fingerprint != old:
                    raise ValueError("request_id conflict")
                return deepcopy(result)
            if self._active is not None or self._poisoned:
                raise RuntimeError("executor busy or requires recovery")
            if len(self._cache) >= self.max_requests:
                raise RuntimeError("request history full; start a new executor explicitly")
            self._active = request_id
            self._cancel.clear()
        result = {"request_id": request_id, "episode_id": self.episode_id,
                  "status": "unknown", "reason": "interrupted", "target": target}
        attempted = False
        try:
            self._resume_verified_hold()
            before = self._read()
            goal = {"joint": target.get("joint", before["joint"]),
                    "gripper": target.get("gripper", before["gripper"])}
            self._emit("requested", target=goal, feedback=before,
                       verification=dict(timeout_s=timeout_s, dwell_s=dwell_s,
                                         tolerance_rad=tolerance_rad, gripper_tolerance=gripper_tolerance))
            if self._cancel.is_set():
                raise RuntimeError("cancel requested before write")
            deadline = time.monotonic() + timeout_s
            attempted = True  # A transport exception can leave an uncertain write.
            self._write(goal, valid_for_s=timeout_s)
            last_seq, last_stamp = before["seq"], before["timestamp_ns"]
            in_band_since = None
            while True:
                if self._cancel.is_set():
                    raise RuntimeError("cancel requested")
                if time.monotonic() >= deadline:
                    raise RuntimeError("verification timeout")
                if not self.lease.alive:
                    raise RuntimeError("control lease lost")
                feedback = self._read()
                if feedback["seq"] < last_seq or feedback["timestamp_ns"] < last_stamp:
                    raise RuntimeError("feedback sequence regressed")
                if feedback["seq"] > last_seq and feedback["timestamp_ns"] > last_stamp:
                    now = time.monotonic()
                    # A long gap cannot count as continuous verification.
                    if (feedback["timestamp_ns"] - last_stamp) / 1e6 > self.max_age_ms:
                        in_band_since = None
                    last_seq, last_stamp = feedback["seq"], feedback["timestamp_ns"]
                    error = float(np.max(np.abs(np.asarray(feedback["joint"]) - goal["joint"])))
                    grip_error = abs(feedback["gripper"] - goal["gripper"])
                    self._emit("feedback", feedback=feedback, joint_error_rad=error, gripper_error=grip_error)
                    if error <= tolerance_rad and grip_error <= gripper_tolerance:
                        if in_band_since is None:
                            in_band_since = now
                        if now - in_band_since >= dwell_s:
                            stop_feedback = self._stop_and_confirm()
                            if (np.max(np.abs(np.asarray(stop_feedback["joint"]) - goal["joint"])) > tolerance_rad
                                    or abs(stop_feedback["gripper"] - goal["gripper"]) > gripper_tolerance):
                                raise RuntimeError("goal tolerance lost during stop verification")
                            result.update(status="succeeded", reason="fresh position feedback verified", feedback=feedback,
                                          stop_confirmed=True, stop_feedback=stop_feedback)
                            break
                    else:
                        in_band_since = None
                time.sleep(self.poll_s)
        except Exception as exc:
            result["reason"] = str(exc)
            if attempted:
                try:
                    result["stop_feedback"] = self._stop_and_confirm()
                    result["stop_confirmed"] = True
                    result["status"] = "cancelled" if self._cancel.is_set() else "failed"
                except Exception as stop_error:
                    result["stop_error"] = str(stop_error)
                    result["status"] = "unknown"
                self._poisoned = True
            else:
                result["status"] = "rejected"
        finally:
            if attempted and result["status"] == "unknown":
                self._poisoned = True
            with self._mutex:
                self._cache[request_id] = (fingerprint, deepcopy(result))
                try:
                    self._emit("finished", result=result)
                finally:
                    self._active = None
        return deepcopy(result)

    def observe(self):
        """Fresh robot feedback, not a policy summary or inferred object state."""
        feedback = self._read()
        return {**feedback, "observation_id": f"{self.component}:{feedback['seq']}:{feedback['timestamp_ns']}",
                "component": self.component, "frame_id": "base", "joint_unit": "rad",
                "age_ms": (time.time_ns() - feedback["timestamp_ns"]) / 1e6}

    def get_status(self):
        with self._mutex:
            return {"active_request": self._active, "requires_recovery": self._poisoned,
                    "closed": self._closed, "lease_alive": bool(self.lease and self.lease.alive),
                    "runtime": self.session.runtime_status(self.component)}

    def _stop_and_confirm(self, timeout_s=1.0):
        if not self.lease.alive:
            raise RuntimeError("control lease lost; cannot request stop")
        before = self._read()
        receipt = self.session.stop_target(self.component)
        if not receipt.get("accepted") or receipt.get("error"):
            raise RuntimeError("stop request rejected")
        self._emit("stop_requested", receipt={k: receipt.get(k) for k in ("accepted", "seq", "error")})
        deadline = time.monotonic() + timeout_s
        previous = None
        stable_since = None
        while time.monotonic() < deadline:
            if not self.lease.alive:
                raise RuntimeError("control lease lost during stop verification")
            status = self.session.runtime_status(self.component) or {}
            feedback = self._read()
            if status.get("stream_seq") == receipt["seq"] and status.get("stream_state") == "stopped":
                if feedback["seq"] > before["seq"] and feedback["timestamp_ns"] > before["timestamp_ns"]:
                    if (feedback["timestamp_ns"] - before["timestamp_ns"]) / 1e6 > self.max_age_ms:
                        stable_since = None
                        previous = None
                    if previous is not None:
                        drift = np.max(np.abs(np.asarray(feedback["joint"]) - previous["joint"]))
                        grip_drift = abs(feedback["gripper"] - previous["gripper"])
                        if drift <= 1e-4 and grip_drift <= 1e-4:
                            if stable_since is None:
                                stable_since = time.monotonic()
                            if time.monotonic() - stable_since >= 0.05:
                                self._stopped = True
                                self._emit("stop_confirmed", feedback=feedback)
                                return feedback
                        else:
                            stable_since = None
                            previous = feedback
                    if previous is None:
                        previous = feedback
                    before = feedback
            time.sleep(self.poll_s)
        raise RuntimeError("position stop could not be verified")

    def _resume_verified_hold(self):
        if self._stopped:
            result = self.session.call(self.component, {"op": "release"})
            if not result.get("accepted") or result.get("error"):
                raise RuntimeError("runtime release failed")
            self._stopped = False

    def recover(self):
        """Explicit recovery: fresh position hold, verify, then release latch.

        Old request IDs retain their original outcomes. No automatic retry.
        """
        with self._mutex:
            if self._active is not None or self._closed:
                raise RuntimeError("cannot recover a busy or closed executor")
            feedback = self._stop_and_confirm()
            self._resume_verified_hold()
            self._poisoned = False
            self._emit("recovered", feedback=feedback)
            return {"status": "ready", "feedback": feedback}

    def cancel(self, request_id):
        """Request interruption, not a physical emergency stop."""
        with self._mutex:
            if self._active != request_id:
                return False
            self._cancel.set()
            return True

    def run(self, requests):
        """Execute an explicit finite sequence; stop at the first non-success."""
        results = []
        for request in requests:
            result = self.execute(**request)
            results.append(result)
            if result["status"] != "succeeded":
                break
        return results

    def close(self):
        with self._mutex:
            if self._closed:
                return
            if self._active is not None:
                raise RuntimeError("cancel active request and wait before close")
            self._closed = True
            try:
                self._stop_and_confirm()
            finally:
                try:
                    self.lease.close()
                finally:
                    self._log.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
