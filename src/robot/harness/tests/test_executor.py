import json
import threading
import time

import pytest

from robot.harness import PositionExecutor
from robot.harness.contract import ActionContract
from robot.harness.runtime_env import RuntimeEnv, _contract_action


class Session:
    def __init__(self):
        self.q = [0.0, 0.0]
        self.gripper = [0.0]
        self.seq = 0
        self.frozen = False
        self.stuck = False
        self.mode = "position"
        self.writes = []
        self.released = False
        self.frame = None

    def read_raw(self, name):
        if self.frozen and self.frame is not None:
            return self.frame
        self.seq += 1
        self.frame = dict(seq=self.seq, timestamp_ns=time.time_ns(), data=dict(
            joint=list(self.q), gripper=list(self.gripper), control_mode=self.mode, servo_fault=""))
        return self.frame

    def acquire_control(self, role, lease_s):
        return 42

    def renew_control(self, token, lease_s):
        return True

    def release_control(self, token):
        self.released = True
        return True

    def write_target(self, name, payload):
        self.writes.append(payload)
        if not self.stuck:
            self.q = payload["joint"]
            self.gripper = [payload["gripper"]]
        return dict(accepted=True, seq=len(self.writes), error="")


def make(tmp_path, session=None, **kw):
    session = session or Session()
    return PositionExecutor(session, lower=[-1, -1], upper=[1, 1],
                            journal=tmp_path / "events.jsonl", poll_s=.002, **kw)


def test_verified_sequence_journal_and_dedup(tmp_path):
    with make(tmp_path) as runner:
        request = dict(request_id="a", joint=[0.1, .2], dwell_s=.01)
        first = runner.execute(**request)
        assert first["status"] == "succeeded"
        writes = len(runner.session.writes)
        first["feedback"]["joint"][0] = 999
        assert runner.execute(**request)["feedback"]["joint"] == [.1, .2]
        assert len(runner.session.writes) == writes
        with pytest.raises(ValueError, match="conflict"):
            runner.execute("a", joint=[.2, .2], dwell_s=.01)
        assert runner.execute("b", gripper=.5, dwell_s=.01)["status"] == "succeeded"
    assert runner.session.released
    events = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text().splitlines()]
    assert [e["event"] for e in events].count("requested") == 2
    assert any(e["event"] == "feedback" and "joint_error_rad" in e for e in events)
    with pytest.raises(FileExistsError):
        make(tmp_path)


@pytest.mark.parametrize("kwargs", [dict(joint=[float("nan"), 0]), dict(joint=[0]),
    dict(joint=[2, 0]), dict(gripper=2), dict(joint=[0, 0], timeout_s=float("inf")),
    dict(joint=[0, 0], dwell_s=4), dict(joint=[[0, 0]])])
def test_invalid_input_never_writes(tmp_path, kwargs):
    with make(tmp_path) as runner:
        count = len(runner.session.writes)
        with pytest.raises(ValueError):
            runner.execute("invalid", **kwargs)
        assert len(runner.session.writes) == count


def test_frozen_frames_never_satisfy_dwell(tmp_path):
    session = Session()
    runner = make(tmp_path, session, max_age_ms=30)
    session.frozen = True
    result = runner.execute("frozen", joint=[0, 0], timeout_s=.1, dwell_s=.04)
    assert result["status"] == "unknown"
    with pytest.raises(RuntimeError, match="recovery"):
        runner.execute("next", joint=[0, 0])
    session.frozen = False
    runner.close()


def test_timeout_stops_task_sequence(tmp_path):
    with make(tmp_path) as runner:
        runner.session.stuck = True
        results = runner.run([dict(request_id="stuck", joint=[.5, .5], timeout_s=.03, dwell_s=.01),
                              dict(request_id="must-not-run", joint=[0, 0])])
        assert len(results) == 1
        assert results[0]["status"] == "unknown"
        assert results[0]["hold_requested"]
        assert "timeout" in results[0]["reason"]


def test_cancel_and_concurrent_request(tmp_path):
    with make(tmp_path) as runner:
        runner.session.stuck = True
        results = []
        thread = threading.Thread(target=lambda: results.append(runner.execute("a", joint=[.5, 0])))
        thread.start()
        try:
            deadline = time.monotonic() + 1
            while len(runner.session.writes) < 2 and time.monotonic() < deadline:
                time.sleep(.001)
            with pytest.raises(RuntimeError, match="busy"):
                runner.execute("b", joint=[0, 0])
            assert runner.cancel("a")
        finally:
            thread.join(timeout=2)
        assert not thread.is_alive()
        assert results[0]["status"] == "unknown"
        assert "cancel" in results[0]["reason"]


def test_mode_and_history_limits(tmp_path):
    session = Session()
    session.mode = "impedance"
    with pytest.raises(RuntimeError, match="position"):
        make(tmp_path, session)
    session.mode = "position"
    with make(tmp_path, session, max_requests=1) as runner:
        runner.execute("a", joint=[0, 0], dwell_s=.01)
        with pytest.raises(RuntimeError, match="history"):
            runner.execute("b", joint=[0, 0])


def test_contract_rejects_nonfinite_and_gripper_shape():
    for action in ({"left_arm_joint": [float("nan")]}, {"left_arm": {"gripper": [0, 1]}}):
        with pytest.raises(ValueError):
            ActionContract().commands(action)
    with pytest.raises(ValueError):
        ActionContract(control_hz=float("nan"))


def test_eef_with_gripper_is_not_lost():
    pose = [0, 0, 0, 1, 0, 0, 0]
    action = _contract_action({"left_ee_pose": pose, "left_ee_joint_state": [.5]})
    assert ActionContract(mode="eef").commands(action)["left_arm"] == {"eef": pose, "gripper": .5}
    with pytest.raises(ValueError):
        ActionContract(mode="joint").commands(action)


def test_stopped_env_rejects_before_touching_session():
    env = RuntimeEnv.__new__(RuntimeEnv)
    env._stop = True
    with pytest.raises(RuntimeError, match="episode"):
        env.take_action({"left_arm_joint": [0]})


def test_lost_lease_does_not_write_recovery_target(tmp_path):
    runner = make(tmp_path)
    runner.lease.alive = False
    count = len(runner.session.writes)
    result = runner.execute("lost", joint=[.1, 0], dwell_s=.01)
    assert result["status"] == "unknown"
    assert "lease" in result["reason"]
    assert len(runner.session.writes) == count
    with pytest.raises(RuntimeError, match="lease"):
        runner.close()
    assert runner.session.released


def test_stale_feedback_rejects_before_target_write(tmp_path):
    runner = make(tmp_path, max_age_ms=10)
    runner.session.frozen = True
    time.sleep(.02)
    count = len(runner.session.writes)
    assert runner.execute("old", joint=[.1, 0])["status"] == "rejected"
    assert len(runner.session.writes) == count
    runner.session.frozen = False
    runner.close()


def test_piper_example_end_to_end(tmp_path):
    import importlib.util
    import subprocess
    import sys
    from pathlib import Path
    if importlib.util.find_spec("pinocchio") is None:
        pytest.skip("Piper Sim needs Pinocchio")
    root = Path(__file__).resolve().parents[4]
    output = tmp_path / "piper.jsonl"
    completed = subprocess.run([sys.executable, str(root / "examples/piper_harness.py"),
                                "--out", str(output)], cwd=root, capture_output=True, text=True, timeout=30)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    events = [json.loads(line) for line in output.read_text().splitlines()]
    outcomes = [event["result"]["status"] for event in events if event["event"] == "finished"]
    assert outcomes == ["succeeded"] * 3


def test_cancellation_during_preflight_does_not_send_goal(tmp_path):
    with make(tmp_path) as runner:
        original = runner._read
        def cancel_before_write():
            runner.cancel("preflight")
            return original()
        runner._read = cancel_before_write
        count = len(runner.session.writes)
        result = runner.execute("preflight", joint=[.5, 0])
        assert result["status"] == "rejected"
        assert "cancel" in result["reason"]
        assert len(runner.session.writes) == count
        runner._read = original
