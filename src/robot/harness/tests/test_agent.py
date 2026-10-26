import json

import pytest

from robot.harness import AgentHarness
from robot.harness.tests.test_executor import make


def agent(tmp_path, runner):
    return AgentHarness(runner, robot_id="test-arm", store_path=tmp_path / "requests.sqlite3")


def test_durable_replay_and_conflict(tmp_path):
    with make(tmp_path) as runner:
        harness = agent(tmp_path, runner)
        args = dict(request_id="one", joint=[.1, .2])
        first = harness.dispatch("move_joint_absolute", args)
        assert first["status"] == "succeeded"
        count = len(runner.session.writes)
        restarted = agent(tmp_path, runner)
        assert restarted.dispatch("move_joint_absolute", args) == first
        assert len(runner.session.writes) == count
        with pytest.raises(ValueError, match="conflict"):
            restarted.dispatch("move_joint_absolute", {**args, "joint": [0, 0]})
        assert restarted.dispatch("observe", {})["observation_id"]


def test_interrupted_request_requires_recovery_never_replays(tmp_path):
    with make(tmp_path) as runner:
        harness = agent(tmp_path, runner)
        args = dict(request_id="interrupted", gripper=.2)
        signature = json.dumps(dict(skill="set_gripper", arguments=args), sort_keys=True)
        harness.store.begin(harness.scope, args["request_id"], signature)
        assert harness.dispatch("set_gripper", args)["status"] == "unknown"
        with pytest.raises(RuntimeError, match="recovery"):
            harness.dispatch("set_gripper", dict(request_id="next", gripper=.3))
        assert harness.dispatch("recover", {})["status"] == "ready"
        assert harness.dispatch("set_gripper", args)["status"] == "unknown"
        assert harness.dispatch("set_gripper", dict(request_id="next", gripper=.3))["status"] == "succeeded"


def test_task_stops_on_failure_and_preflights_all_steps(tmp_path):
    with make(tmp_path) as runner:
        harness = agent(tmp_path, runner)
        count = len(runner.session.writes)
        with pytest.raises(ValueError):
            harness.run_task("malformed", [dict(skill="set_gripper", arguments={"gripper": .4}),
                                           dict(skill="move_eef", arguments={})])
        assert len(runner.session.writes) == count
        plan = [dict(skill="move_joint_absolute", arguments={"joint": [.2, 0.]}),
                dict(skill="set_gripper", arguments={"gripper": .5})]
        runner.execute = lambda rid, **kw: {"request_id": rid, "status": "failed", "reason": "injected execution failure"}
        result = harness.run_task("failed", plan)
        assert len(result["results"]) == 1
        assert result["status"] == "failed"
        assert len(runner.session.writes) == count
        with pytest.raises(RuntimeError, match="recovery"):
            harness.dispatch("set_gripper", dict(request_id="outside", gripper=.5))


@pytest.mark.parametrize("args", [dict(request_id="x", gripper=True), dict(request_id="x", gripper=float("nan")),
                                  dict(request_id="x", gripper="0.5"), dict(request_id="", gripper=.5)])
def test_strict_local_validation(tmp_path, args):
    with make(tmp_path) as runner:
        harness = agent(tmp_path, runner)
        count = len(runner.session.writes)
        with pytest.raises(ValueError):
            harness.dispatch("set_gripper", args)
        assert len(runner.session.writes) == count


def test_piper_kinematics_tool_round_trip():
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    from robot.controller.PiperSim_controller import READY_JOINT
    model = PiperKinematics()
    for point in ("flange", "tool"):
        pose = model.pose(READY_JOINT, point)
        solved = model.solve(READY_JOINT, pose, point=point)
        position, rotation = model.error(solved, pose, point)
        assert position < 1e-4 and rotation < 1e-4
    with pytest.raises(ValueError, match="unit"):
        model.solve(READY_JOINT, [0, 0, 0, 2, 0, 0, 0])


def test_piper_agent_subprocess_restart(tmp_path):
    pytest.importorskip("pinocchio")
    import subprocess
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[4]
    store = tmp_path / "durable.sqlite3"
    for index in range(2):
        output = tmp_path / f"episode{index}.jsonl"
        completed = subprocess.run([sys.executable, str(root / "examples/piper_harness.py"),
                                    "--agent", "--out", str(output), "--store", str(store)],
                                   cwd=root, capture_output=True, text=True, timeout=30)
        assert completed.returncode == 0, completed.stdout + completed.stderr
        events = [json.loads(line) for line in output.read_text().splitlines()]
        finished = [e for e in events if e["event"] == "finished"]
        assert len(finished) == (3 if index == 0 else 0)
        if finished:
            assert all(e["result"]["stop_confirmed"] for e in finished)
