"""Successful stage reviews and explicit continuation must not replay past commands."""
import hashlib
import json
import threading

import pytest

from robot.harness.continuation import load_continuation
from robot.harness.tests.test_policy import AffineKinematics, rows, stage
from robot.harness.tests.test_vision import VisualSession, load_example


def payload(messages):
    return json.loads(messages[1]["content"][0]["text"])


def source_run(tmp_path, rounds=36):
    source = tmp_path / "previous"
    source.mkdir()
    (source / "evidence_notes.json").write_text(json.dumps({
        "version": 1, "task": "lift the orange bowl", "evidence": []}))
    records = [{"round": n, "stage": stage(), "status": "succeeded",
                "action": {"action": "joint_step", "joint_index": 6, "sign": 1, "reason": "old action"},
                "actual_feedback": {"joint_rad": [.1, 1., -1., 0., .5, .4], "gripper": 1,
                                    "objects": "SECRET_OBJECTS"},
                "objects": "SECRET_OBJECTS", "action_resolution": {"pending": "SECRET_OLD_PATH"}}
               for n in range(1, rounds + 1)]
    (source / "turns.jsonl").write_text("\n".join(map(json.dumps, records)) + "\n")
    return source


def test_successful_motion_continues_past_36_with_independent_periodic_reviews(tmp_path, capsys):
    example, session, plans, controls = load_example(), VisualSession(), [], []
    def planner(messages):
        plans.append(payload(messages))
        return {"stages": [stage()]}
    def controller(messages):
        controls.append(payload(messages))
        return {"action": "joint_step", "joint_index": 6,
                "sign": 1 if len(controls) % 2 else -1, "reason": "offline posture change"}
    out = tmp_path / "past36"
    example.run_loop(session, controller, threading.Event(), out, task="lift",
                     planner=planner, legacy_actions=False, kinematics=AffineKinematics(),
                     max_replans=0, stage_rounds=12, interval=.001, max_rounds=40)
    assert len(rows(out / "turns.jsonl")) == 40 and len(session.writes) == 40
    assert len(plans) == 4
    logs = rows(out / "plans.jsonl")
    assert [r["round"] for r in logs] == [0, 12, 24, 36]
    assert all(r["replan"] == 0 for r in logs)
    assert [r["stage_review"] for r in logs] == [0, 1, 2, 3]
    assert all(r["kind"] == "stage_review" for r in logs[1:])
    assert plans[-1]["robot"]["eef_fk"]["tool"]
    assert plans[-1]["scene_reference"]["table_top_z_m"] == 0
    assert "已达到最大控制轮数 40" in capsys.readouterr().out
    assert "SECRET" not in json.dumps(plans)


def test_success_without_measured_change_still_exhausts_stall_budget(tmp_path, capsys):
    example, session, plans = load_example(), VisualSession(), []
    session.gripper = [1.]
    def planner(messages):
        plans.append(messages)
        return {"stages": [stage()]}
    out = tmp_path / "nochange"
    example.run_loop(session, lambda _: {"action": "set_gripper", "gripper": 1, "reason": "already open"},
                     threading.Event(), out, task="lift", planner=planner, legacy_actions=False,
                     max_replans=1, stage_rounds=1, interval=.001, max_rounds=10)
    assert len(rows(out / "turns.jsonl")) == 2 and len(plans) == 2
    assert "已达到重新规划上限" in capsys.readouterr().out


def test_measured_cycle_retains_finite_stall_budget(tmp_path, capsys):
    example, session, count = load_example(), VisualSession(), 0
    def controller(messages):
        nonlocal count
        count += 1
        return {"action": "move", "direction": "forward" if count % 2 else "back", "reason": "cycle"}
    out = tmp_path / "cycle"
    example.run_loop(session, controller, threading.Event(), out, task="lift",
                     planner=lambda _: {"stages": [stage()]}, legacy_actions=False,
                     kinematics=AffineKinematics(), max_replans=1, interval=.001, max_rounds=30)
    assert len(rows(out / "turns.jsonl")) == 8
    assert "已达到重新规划上限" in capsys.readouterr().out


def test_continue_old_logs_reads_current_state_first_and_never_replays_commands(tmp_path):
    source = source_run(tmp_path)
    digest = hashlib.sha256((source / "turns.jsonl").read_bytes()).hexdigest()
    resumed = load_continuation(source)
    assert resumed["last_round"] == 36
    assert len(resumed["progress"].recent) == 8
    assert resumed["progress"].target == {"target": "visible bowl", "affordance": "visible rim"}
    example, session, plans = load_example(), VisualSession(), []
    initial_q = list(session.q)
    def planner(messages):
        plans.append(messages)
        data = payload(messages)
        assert data["planning_context"]["kind"] == "resume"
        assert data["planning_context"]["continued_from_round"] == 36
        assert data["robot"]["joint_rad"] == initial_q
        assert len(data["task_progress"]["recent_actions"]) == 8
        assert sum(p["type"] == "image_url" for p in messages[1]["content"]) == 2
        return {"stages": [stage()]}
    out = tmp_path / "continued"
    example.run_loop(session, lambda _: {"action": "set_gripper", "gripper": 1, "reason": "new decision"},
                     threading.Event(), out, task=resumed["task"], planner=planner,
                     resume=resumed, legacy_actions=False, kinematics=AffineKinematics(),
                     max_rounds=2, interval=.001)
    assert [r["round"] for r in rows(out / "turns.jsonl")] == [37, 38]
    assert all(w["joint"] == initial_q for w in session.writes)
    assert len(session.writes) == 2 and session.released
    assert "SECRET" not in json.dumps(plans)
    assert digest == hashlib.sha256((source / "turns.jsonl").read_bytes()).hexdigest()
    # The new log can itself be continued without an explicit task string.
    assert load_continuation(out)["last_round"] == 38


def test_enter_during_continuation_plan_discards_late_reply(tmp_path):
    resumed = load_continuation(source_run(tmp_path))
    example, session, stop = load_example(), VisualSession(), threading.Event()
    def planner(messages):
        stop.set()
        return {"stages": [stage()]}
    def forbidden(messages):
        pytest.fail("controller must not run after stop")
    example.run_loop(session, forbidden, stop, tmp_path / "stopped", task=resumed["task"],
                     planner=planner, resume=resumed, legacy_actions=False, interval=.001)
    assert not session.writes and session.released


def test_continuation_preserves_close_attempt_without_promoting_it_to_grasp(tmp_path):
    source = source_run(tmp_path, rounds=1)
    path = source / "turns.jsonl"
    row = rows(path)[0]
    row.update(action={"action": "set_gripper", "gripper": 0, "reason": "close"}, status="failed")
    path.write_text(json.dumps(row) + "\n")
    resumed = load_continuation(source)
    assert resumed["progress"].grasp_state == "close_attempted_unverified"
    assert resumed["progress"].completed == []


@pytest.mark.parametrize("invalid", ["empty", "unordered", "legacy", "bad_version"])
def test_invalid_continuations_fail_before_control(tmp_path, invalid):
    source = source_run(tmp_path, rounds=2)
    path = source / "turns.jsonl"
    records = rows(path)
    if invalid == "empty":
        path.write_text("")
    elif invalid == "unordered":
        path.write_text("\n".join(map(json.dumps, reversed(records))))
    elif invalid == "legacy":
        records[0]["stage"] = None
        path.write_text("\n".join(map(json.dumps, records)))
    else:
        metadata = source / "evidence_notes.json"
        metadata.write_text(json.dumps({"version": True, "task": "lift", "evidence": []}))
    with pytest.raises(ValueError):
        load_continuation(source)


def test_cli_rejects_changed_task_on_continuation_before_connecting(tmp_path, monkeypatch, capsys):
    source = source_run(tmp_path, rounds=1)
    example = load_example()
    monkeypatch.setattr("sys.argv", ["piper_mujoco_agent.py", "--resume", str(source), "--task", "different"])
    with pytest.raises(SystemExit) as exc:
        example.main()
    assert exc.value.code == 2
    assert "keeps the original task" in capsys.readouterr().err
