"""Wait is an observation, not a self-unblocking motion or a failed-grasp verdict."""
import json
import threading

import numpy as np

from robot.harness.recovery import recovery_options
from robot.harness.tests.test_policy import AffineKinematics, rows, stage
from robot.harness.tests.test_vision import VisualSession, load_example
from robot.harness.vision import build_messages, observation, one_cm_options, orientation_feedback


def payload(messages):
    return json.loads(messages[1]["content"][0]["text"])


def test_three_waits_replan_with_robot_constraints_then_reopen_and_change_approach(tmp_path):
    example, session, plans, controls = load_example(), VisualSession(), [], []
    session.gripper = [0.]
    def planner(messages):
        plans.append(payload(messages))
        return {"stages": [stage(f"attempt-{len(plans)}")]}
    def controller(messages):
        data = payload(messages); controls.append(data)
        if len(controls) <= 3:
            return {"action": "wait", "reason": "当前图像不变，包夹不明"}
        assert data["recovery_context"]["attempt"] == 1
        if len(controls) == 4:
            return {"action": "set_gripper", "gripper": 1, "reason": "假策略：新图支持空抓后重新张开"}
        return {"action": "move", "direction": "back", "reason": "假策略：有间隙时改变接近位置"}
    out = tmp_path / "retry"
    example.run_loop(session, controller, threading.Event(), out, task="retry same rim", planner=planner,
                     legacy_actions=False, kinematics=AffineKinematics(), max_rounds=5,
                     max_replans=0, stage_rounds=100, interval=.001)
    assert [r["status"] for r in rows(out / "turns.jsonl")] == ["waited"] * 3 + ["succeeded"] * 2
    assert len(plans) == 2
    recovery = plans[-1]
    assert recovery["recovery_context"]["wait_streak"] == 3
    assert recovery["recovery_context"]["grasp_state"] == "not_attempted"  # Closed feedback alone is not an attempt verdict.
    assert recovery["robot"]["gripper"] == 0
    assert recovery["robot"]["eef_fk"]["tool"]
    assert recovery["recovery_options"]["feasible_base_moves"]
    assert recovery["recovery_options"]["joint_steps"]
    assert recovery["scene_reference"]["table_top_z_m"] == 0
    assert recovery["loop_status"]["wait_replans_used"] == 1
    assert controls[1]["loop_status"]["since_last_wait"]["identical_rgb_bytes"] is True
    assert controls[1]["loop_status"]["since_last_wait"]["max_joint_change_rad"] == 0
    assert rows(out / "plans.jsonl")[-1]["kind"] == "wait_recovery"
    assert rows(out / "plans.jsonl")[-1]["replan"] == 0
    assert session.released


def test_wait_recovery_has_its_own_finite_budget_after_normal_budget_is_zero(tmp_path, capsys):
    example, session, plans = load_example(), VisualSession(), []
    def planner(messages):
        plans.append(payload(messages))
        return {"stages": [stage()]}
    out = tmp_path / "bounded"
    example.run_loop(session, lambda _: {"action": "wait", "reason": "仍无证据"}, threading.Event(), out,
                     task="bounded recovery", planner=planner, legacy_actions=False, interval=.001,
                     stage_rounds=100, max_replans=0, max_waits=2, max_wait_replans=2, max_rounds=20)
    assert len(plans) == 3  # Initial + two recovery plans; never unbounded model calls.
    assert len(rows(out / "turns.jsonl")) == 6
    assert "等待恢复规划已用尽 2 次" in capsys.readouterr().out
    assert session.writes == [] and session.released


def test_disabling_wait_replans_retains_pause_without_motion(tmp_path):
    example, session, plans = load_example(), VisualSession(), []
    def planner(messages):
        plans.append(messages)
        return {"stages": [stage()]}
    out = tmp_path / "disabled"
    example.run_loop(session, lambda _: {"action": "wait", "reason": "等待"}, threading.Event(), out,
                     task="no recovery", planner=planner, legacy_actions=False, interval=.001,
                     max_waits=2, max_wait_replans=0, max_rounds=10)
    assert len(plans) == 1 and len(rows(out / "turns.jsonl")) == 2
    assert session.writes == []


def test_stop_during_recovery_planning_never_executes_a_late_reply(tmp_path):
    example, session, stop, calls = load_example(), VisualSession(), threading.Event(), []
    def planner(messages):
        calls.append(messages)
        if len(calls) > 1:
            stop.set()
        return {"stages": [stage()]}
    out = tmp_path / "stop"
    example.run_loop(session, lambda _: {"action": "wait", "reason": "等待"}, stop, out,
                     task="stop", planner=planner, legacy_actions=False, interval=.001, max_waits=2, max_rounds=10)
    assert len(calls) == 2 and len(rows(out / "turns.jsonl")) == 2
    assert session.writes == [] and session.released


def test_exhausted_stage_budget_while_waiting_uses_recovery_before_pausing(tmp_path):
    example, session, plans = load_example(), VisualSession(), []
    def planner(messages):
        plans.append(payload(messages))
        return {"stages": [stage()]}
    example.run_loop(session, lambda _: {"action": "wait", "reason": "仍不清楚"}, threading.Event(), tmp_path / "stage",
                     task="wait", planner=planner, legacy_actions=False, interval=.001,
                     stage_rounds=1, max_replans=1, max_waits=10, max_wait_replans=1, max_rounds=3)
    assert len(plans) == 3
    assert plans[-1]["recovery_context"]["trigger"] == "waiting_after_stage_budget"


def test_unconfirmed_stop_never_starts_wait_recovery(tmp_path, monkeypatch):
    example, session, plans = load_example(), VisualSession(), []
    base = example.PositionExecutor
    class Unconfirmed(base):
        def execute(self, *args, **kwargs):
            return {"status": "unknown", "reason": "stop unconfirmed", "stop_confirmed": False}
        def recover(self):
            raise AssertionError("must not recover unconfirmed execution")
    monkeypatch.setattr(example, "PositionExecutor", Unconfirmed)
    def planner(messages):
        plans.append(messages)
        return {"stages": [stage()]}
    out = tmp_path / "unconfirmed"
    example.run_loop(session, lambda _: {"action": "set_gripper", "gripper": 1, "reason": "张开"},
                     threading.Event(), out, task="retry", planner=planner, legacy_actions=False,
                     interval=.001, max_rounds=10)
    assert len(plans) == 1 and len(rows(out / "turns.jsonl")) == 1
    assert session.writes == []


def test_recovery_predictions_and_planner_input_do_not_copy_hidden_scene_data():
    kin, session = AffineKinematics(), VisualSession()
    q = session.q
    obs, _ = observation(session)
    fk = {"tool": kin.pose(q, "tool"), **orientation_feedback(q, kin),
          "one_cm_translation_options": one_cm_options(q, kin),
          "tool_translation_options": one_cm_options(q, kin, frame="tool"), "objects": "SECRET_OBJECTS"}
    options = recovery_options(q, kin, fk)
    assert options["joint_steps"] and "SECRET" not in json.dumps(options)
    for item in options["joint_steps"]:
        action = item["action"]
        target = np.array(q); target[action["joint_index"] - 1] += action["sign"] * .04
        after = kin.pose(target, "tool")
        np.testing.assert_allclose(item["delta_xyz_base_m"], np.array(after[:3]) - fk["tool"][:3])
    obs.update(eef_fk=fk, recovery_context={"trigger": "repeated_wait"}, recovery_options=options,
               objects="SECRET_OBJECTS", contacts="SECRET_CONTACTS")
    planner = build_messages(obs, "task", [], role="planner")
    verifier = build_messages(obs, "task", [], role="verifier", stage=stage())
    assert "SECRET" not in json.dumps(planner)
    assert "robot" in payload(planner) and "recovery_options" in payload(planner)
    assert set(payload(verifier)) == {"stage"}


def test_controller_can_request_replanning_without_a_wait_streak(tmp_path):
    example, session, plans, controls = load_example(), VisualSession(), [], []
    def planner(messages):
        plans.append(payload(messages))
        return {"stages": [stage()]}
    def controller(messages):
        controls.append(payload(messages))
        if len(controls) == 1:
            return {"action": "replan", "reason": "旧阶段假定持物，新图不支持，需要重抓"}
        return {"action": "set_gripper", "gripper": 1, "reason": "新策略：图像支持张开重试"}
    out = tmp_path / "explicit"
    example.run_loop(session, controller, threading.Event(), out, task="retry", planner=planner,
                     legacy_actions=False, interval=.001, max_replans=0, max_rounds=2)
    assert len(plans) == 2
    assert plans[-1]["recovery_context"]["trigger"] == "controller_replan_request"
    assert plans[-1]["recovery_context"]["wait_streak"] == 0
    assert [r["status"] for r in rows(out / "turns.jsonl")] == ["replan_requested", "succeeded"]
    assert len(session.writes) == 1
