import json
import threading

import pytest

from robot.harness.policy import parse_plan
from robot.harness.progress import EpisodeProgress
from robot.harness.tests.test_policy import stage, rows
from robot.harness.tests.test_vision import VisualSession, load_example


def test_real_planner_shape_with_reason_retains_grasp_and_lift():
    diagnostics = {}
    stages = [stage(), stage("lift", "LIFT")]
    assert parse_plan({"stages": stages, "reason": "先抓取，再验证抬起"}, diagnostics=diagnostics) == stages
    assert diagnostics == {"status": "accepted", "stage_count": 2}
    for extra in ({"reason": ""}, {"reason": "x" * 201}, {"action": "move"}):
        assert parse_plan({"stages": stages, **extra}, diagnostics=diagnostics)[0]["id"] == "task"
        assert diagnostics["status"] == "fallback"


def test_target_is_textual_and_preserved_across_replan_without_mutating_input():
    progress = EpisodeProgress()
    progress.adopt_plan([stage()])
    changed = [{**stage("retry"), "target": "another bowl", "affordance": "other rim"}]
    assert progress.adopt_plan(changed)[0]["target"] == stage()["target"]
    assert changed[0]["target"] == "another bowl"
    snapshot = progress.snapshot()
    snapshot["target_selection"]["target"] = "mutated"
    assert progress.snapshot()["target_selection"]["target"] == stage()["target"]


def motion(dx, status="succeeded"):
    return {"action": {"action": "tool_move", "direction": "x+", "reason": "测试"},
            "status": status, "measured_effect": {"delta_base_m": [dx, 0, 0]}}


def test_cycles_use_actual_feedback_not_requested_tokens_and_require_four_moves():
    progress = EpisodeProgress()
    for i, dx in enumerate([.01, -.01, .01, -.01]):
        progress.record(motion(dx), stage(), i + 1)
    assert progress.stagnation["kind"] == "measured_translation_cycle"
    assert progress.stagnation["net_displacement_m"] == pytest.approx(0)
    progress.record(motion(.01, "failed"), stage(), 5)
    assert progress.stagnation is None
    for i in range(9):
        progress.record(motion(.01), stage(), i + 6)
    assert len(progress.recent) == 8 and progress.stagnation is None


def test_closing_is_only_an_attempt_and_width_or_motion_never_proves_lift():
    progress = EpisodeProgress()
    progress.record({"action": {"action": "set_gripper", "gripper": 0}, "status": "failed"}, stage(), 1)
    assert progress.grasp_state == "close_attempted_unverified"
    assert progress.completed == []
    progress.record({"action": {"action": "done"}, "status": "stage_complete",
                     "verification": {"complete": True, "reason": "碗离开桌面"}}, stage("lift", "LIFT"), 2)
    assert progress.grasp_state == "lift_visually_verified"


def test_loop_keeps_original_constraints_and_replans_with_cycle_and_target(tmp_path):
    from robot.harness.tests.test_policy import AffineKinematics
    example, session = load_example(), VisualSession()
    plans, inputs = [], []
    def planner(messages):
        plans.append(json.loads(messages[1]["content"][0]["text"]))
        return {"stages": [stage()], "reason": "保持同一个碗沿"}
    def controller(messages):
        inputs.append(json.loads(messages[1]["content"][0]["text"]))
        return {"action": "move", "direction": "forward" if len(inputs) % 2 else "back", "reason": "测试往返"}
    output = tmp_path / "cycle"
    example.run_loop(session, controller, threading.Event(), output, task="只用RGB，不用深度",
                     planner=planner, legacy_actions=False, kinematics=AffineKinematics(), interval=.001,
                     stage_rounds=12, max_replans=1, max_rounds=5)
    assert len(plans) == 2
    assert plans[1]["task_progress"]["stagnation"]["kind"] == "measured_translation_cycle"
    assert len(plans[1]["task_progress"]["recent_actions"]) == 4
    assert all(i["task"] == "只用RGB，不用深度" for i in inputs)
    assert len(inputs[-1]["recent_actions"]) == 4
    assert rows(output / "plans.jsonl")[0]["status"] == "accepted"


def test_verifier_receives_stage_start_images_but_no_robot_claims(tmp_path):
    example, session, checks = load_example(), VisualSession(), []
    def verifier(messages):
        checks.append(messages)
        return {"complete": True, "reason": "可见阶段完成"}
    example.run_loop(session, lambda _: {"action": "done", "reason": "请求检查"}, threading.Event(),
                     tmp_path / "verify", task="lift", planner=lambda _: {"stages": [stage()]},
                     verifier=verifier, legacy_actions=False, interval=.001, max_rounds=1)
    content = checks[0][1]["content"]
    assert sum(p["type"] == "image_url" for p in content) == 4
    assert "stage_start head RGB" in content[1]["text"]
    assert set(json.loads(content[0]["text"])) == {"stage"}


def test_later_object_in_multi_object_plan_keeps_its_own_selection():
    progress = EpisodeProgress()
    second = {**stage("second"), "target": "blue bowl", "affordance": "blue near rim"}
    plan = progress.adopt_plan([stage(), stage("lift", "LIFT"), second, stage("lift2", "LIFT")])
    assert plan[2]["target"] == "blue bowl"
    progress.enter_stage(plan[2])
    assert progress.target == {"target": "blue bowl", "affordance": "blue near rim"}


def test_replan_retains_attempt_and_history_but_does_not_retrigger_old_cycle():
    progress = EpisodeProgress()
    progress.adopt_plan([stage()])
    progress.grasp_state = "close_attempted_unverified"
    for i, dx in enumerate([.01, -.01, .01, -.01]):
        progress.record(motion(dx), stage(), i + 1)
    assert progress.stagnation
    progress.begin_plan()
    progress.enter_stage(stage())
    assert progress.grasp_state == "close_attempted_unverified"
    progress.record(motion(.01), stage(), 5)
    assert len(progress.recent) == 5 and progress.stagnation is None


@pytest.mark.parametrize("index", range(1, 7))
@pytest.mark.parametrize("sign", [-1, 1])
def test_discrete_joint_posture_step_uses_existing_limits(index, sign):
    from robot.harness.policy import interpret
    q = [0, 1.3, -1, 0, .5, 0]
    target, info = interpret({"action": "joint_step", "joint_index": index, "sign": sign, "reason": "调姿"}, q, None)
    expected = q.copy()
    expected[index - 1] += sign * .04
    assert target["joint"] == pytest.approx(expected)
    assert info["delta_rad"] == sign * .04


@pytest.mark.parametrize("index,sign", [(0, 1), (7, 1), (True, 1), (1, True), (1, 1.), (1, 0)])
def test_joint_step_rejects_ambiguous_or_out_of_range_tokens(index, sign):
    from robot.harness.policy import parse_action
    with pytest.raises(ValueError):
        parse_action({"action": "joint_step", "joint_index": index, "sign": sign, "reason": "调姿"})


def test_joint_step_rejects_limit_violation_before_write():
    from robot.harness.policy import interpret
    from robot.harness.vision import UPPER
    with pytest.raises(ValueError, match="limits"):
        interpret({"action": "joint_step", "joint_index": 1, "sign": 1, "reason": "调姿"}, UPPER, None)
