"""Variable translation contract, segmented execution, and explicit static height inputs."""
import json
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot.harness.policy import interpret, parse_action
from robot.harness.scene_reference import load_height_reference, height_feedback
from robot.harness.tests.test_policy import AffineKinematics, stage
from robot.harness.tests.test_vision import VisualSession, load_example
from robot.harness.vision import build_messages, observation, resolve_action, Stopped


@pytest.mark.parametrize("name,direction", [("move", "forward"), ("tool_move", "x+")])
@pytest.mark.parametrize("cm", [1, 1.5, 2, 3.5, 5])
def test_variable_request_preserves_direction_and_preflights_bounded_waypoints(name, direction, cm):
    kin, q = AffineKinematics(), VisualSession().q
    q[3:] = [.3, .4, -.2]
    before = np.asarray(kin.pose(q, "tool"))
    target, info = interpret(dict(action=name, direction=direction, step_cm=cm, reason="选择较大接近步长"), q, kin)
    after = np.asarray(kin.pose(target["joint"], "tool"))
    delta = np.array([cm / 100, 0, 0])
    if name == "tool_move":
        delta = Rotation.from_quat(before[[4, 5, 6, 3]]).apply(delta)
    np.testing.assert_allclose(after[:3] - before[:3], delta, atol=1e-12)
    np.testing.assert_allclose(after[3:], before[3:], atol=1e-12)
    assert len(info["waypoints"]) == max(1, int(np.ceil(cm)))
    for waypoint in info["waypoints"]:
        assert np.max(np.abs(np.array(waypoint["joint"]) - q)) <= .08
        q = waypoint["joint"]


@pytest.mark.parametrize("cm", [True, False, 0, -.1, .99, 5.01, float("nan"), float("inf"), "3", [3]])
def test_invalid_centimetres_cannot_create_a_target(cm):
    with pytest.raises(ValueError, match="step_cm"):
        parse_action(dict(action="tool_move", direction="z+", step_cm=cm, reason="调试"))


def test_mixed_tool_translation_rotation_uses_original_frame_for_whole_path():
    kin, q = AffineKinematics(), VisualSession().q
    q[3:] = [.3, .4, -.2]
    before = np.asarray(kin.pose(q, "tool"))
    r = Rotation.from_quat(before[[4, 5, 6, 3]])
    _, info = resolve_action(dict(action="move_eef_delta", frame="tool", point="tool",
                                 delta_xyz=[.03, 0, 0], delta_rotvec=[0, .09, 0]), q, kin)
    for i, waypoint in enumerate(info["waypoints"], 1):
        pose = np.asarray(waypoint["pose"])
        np.testing.assert_allclose(pose[:3] - before[:3], r.apply([.01 * i, 0, 0]), atol=1e-12)
        expected = r * Rotation.from_rotvec([0, .03 * i, 0])
        np.testing.assert_allclose(Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix(), expected.as_matrix(), atol=1e-12)


def test_loop_executes_five_cm_as_five_verified_segments_and_reports_height(tmp_path):
    example, session, inputs = load_example(), VisualSession(), []
    kin = AffineKinematics()
    before = np.asarray(kin.pose(session.q, "tool"))
    def controller(messages):
        inputs.append(json.loads(messages[1]["content"][0]["text"]))
        return dict(action="move", direction="forward", step_cm=5, reason="空旷路径大步接近")
    out = tmp_path / "five-cm"
    example.run_loop(session, controller, threading.Event(), out, task="approach", legacy_actions=False,
                     planner=lambda _: {"stages": [stage()]}, kinematics=kin, max_rounds=1, interval=.001)
    row = json.loads((out / "turns.jsonl").read_text())
    assert row["status"] == "succeeded" and len(row["execution_segments"]) == 5
    assert len(session.writes) == 5
    assert np.linalg.norm(np.asarray(kin.pose(session.q, "tool"))[:3] - before[:3]) == pytest.approx(.05)
    assert inputs[0]["translation_limits"]["max_cm"] == 5
    ref = inputs[0]["scene_reference"]
    assert ref["table_top_z_m"] == 0 and ref["bowl_rim_z_approx_m"] == .05
    assert ref["tool_height_above_table_m"] == before[2]


def test_late_ik_failure_rejects_entire_request_before_writing(tmp_path):
    example, session = load_example(), VisualSession()
    class LateFailure(AffineKinematics):
        def solve(self, joint, pose, **kw):
            if pose[0] >= .025:
                raise RuntimeError("no further reach")
            return super().solve(joint, pose, **kw)
    example.run_loop(session, lambda _: dict(action="move", direction="forward", step_cm=5, reason="验证预检查"),
                     threading.Event(), tmp_path / "preflight", task="move", legacy_actions=False,
                     planner=lambda _: {"stages": [stage()]}, kinematics=LateFailure(), max_rounds=1)
    assert session.writes == []
    row = json.loads((tmp_path / "preflight/turns.jsonl").read_text())
    assert row["status"] == "rejected"


@pytest.mark.parametrize("mode", ["failure", "cancel"])
def test_segment_failure_or_cancel_never_executes_remaining_waypoints(mode):
    example, stop = load_example(), threading.Event()
    kin, q = AffineKinematics(), VisualSession().q
    target, info = interpret(dict(action="move", direction="forward", step_cm=5, reason="分段"), q, kin)
    class Executor:
        calls = []
        def observe(self):
            return {"joint": q}
        def execute(self, rid, **kw):
            self.calls.append(rid)
            if mode == "cancel":
                stop.set()
            return {"status": "failed" if mode == "failure" else "succeeded", "reason": "test"}
    executor = Executor()
    if mode == "cancel":
        with pytest.raises(Stopped):
            example.execute_motion(executor, "test", target, info, kin, stop)
    else:
        outcome, segments = example.execute_motion(executor, "test", target, info, kin, stop)
        assert outcome["status"] == "failed" and len(segments) == 1
    assert len(executor.calls) == 1


def test_static_table_and_current_tool_height_are_distinct_and_verifier_gets_neither():
    reference = load_height_reference()
    assert reference["table_top_z_m"] == 0
    value = height_feedback(reference, {"tool": [0, 0, .12]})
    assert value["tool_height_above_table_m"] == .12
    assert value["bowl_rim_z_approx_m"] == .05
    assert "not current object state" in value["object_reference"]
    obs, _ = observation(VisualSession())
    obs["scene_reference"] = value
    controller = build_messages(obs, "task", [], stage=stage())
    verifier = build_messages(obs, "task", [], stage=stage(), role="verifier")
    assert "table_top_z_m" in json.dumps(controller)
    assert "table_top_z_m" not in json.dumps(verifier)
    assert all(key not in json.loads(controller[1]["content"][0]["text"]) for key in ("objects", "contacts", "depth_m"))


def test_custom_scene_reference_follows_table_top_and_does_not_assume_bowl_height(tmp_path):
    path = tmp_path / "table.xml"
    path.write_text('<mujoco><worldbody><geom name="table" type="box" pos="0 0 0.7" size="1 1 0.1"/></worldbody></mujoco>')
    value = load_height_reference(path)
    assert value["table_top_z_m"] == pytest.approx(.8)
    assert "bowl_rim_z_approx_m" not in value
    measured = height_feedback(value, {"tool": [0, 0, .9]})
    assert measured["tool_height_above_table_m"] == pytest.approx(.1)
    path.write_text('<mujoco><worldbody><geom name="table" type="box" euler="0 0 0.2" size="1 1 0.1"/></worldbody></mujoco>')
    assert not load_height_reference(path)["available"]


def test_failure_after_partial_motion_reports_partial_state_and_stops_path():
    example, stop = load_example(), threading.Event()
    kin, q = AffineKinematics(), VisualSession().q
    initial_x = kin.pose(q, "tool")[0]
    target, info = interpret(dict(action="move", direction="forward", step_cm=5, reason="分段反馈"), q, kin)
    class Executor:
        def __init__(self):
            self.q, self.calls = q.copy(), 0
        def observe(self):
            return {"joint": self.q.copy()}
        def execute(self, rid, **kw):
            self.calls += 1
            failed = self.calls == 3
            if not failed:
                self.q = kw["joint"]
            return {"status": "failed" if failed else "succeeded", "reason": "verification timeout" if failed else "verified",
                    "stop_confirmed": True, "stop_feedback": {"joint": self.q.copy(), "gripper": 0.}}
    executor = Executor()
    result, segments = example.execute_motion(executor, "partial", target, info, kin, stop)
    assert result["status"] == "failed" and executor.calls == 3
    assert [x["status"] for x in segments] == ["succeeded", "succeeded", "failed"]
    assert kin.pose(result["stop_feedback"]["joint"], "tool")[0] - initial_x == pytest.approx(.02)


def test_loop_uses_attached_service_scene_path_instead_of_assuming_default_table(tmp_path):
    from types import SimpleNamespace
    path = tmp_path / "custom.xml"
    path.write_text('<mujoco><worldbody><geom name="table" type="box" pos="0 0 0.7" size="1 1 0.1"/></worldbody></mujoco>')
    example, session, received = load_example(), VisualSession(), []
    session._config = SimpleNamespace(components=[SimpleNamespace(name="left_arm", args={"scene_path": str(path)})])
    def controller(messages):
        received.append(json.loads(messages[1]["content"][0]["text"]))
        return {"action": "wait", "reason": "读取桌面基准"}
    example.run_loop(session, controller, threading.Event(), tmp_path / "custom", task="observe",
                     planner=lambda _: {"stages": [stage()]}, kinematics=AffineKinematics(),
                     legacy_actions=False, max_rounds=1)
    assert received[0]["scene_reference"]["table_top_z_m"] == pytest.approx(.8)
    assert "bowl_rim_z_approx_m" not in received[0]["scene_reference"]


@pytest.mark.parametrize("hold_status", ["succeeded", "unknown"])
def test_feedback_drift_invalidating_first_waypoint_is_logged_not_raised(hold_status):
    example, stop = load_example(), threading.Event()
    kin, q = AffineKinematics(), VisualSession().q
    target, info = interpret(dict(action="move", direction="forward", step_cm=5, reason="变化后不发旧目标"), q, kin)
    drifted = q.copy(); drifted[0] -= .2
    class Executor:
        calls = []
        def observe(self):
            return {"joint": drifted}
        def execute(self, rid, **kw):
            self.calls.append((rid, kw))
            return {"status": hold_status, "reason": "verified" if hold_status == "succeeded" else "unconfirmed stop",
                    "stop_confirmed": hold_status == "succeeded", "stop_feedback": {"joint": drifted}}
    executor = Executor()
    result, segments = example.execute_motion(executor, "drift", target, info, kin, stop)
    assert result["status"] == ("failed" if hold_status == "succeeded" else "unknown")
    assert len(executor.calls) == 1
    assert executor.calls[0] == ("drift-revalidation-hold", {"joint": drifted})
    assert segments[0]["status"] == "rejected" and segments[0]["hold_status"] == hold_status
