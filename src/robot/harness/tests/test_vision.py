import base64
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import pytest

from robot.harness.vision import (Stopped, build_messages, chat_action, deepseek_action, interruptible_call,
                                  observation, resolve_action, validate_action)
from robot.harness.tests.test_executor import Session


class VisualSession(Session):
    def __init__(self):
        super().__init__()
        self.q = [0, 1.3, -1, 0, .5, 0]

    def describe(self):
        return [dict(name="left_arm", class_path="robot.controller.PiperMuJoCo_controller",
                     class_name="PiperMuJoCoController")]

    def read_raw(self, name):
        frame = super().read_raw(name)
        frame["data"].update(objects="SECRET_OBJECT_POSES", contacts="SECRET_CONTACTS",
            cameras={key: dict(rgb=np.zeros((24, 32, 3), dtype=np.uint8),
                               timestamp_ns=time.time_ns(), depth_m="SECRET_DEPTH",
                               world_from_camera="SECRET_EXTRINSICS", intrinsics="SECRET_INTRINSICS")
                     for key in ("head", "wrist")}, hidden="SECRET_FUTURE_FIELD")
        return frame


def load_example():
    examples = Path(__file__).resolve().parents[4] / "examples"
    sys.path.insert(0, str(examples))
    try:
        spec = importlib.util.spec_from_file_location("visual_example", examples / "piper_mujoco_agent.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(examples))


def test_actual_http_payload_excludes_truth_and_encodes_two_images(monkeypatch):
    obs, _ = observation(VisualSession())
    messages = build_messages(obs, "look", [])
    def urlopen(request, timeout):
        payload = json.loads(request.data)
        assert "SECRET" not in json.dumps(payload)
        parts = payload["messages"][1]["content"]
        images = [p["image_url"]["url"] for p in parts if p["type"] == "image_url"]
        assert len(images) == 2
        assert all(base64.b64decode(i.split(",")[1]).startswith(b"\x89PNG\r\n\x1a\n") for i in images)
        assert set(json.loads(parts[0]["text"])["robot"]) == {"joint_rad", "gripper"}
        import io
        return io.BytesIO(json.dumps({"choices": [{"finish_reason": "stop", "message": {
            "content": '{"action":"wait"}'}}]}).encode())
    monkeypatch.setattr("urllib.request.urlopen", urlopen)
    assert deepseek_action(messages, "not-a-real-key") == {"action": "wait"}


def test_custom_chat_endpoint_has_no_deepseek_only_parameters(monkeypatch):
    def urlopen(request, timeout):
        payload = json.loads(request.data)
        assert request.full_url == "http://127.0.0.1:3425/v1/chat/completions"
        assert payload["model"] == "codex/gpt-6.1-sol"
        assert "thinking" not in payload and "temperature" not in payload
        assert "test-key" not in request.data.decode()
        import io
        return io.BytesIO(json.dumps({"choices":[{"finish_reason":"stop","message":{
            "content":'{"action":"wait","reason":"test"}'}}]}).encode())
    monkeypatch.setattr("urllib.request.urlopen", urlopen)
    assert chat_action([], "test-key", "codex/gpt-6.1-sol", "http://127.0.0.1:3425/v1/")["action"] == "wait"


def test_stale_and_repeated_cameras_rejected():
    session = VisualSession()
    obs, stamps = observation(session)
    with pytest.raises(RuntimeError, match="advance"):
        observation(session, {k: time.time_ns() + 10**9 for k in stamps})
    frame = session.read_raw("left_arm")
    frame["data"]["cameras"]["head"]["timestamp_ns"] -= 2_000_000_000
    session.read_raw = lambda name: frame
    with pytest.raises(RuntimeError, match="stale"):
        observation(session)


@pytest.mark.parametrize("action", [
    {"action": "move_joint_absolute", "joint": [1]*6},
    {"action": "set_gripper", "gripper": True},
    {"action": "set_gripper", "gripper": float("nan")},
    {"action": "wait", "objects": {}}, {"action": "shell", "command": "anything"},
])
def test_actions_reject_unbounded_or_extra_inputs(action):
    with pytest.raises(ValueError):
        validate_action(action, [0]*6)


def test_stop_during_model_wait_discards_late_result():
    stop = threading.Event()
    timer = threading.Timer(.05, stop.set)
    timer.start()
    started = time.monotonic()
    with pytest.raises(Stopped):
        interruptible_call(lambda: time.sleep(1), stop)
    assert time.monotonic() - started < .5
    timer.join()


def test_loop_multiple_turns_and_cleanup(tmp_path):
    session, stop = VisualSession(), threading.Event()
    calls = []
    def policy(messages):
        calls.append(messages)
        return {"action": "wait"}
    load_example().run_loop(session, policy, stop, tmp_path / "run", task="look",
                            interval=.001, max_rounds=3)
    assert len(calls) == 3
    assert session.released
    assert "SECRET" not in (tmp_path / "run/latest_messages.json").read_text()


def test_stop_during_motion_cancels_and_releases(tmp_path):
    session, stop = VisualSession(), threading.Event()
    session.stuck = True
    timer = None
    def policy(messages):
        nonlocal timer
        timer = threading.Timer(.08, stop.set)
        timer.start()
        q = list(session.q); q[1] += .05
        return {"action": "move_joint_absolute", "joint": q}
    load_example().run_loop(session, policy, stop, tmp_path / "run", task="move")
    timer.join()
    rows = [json.loads(s) for s in (tmp_path / "run/control_000000.jsonl").read_text().splitlines()]
    result = next(r["result"] for r in rows if r["event"] == "finished")
    assert result["status"] == "cancelled" and result["stop_confirmed"]
    assert session.released


def test_stop_during_model_wait_never_executes_reply(tmp_path):
    session, stop = VisualSession(), threading.Event()
    def policy(messages):
        stop.set()
        return {"action": "set_gripper", "gripper": 1}
    load_example().run_loop(session, policy, stop, tmp_path / "run", task="move")
    assert session.released
    assert all(w["gripper"] == 0 for w in session.writes)  # constructor hold only


def test_wait_reason_and_no_progress_limit(tmp_path, capsys):
    session, stop = VisualSession(), threading.Event()
    action = {"action": "wait", "reason": "无法确定像素与关节动作的关系"}
    assert validate_action(action, session.q) == {}
    calls = []
    def policy(messages):
        calls.append(messages)
        return action
    load_example().run_loop(session, policy, stop, tmp_path / "run", task="stack",
                            interval=.001, max_rounds=10, max_waits=3)
    assert len(calls) == 3 and session.released
    assert "已暂停模型调用" in capsys.readouterr().out
    with pytest.raises(ValueError):
        validate_action({"action": "wait", "reason": {"hidden": "data"}}, session.q)


def test_pinocchio_eef_loop_and_model_input(tmp_path):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin = PiperKinematics()
    session, stop = VisualSession(), threading.Event()
    initial = kin.pose(session.q, "tool")
    desired = list(initial); desired[2] += .01
    def policy(messages):
        assert "SECRET" not in json.dumps(messages)
        state = json.loads(messages[1]["content"][0]["text"])["robot"]
        np.testing.assert_allclose(state["eef_fk"]["tool"], initial)
        return dict(action="move_eef", pose=desired, frame="base", point="tool")
    load_example().run_loop(session, policy, stop, tmp_path / "eef", task="raise 1cm",
                            max_rounds=1, kinematics=kin)
    row = json.loads((tmp_path / "eef/turns.jsonl").read_text())
    assert row["status"] == "succeeded"
    assert row["eef_verification"]["position_error_m"] < .001
    assert session.released


@pytest.mark.parametrize("change", [
    {"frame": "world"}, {"point": "bowl"}, {"pose": [0]*7},
    {"pose": [True, 0, 0, 1, 0, 0, 0]}, {"pose": [float("nan"), 0, 0, 1, 0, 0, 0]},
])
def test_eef_rejects_invalid_pose_contract(change):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin = PiperKinematics(); q = [0, 1.3, -1, 0, .5, 0]
    action = dict(action="move_eef", pose=kin.pose(q, "tool"), frame="base", point="tool")
    action.update(change)
    with pytest.raises(ValueError):
        validate_action(action, q, kin)


def test_eef_limits_and_failed_ik_cannot_reach_executor(monkeypatch):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin = PiperKinematics(); q = [0, 1.3, -1, 0, .5, 0]
    action = dict(action="move_eef", pose=kin.pose(q, "tool"), frame="base", point="tool")
    action["pose"][2] += .03
    with pytest.raises(ValueError, match="EEF step"):
        validate_action(action, q, kin)
    action["pose"][2] -= .02
    monkeypatch.setattr(kin, "solve", lambda *a, **kw: [0, 1.5, -1, 0, .5, 0])
    with pytest.raises(ValueError, match="joint step"):
        validate_action(action, q, kin)
    def failed(*a, **kw):
        raise RuntimeError("did not converge")
    monkeypatch.setattr(kin, "solve", failed)
    with pytest.raises(ValueError, match="did not converge"):
        validate_action(action, q, kin)


def test_delta_one_centimetre_and_temporal_rgb_boundary(tmp_path):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin = PiperKinematics(); session = VisualSession(); calls = []
    def policy(messages):
        calls.append(messages)
        if len(calls) == 1:
            return dict(action="move_eef_delta", delta_xyz=[0,0,.01], frame="base", point="tool")
        assert len([c for c in messages[1]["content"] if c["type"] == "image_url"]) == 4
        assert "SECRET" not in json.dumps(messages)
        feedback = json.loads(messages[1]["content"][0]["text"])["recent_actions"][0]
        assert feedback["action_resolution"]["scale"] == 1
        return dict(action="wait")
    load_example().run_loop(session, policy, threading.Event(), tmp_path / "run", task="visual probe",
                            interval=.001, max_rounds=2, kinematics=kin)
    assert len(calls) == 2
    action = dict(action="move_eef_delta", delta_xyz=[0,0,.003], frame="base", point="tool")
    with pytest.raises(ValueError, match="exactly 0.01"):
        resolve_action(action, session.q, kin)


def test_eef_rotation_holds_tool_point_and_reduces_downward_tilt(tmp_path):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    from robot.harness.vision import orientation_feedback
    from scipy.spatial.transform import Rotation
    kin = PiperKinematics(); session = VisualSession(); q = list(session.q)
    original = np.asarray(kin.pose(q, "tool"))
    before = orientation_feedback(q, kin)
    assert 48 < before["tool_down_tilt_deg"] < 50
    action = before["orientation_hint"]["action"]
    resolved, info = resolve_action(action, q, kin)
    np.testing.assert_allclose(resolved["pose"][:3], original[:3], atol=1e-12)
    new = np.asarray(resolved["pose"])
    expected = Rotation.from_rotvec(action["delta_rotvec"]) * Rotation.from_quat(original[[4,5,6,3]])
    np.testing.assert_allclose(Rotation.from_quat(new[[4,5,6,3]]).as_matrix(), expected.as_matrix(), atol=1e-12)
    def policy(messages):
        data=json.loads(messages[1]["content"][0]["text"])
        assert data["scene_priors"]["nominal_bowl_rim_z_m"] == .05
        assert "SECRET" not in json.dumps(messages)
        return action
    load_example().run_loop(session, policy, threading.Event(), tmp_path / "rotate", task="downward",
                            max_rounds=1, kinematics=kin)
    after = orientation_feedback(session.q, kin)
    assert after["tool_down_tilt_deg"] < before["tool_down_tilt_deg"] - .5
    np.testing.assert_allclose(kin.pose(session.q, "tool")[:3], original[:3], atol=.001)
    result = json.loads((tmp_path / "rotate/turns.jsonl").read_text())
    assert result["status"] == "succeeded"


@pytest.mark.parametrize("rv", [[0,.16,0], [True,0,0], [float("nan"),0,0], [0,.04]])
def test_rotation_vector_rejects_invalid_input(rv):
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    kin=PiperKinematics(); q=[0,1.3,-1,0,.5,0]
    with pytest.raises(ValueError, match="delta_rotvec"):
        resolve_action(dict(action="move_eef_delta",delta_xyz=[0,0,0],delta_rotvec=rv,frame="base",point="tool"),q,kin)


def test_base_guide_stays_fixed_while_tool_axes_rotate():
    pytest.importorskip("pinocchio")
    from robot.harness.kinematics import PiperKinematics
    from robot.harness.vision import orientation_feedback
    kin=PiperKinematics(); session=VisualSession(); guides=[]; axes=[]
    for q1 in (0., .2):
        session.q[0]=q1
        obs,_=observation(session)
        obs["eef_fk"]=orientation_feedback(session.q,kin)
        messages=build_messages(obs,"check coordinate directions",[])
        assert "SECRET" not in json.dumps(messages)
        payload=json.loads(messages[1]["content"][0]["text"])
        guides.append(payload["coordinate_guide"])
        fk=payload["robot"]["eef_fk"]
        matrix=np.column_stack([fk["tool_"+a+"_axis_base"] for a in "xyz"])
        np.testing.assert_allclose(matrix.T@matrix,np.eye(3),atol=1e-12)
        assert np.linalg.det(matrix) == pytest.approx(1.)
        axes.append(matrix)
    assert guides[0] == guides[1]
    assert not np.allclose(axes[0],axes[1])
    assert guides[0]["one_cm_examples_m"]["down"] == [0,0,-.01]
    assert guides[0]["one_cm_examples_m"]["left"] == [0,.01,0]


def test_failed_move_replans_with_feedback_and_blocks_identical_retry(tmp_path, monkeypatch):
    example=load_example(); base=example.PositionExecutor
    class FastExecutor(base):
        def execute(self, rid, **kw):
            return super().execute(rid,timeout_s=.08,dwell_s=.01,**kw)
    monkeypatch.setattr(example,"PositionExecutor",FastExecutor)
    session=VisualSession();session.stuck=True;calls=[]
    first_q=list(session.q);first_q[1]+=.03
    first=dict(action="move_joint_absolute",joint=first_q)
    def policy(messages):
        calls.append(messages)
        history=json.loads(messages[1]["content"][0]["text"])["recent_actions"]
        if len(calls)==1:return first
        if len(calls)==2:
            assert history[-1]["status"]=="failed"
            assert history[-1]["recovery"]["status"]=="ready"
            assert history[-1]["actual_feedback"]["joint_rad"]==session.q
            return {**first,"joint":[float(x) for x in first["joint"]]}  # numeric spelling is not a new plan
        assert history[-1]["status"]=="rejected"
        session.stuck=False;q=list(session.q);q[0]+=.02
        return dict(action="move_joint_absolute",joint=q)
    example.run_loop(session,policy,threading.Event(),tmp_path/"replan",task="adjust after failure",interval=.001,max_rounds=3)
    rows=[json.loads(x) for x in (tmp_path/"replan/turns.jsonl").read_text().splitlines()]
    assert [x["status"] for x in rows]==["failed","rejected","succeeded"]
    assert len(session.writes)==2 and session.released


def test_unknown_stop_does_not_recover_or_call_model_again(tmp_path,monkeypatch):
    example=load_example();base=example.PositionExecutor
    class UnknownExecutor(base):
        def execute(self,rid,**kw):
            return dict(status="unknown",reason="transport error",stop_confirmed=False)
        def recover(self):pytest.fail("unconfirmed stop cannot auto-recover")
    monkeypatch.setattr(example,"PositionExecutor",UnknownExecutor)
    session=VisualSession();calls=[]
    def policy(messages):
        calls.append(messages);q=list(session.q);q[0]+=.02
        return dict(action="move_joint_absolute",joint=q)
    example.run_loop(session,policy,threading.Event(),tmp_path/"unknown",task="move",max_rounds=3)
    assert len(calls)==1 and session.released


def test_consecutive_execution_failures_are_bounded(tmp_path,monkeypatch):
    example=load_example();base=example.PositionExecutor
    class FastExecutor(base):
        def execute(self,rid,**kw):return super().execute(rid,timeout_s=.08,dwell_s=.01,**kw)
    monkeypatch.setattr(example,"PositionExecutor",FastExecutor)
    session=VisualSession();session.stuck=True;calls=[]
    def policy(messages):
        calls.append(messages);q=list(session.q);q[0]+=.02*len(calls)
        return dict(action="move_joint_absolute",joint=q)
    example.run_loop(session,policy,threading.Event(),tmp_path/"bounded",task="move",interval=.001,max_rounds=10,max_failures=3)
    rows=[json.loads(x) for x in (tmp_path/"bounded/turns.jsonl").read_text().splitlines()]
    assert len(calls)==3
    assert [x.get("recovery",{}).get("status") for x in rows]==["ready","ready",None]
    assert session.released


def test_enter_after_failed_motion_skips_recovery(tmp_path,monkeypatch):
    example=load_example();base=example.PositionExecutor;stop=threading.Event()
    class InterruptedExecutor(base):
        def execute(self,rid,**kw):
            result=super().execute(rid,timeout_s=.08,dwell_s=.01,**kw)
            stop.set();return result
        def recover(self):pytest.fail("Enter must prevent further recovery")
    monkeypatch.setattr(example,"PositionExecutor",InterruptedExecutor)
    session=VisualSession();session.stuck=True
    def policy(messages):
        q=list(session.q);q[0]+=.02
        return dict(action="move_joint_absolute",joint=q)
    example.run_loop(session,policy,stop,tmp_path/"enter",task="move")
    assert session.released
