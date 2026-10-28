"""Tool/body increments, wrist references and the actual layered execution path."""
import json
import threading

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot.harness.policy import interpret, parse_action, wrist_tool_guide
from robot.harness.vision import build_messages, observation, resolve_action
from robot.harness.tests.test_policy import AffineKinematics, camera, stage
from robot.harness.tests.test_vision import VisualSession, load_example


@pytest.mark.parametrize("axis", list("xyz"))
@pytest.mark.parametrize("sign", [-1, 1])
def test_body_translation_rotates_with_tool_and_preserves_orientation(axis, sign):
    kin, q = AffineKinematics(), VisualSession().q
    q[3:] = [.3, .4, -.2]  # Nontrivial orientation; base and tool differ.
    before = np.asarray(kin.pose(q, "tool"))
    r = Rotation.from_quat(before[[4, 5, 6, 3]])
    action = dict(action="tool_move", direction=axis + ("+" if sign > 0 else "-"), reason="腕图中碗沿偏离夹指间隙")
    target, info = interpret(action, q, kin)
    after = np.asarray(kin.pose(target["joint"], "tool"))
    delta = np.eye(3)["xyz".index(axis)] * sign * .01
    np.testing.assert_allclose(after[:3] - before[:3], r.apply(delta), atol=1e-12)
    np.testing.assert_allclose(after[3:], before[3:], atol=1e-12)
    assert np.linalg.norm(info["resolved_delta_xyz_m"]) == pytest.approx(.01)
    assert info["requested_frame"] == "tool" and info["frame"] == "base"


@pytest.mark.parametrize("axis", list("xyz"))
@pytest.mark.parametrize("sign", [-1, 1])
def test_body_rotation_right_multiplies_and_holds_tool_origin(axis, sign):
    kin, q = AffineKinematics(), VisualSession().q
    q[3:] = [.3, .4, -.2]
    before = np.asarray(kin.pose(q, "tool"))
    r = Rotation.from_quat(before[[4, 5, 6, 3]])
    target, info = interpret(dict(action="tool_rotate", axis=axis, sign=sign, reason="调整夹口朝向"), q, kin)
    after = np.asarray(kin.pose(target["joint"], "tool"))
    rv = np.eye(3)["xyz".index(axis)] * sign * .04
    expected = r * Rotation.from_rotvec(rv)
    np.testing.assert_allclose(Rotation.from_quat(after[[4, 5, 6, 3]]).as_matrix(), expected.as_matrix(), atol=1e-12)
    np.testing.assert_allclose(after[:3], before[:3], atol=1e-12)
    np.testing.assert_allclose(info["delta_rotvec_base_rad"], r.apply(rv), atol=1e-12)


def test_combined_body_delta_uses_axes_before_rotation_and_keeps_limits():
    kin, q = AffineKinematics(), VisualSession().q
    q[3:] = [.3, .4, -.2]
    before = np.asarray(kin.pose(q, "tool"))
    r = Rotation.from_quat(before[[4, 5, 6, 3]])
    action = dict(action="move_eef_delta", frame="tool", point="tool",
                  delta_xyz=[0, 0, .01], delta_rotvec=[.04, 0, 0])
    resolved, _ = resolve_action(action, q, kin)
    np.testing.assert_allclose(resolved["pose"][:3], before[:3] + r.apply([0, 0, .01]))
    for change, message in [({"point": "flange"}, "point=tool"),
                            ({"delta_xyz": [0, 0, .005]}, "0.01 to 0.05"),
                            ({"delta_rotvec": [.16, 0, 0]}, "0.15"),
                            ({"frame": "camera"}, "base or tool")]:
        with pytest.raises(ValueError, match=message):
            resolve_action({**action, **change}, q, kin)


@pytest.mark.parametrize("extra", [dict(action="tool_move", direction="up"),
    dict(action="tool_move", direction=["x+"]), dict(action="tool_rotate", axis="screen", sign=1),
    dict(action="tool_rotate", axis="z", sign=True), dict(action="tool_rotate", axis="z", sign=1.),
    dict(action="tool_rotate", axis="z", sign=0), dict(action="tool_move", direction="z+", metres=1)])
def test_bad_tool_tokens_are_rejected(extra):
    with pytest.raises(ValueError):
        parse_action({**extra, "reason": "调整"})


def test_wrist_reference_is_robot_only_and_depth_independent():
    kin, q = AffineKinematics(), VisualSession().q
    cameras = {"wrist": camera()}
    cameras["wrist"].update(depth_m="SECRET_DEPTH", objects="SECRET_OBJECTS", timestamp_ns=123)
    guide = wrist_tool_guide(q, kin, cameras)
    assert guide["available"] and guide["image_timestamp_ns"] == 123
    assert "SECRET" not in json.dumps(guide)
    pose = np.asarray(kin.pose(q, "tool"))
    r = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()
    np.testing.assert_allclose(np.column_stack([guide["tool_axes_optical"][a] for a in "xyz"]), r)
    # Rotate the camera: RDF hints must change, not silently use base axes.
    cameras["wrist"]["world_from_camera"][3:] = [2**-.5, 0, 0, 2**-.5]
    turned = wrist_tool_guide(q, kin, cameras)
    expected = Rotation.from_euler("z", -90, degrees=True).as_matrix() @ r
    np.testing.assert_allclose(np.column_stack([turned["tool_axes_optical"][a] for a in "xyz"]), expected, atol=1e-12)
    cameras["wrist"]["world_from_camera"] = "SECRET_BAD_CALIBRATION"
    assert not wrist_tool_guide(q, kin, cameras)["available"]


def test_layered_loop_executes_tool_motion_and_exposes_feedback(tmp_path):
    example, session, kin = load_example(), VisualSession(), AffineKinematics()
    session.q[3:] = [.3, .4, -.2]
    before = np.asarray(kin.pose(session.q, "tool"))
    calls = []
    def policy(messages):
        data = json.loads(messages[1]["content"][0]["text"])
        calls.append(data)
        assert "SECRET" not in json.dumps(messages)
        assert "tool_translation_options" in data["robot"]["eef_fk"]
        if len(calls) == 1:
            return dict(action="tool_move", direction="x+", reason="按腕图减小碗沿与夹口偏差")
        assert data["recent_actions"][0]["status"] == "succeeded"
        assert data["recent_actions"][0]["action_resolution"]["requested_frame"] == "tool"
        return dict(action="wait", reason="复查腕部视角")
    example.run_loop(session, policy, threading.Event(), tmp_path / "tool-loop", task="grasp",
                     planner=lambda _: {"stages": [stage()]}, kinematics=kin,
                     legacy_actions=False, max_rounds=2, interval=.001)
    after = np.asarray(kin.pose(session.q, "tool"))
    expected = Rotation.from_quat(before[[4, 5, 6, 3]]).apply([.01, 0, 0])
    np.testing.assert_allclose(after[:3] - before[:3], expected, atol=.001)
    assert session.released


def test_wrist_reference_only_sent_to_controller():
    obs, _ = observation(VisualSession())
    obs["wrist_tool_guide"] = {"available": False}
    for role in ("controller", "planner", "verifier"):
        messages = build_messages(obs, "grasp", [], role=role, stage=stage())
        data = json.loads(messages[1]["content"][0]["text"])
        assert ("wrist_tool_guide" in data) == (role == "controller")


def test_tool_motion_rejects_pose_drift_during_model_call(tmp_path):
    session, kin = VisualSession(), AffineKinematics()
    session.q[3:] = [.3, .4, -.2]
    def policy(_):
        session.q[3] += .03  # External drift after the pictured pose.
        return dict(action="tool_move", direction="x+", reason="腕图对齐")
    output = tmp_path / "drift"
    load_example().run_loop(session, policy, threading.Event(), output, task="grasp",
        planner=lambda _: {"stages": [stage()]}, kinematics=kin,
        legacy_actions=False, max_rounds=1, interval=.001)
    record = json.loads((output / "turns.jsonl").read_text())
    assert record["status"] == "rejected" and "changed since pictured" in record["reason"]
    assert session.writes == [] and session.released


def test_fast_planner_waits_for_advancing_camera_and_enter_can_cancel(tmp_path, monkeypatch):
    example, session = load_example(), VisualSession()
    original = example.observation
    attempts = []
    def acquisition(session, previous, **kwargs):
        attempts.append(previous)
        if len(attempts) == 2:
            raise RuntimeError("camera did not advance")
        return original(session, previous, **kwargs)
    monkeypatch.setattr(example, "observation", acquisition)
    output = tmp_path / "new-frame"
    example.run_loop(session, lambda _: dict(action="wait", reason="重新观察"),
        threading.Event(), output, task="grasp", planner=lambda _: {"stages": [stage()]},
        legacy_actions=False, max_rounds=1, interval=.001)
    assert len(attempts) == 3
    assert json.loads((output / "turns.jsonl").read_text())["status"] == "waited"

    stop = threading.Event()
    def cancelled(session, previous, **kwargs):
        stop.set()
        raise RuntimeError("camera did not advance")
    monkeypatch.setattr(example, "observation", cancelled)
    session = VisualSession()
    example.run_loop(session, lambda _: pytest.fail("cancelled acquisition must not call model"),
                     stop, tmp_path / "cancelled", task="grasp", max_rounds=1)
    assert session.writes == [] and session.released


def test_wrist_tool_origin_is_not_clamped_or_invented_when_behind_camera():
    kin, q = AffineKinematics(), VisualSession().q
    value = camera(origin=(0, 0, 2))
    guide = wrist_tool_guide(q, kin, {"wrist": value})
    assert guide["available"] and guide["tool_origin_pixel"] is None
    assert not guide["tool_origin_in_image"]
    value = camera(origin=(10, 0, -2))
    guide = wrist_tool_guide(q, kin, {"wrist": value})
    assert guide["tool_origin_pixel"][0] < 0 and not guide["tool_origin_in_image"]
