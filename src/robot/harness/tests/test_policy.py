"""Offline geometry, payload boundaries, and layered visual-loop behavior."""
from copy import deepcopy
from importlib import resources
import json
from pathlib import Path
import threading
import time
import tomllib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot.harness.policy import (interpret, load_prompt, parse_action, parse_plan,
                                  parse_verification, pixel_to_base)
from robot.harness.tests.test_vision import VisualSession, load_example
from robot.harness.vision import LOWER, UPPER, build_messages, observation


@pytest.fixture(autouse=True)
def forbid_live_api(monkeypatch):
    def blocked(*args, **kwargs):
        pytest.fail("policy tests must not call a model API")
    monkeypatch.setattr("urllib.request.urlopen", blocked)


def stage(identifier="grasp", motion="GRASP"):
    return dict(id=identifier, target="visible bowl", affordance="visible rim",
                motion=motion, description="Act from the current images.",
                completion=f"Visible evidence for {identifier}.")


def camera(*, origin=(0, 0, 0), depth=True):
    value = dict(intrinsics=np.array([[4., 0, 5], [0, 5., 4], [0, 0, 1]]),
                 world_from_camera=np.array([*origin, 1., 0, 0, 0]),
                 rgb_shape=(9, 11))
    if depth:
        value["depth_m"] = np.full((9, 11), 2.)
    return value


def stereo_cameras():
    cameras = {"head": camera(depth=False), "wrist": camera(origin=(1, 0, 0), depth=False)}
    for value in cameras.values():
        value.update(intrinsics=np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1]]),
                     rgb_shape=(101, 101), depth_m="SECRET_MALFORMED_DEPTH")
    return cameras


class CalibratedSession(VisualSession):
    def __init__(self):
        super().__init__()
        self.cameras = {name: camera() for name in ("head", "wrist")}
        for value in self.cameras.values():
            value["rgb"] = np.zeros((9, 11, 3), dtype=np.uint8)

    def read_raw(self, name):
        frame = super().read_raw(name)
        frame["data"]["cameras"] = {
            key: {**value, "timestamp_ns": time.time_ns(), "hidden": "SECRET_CAMERA_FIELD"}
            for key, value in self.cameras.items()}
        return frame


class AffineKinematics:
    """Deterministic local FK/IK to exercise real validation without optional assets."""
    def pose(self, joint, point):
        q = np.asarray(joint)
        xyzw = Rotation.from_rotvec(q[3:]).as_quat()
        return [*(q[:3] * .5), *xyzw[[3, 0, 1, 2]]]

    def solve(self, joint, pose, *, frame, point):
        pose = np.asarray(pose)
        return np.concatenate((pose[:3] * 2,
                               Rotation.from_quat(pose[[4, 5, 6, 3]]).as_rotvec())).tolist()

    def error(self, joint, pose, point):
        actual, desired = np.asarray(self.pose(joint, point)), np.asarray(pose)
        a = Rotation.from_quat(actual[[4, 5, 6, 3]])
        b = Rotation.from_quat(desired[[4, 5, 6, 3]])
        return float(np.linalg.norm(actual[:3] - desired[:3])), float((b * a.inv()).magnitude())


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


@pytest.mark.parametrize("matrix_pose", [False, True], ids=["xyz-wxyz", "matrix"])
def test_off_axis_depth_is_optical_z_and_camera_pose_transforms_into_base(matrix_pose):
    value = camera()
    # Camera rotated +90 degrees about base Z, with a translated optical origin.
    if matrix_pose:
        value["world_from_camera"] = np.array([
            [0, -1, 0, .4], [1, 0, 0, -.2], [0, 0, 1, .3], [0, 0, 0, 1]])
    else:
        value["world_from_camera"] = [.4, -.2, .3, 2 ** -.5, 0, 0, 2 ** -.5]
    result = pixel_to_base({"head": value}, "head", [8, 6])
    # Optical point is [1.5, .8, 2], not a unit ray scaled to length 2.
    np.testing.assert_allclose(result["point_base_m"], [-.4, 1.3, 2.3], atol=1e-12)
    assert result["depth_m"] == 2 and result["method"] == "depth"
    assert result["pixel"] == [8, 6]
    assert set(result) == {"point_base_m", "method", "camera", "pixel", "depth_m"}


def test_stereo_intersects_corresponding_rays_without_reading_depth():
    result = pixel_to_base(stereo_cameras(), "head", [60, 55], second=("wrist", [10, 55]))
    np.testing.assert_allclose(result["point_base_m"], [.2, .1, 2], atol=1e-12)
    assert result["method"] == "two_view" and result["residual_m"] < 1e-12
    assert result["cameras"] == ["head", "wrist"]
    assert result["pixels"] == [[60, 55], [10, 55]]
    assert "depth_m" not in result and "intrinsics" not in result


@pytest.mark.parametrize("first,second,other,error", [
    ([50, 50], [50, 50], "wrist", "parallel"),
    ([40, 50], [60, 50], "wrist", "in front"),
    ([60, 55], [10, 85], "wrist", "disagree"),
    ([60, 55], [10, 55], "head", "different cameras"),
    ([101, 55], [10, 55], "wrist", "outside"),
])
def test_stereo_rejects_degenerate_or_inconsistent_correspondences(first, second, other, error):
    with pytest.raises(ValueError, match=error):
        pixel_to_base(stereo_cameras(), "head", first, second=(other, second))


@pytest.mark.parametrize("pixel", [None, (1, 2), [], [1], [1, 2, 3], [True, 2],
                                   ["1", 2], [float("nan"), 2], [float("inf"), 2],
                                   [-.1, 2], [10.01, 2], [2, 8.01]])
def test_pixel_query_rejects_invalid_or_out_of_image_coordinates(pixel):
    with pytest.raises(ValueError):
        pixel_to_base({"head": camera()}, "head", pixel)


@pytest.mark.parametrize("depth", [0, -1, 5.01, float("nan"), float("inf")])
def test_depth_query_requires_a_finite_positive_bounded_sample(depth):
    value = camera()
    value["depth_m"][4, 5] = depth
    with pytest.raises(ValueError, match="finite depth"):
        pixel_to_base({"head": value}, "head", [5, 4])


@pytest.mark.parametrize("change", [
    {"depth_m": None}, {"depth_m": np.ones(9)}, {"rgb_shape": (8, 11)},
    {"intrinsics": np.eye(2)},
    {"intrinsics": [[0, 0, 5], [0, 5, 4], [0, 0, 1]]},
    {"intrinsics": [[4, .1, 5], [0, 5, 4], [0, 0, 1]]},
    {"intrinsics": [[4, 0, 5], [0, 5, 4], [0, 1, 1]]},
    {"intrinsics": np.full((3, 3), np.nan)},
    {"world_from_camera": [0, 0, 0, 2, 0, 0, 0]},
    {"world_from_camera": [0, 0, 0, 0, 0, 0, 0]},
    {"world_from_camera": [0, 0, 0, 1, 0, 0]},
    {"world_from_camera": [float("nan"), 0, 0, 1, 0, 0, 0]},
    {"world_from_camera": np.diag([2., 1, 1, 1])},
    {"world_from_camera": np.diag([-1., 1, 1, 1])},
    {"world_from_camera": np.diag([1., 1, 1, 2])},
])
def test_query_rejects_invalid_depth_calibration_and_camera_pose(change):
    value = camera()
    value.update(change)
    with pytest.raises(ValueError):
        pixel_to_base({"head": value}, "head", [5, 4])


@pytest.mark.parametrize("missing", ["depth_m", "intrinsics", "world_from_camera"])
def test_depth_query_does_not_guess_missing_measurements(missing):
    value = camera()
    del value[missing]
    with pytest.raises(ValueError):
        pixel_to_base({"head": value}, "head", [5, 4])
    with pytest.raises(ValueError):
        pixel_to_base({}, "head", [5, 4])


@pytest.mark.parametrize("stamp", [None, 0, -1, True, "19000000000", 19_000_000_000.,
                                    float("nan"), float("inf"), [], {}, 20_000_000_001])
def test_point_action_rejects_missing_malformed_or_future_timestamp(monkeypatch, stamp):
    now = 20_000_000_000
    monkeypatch.setattr("robot.harness.policy.time.time_ns", lambda: now)
    value = camera()
    if stamp is not None:
        value["timestamp_ns"] = stamp
    with pytest.raises(ValueError, match="snapshot.*timestamp_ns.*missing or invalid|snapshot.*future"):
        interpret(dict(action="point", camera="head", pixel=[5, 4], reason="查看目标"),
                  [0] * 6, None, cameras={"head": value})


@pytest.mark.parametrize("cameras", [None, {}, {"head": None}, {"head": []}])
def test_point_action_requires_the_pictured_snapshot(cameras):
    with pytest.raises(ValueError, match="snapshot.*missing or invalid"):
        interpret(dict(action="point", camera="head", pixel=[5, 4], reason="查看目标"),
                  [], None, cameras=cameras)


@pytest.mark.parametrize("delay_ns", [1_000_000_001, 60_000_000_000, 86_400_000_000_000])
def test_delayed_point_uses_exact_validated_snapshot_as_historical_evidence(monkeypatch, delay_ns):
    session, local = CalibratedSession(), {}
    obs, stamps = observation(session, local=local)
    for name in local:
        assert local[name]["timestamp_ns"] == stamps[name]
        assert not np.shares_memory(local[name]["depth_m"], session.cameras[name]["depth_m"])
        session.cameras[name]["depth_m"][:] = 4
        session.cameras[name]["intrinsics"][0, 0] = 100
        session.cameras[name]["world_from_camera"][0] = 10
    now = max(stamps.values()) + delay_ns
    monkeypatch.setattr("robot.harness.policy.time.time_ns", lambda: now)
    payload, resolution = interpret(
        dict(action="point", camera="head", pixel=[8, 6], reason="查看碗沿"),
        obs["joint_rad"], None, cameras=local)
    assert payload is None and session.writes == []
    np.testing.assert_allclose(resolution["point_base_m"], [1.5, .8, 2])
    assert resolution["image_timestamps_ns"] == {"head": stamps["head"]}
    assert resolution["queried_at_ns"] == now
    assert resolution["image_age_ms_at_query"] == {"head": (now - stamps["head"]) / 1e6}
    assert resolution["source"] == "captured_snapshot"
    assert resolution["evidence"] == "historical" and resolution["live_motion_target"] is False
    assert "depth_m" not in obs and "intrinsics" not in obs


@pytest.mark.parametrize("direction,expected", [
    ("forward", [.01, 0, 0]), ("back", [-.01, 0, 0]),
    ("left", [0, .01, 0]), ("right", [0, -.01, 0]),
    ("up", [0, 0, .01]), ("down", [0, 0, -.01]),
])
def test_discrete_moves_resolve_to_exactly_one_centimetre_with_local_joint_limits(direction, expected):
    kin, joint = AffineKinematics(), VisualSession().q
    before = kin.pose(joint, "tool")
    target, info = interpret(dict(action="move", direction=direction, reason="修正可见偏差"), joint, kin)
    after = kin.pose(target["joint"], "tool")
    np.testing.assert_allclose(np.asarray(after[:3]) - before[:3], expected, atol=1e-12)
    np.testing.assert_allclose(after[3:], before[3:], atol=1e-12)
    assert info["scale"] == 1 and info["direction"] == direction
    assert np.max(np.abs(np.asarray(target["joint"]) - joint)) <= .08
    assert np.all(np.asarray(target["joint"]) >= LOWER)
    assert np.all(np.asarray(target["joint"]) <= UPPER)
    assert joint == VisualSession().q


@pytest.mark.parametrize("sense", ["down", "cw", "ccw"])
def test_discrete_rotations_hold_tool_point_and_apply_bounded_base_rotation(sense):
    kin, joint = AffineKinematics(), VisualSession().q
    before = np.asarray(kin.pose(joint, "tool"))
    target, info = interpret(dict(action="rotate", sense=sense, reason="调整可见姿态"), joint, kin)
    after = np.asarray(kin.pose(target["joint"], "tool"))
    a, b = Rotation.from_quat(before[[4, 5, 6, 3]]), Rotation.from_quat(after[[4, 5, 6, 3]])
    np.testing.assert_allclose(after[:3], before[:3], atol=1e-12)
    assert (b * a.inv()).magnitude() == pytest.approx(.04)
    if sense == "down":
        assert b.as_matrix()[2, 2] < a.as_matrix()[2, 2]
    else:
        sign = 1 if sense == "ccw" else -1
        np.testing.assert_allclose((b * a.inv()).as_rotvec(), sign * .04 * a.as_matrix()[:, 2], atol=1e-12)
    assert info["sense"] == sense


def test_interpret_rejects_joint_limits_failed_ik_and_missing_kinematics(monkeypatch):
    action = dict(action="move", direction="forward", reason="向前调整")
    kin, joint = AffineKinematics(), VisualSession().q
    with pytest.raises(ValueError, match="kinematics"):
        interpret(action, joint, None)
    at_limit = list(joint)
    at_limit[0] = UPPER[0]
    with pytest.raises(ValueError, match="joint limits"):
        interpret(action, at_limit, kin)
    def failed(*args, **kwargs):
        raise RuntimeError("IK did not converge")
    monkeypatch.setattr(kin, "solve", failed)
    with pytest.raises(ValueError, match="did not converge"):
        interpret(action, joint, kin)


@pytest.mark.parametrize("action", [None, [], {}, {"action": 1, "reason": "x"},
    {"action": "wait"}, {"action": "wait", "reason": " "},
    {"action": "wait", "reason": "x" * 201}, {"action": "wait", "reason": {}},
    {"action": "wait", "reason": "x", "objects": {}},
    {"action": "move", "direction": "diagonal", "reason": "x"},
    {"action": "move", "direction": ["up"], "reason": "x"},
    {"action": "rotate", "sense": "yaw", "reason": "x"},
    {"action": "set_gripper", "gripper": True, "reason": "x"},
    {"action": "set_gripper", "gripper": .5, "reason": "x"},
    {"action": "set_gripper", "gripper": float("nan"), "reason": "x"},
    {"action": "point", "camera": "object", "pixel": [1, 2], "reason": "x"},
    {"action": "point", "camera": "head", "pixel": [1, 2], "second": {}, "reason": "x"},
    {"action": "point", "camera": "head", "pixel": [1, 2],
     "second": {"camera": "head", "pixel": [1, 2]}, "reason": "x"},
    {"action": "move_joint_absolute", "joint": [0] * 6, "reason": "x"},
    {"action": "shell", "command": "anything", "reason": "x"},
])
def test_layered_action_schema_rejects_invalid_or_legacy_inputs(action):
    with pytest.raises(ValueError):
        parse_action(action)


@pytest.mark.parametrize("name", ["wait", "done", "replan"])
def test_wait_and_done_are_nonmoving_actions(name):
    payload, info = interpret(dict(action=name, reason="检查当前图像"), [], None)
    assert payload == {} and info == {"visual_action": name}


def test_plan_validation_preserves_order_and_falls_back_without_inventing_a_path():
    expected = [stage(), stage("lift", "LIFT")]
    assert parse_plan(json.dumps({"stages": expected})) == expected
    invalid = [None, "not json", [], {}, {"stages": []},
               {"stages": [stage()] * 2}, {"stages": [stage(str(i)) for i in range(7)]},
               {"stages": [{**stage(), "motion": "PUSH"}]},
               {"stages": [{**stage(), "completion": " "}]},
               {"stages": [{k: v for k, v in stage().items() if k != "affordance"}]},
               {"stages": [stage()], "code": "SECRET_EXECUTABLE"}]
    for value in invalid:
        fallback = parse_plan(value, task="Lift the visible bowl")
        assert len(fallback) == 1
        assert fallback[0]["motion"] == "APPROACH"
        assert fallback[0]["target"] == fallback[0]["completion"] == "Lift the visible bowl"
        assert "SECRET" not in json.dumps(fallback)


@pytest.mark.parametrize("payload", [None, {}, {"complete": 1, "reason": "visible"},
    {"complete": True, "reason": " "}, {"complete": True, "reason": "x" * 201},
    {"complete": True, "reason": "visible", "action": "move"}])
def test_verifier_rejects_non_boolean_or_unbounded_claims(payload):
    with pytest.raises(ValueError):
        parse_verification(payload)


def test_verifier_parses_independent_image_evidence():
    assert parse_verification('{"complete":false,"reason":"  occluded  "}') == {
        "complete": False, "reason": "occluded", "source": "image_verification"}


@pytest.mark.parametrize("role", ["planner", "controller", "verifier"])
def test_role_payload_boundary_keeps_scene_truth_and_local_geometry_private(role):
    local = {}
    obs, _ = observation(VisualSession(), local=local)
    obs.update(objects="SECRET_OBJECT", contacts="SECRET_CONTACT", scene_priors="SECRET_PRIORS",
               depth_m="SECRET_DEPTH", intrinsics="SECRET_K", world_from_camera="SECRET_POSE",
               previous_rgb=obs["rgb"], eef_fk={"tool": [0, 0, .2, 1, 0, 0, 0], "hidden": "SECRET_FK"})
    history = [{"action": {"action": "wait"}, "status": "old"},
               {"action": {"action": "wait"}, "status": "waited", "hidden": "SECRET_HISTORY"}]
    messages = build_messages(obs, "TASK_ONLY_FOR_PLANNER", history, role=role, stage=stage())
    encoded = json.dumps(messages)
    assert "SECRET" not in encoded and "scene_priors" not in encoded
    assert set(local["head"]) == {"timestamp_ns", "rgb_shape", "rgb", "depth_m", "intrinsics", "world_from_camera"}
    content = messages[1]["content"]
    data = json.loads(content[0]["text"])
    assert sum(part["type"] == "image_url" for part in content) == (2 if role == "verifier" else 4)
    if role == "controller":
        assert data["task"] == "TASK_ONLY_FOR_PLANNER" and data["stage"] == stage()
        assert data["recent_actions"] == [{"action": {"action": "wait"}, "status": "old"},
                                          {"action": {"action": "wait"}, "status": "waited"}]
        assert data["robot"]["joint_limits_rad"] == {"lower": LOWER, "upper": UPPER}
    elif role == "verifier":
        assert data == {"stage": {"id": "grasp", "completion": stage()["completion"]}}
    else:
        assert data == {"task": "TASK_ONLY_FOR_PLANNER"}


def test_role_prompts_are_packaged_and_height_priors_are_explicitly_not_contact():
    root = Path(__file__).resolve().parents[4]
    config = tomllib.loads((root / "pyproject.toml").read_text())
    patterns = config["tool"]["setuptools"]["package-data"]["robot.harness"]
    assert "prompts/*.txt" in patterns
    for name in ("common", "controller", "planner", "verify", "legacy", "tools", "grasp", "grounded"):
        assert resources.files("robot.harness").joinpath("prompts", f"{name}.txt").read_text().strip() == load_prompt(name)
    controller = "\n".join(load_prompt(name) for name in ("controller", "tools", "grasp")).lower()
    for reference in ("orange", "0.13503", "rim-on-right"):
        assert reference not in controller
    assert "not live bowl pose/depth/contact feedback" in controller
    assert "nominal tool origin is not the lowest" in controller
    assert "current captured rgb image" in controller and "image_timestamps_ns" in controller
    assert "historical evidence" in controller and "not a live motion target" in controller
    assert "image_age_ms_at_query" in controller and "queried_at_ns" in controller


def test_layered_loop_advances_only_after_independent_verification(tmp_path, monkeypatch, capsys):
    example, session = load_example(), VisualSession()
    def no_recovery(self):
        pytest.fail("image verification cannot trigger executor recovery")
    monkeypatch.setattr(example.PositionExecutor, "recover", no_recovery)
    calls, checks = [], iter([False, True, True])
    plan = [stage(), stage("lift", "LIFT")]
    def planner(messages):
        calls.append(("planner", messages))
        return {"stages": plan}
    def controller(messages):
        calls.append(("controller", messages))
        return {"action": "done", "reason": "请求独立检查"}
    def verifier(messages):
        calls.append(("verifier", messages))
        data = json.loads(messages[1]["content"][0]["text"])
        assert set(data) == {"stage"}
        assert set(data["stage"]) == {"id", "completion"}
        return {"complete": next(checks), "reason": "当前图像中的可见证据"}
    output = tmp_path / "complete"
    example.run_loop(session, controller, threading.Event(), output, task="lift",
                     planner=planner, verifier=verifier, legacy_actions=False,
                     interval=.001, max_rounds=6)
    turns = rows(output / "turns.jsonl")
    assert [row["status"] for row in turns] == ["not_complete", "stage_complete", "stage_complete"]
    assert [row["stage"]["id"] for row in turns] == ["grasp", "grasp", "lift"]
    assert all(row["verification"]["source"] == "image_verification" for row in turns)
    stamps = [row["verification_image_timestamps_ns"] for row in turns]
    assert all(stamps[i][name] < stamps[i + 1][name] for i in range(2) for name in ("head", "wrist"))
    audit = rows(output / "model_calls.jsonl")
    assert [(row["role"], row["messages"]) for row in audit if row["event"] == "request"] == calls
    assert [role for role, _ in calls] == ["planner", "controller", "verifier", "controller", "verifier", "controller", "verifier"]
    assert json.loads((output / "latest_messages.json").read_text()) == calls[-1][1]
    notes = json.loads((output / "evidence_notes.json").read_text())
    assert [item["complete"] for item in notes["evidence"]] == [False, True, True]
    assert session.released and session.writes == []
    assert "任务已由独立图像验证完成" in capsys.readouterr().out


@pytest.mark.parametrize("planner_failure", ["exception", "invalid"])
def test_planner_failure_degrades_to_one_stage_and_records_exact_call_boundary(tmp_path, planner_failure):
    example, session, received = load_example(), VisualSession(), []
    def planner(messages):
        if planner_failure == "exception":
            raise RuntimeError("SECRET_TRANSPORT_DETAILS")
        return {"stages": [{"target": "missing stage contract"}]}
    def controller(messages):
        received.append(json.loads(messages[1]["content"][0]["text"]))
        return {"action": "wait", "reason": "等待可见证据"}
    output = tmp_path / "fallback"
    example.run_loop(session, controller, threading.Event(), output, task="visible goal",
                     planner=planner, legacy_actions=False, interval=.001, max_rounds=1)
    assert received[0]["stage"]["completion"] == "visible goal"
    assert received[0]["stage"]["motion"] == "APPROACH"
    audit = rows(output / "model_calls.jsonl")
    assert [row["event"] for row in audit] == ["request", "error" if planner_failure == "exception" else "response", "request", "response"]
    assert "SECRET" not in json.dumps(audit)
    assert session.released and session.writes == []


@pytest.mark.parametrize("failure", ["exception", "invalid"])
def test_verifier_failure_does_not_complete_or_recover_the_task(tmp_path, monkeypatch, capsys, failure):
    example, session = load_example(), VisualSession()
    def no_recovery(self):
        pytest.fail("verifier failure is not an execution failure")
    monkeypatch.setattr(example.PositionExecutor, "recover", no_recovery)
    def verifier(messages):
        if failure == "exception":
            raise RuntimeError("SECRET_PROVIDER_DETAILS")
        return {"complete": "true", "reason": "unsupported claim"}
    output = tmp_path / "unverified"
    example.run_loop(session, lambda _: {"action": "done", "reason": "请求检查"},
                     threading.Event(), output, task="lift", planner=lambda _: {"stages": [stage()]},
                     verifier=verifier, legacy_actions=False, interval=.001, max_rounds=2)
    turns = rows(output / "turns.jsonl")
    assert [row["status"] for row in turns] == ["not_complete", "not_complete"]
    assert all(not row["verification"]["complete"] for row in turns)
    assert session.released and session.writes == []
    assert "任务未确认完成" in capsys.readouterr().out
    assert "SECRET" not in (output / "model_calls.jsonl").read_text()


def test_controller_exception_logs_type_and_releases_control(tmp_path):
    example, session = load_example(), VisualSession()
    def controller(messages):
        raise RuntimeError("SECRET_NETWORK_DETAILS")
    output = tmp_path / "controller-error"
    with pytest.raises(RuntimeError, match="SECRET_NETWORK_DETAILS"):
        example.run_loop(session, controller, threading.Event(), output, task="lift",
                         planner=lambda _: {"stages": [stage()]}, legacy_actions=False, max_rounds=1)
    assert session.released and session.writes == []
    assert rows(output / "model_calls.jsonl")[-1] == {
        "round": 1, "role": "controller", "event": "error", "error_type": "RuntimeError"}
    assert "SECRET" not in (output / "model_calls.jsonl").read_text()
    assert json.loads((output / "evidence_notes.json").read_text())["evidence"] == []


def test_invalid_controller_reply_is_not_echoed_into_the_next_model_input(tmp_path):
    example, session, calls = load_example(), VisualSession(), []
    def controller(messages):
        calls.append(messages)
        assert "SECRET" not in json.dumps(messages)
        if len(calls) == 1:
            return {"action": "shell", "command": "SECRET_UNTRUSTED_COMMAND", "reason": "invalid"}
        recent = json.loads(messages[1]["content"][0]["text"])["recent_actions"]
        assert len(recent) == 1 and recent[-1]["status"] == "rejected"
        assert recent[-1]["action"] == "invalid"
        return {"action": "wait", "reason": "重新观察"}
    output = tmp_path / "invalid"
    example.run_loop(session, controller, threading.Event(), output, task="lift",
                     planner=lambda _: {"stages": [stage()]}, legacy_actions=False,
                     interval=.001, max_rounds=2)
    assert [row["status"] for row in rows(output / "turns.jsonl")] == ["rejected", "waited"]
    assert session.released and session.writes == []


def test_layered_execution_failure_recovers_then_blocks_identical_action(tmp_path, monkeypatch):
    example, session, calls = load_example(), VisualSession(), []
    base = example.PositionExecutor
    class FastExecutor(base):
        def execute(self, request_id, **kwargs):
            return super().execute(request_id, timeout_s=.08, dwell_s=.01, **kwargs)
    monkeypatch.setattr(example, "PositionExecutor", FastExecutor)
    session.stuck = True
    def controller(messages):
        data = json.loads(messages[1]["content"][0]["text"])
        calls.append(data)
        if len(calls) == 2:
            assert data["recent_actions"][-1]["recovery"]["status"] == "ready"
            assert data["last_failed_action"]["actual_feedback"]["gripper"] == 0
            assert data["blocked_action"] == {"action": "set_gripper", "gripper": 1.0}
            assert data["consecutive_failures"] == 1
        if len(calls) == 3:
            assert data["recent_actions"][-1]["status"] == "rejected"
            session.stuck = False
            return {"action": "set_gripper", "gripper": 0, "reason": "改用不同动作"}
        return {"action": "set_gripper", "gripper": 1.0, "reason": "尝试张开"}
    output = tmp_path / "execution-failure"
    example.run_loop(session, controller, threading.Event(), output, task="open",
                     planner=lambda _: {"stages": [stage()]}, legacy_actions=False,
                     interval=.001, max_rounds=3)
    turns = rows(output / "turns.jsonl")
    assert [row["status"] for row in turns] == ["failed", "rejected", "succeeded"]
    assert turns[0]["stop_confirmed"] and turns[0]["recovery"]["status"] == "ready"
    assert "same action failed" in turns[1]["reason"]
    assert len(session.writes) == 2 and session.released


def test_stalled_stage_replanning_is_bounded(tmp_path, capsys):
    example, session, plans = load_example(), VisualSession(), []
    def planner(messages):
        plans.append(messages)
        return {"stages": [stage(f"attempt-{len(plans)}")]}
    output = tmp_path / "replans"
    example.run_loop(session, lambda _: {"action": "wait", "reason": "证据不足"},
                     threading.Event(), output, task="lift", planner=planner,
                     legacy_actions=False, stage_rounds=1, max_replans=1,
                     interval=.001, max_rounds=8, max_waits=10, max_wait_replans=0)
    assert len(plans) == 2
    assert [row["stage"]["id"] for row in rows(output / "turns.jsonl")] == ["attempt-1", "attempt-2"]
    assert "已达到重新规划上限" in capsys.readouterr().out
    assert session.released and session.writes == []


def test_dry_run_skips_all_external_roles_replanning_and_motion(tmp_path, capsys):
    example, session = load_example(), VisualSession()
    original_joint, original_gripper = list(session.q), list(session.gripper)
    def forbidden(messages):
        pytest.fail("dry run must not call the supplied planner/controller/verifier")
    output = tmp_path / "dry"
    example.run_loop(session, forbidden, threading.Event(), output, task="lift",
                     planner=forbidden, verifier=forbidden, dry_run=True, legacy_actions=False,
                     stage_rounds=1, max_replans=0, interval=.001, max_rounds=3, max_waits=10)
    assert [row["status"] for row in rows(output / "turns.jsonl")] == ["waited"] * 3
    audit = rows(output / "model_calls.jsonl")
    assert {row["role"] for row in audit} == {"controller"}
    assert sum(row["event"] == "request" for row in audit) == 3
    assert all(row["raw"]["action"] == "wait" for row in audit if row["event"] == "response")
    assert not any(row["event"] == "requested" for row in rows(output / "control_000000.jsonl"))
    assert session.q == original_joint and session.gripper == original_gripper
    assert session.writes == [] and session.released
    assert json.loads((output / "evidence_notes.json").read_text())["evidence"] == []
    assert "任务未确认完成" in capsys.readouterr().out


def test_delayed_point_loop_exposes_historical_query_with_original_image_identity(tmp_path, monkeypatch):
    example, session, calls = load_example(), CalibratedSession(), []
    real_time_ns, offset = time.time_ns, [0]
    monkeypatch.setattr("robot.harness.policy.time.time_ns", lambda: real_time_ns() + offset[0])
    def no_motion(*args, **kwargs):
        pytest.fail("point queries must never submit motion")
    monkeypatch.setattr(example.PositionExecutor, "execute", no_motion)
    def controller(messages):
        calls.append(messages)
        assert "SECRET" not in json.dumps(messages)
        data = json.loads(messages[1]["content"][0]["text"])
        if len(calls) == 1:
            session.cameras["head"]["depth_m"][:] = 4
            session.cameras["head"]["intrinsics"][0, 0] = 100
            session.cameras["head"]["world_from_camera"][0] = 10
            offset[0] += 2_000_000_000  # Model latency, without waiting or calling a live model.
            return {"action": "point", "camera": "head", "pixel": [8, 6], "reason": "测量可见目标"}
        query = data["point_query"]
        np.testing.assert_allclose(query["point_base_m"], [1.5, .8, 2])
        assert query["depth_m"] == 2
        source = json.loads(calls[0][1]["content"][0]["text"])["image_timestamps_ns"]["head"]
        assert query["image_timestamps_ns"] == {"head": source}
        assert query["image_age_ms_at_query"]["head"] >= 2000
        assert query["queried_at_ns"] >= source + 2_000_000_000
        assert query["source"] == "captured_snapshot"
        assert query["evidence"] == "historical" and query["live_motion_target"] is False
        assert data["image_timestamps_ns"]["head"] > source
        labels = [item["text"] for item in messages[1]["content"][1:] if item["type"] == "text"]
        assert f"previous head RGB view; captured timestamp_ns={source}" in labels
        assert data["recent_actions"][-1]["status"] == "queried"
        assert "intrinsics" not in query and "world_from_camera" not in query
        return {"action": "wait", "reason": "依据测量重新观察"}
    output = tmp_path / "point-query"
    example.run_loop(session, controller, threading.Event(), output, task="point at bowl",
                     planner=lambda _: {"stages": [stage()]}, legacy_actions=False,
                     interval=.001, max_rounds=2)
    turns = rows(output / "turns.jsonl")
    assert [row["status"] for row in turns] == ["queried", "waited"]
    assert not any(row["event"] == "requested" for row in rows(output / "control_000000.jsonl"))
    assert session.released and session.writes == []


@pytest.mark.parametrize("bad_stamp", ["missing", "future", "malformed"])
def test_point_loop_rejects_invalid_snapshot_timestamp_without_motion(tmp_path, monkeypatch, bad_stamp):
    example, session = load_example(), CalibratedSession()
    acquire = example.observation
    def corrupted_local_snapshot(*args, **kwargs):
        obs, stamps = acquire(*args, **kwargs)
        if kwargs.get("local") is not None:
            value = kwargs["local"]["head"]
            if bad_stamp == "missing":
                del value["timestamp_ns"]
            else:
                value["timestamp_ns"] = time.time_ns() + 60_000_000_000 if bad_stamp == "future" else "invalid"
        return obs, stamps
    monkeypatch.setattr(example, "observation", corrupted_local_snapshot)
    def no_motion(*args, **kwargs):
        pytest.fail("invalid point timestamps must never submit motion")
    monkeypatch.setattr(example.PositionExecutor, "execute", no_motion)
    output = tmp_path / "invalid-point"
    example.run_loop(session, lambda _: {"action": "point", "camera": "head", "pixel": [8, 6], "reason": "查看目标"},
                     threading.Event(), output, task="point at bowl", planner=lambda _: {"stages": [stage()]},
                     legacy_actions=False, interval=.001, max_rounds=1)
    turns = rows(output / "turns.jsonl")
    assert len(turns) == 1 and turns[0]["status"] == "rejected"
    assert "timestamp_ns" in turns[0]["reason"]
    assert not any(row["event"] == "requested" for row in rows(output / "control_000000.jsonl"))
    assert session.released and session.writes == []


@pytest.mark.parametrize("bad_stamp", [None, 0, -1, True, "invalid", 20_000_000_001])
@pytest.mark.parametrize("invalid_camera", ["head", "wrist"])
def test_stereo_point_checks_both_timestamps_and_accepts_delayed_snapshots(monkeypatch, bad_stamp, invalid_camera):
    now = 20_000_000_000
    monkeypatch.setattr("robot.harness.policy.time.time_ns", lambda: now)
    cameras = stereo_cameras()
    stamps = {"head": now - 2_000_000_000, "wrist": now - 2_100_000_000}
    for name, value in cameras.items():
        value["timestamp_ns"] = stamps[name]
    cameras[invalid_camera]["timestamp_ns"] = bad_stamp
    action = dict(action="point", camera="head", pixel=[60, 55],
                  second={"camera": "wrist", "pixel": [10, 55]}, reason="用两幅图测量")
    with pytest.raises(ValueError, match="snapshot.*timestamp_ns"):
        interpret(action, [], None, cameras=cameras)
    cameras[invalid_camera]["timestamp_ns"] = stamps[invalid_camera]
    target, info = interpret(action, [], None, cameras=cameras)
    assert target is None and info["method"] == "two_view"
    assert info["image_timestamps_ns"] == stamps
    assert info["image_age_ms_at_query"] == {"head": 2000., "wrist": 2100.}
    assert info["evidence"] == "historical" and info["live_motion_target"] is False
    np.testing.assert_allclose(info["point_base_m"], [.2, .1, 2], atol=1e-12)


def test_notes_reuse_requires_explicit_matching_task_and_stays_out_of_verifier(tmp_path):
    example = load_example()
    evidence = [{"round": 1, "stage": "grasp", "complete": False, "reason": "目标被遮挡"}]
    path = tmp_path / "notes.json"
    path.write_text(json.dumps({"version": 1, "task": "lift", "evidence": evidence}), encoding="utf-8")
    assert example.load_notes(path, "lift") == evidence
    with pytest.raises(ValueError, match="exactly this task"):
        example.load_notes(path, "different task")
    obs, _ = observation(VisualSession())
    for role in ("planner", "controller", "verifier"):
        messages = build_messages(obs, "lift", [], role=role, stage=stage(), notes=evidence)
        data = json.loads(messages[1]["content"][0]["text"])
        assert data.get("prior_notes") == (None if role == "verifier" else evidence)
    assert "prior_notes" not in json.loads(build_messages(obs, "lift", [])[1]["content"][0]["text"])


@pytest.mark.parametrize("change", [
    {"version": True}, {"evidence": [{"round": 1, "code": "untrusted"}]},
    {"evidence": [{"round": 1, "stage": "s", "complete": 1, "reason": "visible"}]},
    {"evidence": [{"round": 1, "stage": "s", "complete": True, "reason": "x" * 201}]},
    {"evidence": [{"round": 1, "stage": "s", "complete": False, "reason": "visible"}] * 9},
    {"code": "untrusted instructions"},
])
def test_notes_loader_rejects_unbounded_or_non_evidence_records(tmp_path, change):
    path = tmp_path / "notes.json"
    value = {"version": 1, "task": "lift", "evidence": []}
    value.update(deepcopy(change))
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ValueError):
        load_example().load_notes(path, "lift")


def test_notes_loader_rejects_oversized_file(tmp_path):
    path = tmp_path / "notes.json"
    path.write_text(" " * 16_385, encoding="utf-8")
    with pytest.raises(ValueError, match="too large"):
        load_example().load_notes(path, "lift")
