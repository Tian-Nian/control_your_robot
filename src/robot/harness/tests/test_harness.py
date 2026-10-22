import json
import os
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from robot.harness.contract import HAND_DOF, components_from_action
from robot.harness.episode import Episode
from robot.harness.gate import CommandRejected, validate_joints
from robot.harness.harness import RobotHarness
from robot.harness.motion import JointMotion, path_samples
from robot.harness.observe import read_observation
from robot.runtime.discover import write_record
from robot.runtime.node import run_bundle, run_component
from robot.runtime.session import Session
from robot.runtime.transport import Bus, SlotSpec


def test_gate_rejects_limits_step_clearance_and_fk_error():
    with pytest.raises(CommandRejected, match="限位"):
        validate_joints([0.0], [1.2], lower=[-1.0], upper=[1.0], max_step_rad=2.0)
    with pytest.raises(CommandRejected, match="单步"):
        validate_joints([0.0], [0.4], lower=[-2.0], upper=[2.0], max_step_rad=0.2)
    with pytest.raises(CommandRejected, match="间隙"):
        validate_joints(
            [0.0],
            [0.1],
            lower=[-2.0],
            upper=[2.0],
            max_step_rad=0.2,
            min_clearance_m=0.04,
            clearance=lambda _sample: 0.01,
        )
    with pytest.raises(CommandRejected, match="FK"):
        validate_joints(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            lower=[-2.0] * 3,
            upper=[2.0] * 3,
            max_step_rad=0.2,
            eef=[0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            flange=lambda _sample: np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]),
            max_fk_error_m=0.008,
        )


def test_trajectory_samples_stay_inside_the_step():
    samples = path_samples([0.0, 0.0], [0.3, -0.1], 0.1)
    assert np.allclose(samples[-1], [0.3, -0.1])
    previous = np.zeros(2)
    for sample in samples:
        assert float(np.max(np.abs(sample - previous))) <= 0.1 + 1e-9
        previous = sample
    checked = validate_joints(
        [0.0],
        [0.3],
        lower=[-1.0],
        upper=[1.0],
        max_step_rad=0.2,
        trajectory=True,
        path_step_rad=0.1,
        min_clearance_m=0.04,
        clearance=lambda sample: 0.05 if abs(float(sample[0])) <= 0.3 else 0.0,
    )
    assert np.isclose(checked[0], 0.3)
    with pytest.raises(CommandRejected, match="间隙"):
        validate_joints(
            [0.0],
            [0.5],
            lower=[-1.0],
            upper=[1.0],
            max_step_rad=0.2,
            trajectory=True,
            path_step_rad=0.1,
            min_clearance_m=0.04,
            clearance=lambda sample: 0.0 if float(sample[0]) > 0.2 else 0.1,
        )


def test_joint_motion_reaches_the_goal_in_limited_steps():
    motion = JointMotion([0.0, 0.0], [0.3, -0.1], max_step_rad=0.1, tol_rad=0.02, timeout_s=1.0, seq=3)
    seen = []
    while not motion.command_reached():
        seen.append(motion.advance())
    assert len(seen) == 3
    assert np.allclose(seen[-1], [0.3, -0.1])
    assert motion.tracked([0.29, -0.09])
    assert not motion.tracked([0.0, 0.0])


def test_contract_maps_training_joint_pose_and_hand_order():
    commands = components_from_action(
        {
            "left_arm_joint": [0.1, 0.2, 0.3],
            "right_ee_pose": [0.4, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0],
            "left_ee_joint_state": np.arange(HAND_DOF),
        }
    )
    assert commands["left_arm"]["joint"] == [0.1, 0.2, 0.3]
    assert commands["right_arm"]["eef"][0] == 0.4
    assert commands["left_hand"]["joint"] == list(range(HAND_DOF))
    with pytest.raises(ValueError, match="20"):
        components_from_action({"right_hand_joint": [0.0, 1.0]})


def test_observation_reports_skew_and_episode_keeps_image_shape(tmp_path):
    now = time.time_ns()
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    image[0, 0, 0] = 7

    class Session:
        def describe(self):
            return [{"name": "left_arm"}, {"name": "cam_head"}]

        def read_raw(self, name):
            if name == "left_arm":
                return {"seq": 1, "timestamp_ns": now, "data": {"joint": np.array([0.2, 0.0])}}
            return {"seq": 2, "timestamp_ns": now - 30_000_000, "data": {"color": image}}

    observation = read_observation(Session(), max_skew_ms=80.0)
    assert observation["within_skew"] is True
    assert observation["skew_ms"] == pytest.approx(30.0)
    assert observation["components"]["cam_head"]["data"]["color"][0, 0, 0] == 7

    episode = Episode()
    episode.start(tmp_path / "episode.json")
    episode.record(observation, {"left_arm": {"joint": [0.2, 0.0]}})
    path = episode.finish(True, "放到了桌上")
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert saved["success"] is True
    assert saved["reason"] == "放到了桌上"
    color = saved["steps"][0]["state"]["components"]["cam_head"]["color"]
    assert color["shape"] == [2, 2, 3]
    assert 7 not in color["shape"]


class _JointArm:
    def __init__(self):
        self.joint = np.zeros(3)
        self.calls = []
        self.mode = "impedance"

    def set_up(self):
        return None

    def set_collect_info(self, _info):
        return None

    def get_state(self):
        return {"joint": self.joint.copy()}

    def get(self):
        return {"joint": self.joint.copy(), "timestamp": time.time_ns()}

    def move(self, data, is_delta=False):
        if "control_mode" in data and "joint" not in data:
            self.mode = data["control_mode"]
            return
        self.calls.append(list(data["joint"]))
        self.joint = np.asarray(data["joint"], dtype=float)

    def reset(self, start_state=None):
        goal = np.zeros(3) if start_state is None else np.asarray(start_state, dtype=float)
        self.move({"joint": goal})


def _run(shm, spec, arm):
    stop = threading.Event()
    thread = threading.Thread(
        target=run_component,
        args=(shm, spec, stop),
        kwargs={"factory": lambda: arm},
        daemon=True,
    )
    thread.start()
    return stop, thread


def test_one_command_per_tick_and_bundle_reports_hz():
    shm = f"cyr_tick_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=8)
    arm = _JointArm()
    arm.move_times = []
    arm.get_times = []
    original_move = arm.move
    original_get = arm.get

    def move(data, is_delta=False):
        arm.move_times.append(time.monotonic())
        original_move(data, is_delta=is_delta)

    def get():
        arm.get_times.append(time.monotonic())
        return original_get()

    arm.move = move
    arm.get = get
    spec = {"name": "arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}
    for goal in (1.0, 2.0, 3.0):
        bus.write_cmd("arm", {"joint": [goal, 0.0, 0.0]})
    stop, thread = _run(shm, spec, arm)
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not (
            arm.move_times and arm.get_times and arm.get_times[0] > arm.move_times[0]
        ):
            time.sleep(0.005)
        moves_before_get = sum(1 for stamp in arm.move_times if stamp < arm.get_times[0])
        assert moves_before_get == 1
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and arm.joint[0] != 3.0:
            time.sleep(0.01)
        assert arm.joint[0] == 3.0
    finally:
        stop.set()
        thread.join(timeout=2)
        bus.unlink()

    shm = f"cyr_hz_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=4)
    bundle_spec = {
        "name": "arm",
        "class_path": "robot.controller.TestArm_controller",
        "class_name": "TestArmController",
        "args": {"name": "arm", "DoFs": 3, "INFO": "ERROR"},
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
    }
    stop = threading.Event()
    thread = threading.Thread(target=run_bundle, args=(shm, [bundle_spec], stop), daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 2
        status = None
        while time.monotonic() < deadline:
            status = bus.read_status("arm")
            if status and status["hz"] > 10:
                break
            time.sleep(0.05)
        assert status is not None and status["hz"] > 10
    finally:
        stop.set()
        thread.join(timeout=2)
        bus.unlink()


def test_gate_rejection_holds_component_without_latching_service(tmp_path):
    shm = f"cyr_gate_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=4)
    config = tmp_path / "gate.yml"
    config.write_text(
        f"""
name: gate
shm_name: {shm}
cmd_capacity: 4
components:
  - name: arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
""",
        encoding="utf-8",
    )
    write_record({"pid": os.getpid(), "config_path": str(config), "shm_name": shm, "components": ["arm"], "name": "gate"})
    arm = _JointArm()
    spec = {
        "name": "arm",
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
        "gate": {"lower": [-1.0, -1.0, -1.0], "upper": [1.0, 1.0, 1.0], "max_step_rad": 0.2},
    }
    stop, thread = _run(shm, spec, arm)
    session = Session.attach()
    try:
        rejected = session.call("arm", {"joint": [0.5, 0.0, 0.0]})
        assert rejected["accepted"] is False
        assert "单步" in rejected["error"]
        assert bus.read_safety()["latched"] is False
        assert np.allclose(arm.joint, 0)
        held = session.call("arm", {"joint": [0.1, 0.0, 0.0]})
        assert held["accepted"] is False
        assert "保持" in held["error"]
        released = session.call("arm", {"op": "release"})
        assert released["accepted"] is True
        moved = session.call("arm", {"joint": [0.1, 0.0, 0.0]})
        assert moved["accepted"] is True
        assert np.allclose(arm.joint, [0.1, 0.0, 0.0])
    finally:
        session.close()
        stop.set()
        thread.join(timeout=2)
        bus.unlink()


def test_motion_interpolates_and_tracking_timeout_holds():
    shm = f"cyr_motion_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=4)
    arm = _JointArm()
    spec = {
        "name": "arm",
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
        "motion": {"enabled": True, "max_step_rad": 0.1, "tol_rad": 0.02, "timeout_s": 2},
    }
    stop, thread = _run(shm, spec, arm)
    try:
        bus.write_cmd("arm", {"joint": [0.25, 0.0, 0.0]})
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not np.allclose(arm.joint, [0.25, 0.0, 0.0]):
            time.sleep(0.01)
        assert [round(item[0], 2) for item in arm.calls] == [0.1, 0.2, 0.25]
    finally:
        stop.set()
        thread.join(timeout=2)
        bus.unlink()

    shm = f"cyr_timeout_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=4)

    class Stuck(_JointArm):
        def move(self, data, is_delta=False):
            self.calls.append(list(data["joint"]))

    stuck = Stuck()
    spec = {
        "name": "arm",
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
        "motion": {"enabled": True, "max_step_rad": 0.1, "tol_rad": 0.02, "timeout_s": 0.2},
    }
    stop, thread = _run(shm, spec, stuck)
    try:
        bus.write_cmd("arm", {"joint": [0.5, 0.0, 0.0]})
        deadline = time.monotonic() + 2
        status = None
        while time.monotonic() < deadline:
            status = bus.read_status("arm")
            if status and status.get("last_cmd_error") == "跟踪超时":
                break
            time.sleep(0.02)
        assert status is not None
        assert status["last_cmd_error"] == "跟踪超时"
        assert bus.read_safety()["latched"] is False
        assert np.allclose(stuck.joint, 0)
        bus.write_cmd("arm", {"joint": [0.0, 0.0, 0.0]})
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            status = bus.read_status("arm")
            if status and "保持" in str(status.get("last_cmd_error")):
                break
            time.sleep(0.02)
        assert "保持" in status["last_cmd_error"]
    finally:
        stop.set()
        thread.join(timeout=2)
        bus.unlink()


def test_rejected_command_keeps_mode_and_service(tmp_path):
    shm = f"cyr_mode_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("arm", 65536)], cmd_capacity=4)
    config = tmp_path / "mode.yml"
    config.write_text(
        f"""
name: mode
shm_name: {shm}
cmd_capacity: 4
components:
  - name: arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
""",
        encoding="utf-8",
    )
    write_record({"pid": os.getpid(), "name": "mode", "config_path": str(config), "shm_name": shm, "components": ["arm"]})
    arm = _JointArm()
    spec = {
        "name": "arm",
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
        "gate": {"lower": [-1.0, -1.0, -1.0], "upper": [1.0, 1.0, 1.0], "max_step_rad": 0.2},
    }
    stop, thread = _run(shm, spec, arm)
    session = Session.attach()
    try:
        rejected = session.call("arm", {"control_mode": "position", "joint": [0.5, 0.0, 0.0]})
        assert rejected["accepted"] is False
        assert "单步" in rejected["error"]
        assert arm.mode == "impedance"
        assert np.allclose(arm.joint, 0)
        assert bus.read_safety()["latched"] is False
        assert session.call("arm", {"op": "release"})["accepted"] is True
        bad_mode = session.call("arm", {"control_mode": "swing", "joint": [0.1, 0.0, 0.0]})
        assert bad_mode["accepted"] is False
        assert "control_mode" in bad_mode["error"]
        assert arm.mode == "impedance"
        assert np.allclose(arm.joint, 0)
        assert bus.read_safety()["latched"] is False
        assert session.call("arm", {"op": "release"})["accepted"] is True
        bad_delta = session.call("arm", {"joint": ["a", "b", "c"]}, is_delta=True)
        assert bad_delta["accepted"] is False
        assert "数字" in bad_delta["error"]
        assert bus.read_safety()["latched"] is False
        assert session.call("arm", {"op": "release"})["accepted"] is True
        moved = session.call("arm", {"control_mode": "position", "joint": [0.1, 0.0, 0.0]})
        assert moved["accepted"] is True
        assert arm.mode == "position"
        assert np.allclose(arm.joint, [0.1, 0.0, 0.0])
    finally:
        session.close()
        stop.set()
        thread.join(timeout=2)
        bus.unlink()


def test_harness_step_records_action(tmp_path):
    shm = f"cyr_ep_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("left_arm", 65536), SlotSpec("left_hand", 65536)], cmd_capacity=4)
    config = tmp_path / "episode.yml"
    config.write_text(
        f"""
name: episode
shm_name: {shm}
cmd_capacity: 4
components:
  - name: left_arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
  - name: left_hand
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
""",
        encoding="utf-8",
    )
    write_record(
        {
            "pid": os.getpid(),
            "name": "episode",
            "config_path": str(config),
            "shm_name": shm,
            "components": ["left_arm", "left_hand"],
        }
    )
    arm = _JointArm()
    hand = _JointArm()
    hand.joint = np.zeros(HAND_DOF)

    def hand_move(data, is_delta=False):
        hand.calls.append(list(data["joint"]))
        hand.joint = np.asarray(data["joint"], dtype=float)

    hand.move = hand_move
    stop_arm, arm_thread = _run(shm, {"name": "left_arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}, arm)
    stop_hand, hand_thread = _run(
        shm, {"name": "left_hand", "publish": ["joint"], "rate_hz": 50, "setup": {}}, hand
    )
    session = Session.attach("episode")
    harness = RobotHarness(session, mode="joint", control_hz=15)
    try:
        deadline = time.monotonic() + 2
        ready = None
        while time.monotonic() < deadline:
            ready = harness.get_obs()
            if ready["within_skew"]:
                break
            time.sleep(0.02)
        assert ready is not None and ready["within_skew"] is True
        harness.start_episode(tmp_path / "ep.json")
        outcome = harness.step(
            {"left_arm_joint": [0.2, 0.0, 0.0], "left_ee_joint_state": np.zeros(HAND_DOF)}
        )
        assert outcome["result"]["left_arm"]["accepted"] is True
        assert outcome["result"]["left_hand"]["accepted"] is True
        assert outcome["observation"]["within_skew"] is True
        path = harness.finish_episode(False, "没抓住")
        saved = json.loads(path.read_text(encoding="utf-8"))
        assert saved["success"] is False
        assert saved["steps"][0]["action"]["left_arm_joint"] == [0.2, 0.0, 0.0]
    finally:
        session.close()
        stop_arm.set()
        stop_hand.set()
        arm_thread.join(timeout=2)
        hand_thread.join(timeout=2)
        bus.unlink()


def test_attach_requires_a_name_when_two_services_are_live(tmp_path):
    first = tmp_path / "a.yml"
    second = tmp_path / "b.yml"
    for path, shm, service_name in (
        (first, "cyr_a", "alpha"),
        (second, "cyr_b", "beta"),
    ):
        path.write_text(
            f"""
name: {service_name}
shm_name: {shm}
cmd_capacity: 2
components:
  - name: left_arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 10
    max_bytes: 4096
""",
            encoding="utf-8",
        )
    write_record({"pid": os.getpid(), "name": "alpha", "config_path": str(first), "shm_name": "cyr_a", "components": []})
    write_record({"pid": os.getpid(), "name": "beta", "config_path": str(second), "shm_name": "cyr_b", "components": []})
    with pytest.raises(RuntimeError, match="多台服务"):
        Session.attach()
    with pytest.raises(RuntimeError, match="没有服务"):
        Session.attach("missing")


def test_marvin_body_home_palm_clearance_and_ik():
    pinocchio = pytest.importorskip("pinocchio")
    del pinocchio
    path = Path(__file__).resolve().parents[4] / "config" / "bodies" / "marvin.yml"
    if not path.is_file():
        pytest.skip("没有天机身体描述")
    from robot.harness.body import BodyModel

    body = BodyModel.load(path)
    if not Path(body.urdf_path).is_file():
        pytest.skip("没有天机 URDF")
    home = body.home("right")
    flange = body.flange_pose("right", home)
    palm = body.palm_pose("right", home)
    assert flange.shape == (7,)
    assert np.allclose(flange[:3], [0.2813, -0.1744, -0.2776], atol=0.01)
    offset = body.flange_to_palm("right")
    assert np.linalg.norm(offset) == pytest.approx(np.linalg.norm(palm[:3] - flange[:3]), abs=1e-6)
    assert float(np.linalg.norm(offset)) > 0.1
    assert body.clearance("right", home) > 0.04
    assert body.clearance("left", body.home("left")) > 0.04
    target = flange.copy()
    target[0] += 0.02
    solved = body.ik("right", home, target)
    reached = body.flange_pose("right", solved)
    assert float(np.linalg.norm(reached[:3] - target[:3])) < 0.008
    assert body.clearance("right", solved) > 0.04
    low, high = body.limits("right")
    assert np.all(home > low - 1e-6) and np.all(home < high + 1e-6)
    with pytest.raises(CommandRejected, match="xyz"):
        body.ik("right", home, [0.1, 0.2])


def test_bad_eef_and_stand_collision_hold_without_latch(tmp_path):
    pinocchio = pytest.importorskip("pinocchio")
    del pinocchio
    path = Path(__file__).resolve().parents[4] / "config" / "bodies" / "marvin.yml"
    if not path.is_file():
        pytest.skip("没有天机身体描述")
    from robot.harness.body import BodyModel

    body = BodyModel.load(path)
    if not Path(body.urdf_path).is_file():
        pytest.skip("没有天机 URDF")
    home = body.home("right")
    flange = body.flange_pose("right", home)
    low, _high = body.limits("right")
    colliding = home.copy()
    colliding[1] = low[1]

    class Arm:
        def __init__(self):
            self.joint = home.copy()
            self.moves = 0

        def set_up(self):
            return None

        def set_collect_info(self, _info):
            return None

        def get_state(self):
            return {"joint": self.joint.copy()}

        def get(self):
            return {"joint": self.joint.copy(), "timestamp": time.time_ns()}

        def move(self, data, is_delta=False):
            self.moves += 1
            self.joint = np.asarray(data["joint"], dtype=float)

    shm = f"cyr_body_{time.time_ns()}"
    bus = Bus.create(shm, [SlotSpec("right_arm", 65536)], cmd_capacity=4)
    config = tmp_path / "body.yml"
    config.write_text(
        f"""
name: body
shm_name: {shm}
cmd_capacity: 4
components:
  - name: right_arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
""",
        encoding="utf-8",
    )
    write_record(
        {"pid": os.getpid(), "name": "body", "config_path": str(config), "shm_name": shm, "components": ["right_arm"]}
    )
    arm = Arm()
    spec = {
        "name": "right_arm",
        "publish": ["joint"],
        "rate_hz": 50,
        "setup": {},
        "body": str(path),
        "gate": {"arm": "right", "max_step_rad": 0.2, "max_fk_error_m": 0.008, "min_clearance_m": 0.04},
        "motion": {"enabled": True, "max_step_rad": 0.05, "tol_rad": 0.02, "timeout_s": 4},
    }
    stop, thread = _run(shm, spec, arm)
    session = Session.attach()
    moved = home.copy()
    try:
        short = session.call("right_arm", {"eef": [0.1, 0.2]}, timeout=3)
        assert short["accepted"] is False
        assert "xyz" in short["error"]
        assert bus.read_safety()["latched"] is False
        assert arm.moves == 0
        assert np.allclose(arm.joint, home)
        assert session.call("right_arm", {"op": "release"})["accepted"] is True
        blocked = session.call("right_arm", {"joint": colliding.tolist()}, timeout=5)
        assert blocked["accepted"] is False
        assert "间隙" in blocked["error"]
        assert bus.read_safety()["latched"] is False
        assert arm.moves == 0
        assert np.allclose(arm.joint, home)
        assert session.call("right_arm", {"op": "release"})["accepted"] is True
        shifted = session.call("right_arm", {"eef": [0.02, 0.0, 0.0]}, is_delta=True, timeout=5)
        assert shifted["accepted"] is True
        assert bus.read_safety()["latched"] is False
        assert arm.moves > 0
        moved = arm.joint.copy()
    finally:
        session.close()
        stop.set()
        thread.join(timeout=2)
        bus.unlink()
    from robot.runtime.frames import compose_delta

    reached = body.flange_pose("right", moved)
    expected = compose_delta(flange, [0.02, 0.0, 0.0])
    assert np.allclose(reached[:3], expected[:3], atol=0.008)
