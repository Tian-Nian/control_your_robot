"""伺服模式、PiperSim 周期积分，以及工具包检查。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from robot.controller.Y1_controller import Y1Controller
from robot.controller.arm_controller import normalize_arm_mode
from robot.harness.gate import CommandRejected
from robot.runtime.config import load_config
from robot.runtime.node import _apply_command, _apply_mode, _servo

ROOT = Path(__file__).resolve().parents[4]


def _load(alias: str, path: Path):
    spec = importlib.util.spec_from_file_location(alias, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[alias] = module
    spec.loader.exec_module(module)
    return module


def test_standard_mode_aliases_and_runtime_whitelist():
    assert normalize_arm_mode("gravity") == "gravity_compensation"
    assert normalize_arm_mode("mit_control") == "mit"
    assert normalize_arm_mode("pos") == "position"

    class Arm:
        def __init__(self):
            self.moves = []

        def move(self, data, is_delta=False):
            self.moves.append(data)

    arm = Arm()
    _apply_mode(arm, "mit")
    _apply_mode(arm, "gravity_compensation")
    assert arm.moves == [{"control_mode": "mit"}, {"control_mode": "gravity_compensation"}]
    with pytest.raises(CommandRejected, match="gravity_compensation"):
        _apply_mode(arm, "swing")


def test_servo_step_is_optional():
    class Device:
        def __init__(self):
            self.dts = []

        def servo_step(self, dt):
            self.dts.append(dt)

    device = Device()
    _servo(device, 0.02)
    assert device.dts == [0.02]
    _servo(object(), 0.02)


def test_declared_modes_match_each_sdk():
    from robot.controller.Marvin_arm_controller import MarvinArmController
    from robot.controller.PiperSim_controller import PiperSimController

    assert PiperSimController.supported_modes == ("position", "mit", "gravity_compensation", "impedance")
    assert MarvinArmController.supported_modes == ("position", "impedance", "cartesian", "force")
    arm = Y1Controller("y1")
    assert arm.supported_modes == ("position", "mit", "gravity_compensation", "impedance")
    with pytest.raises(ValueError, match="URDF"):
        arm.set_control_mode("gravity")
    with pytest.raises(ValueError, match="URDF"):
        arm.set_control_mode("impedance")


def test_example_yamls_select_the_startup_mode():
    plain = load_config(ROOT / "config" / "runtime" / "piper_sim.yml")
    gravity = load_config(ROOT / "config" / "runtime" / "piper_sim_gravity.yml")
    impedance = load_config(ROOT / "config" / "runtime" / "piper_sim_impedance.yml")
    assert plain.components[0].setup == {}
    assert gravity.components[0].setup["control_mode"] == "gravity_compensation"
    assert "control_mode" in gravity.components[0].publish
    assert impedance.components[0].setup["control_mode"] == "impedance"
    assert impedance.components[0].setup["joint_K"][0] == 16
    assert impedance.components[0].setup["joint_D"][0] == 1.8


def test_piper_sim_gravity_holds_and_impedance_tracks():
    pytest.importorskip("pinocchio")
    pytest.importorskip("rerun")
    from robot.controller.PiperSim_controller import PiperSimController

    arm = PiperSimController("left", spawn=False)
    arm.set_up(control_mode="gravity_compensation")
    arm.set_collect_info(["joint", "control_mode", "servo_fault"])
    start = arm.get()["joint"].copy()
    for _ in range(25):
        arm.servo_step(0.02)
    held = arm.get()
    assert held["control_mode"] == "gravity_compensation"
    assert held["servo_fault"] == ""
    assert np.allclose(held["joint"], start, atol=0.02)

    arm.set_control_mode("impedance")
    parked = arm.get()["joint"].copy()
    goal = parked.copy()
    goal[1] += 0.1
    arm.set_joint(goal)
    assert np.allclose(arm.get()["joint"], parked, atol=1e-8)
    for _ in range(40):
        arm.servo_step(0.02)
    moved = arm.get()
    assert moved["servo_fault"] == ""
    assert float(moved["joint"][1]) > float(parked[1]) + 0.01
    assert float(moved["joint"][1]) < float(goal[1]) + 0.02

    with pytest.raises(ValueError, match="cartesian"):
        arm.set_control_mode("cartesian")

    loaded = PiperSimController("loaded", spawn=False)
    loaded.set_up(control_mode="position", plant_mass=0.35, plant_com=[0.02, -0.01, 0.06])
    with pytest.raises(ValueError, match="末端负载"):
        loaded.set_control_mode("impedance")
    loaded.set_identified_payload(0.35, [0.02, -0.01, 0.06])
    loaded.set_control_mode("impedance")
    assert loaded.control_mode == "impedance"


def test_piper_sim_position_teleports_and_mit_integrates():
    pytest.importorskip("pinocchio")
    pytest.importorskip("rerun")
    from robot.controller.PiperSim_controller import PiperSimController

    arm = PiperSimController("left", spawn=False)
    arm.set_up()
    start = arm.get_state()["joint"].copy()
    target = start.copy()
    target[0] = 0.2
    arm.set_joint(target)
    assert np.allclose(arm.get_state()["joint"], target)
    arm.servo_step(0.02)
    assert np.allclose(arm.get_state()["joint"], target)

    current = arm.get_state()["joint"].copy()
    arm.set_mit(current, np.zeros(6), np.zeros(6))
    for _ in range(10):
        arm.servo_step(0.02)
    assert arm.control_mode == "mit"
    assert np.allclose(arm.get_state()["joint"], current, atol=0.03)
    offset = current.copy()
    offset[1] += 0.1
    arm.handle_runtime_op({"op": "mit", "q_des": offset, "kp": [16] * 6, "kd": [1.8] * 6})
    for _ in range(40):
        arm.servo_step(0.02)
    assert float(arm.get_state()["joint"][1]) > float(current[1]) + 0.01


def test_gravity_and_impedance_checks_ask_until_the_mode_is_up():
    gravity = _load("gravity_check", ROOT / "toolkits" / "gravity_compensation" / "check.py")
    impedance = _load("impedance_check", ROOT / "toolkits" / "impedance" / "check.py")
    missing = gravity.evaluate([], {}, service_up=False)
    assert missing["ok"] is False
    assert "piper_sim_gravity.yml" in missing["questions"][0]

    component = {
        "name": "left_arm",
        "class_path": "robot.controller.PiperSim_controller",
        "class_name": "PiperSimController",
        "publish": ["joint", "eef", "gripper", "result", "control_mode", "servo_fault"],
        "setup": {"control_mode": "gravity_compensation"},
    }
    ready = gravity.evaluate(
        [component],
        {"left_arm": {"control_mode": "gravity_compensation"}},
        tool_in_urdf=True,
    )
    assert ready["ok"] is True
    assert ready["arm"] == "left_arm"
    needs_samples = gravity.evaluate([component], {"left_arm": {"control_mode": "position"}})
    assert needs_samples["ok"] is False
    assert "cyr" in " ".join(needs_samples["ask"])
    waiting = gravity.evaluate(
        [component],
        {"left_arm": {"control_mode": "position"}},
        tool_in_urdf=True,
    )
    assert waiting["ok"] is False
    holding = gravity.evaluate(
        [component],
        {"left_arm": {"control_mode": "gravity_compensation"}},
        verify_hold=True,
    )
    assert holding["ok"] is True

    imp = dict(component)
    imp["setup"] = {"control_mode": "impedance"}
    needs_gains = impedance.evaluate([imp], {"left_arm": {"control_mode": "impedance"}})
    assert needs_gains["ok"] is False
    assert "joint_K" in needs_gains["questions"][0]
    imp["setup"] = {"control_mode": "impedance", "joint_K": [16] * 6, "joint_D": [1.8] * 6}
    tracking = impedance.evaluate([imp], {"left_arm": {"control_mode": "impedance"}})
    assert tracking["ok"] is True


def test_unmodeled_payload_is_fit_from_held_poses():
    pytest.importorskip("pinocchio")
    pytest.importorskip("rerun")
    sys.path.insert(0, str(ROOT / "toolkits" / "gravity_compensation"))
    from collect import demo

    report = demo()
    assert abs(report["mass_kg"] - report["plant_mass"]) < 0.05
    assert np.allclose(report["com_m"], report["plant_com"], atol=0.02)
    assert report["hold_delta"] < 0.05
    assert report["unidentified_fault"]


def test_payload_ops_reach_the_arm():
    class Arm:
        def __init__(self):
            self.ops = []

        def handle_runtime_op(self, data):
            self.ops.append(dict(data))

    arm = Arm()
    spec = {"name": "left_arm"}
    _apply_command(spec, arm, {"seq": 1, "is_delta": False, "data": {"op": "hold_torque"}})
    _apply_command(
        spec,
        arm,
        {"seq": 2, "is_delta": False, "data": {"op": "set_payload", "mass": 0.2, "com": [0, 0, 0.05]}},
    )
    assert [item["op"] for item in arm.ops] == ["hold_torque", "set_payload"]


def test_mit_servo_refreshes_gravity_and_impedance_without_integrating():
    pytest.importorskip("pinocchio")
    pytest.importorskip("rerun")
    from robot.controller.PiperSim_controller import PiperSimController
    from robot.controller.arm_servo import ArmServoLaw, MitServo

    ready = np.array([0.0, 1.3, -1.0, 0.0, 0.5, 0.0])
    arm = PiperSimController("left", spawn=False)
    arm.set_up(control_mode="gravity_compensation")
    sim_gravity = arm.gravity_compensation()

    class Drive(MitServo):
        def __init__(self):
            self.name = "drive"
            self.control_mode = "gravity_compensation"
            self.joint = ready.copy()
            self.sent = []

        def get_state(self):
            return {"joint": self.joint.copy(), "force": sim_gravity.copy()}

        def _send_mit_refresh(self, terms):
            self.sent.append({key: np.asarray(value).copy() for key, value in terms.items()})

    law = ArmServoLaw(arm.urdf_path, apply_scale=False)
    drive = Drive()
    drive.bind_mit_servo(law)
    drive.control_mode = "impedance"
    with pytest.raises(ValueError, match="末端负载"):
        drive.servo_step(0.02)
    drive.control_mode = "gravity_compensation"
    drive.sent.clear()
    drive.servo_step(0.02)
    assert drive.control_mode == "gravity_compensation"
    assert np.allclose(drive.sent[0]["kp"], 0.0)
    assert np.allclose(drive.sent[0]["tau_ff"], sim_gravity, atol=1e-8)
    assert np.allclose(drive.joint, ready)

    scaled = ArmServoLaw(arm.urdf_path, torque_scale=[1, 1, 1, -1, 1, 1], apply_scale=True)
    scaled.set_payload(0.2, [0.01, 0.0, 0.04])
    law.set_payload(0.2, [0.01, 0.0, 0.04])
    raw = law.compensation(ready)
    flipped = scaled.compensation(ready)
    assert np.allclose(flipped[:3], raw[:3])
    assert np.isclose(flipped[3], -raw[3])

    drive.control_mode = "impedance"
    with pytest.raises(ValueError, match="joint_K"):
        drive.servo_step(0.02)
    law.set_gains([4, 4, 4, 2, 2, 2], [0.3] * 6)
    drive.note_joint_target(ready + np.array([0, 0.1, 0, 0, 0, 0]))
    drive.servo_step(0.02)
    assert drive.control_mode == "impedance"
    assert np.allclose(drive.sent[-1]["kp"], [4, 4, 4, 2, 2, 2])
    assert np.allclose(drive.sent[-1]["kd"], 0.3)
    assert np.allclose(drive.sent[-1]["tau_ff"], raw, atol=1e-8)
    assert np.allclose(drive.sent[-1]["q_des"][1], ready[1] + 0.1)

    sample = scaled.hold(ready, flipped)
    assert np.allclose(sample["torque"], raw, atol=1e-8)
    assert np.allclose(sample["link_gravity"], law.link_gravity(ready), atol=1e-8)


def test_collect_parses_a_cyr_joint_target():
    collect = _load("gravity_collect", ROOT / "toolkits" / "gravity_compensation" / "collect.py")
    assert collect._parse_joint("") is None
    assert collect._parse_joint("[0, 1.3, -1, 0, 0.5, 0]")[1] == 1.3
    assert collect._parse_joint("0 1.3 -1 0 0.5 0")[2] == -1.0


def test_handeye_check_asks_when_the_service_has_no_camera():
    handeye = _load("handeye_check", ROOT / "toolkits" / "handeye" / "check.py")
    config = handeye.load_config(ROOT / "toolkits" / "handeye" / "configs" / "default.yml")
    report = handeye.evaluate(
        [{"name": "left_arm", "publish": ["joint", "eef"]}],
        {"left_arm": {"eef": [0, 0, 0, 1, 0, 0, 0]}},
        config,
        opencv_ok=True,
    )
    text = " ".join(report["questions"])
    assert report["ok"] is False
    assert "相机" in text
    assert "眼" in text
    assert "Charuco" in text

    config = dict(config)
    config["arm"] = "left_arm"
    config["camera"] = "cam_head"
    config["setup"] = "eye_in_hand"
    config["camera_matrix"] = np.eye(3).tolist()
    ready = handeye.evaluate(
        [
            {"name": "left_arm", "publish": ["eef"]},
            {"name": "cam_head", "publish": ["color"]},
        ],
        {"left_arm": {"eef": [0, 0, 0, 1, 0, 0, 0]}},
        config,
        opencv_ok=True,
        confirm_board=True,
    )
    assert ready["ok"] is True


def test_handeye_solver_recovers_a_fixed_transform():
    cv2 = pytest.importorskip("cv2")
    solver = _load("handeye_solve", ROOT / "toolkits" / "handeye" / "solve.py")
    rng = np.random.default_rng(3)
    camera_to_gripper = _pose(rng)
    base_to_target = _pose(rng)
    grippers = []
    targets = []
    for _ in range(10):
        gripper_to_base = _pose(rng)
        target_to_camera = _invert(camera_to_gripper) @ _invert(gripper_to_base) @ base_to_target
        grippers.append(gripper_to_base)
        targets.append(target_to_camera)
    found = solver.solve_handeye(grippers, targets, "eye_in_hand", cv2)
    assert found["meaning"] == "camera_to_gripper"
    assert np.allclose(found["transform"], camera_to_gripper, atol=1e-4)

    camera_to_base = _pose(rng)
    gripper_to_target = _pose(rng)
    bases = []
    boards = []
    for _ in range(10):
        gripper_to_base = _pose(rng)
        target_to_camera = _invert(camera_to_base) @ gripper_to_base @ gripper_to_target
        bases.append(gripper_to_base)
        boards.append(target_to_camera)
    external = solver.solve_handeye(bases, boards, "eye_to_hand", cv2)
    assert external["meaning"] == "camera_to_base"
    assert np.allclose(external["transform"], camera_to_base, atol=1e-4)


def test_charuco_detector_sees_a_generated_board():
    cv2 = pytest.importorskip("cv2")
    board_mod = _load("handeye_board", ROOT / "toolkits" / "handeye" / "board.py")
    config = board_mod.load_config(ROOT / "toolkits" / "handeye" / "configs" / "default.yml")
    board, _dictionary = board_mod.make_board(config, cv2)
    image = board.generateImage((1400, 1000))
    bordered = cv2.copyMakeBorder(image, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    corners, ids, _board = board_mod.detect_charuco(bordered, config, cv2)
    assert ids is not None and len(ids) >= 8
    assert corners is not None


def _pose(rng) -> np.ndarray:
    axis = rng.normal(size=3)
    axis = axis / np.linalg.norm(axis)
    angle = float(rng.uniform(0.3, 1.4))
    skew = np.array(
        [[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]],
        dtype=float,
    )
    rotation = np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = rng.normal(size=3) * 0.4
    return out


def _invert(pose: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = pose[:3, :3].T
    out[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return out
