from pathlib import Path

import numpy as np

from types import SimpleNamespace

from robot.runtime.config import load_config
from robot.runtime.session import _accepts_commands
from robot.runtime.supervisor import process_groups
from robot.controller.Marvin_arm_controller import (
    MarvinArmController,
    clear_shared_robots,
    flange_pose,
    joints_from_feedback,
)
from robot.controller.WujiHand_controller import WujiHandController, _deploy_wuji_site, joints_from_frame
from robot.sensor.MoxianTactile_sensor import CHANNELS, parse_frame

_ROOT = Path(__file__).resolve().parents[4]
_MOTION = (
    "set_joint_position_cmd",
    "set_position_state",
    "set_imp",
    "joint_command",
    "mit_params",
    "effort_limit",
    ".enable(",
    "clear_fault",
    "set_tool",
    ".disable(",
)


def test_marvin_lab_config_commands_arms_and_hands():
    config = load_config(_ROOT / "config" / "runtime" / "marvin_lab.yml")
    assert config.shm_name == "cyr_marvin_lab"
    names = [item.name for item in config.components]
    assert names == [
        "left_arm",
        "right_arm",
        "left_hand",
        "right_hand",
        "left_glove",
        "right_glove",
        "cam_head",
        "cam_left_wrist",
        "cam_right_wrist",
    ]
    commanded = {"left_arm", "right_arm", "left_hand", "right_hand"}
    for item in config.components:
        assert _accepts_commands(item.class_path) is (item.name in commanded)
    left_arm = next(item for item in config.components if item.name == "left_arm")
    assert left_arm.setup["control_mode"] == "impedance"
    assert left_arm.setup["joint_K"] == [16.0, 16.0, 16.0, 8.0, 8.0, 8.0, 8.0]
    assert left_arm.setup["vel_ratio"] == 50


def test_reader_sources_have_no_motion_api():
    sensor_dir = _ROOT / "src" / "robot" / "sensor"
    for name in (
        "MoxianTactile_sensor.py",
        "Orbbec_sensor.py",
    ):
        text = (sensor_dir / name).read_text(encoding="utf-8")
        for token in _MOTION:
            assert token not in text, f"{name} contains {token}"


def test_marvin_feedback_is_radians_and_stop_only_releases(monkeypatch):
    monkeypatch.setattr("robot.controller.Marvin_arm_controller.time.sleep", lambda _seconds: None)
    feedback = {
        "outputs": [
            {"fb_joint_pos": [0, 10, 0, 0, 0, 0, 0]},
            {"fb_joint_pos": [0, -20, 0, 0, 0, 0, 0]},
        ],
        "states": [{"cur_state": 1}, {"cur_state": 0}],
    }
    left, left_mode = joints_from_feedback(feedback, "A")
    right, right_mode = joints_from_feedback(feedback, "B")
    assert np.isclose(left[1], np.deg2rad(10))
    assert np.isclose(right[1], np.deg2rad(-20))
    assert (left_mode, right_mode) == (1, 0)

    clear_shared_robots()
    calls = []

    class Robot:
        def subscribe(self, _dcss):
            calls.append("subscribe")
            return {
                "outputs": feedback["outputs"],
                "states": [{"cur_state": self.mode["A"]}, {"cur_state": self.mode["B"]}],
            }

        def set_joint_position_cmd(self, arm, joint):
            calls.append(("joint", arm, list(joint)))
            return True

        def disable(self, arm):
            self.mode[arm] = 0
            calls.append(("disable", arm))
            return True

        def set_position_state(self, arm, velRatio, AccRatio):
            self.mode[arm] = 1
            calls.append(("position", arm, velRatio, AccRatio))
            return True

        def set_imp_joint_state(self, arm, velRatio, AccRatio, K, D):
            self.mode[arm] = 3
            calls.append(("impedance", arm, list(K), list(D)))
            return True

        def set_tool(self, arm, kine_para, dyn_para):
            calls.append(("tool", arm, list(dyn_para)))
            return True

        def release_robot(self):
            calls.append("release")

        def __init__(self):
            self.mode = {"A": 1, "B": 0}

    class Kine:
        def fk(self, joints):
            native = np.eye(4)
            native[0, 3] = 1000.0
            return native

    robot = Robot()

    def connect(_ip):
        calls.append("connect")
        return robot, object()

    left_arm = MarvinArmController("left_arm")
    right_arm = MarvinArmController("right_arm")
    left_arm.set_up("192.168.1.190", side="left", arm_id="A", _connect=connect, _kine=Kine())
    right_arm.set_up("192.168.1.190", side="right", arm_id="B", _connect=connect, _kine=Kine())
    left_info = left_arm.get_information()
    right_info = right_arm.get_information()
    assert calls.count("connect") == 1
    assert left_info["control_mode"] == "impedance"
    assert any(item[0] == "impedance" and item[2][0] == 16.0 and item[3][0] == 1.8 for item in calls)
    assert np.allclose(left_info["joint"], left)
    assert np.allclose(right_info["joint"], right)
    assert left_info["eef"].shape == (7,)
    assert np.allclose(left_info["eef"], [1.0, 0.0, 0.0, np.sqrt(0.5), -np.sqrt(0.5), 0.0, 0.0])
    assert np.allclose(right_info["eef"], [1.0, 0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0, 0.0])
    left_arm.move({"control_mode": "position", "joint": left.tolist()})
    joint_cmds = [item for item in calls if isinstance(item, tuple) and item[0] == "joint" and item[1] == "A"]
    assert np.allclose(joint_cmds[-1][2], np.degrees(left))
    assert left_arm.get_information()["control_mode"] == "position"
    assert left_arm.safe_stop() == "stop"
    assert "release" not in calls
    assert right_arm.safe_stop() == "stop"
    assert calls[-1] == "release"
    clear_shared_robots()


def test_flange_pose_is_meters_and_wxyz():
    native = np.eye(4)
    native[0, 3] = 250.0
    pose = flange_pose(native, "left")
    assert np.allclose(pose, [0.25, 0.0, 0.0, np.sqrt(0.5), -np.sqrt(0.5), 0.0, 0.0])


def test_same_controller_arms_share_one_process():
    groups = process_groups(
        [
            SimpleNamespace(name="left_arm", class_name="MarvinArmController", setup={"robot_ip": "192.168.1.190"}),
            SimpleNamespace(name="left_hand", class_name="WujiHandController", setup={"address": "192.168.1.110:7447"}),
            SimpleNamespace(name="right_arm", class_name="MarvinArmController", setup={"robot_ip": "192.168.1.190"}),
        ]
    )
    assert [[item.name for item in group] for group in groups] == [
        ["left_arm", "right_arm"],
        ["left_hand"],
    ]


def test_wuji_prefers_deploy_conda_sdk(monkeypatch, tmp_path):
    import sys

    minor = f"python{sys.version_info.major}.{sys.version_info.minor}"
    site = tmp_path / "xone_new" / "lib" / minor / "site-packages"
    (site / "wuji_sdk").mkdir(parents=True)
    monkeypatch.delenv("CYR_WUJI_SITE", raising=False)
    monkeypatch.setattr(sys, "prefix", str(tmp_path / "Xone"))
    assert _deploy_wuji_site() == site
    monkeypatch.setattr(sys, "prefix", str(tmp_path / "xone_new"))
    assert _deploy_wuji_site() is None


def test_wuji_enables_and_sends_joint_command(monkeypatch):
    class Joint:
        def __init__(self, nid, position):
            self.nid = nid
            self.position = position

    class Frame:
        joints = [Joint(1, 0.2), Joint(6, -0.4)]

    flat = joints_from_frame(Frame())
    assert flat.shape == (20,)
    assert np.isclose(flat[0], 0.2)
    assert np.isclose(flat[4], -0.4)

    class Sub:
        def recv(self):
            return Frame()

        def close(self):
            calls.append("close")

    class Pub:
        def send(self, commands):
            calls.append(("send", len(commands), commands[0].position))

    class Knob:
        def set(self, value):
            calls.append(("set", value))

    class Command:
        def __init__(self, position, velocity, effort):
            self.position = position
            self.velocity = velocity
            self.effort = effort

    class Device:
        def joint_states(self):
            calls.append("joint_states")
            return self

        def subscribe(self):
            calls.append("subscribe")
            return Sub()

        def mit_params(self):
            return Knob()

        def effort_limit(self):
            return Knob()

        def enable(self):
            calls.append("enable")

        def disable(self):
            calls.append("disable")

        def joint_command(self):
            return self

        def publish(self):
            return Pub()

        def disconnect(self):
            calls.append("disconnect")

    calls = []
    monkeypatch.setattr(
        "robot.controller.WujiHand_controller._load_wuji_sdk",
        lambda: SimpleNamespace(JointCommand=Command),
    )
    hand = WujiHandController("left_hand")
    hand.set_up("left", "192.168.1.110:7447", _connect=lambda side, address: Device())
    assert np.allclose(hand.get_information()["joint"], flat)
    assert "enable" in calls
    assert ("set", (3.0, 0.1)) in calls
    assert ("set", 1.5) in calls
    hand.move({"joint": flat})
    assert calls[-1][0] == "send"
    assert calls[-1][1] == 20
    hand.move({"joint": np.zeros(20)})
    for _ in range(5):
        hand.get_information()
    sent = [item[2] for item in calls if isinstance(item, tuple) and item[0] == "send"]
    assert sent[0] == 0.2
    assert sent[-1] == 0.0
    assert sent[-2] == 0.0
    assert hand.safe_stop() == "stop"
    assert "disable" in calls
    assert calls[-1] == "disconnect"


def test_moxian_frame_parses_pressure():
    payload = bytes(range(256)) + bytes([7] * (CHANNELS - 256))
    body = bytes([0xA5, 0x5A, 0x01, 0xD2, 0x01, 0x01]) + payload
    checksum = sum(body) & 0xFFFF
    frame = body + bytes([checksum & 0xFF, checksum >> 8])
    parsed = parse_frame(frame)
    assert parsed is not None
    assert parsed.shape == (CHANNELS,)
    assert int(parsed[0]) == 0
    assert int(parsed[-1]) == 7
    assert parse_frame(frame[:-1] + b"\x00") is None
