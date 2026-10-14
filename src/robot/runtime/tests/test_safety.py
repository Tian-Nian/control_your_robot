import threading
import time
import uuid
from pathlib import Path

import numpy as np
import pytest

from robot.runtime.client import RuntimeClient
from robot.runtime.log import log_path
from robot.runtime.node import run_component
from robot.runtime.service import start, status, stop
from robot.runtime.transport import Bus, SafetyStopError, SlotSpec


def _shm() -> str:
    return f"cyr_{uuid.uuid4().hex[:12]}"


def test_latch_rejects_later_commands():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=4)
    try:
        assert bus.latch_safety("left_arm 命令失败") is True
        assert bus.latch_safety("后来的原因") is False
        safety = bus.read_safety()
        assert safety["latched"] is True
        assert safety["reason"] == "left_arm 命令失败"
        with pytest.raises(SafetyStopError):
            bus.write_cmd("left_arm", {"joint": [1.0]})
    finally:
        bus.unlink()


def test_move_failure_safe_stops_and_blocks_commands():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("arm", 4096)], cmd_capacity=4)

    class Arm:
        def __init__(self):
            self.stopped = False
            self.joint = [0.0]

        def set_up(self):
            return None

        def set_collect_info(self, _info):
            return None

        def get(self):
            return {"joint": list(self.joint), "timestamp": time.time_ns()}

        def move(self, _data, is_delta=False):
            raise RuntimeError("servo fault")

        def safe_stop(self):
            self.stopped = True
            return "hold"

    arm = Arm()
    stop_event = threading.Event()

    def _run():
        try:
            run_component(shm, {"name": "arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}, stop_event, factory=lambda: arm)
        except SystemExit:
            return

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    client = RuntimeClient(shm)
    try:
        client.write_cmd("arm", {"joint": [1.0]})
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and thread.is_alive():
            time.sleep(0.02)
        assert arm.stopped is True
        assert arm.joint == [0.0]
        assert bus.read_safety()["latched"] is True
        with pytest.raises(SafetyStopError):
            client.write_cmd("arm", {"joint": [2.0]})
    finally:
        stop_event.set()
        thread.join(timeout=2)
        client.close()
        bus.unlink()


def test_arm_holds_and_mobile_zeros_on_safe_stop():
    from robot.controller.TestArm_controller import TestArmController
    from robot.controller.TestMobile_controller import TestMobileController

    arm = TestArmController("left_arm", DoFs=6, INFO="ERROR")
    arm.set_up()
    arm.set_collect_info(["joint", "eef", "gripper"])
    arm.set_joint(np.zeros(6))
    assert arm.safe_stop() == "hold"
    arm.move({"joint": np.ones(6)})
    np.testing.assert_allclose(arm.now_state["joint"], np.zeros(6))

    mobile = TestMobileController("test_mobile", INFO="ERROR")
    mobile.set_up()
    mobile.set_move_velocity(np.ones(6))
    assert mobile.safe_stop() == "zero_velocity"
    np.testing.assert_allclose(mobile.velocity, np.zeros(6))
    mobile.move({"move_velocity": np.ones(6)})
    np.testing.assert_allclose(mobile.velocity, np.zeros(6))


def test_service_stop_safe_stops_the_device(tmp_path):
    marker = tmp_path / "hold.txt"
    config = _config(tmp_path, _shm(), marker, crash=False)
    assert start(config) == 0
    try:
        assert stop() == 0
    finally:
        stop()
    assert marker.read_text(encoding="utf-8") == "stopped"


def test_component_crash_safe_stops_the_sibling(tmp_path):
    marker = tmp_path / "hold.txt"
    shm = _shm()
    config = _config(tmp_path, shm, marker, crash=True)
    assert start(config) == 0
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and status() == 0:
            time.sleep(0.05)
        assert status() == 3
        assert marker.read_text(encoding="utf-8") == "stopped"
        log_text = Path(log_path()).read_text(encoding="utf-8")
        assert "安全" in log_text
        assert "crash" in log_text or "组件退出" in log_text
    finally:
        stop()


def _config(directory: Path, shm: str, marker: Path, crash: bool) -> Path:
    crash_yaml = ""
    if crash:
        crash_yaml = """
  - name: crash
    class_path: robot.runtime.tests.safety_devices
    class_name: CrashDevice
    args: {}
    setup: {}
    publish: [joint]
    rate_hz: 20
    max_bytes: 4096
"""
    path = directory / "runtime.yml"
    path.write_text(
        f"""
name: safety
shm_name: {shm}
cmd_capacity: 4
components:
  - name: hold
    class_path: robot.runtime.tests.safety_devices
    class_name: HoldDevice
    args: {{marker: "{marker}"}}
    setup: {{}}
    publish: [joint]
    rate_hz: 20
    max_bytes: 4096
{crash_yaml}
""",
        encoding="utf-8",
    )
    return path
