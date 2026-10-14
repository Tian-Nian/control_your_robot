import json
import os
import threading
import time
from pathlib import Path

import numpy as np

from robot.runtime.discover import write_record
from robot.runtime.node import run_component
from robot.runtime.session import Session
from robot.runtime.transport import Bus, SlotSpec


def _shm() -> str:
    return f"cyr_{time.time_ns()}"


class _Camera:
    def set_up(self):
        return None

    def set_collect_info(self, info):
        self.info = list(info)

    def get(self):
        image = np.zeros((4, 4, 3), dtype=np.uint8)
        image[0, 0, 0] = 9
        return {"color": image, "timestamp": time.time_ns()}


def _config(path: Path, shm: str) -> None:
    path.write_text(
        f"""
name: session_test
shm_name: {shm}
cmd_capacity: 8
components:
  - name: left_arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    args: {{name: left_arm, DoFs: 6, INFO: ERROR}}
    publish: [joint]
    rate_hz: 50
    max_bytes: 65536
  - name: cam_head
    class_path: robot.sensor.TestVision_sensor
    class_name: TestVisonSensor
    args: {{name: cam_head, INFO: ERROR}}
    publish: [color]
    rate_hz: 50
    max_bytes: 65536
""",
        encoding="utf-8",
    )


def test_session_call_reset_snapshot_and_safety(tmp_path):
    from robot.controller.TestArm_controller import TestArmController

    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 65536), SlotSpec("cam_head", 65536)], cmd_capacity=8)
    config_path = tmp_path / "runtime.yml"
    _config(config_path, shm)
    write_record(
        {
            "pid": os.getpid(),
            "config_path": str(config_path),
            "shm_name": shm,
            "components": ["left_arm", "cam_head"],
        }
    )
    stop = threading.Event()
    arm = TestArmController("left_arm", DoFs=6, INFO="ERROR")
    arm_spec = {"name": "left_arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}
    cam_spec = {"name": "cam_head", "publish": ["color"], "rate_hz": 50, "setup": {}}
    threads = [
        threading.Thread(
            target=run_component,
            args=(shm, arm_spec, stop),
            kwargs={"factory": lambda: arm},
            daemon=True,
        ),
        threading.Thread(
            target=run_component,
            args=(shm, cam_spec, stop),
            kwargs={"factory": _Camera},
            daemon=True,
        ),
    ]
    for thread in threads:
        thread.start()
    session = Session.attach()
    try:
        described = {item["name"]: item for item in session.describe()}
        assert described["left_arm"]["accepts_commands"] is True
        assert described["left_arm"]["publish"] == ["joint"]
        assert described["cam_head"]["accepts_commands"] is False

        refused = session.call("cam_head", {"color": 1})
        assert refused["accepted"] is False
        assert refused["error"] == "组件不接受命令"

        moved = session.call("left_arm", {"joint": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]})
        assert moved["accepted"] is True
        assert moved["seq"] == 1
        assert np.allclose(moved["state"]["data"]["joint"], [0.2] * 6)

        reset = session.reset("left_arm")
        assert reset["accepted"] is True
        assert np.allclose(reset["state"]["data"]["joint"], [0.0] * 6)

        picture = None
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            view = session.snapshot()["components"]["cam_head"]["data"]
            if view and view.get("color"):
                picture = view["color"]
                break
            time.sleep(0.02)
        assert picture is not None
        assert picture["kind"] == "ndarray"
        assert picture["shape"] == [4, 4, 3]
        assert picture["nbytes"] == 48
        assert 9 not in picture

        bus.latch_safety("测试锁存")
        blocked = session.call("left_arm", {"joint": [0.5] * 6})
        assert blocked["accepted"] is False
        assert "测试锁存" in blocked["error"]
    finally:
        session.close()
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()


def test_cli_snapshot_and_call_are_one_json_line(tmp_path, capsys):
    from robot.runtime.__main__ import main

    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("left_arm", 65536), SlotSpec("cam_head", 65536)], cmd_capacity=8)
    config_path = tmp_path / "runtime.yml"
    _config(config_path, shm)
    write_record(
        {
            "pid": os.getpid(),
            "config_path": str(config_path),
            "shm_name": shm,
            "components": ["left_arm", "cam_head"],
        }
    )
    stop = threading.Event()
    from robot.controller.TestArm_controller import TestArmController

    arm = TestArmController("left_arm", DoFs=6, INFO="ERROR")
    threads = [
        threading.Thread(
            target=run_component,
            args=(shm, {"name": "left_arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}, stop),
            kwargs={"factory": lambda: arm},
            daemon=True,
        ),
        threading.Thread(
            target=run_component,
            args=(shm, {"name": "cam_head", "publish": ["color"], "rate_hz": 50, "setup": {}}, stop),
            kwargs={"factory": _Camera},
            daemon=True,
        ),
    ]
    for thread in threads:
        thread.start()
    try:
        time.sleep(0.3)
        assert main(["snapshot"]) == 0
        line = capsys.readouterr().out.strip().splitlines()[-1]
        payload = json.loads(line)
        assert payload["components"]["cam_head"]["data"]["color"]["kind"] == "ndarray"
        assert main(["call", "left_arm", json.dumps({"op": "reset"})]) == 0
        result = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
        assert result["accepted"] is True
        assert result["state"]["data"]["joint"] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        bus.unlink()
