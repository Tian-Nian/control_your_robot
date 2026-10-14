import threading
import time
import uuid

from robot.runtime.client import RuntimeClient
from robot.runtime.node import run_component
from robot.runtime.transport import Bus, SlotSpec


def _shm():
    return f"cyr_{uuid.uuid4().hex[:12]}"


def test_write_cmd_returns_before_move_finishes():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("arm", 4096)], cmd_capacity=8)
    started = threading.Event()
    release = threading.Event()

    class Arm:
        def __init__(self):
            self.joint = [0.0]

        def set_up(self):
            return None

        def set_collect_info(self, info):
            self.info = list(info)

        def get(self):
            return {"joint": list(self.joint), "timestamp": time.time_ns()}

        def move(self, data, is_delta=False):
            started.set()
            assert release.wait(2)
            self.joint = list(data["joint"])

    spec = {"name": "arm", "publish": ["joint"], "rate_hz": 50, "setup": {}}
    stop = threading.Event()
    arm = Arm()
    thread = threading.Thread(target=run_component, args=(shm, spec, stop), kwargs={"factory": lambda: arm}, daemon=True)
    thread.start()
    client = RuntimeClient(shm)
    try:
        first = client.write_cmd("arm", {"joint": [1.0]})
        assert started.wait(2)
        before = time.monotonic()
        second = client.write_cmd("arm", {"joint": [2.0]})
        elapsed = time.monotonic() - before
        read_seq, write_seq, _drops = bus.cmd_cursors("arm")
        assert elapsed < 0.2
        assert second == write_seq
        assert read_seq < second
        assert first == 1
        release.set()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and arm.joint != [2.0]:
            time.sleep(0.01)
        assert arm.joint == [2.0]
    finally:
        release.set()
        stop.set()
        thread.join(timeout=2)
        client.close()
        bus.unlink()


def test_oversize_frame_increments_drops_and_keeps_payload():
    shm = _shm()
    bus = Bus.create(shm, [SlotSpec("cam", 256)], cmd_capacity=2)

    class Camera:
        def __init__(self):
            self.calls = 0

        def set_up(self):
            return None

        def set_collect_info(self, info):
            return None

        def get(self):
            self.calls += 1
            if self.calls == 1:
                return {"color": "ok", "timestamp": time.time_ns()}
            return {"color": "x" * 1000, "timestamp": time.time_ns()}

    spec = {"name": "cam", "publish": ["color"], "rate_hz": 50, "setup": {}}
    stop = threading.Event()
    thread = threading.Thread(
        target=run_component,
        args=(shm, spec, stop),
        kwargs={"factory": lambda: Camera()},
        daemon=True,
    )
    thread.start()
    client = RuntimeClient(shm)
    try:
        deadline = time.monotonic() + 2
        status = None
        while time.monotonic() < deadline:
            status = client.read_status("cam")
            if status and status["drops"] > 0:
                break
            time.sleep(0.02)
        assert status is not None and status["drops"] > 0
        state = client.read_state("cam")
        assert state["data"]["color"] == "ok"
        assert "max_bytes" in status["error"]
    finally:
        stop.set()
        thread.join(timeout=2)
        client.close()
        bus.unlink()
