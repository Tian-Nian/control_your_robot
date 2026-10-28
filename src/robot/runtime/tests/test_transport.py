import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import numpy as np
import pytest

from robot.runtime.client import RuntimeClient
from robot.runtime.transport import Bus, BusBusyError, SlotSpec, pid_alive


def _slots():
    return [SlotSpec("left_arm", 4096), SlotSpec("cam_head", 4096)]


def _name():
    return f"cyr_{uuid.uuid4().hex[:12]}"


def test_command_ring_drops_oldest():
    shm = _name()
    bus = Bus.create(shm, [SlotSpec("left_arm", 4096)], cmd_capacity=8)
    try:
        for index in range(10):
            seq = bus.write_cmd("left_arm", {"joint": [index]})
            assert seq == index + 1
        _read, write, drops = bus.cmd_cursors("left_arm")
        assert write == 10
        assert drops == 2
        got = []
        while True:
            command = bus.pop_cmd("left_arm")
            if command is None:
                break
            got.append(command["seq"])
        assert got == [3, 4, 5, 6, 7, 8, 9, 10]
    finally:
        bus.unlink()


def test_oversize_state_keeps_previous_frame():
    shm = _name()
    bus = Bus.create(shm, [SlotSpec("cam", 128)], cmd_capacity=2)
    try:
        assert bus.write_state("cam", {"timestamp_ns": 10, "data": {"color": "ok"}})
        first = bus.read_state("cam")
        assert bus.write_state("cam", {"timestamp_ns": 11, "data": {"color": "x" * 500}}) is False
        second = bus.read_state("cam")
        assert second["seq"] == first["seq"]
        assert second["data"]["color"] == "ok"
    finally:
        bus.unlink()


def test_two_clients_see_the_same_frame():
    shm = _name()
    bus = Bus.create(shm, _slots(), cmd_capacity=4)
    try:
        joint = np.arange(6, dtype=np.float64)
        assert bus.write_state("left_arm", {"timestamp_ns": 42, "data": {"joint": joint}})
        first = RuntimeClient(shm)
        second = RuntimeClient(shm)
        try:
            left = first.read_state("left_arm")
            right = second.read_state("left_arm")
            assert left["seq"] == right["seq"]
            assert left["timestamp_ns"] == 42
            np.testing.assert_array_equal(left["data"]["joint"], right["data"]["joint"])
        finally:
            first.close()
            second.close()
    finally:
        bus.unlink()


def test_live_owner_rejects_a_second_segment():
    shm = _name()
    bus = Bus.create(shm, _slots(), cmd_capacity=2)
    try:
        assert pid_alive(os.getpid())
        with pytest.raises(BusBusyError):
            Bus.create(shm, _slots(), cmd_capacity=2)
    finally:
        bus.unlink()


def test_attacher_exit_keeps_the_segment():
    shm = _name()
    bus = Bus.create(shm, _slots(), cmd_capacity=2)
    try:
        assert bus.write_state("left_arm", {"timestamp_ns": 1, "data": {"joint": [1.0]}})
        src = str(Path(__file__).resolve().parents[3])
        env = os.environ.copy()
        env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        script = (
            "import sys\n"
            "from robot.runtime.client import RuntimeClient\n"
            "client = RuntimeClient(sys.argv[1])\n"
            "state = client.read_state('left_arm')\n"
            "assert state['data']['joint'] == [1.0]\n"
            "client.close()\n"
        )
        subprocess.run([sys.executable, "-c", script, shm], check=True, env=env)
        time.sleep(0.5)
        again = Bus.attach(shm)
        try:
            assert again.read_state("left_arm")["data"]["joint"] == [1.0]
        finally:
            again.close()
    finally:
        bus.unlink()


def test_zombie_is_not_alive():
    pid = os.fork()
    if pid == 0:
        os._exit(0)
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and pid_alive(pid):
            time.sleep(0.01)
        assert not pid_alive(pid)
    finally:
        os.waitpid(pid, 0)


def test_version_two_segment_can_be_read_but_not_streamed():
    from robot.utils.base.msgpack_numpy import packb

    from robot.runtime.transport import (
        CMD_SLOT_MAX,
        MAGIC,
        STATUS_MAX,
        LayoutMismatchError,
        _CMD_HDR,
        _DIR,
        _HEADER,
        _SAFETY,
        _SLOT_HDR,
        _align,
        _open_shared_memory,
        _safety_offset,
    )

    shm_name = _name()
    state_max = 4096
    cmd_capacity = 2
    cursor = _align(_safety_offset() + _SAFETY.size)
    state_off = cursor
    cursor = _align(cursor + _SLOT_HDR.size + state_max)
    cmd_off = cursor
    cursor = _align(cursor + _CMD_HDR.size + cmd_capacity * (_SLOT_HDR.size + CMD_SLOT_MAX))
    status_off = cursor
    cursor = _align(cursor + _SLOT_HDR.size + STATUS_MAX)
    created = _open_shared_memory(shm_name, create=True, size=cursor)
    try:
        _HEADER.pack_into(created.buf, 0, MAGIC, 2, os.getpid(), 1, cmd_capacity, 0)
        _DIR.pack_into(
            created.buf,
            _HEADER.size,
            b"left_arm",
            state_max,
            CMD_SLOT_MAX,
            STATUS_MAX,
            state_off,
            cmd_off,
            status_off,
        )
        payload = packb({"timestamp_ns": 5, "data": {"joint": [0.2]}}, use_bin_type=True)
        created.buf[state_off + _SLOT_HDR.size : state_off + _SLOT_HDR.size + len(payload)] = payload
        _SLOT_HDR.pack_into(created.buf, state_off, 1, len(payload), 0)
    finally:
        created.close()
    bus = Bus.attach(shm_name)
    try:
        assert bus.version == 2
        assert bus.read_state("left_arm")["data"]["joint"] == [0.2]
        assert bus.read_control() == {"held": False, "role": "", "label": ""}
        assert bus.read_stream("left_arm") is None
        with pytest.raises(LayoutMismatchError, match="cyr restart"):
            bus.write_stream("left_arm", {"joint": [0.3]})
        with pytest.raises(LayoutMismatchError, match="cyr restart"):
            bus.acquire_control("teleop")
        assert bus.read_state("left_arm")["data"]["joint"] == [0.2]
        assert bus.write_cmd("left_arm", {"joint": [0.4]}) == 1
    finally:
        bus.unlink()


def test_dead_owner_is_replaced(monkeypatch):
    shm = _name()
    bus = Bus.create(shm, _slots(), cmd_capacity=2, owner_pid=999999)
    bus.close()
    monkeypatch.setattr("robot.runtime.transport.pid_alive", lambda pid: pid != 999999)
    replacement = Bus.create(shm, _slots(), cmd_capacity=2)
    try:
        assert replacement.read_state("left_arm") is None
    finally:
        replacement.unlink()
