import os
import select
import signal
import struct
import subprocess
import sys
import time
import uuid
from pathlib import Path

import numpy as np
import pytest

from robot.runtime.client import RuntimeClient
from robot.runtime.service import start, status, stop


def _write_config(directory: Path, shm: str, include_camera: bool) -> Path:
    camera = ""
    if include_camera:
        camera = """
  - name: cam_head
    class_path: robot.sensor.TestVision_sensor
    class_name: TestVisonSensor
    args: {name: cam_head, INFO: DEBUG}
    setup: {is_depth: false}
    publish: [color]
    rate_hz: 30
    max_bytes: 2097152
"""
    path = directory / "runtime.yml"
    path.write_text(
        f"""
name: t
shm_name: {shm}
cmd_capacity: 8
components:
  - name: left_arm
    class_path: robot.controller.TestArm_controller
    class_name: TestArmController
    args: {{name: left_arm, DoFs: 6, INFO: DEBUG}}
    setup: {{}}
    publish: [joint, eef, gripper]
    rate_hz: 100
    max_bytes: 65536
{camera}
""",
        encoding="utf-8",
    )
    return path


@pytest.fixture
def running(tmp_path, request):
    include_camera = bool(getattr(request, "param", False))
    shm = f"cyr_{uuid.uuid4().hex[:12]}"
    config = _write_config(tmp_path, shm, include_camera)
    assert start(config) == 0
    try:
        yield shm
    finally:
        stop()


def test_arm_rate_and_command(running):
    shm = running
    client = RuntimeClient(shm)
    try:
        time.sleep(0.2)
        before = client.read_status("left_arm")
        assert before and before["alive"]
        time.sleep(1.0)
        after = client.read_status("left_arm")
        assert after["state_seq"] - before["state_seq"] >= 80
        ages = []
        for _ in range(50):
            state = client.read_state("left_arm")
            ages.append(time.time_ns() - int(state["timestamp_ns"]))
        ages.sort()
        assert ages[len(ages) // 2] < 50_000_000
        joint = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        started = time.monotonic()
        client.write_cmd("left_arm", {"joint": joint})
        assert time.monotonic() - started < 0.2
        deadline = time.monotonic() + 1.0
        seen = None
        while time.monotonic() < deadline:
            seen = client.read_state("left_arm")
            if seen is not None and np.allclose(seen["data"]["joint"], joint):
                break
            time.sleep(0.01)
        assert seen is not None
        np.testing.assert_allclose(seen["data"]["joint"], joint)
    finally:
        client.close()


def test_echo_prints_the_latest_joint(running, capsys):
    from robot.runtime.__main__ import echo

    time.sleep(0.2)
    assert echo("left_arm") == 0
    text = capsys.readouterr().out
    assert "seq=" in text
    assert "joint" in text


def test_start_is_idempotent_and_restart_resets(tmp_path, capsys):
    shm = f"cyr_{uuid.uuid4().hex[:12]}"
    config = _write_config(tmp_path, shm, include_camera=False)
    assert start(config) == 0
    try:
        first = status()
        assert first == 0
        text = capsys.readouterr().out
        assert "running" in text
        pid = int(text.split("pid=")[1].split()[0])
        assert start(config) == 0
        assert "已在运行" in capsys.readouterr().out
        assert status() == 0
        again = capsys.readouterr().out
        assert f"pid={pid}" in again
        from robot.runtime.service import restart

        assert restart() == 0
        assert status() == 0
        restarted = capsys.readouterr().out
        new_pid = int(restarted.split("pid=")[1].split()[0])
        assert new_pid != pid
        client = RuntimeClient(shm)
        try:
            time.sleep(0.3)
            early = client.read_status("left_arm")["state_seq"]
            time.sleep(0.4)
            later = client.read_status("left_arm")["state_seq"]
            assert later > early
        finally:
            client.close()
    finally:
        stop()
    assert status() == 3
    assert "stopped" in capsys.readouterr().out
    from multiprocessing import shared_memory

    with pytest.raises(FileNotFoundError):
        shared_memory.SharedMemory(name=shm, create=False)


def test_hmi_quits_without_stopping_the_service(tmp_path):
    pytest.importorskip("curses")
    shm = f"cyr_{uuid.uuid4().hex[:12]}"
    config = _write_config(tmp_path, shm, include_camera=True)
    assert start(config) == 0
    master, slave = os.openpty()
    import fcntl
    import termios

    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 40, 120, 0, 0))
    env = os.environ.copy()
    src = str(Path(__file__).resolve().parents[3])
    env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env["TERM"] = "xterm-256color"
    process = subprocess.Popen(
        [sys.executable, "-m", "robot.runtime", "hmi"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        env=env,
        start_new_session=True,
    )
    os.close(slave)
    try:
        output = b""
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and b"left_arm" not in output:
            ready, _, _ = select.select([master], [], [], 0.2)
            if ready:
                try:
                    output += os.read(master, 8192)
                except OSError:
                    break
        assert b"left_arm" in output
        assert b"cam_head" in output
        os.write(master, b"jq")
        # Keep draining curses output: a full PTY buffer can block redraw/exit
        # on macOS even after q has been delivered.
        deadline = time.monotonic() + 5
        while process.poll() is None and time.monotonic() < deadline:
            ready, _, _ = select.select([master], [], [], .1)
            if ready:
                try:
                    os.read(master, 65536)
                except OSError:
                    break
        process.wait(timeout=1)
        time.sleep(0.5)
        # POSIX shared memory is not mounted at /dev/shm on macOS.
        from robot.runtime.transport import Bus
        attached = Bus.attach(shm)
        attached.close()
        assert status() == 0
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
        os.close(master)
        stop()


def test_hmi_x_stops_the_service(tmp_path):
    pytest.importorskip("curses")
    shm = f"cyr_{uuid.uuid4().hex[:12]}"
    config = _write_config(tmp_path, shm, include_camera=False)
    assert start(config) == 0
    master, slave = os.openpty()
    import fcntl
    import termios

    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 40, 120, 0, 0))
    env = os.environ.copy()
    src = str(Path(__file__).resolve().parents[3])
    env["PYTHONPATH"] = src + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
    env["TERM"] = "xterm-256color"
    process = subprocess.Popen(
        [sys.executable, "-m", "robot.runtime", "hmi"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        env=env,
        start_new_session=True,
    )
    os.close(slave)
    try:
        output = b""
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and b"left_arm" not in output:
            ready, _, _ = select.select([master], [], [], 0.2)
            if ready:
                try:
                    output += os.read(master, 8192)
                except OSError:
                    break
        assert b"left_arm" in output
        os.write(master, b"xy")
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and status() == 0:
            _drain_pty(master)
            time.sleep(0.1)
        assert status() == 3
        os.write(master, b"q")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and process.poll() is None:
            _drain_pty(master)
            time.sleep(0.05)
        assert process.poll() is not None
        process.wait(timeout=2)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)
        os.close(master)
        stop()


def _drain_pty(master: int) -> None:
    while True:
        ready, _, _ = select.select([master], [], [], 0)
        if not ready:
            return
        try:
            os.read(master, 8192)
        except OSError:
            return
