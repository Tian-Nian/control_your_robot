"""Reopenable read-only Rerun bridge for an existing Piper MuJoCo service.

Only copies published poses into a local render model and performs FK. It never
steps physics, acquires control, or writes a command to the running service.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
PORT = 9879


def sync_view(model, state):
    model.data.qpos[model._qadr[:6]] = state["joint"]
    model.data.qpos[model._qadr[6:]] = state["gripper_joint_m"]
    for name, item in state["objects"].items():
        adr = model.model.joint(name + "_free").qposadr[0]
        model.data.qpos[adr:adr+7] = item["pose"]
    model.mj.mj_forward(model.model, model.data)
    model.data.time = float(state["sim_time"])
    model._cameras = state["cameras"]


def ensure_viewer(directory):
    directory.mkdir(parents=True, exist_ok=True)
    pidfile = directory / "viewer.json"
    request = directory / "viewer.reopen"
    ready = directory / "viewer.ready"
    pid = None
    try:
        pid = int(json.loads(pidfile.read_text())["pid"])
        command = subprocess.run(["ps", "-p", str(pid), "-o", "command="], capture_output=True, text=True).stdout
        if str(Path(__file__).resolve()) not in command:
            pid = None
    except (OSError, ValueError, KeyError):
        pass
    ready.unlink(missing_ok=True)
    request.touch()
    def launch():
        with (directory / "viewer.log").open("ab") as log:
            env = os.environ.copy()
            for key in ("DEEPSEEK_API_KEY", "CYR_VISION_API_KEY", "OPENAI_API_KEY"):
                env.pop(key, None)
            return subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--directory", str(directory)],
                cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    process = launch() if pid is None else None
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        if ready.exists():
            print("Rerun 已打开：当前场景、头部和腕部视角（独立只读窗口）。", flush=True)
            return
        if process is not None and process.poll() is not None:
            break
        # A bridge attached before a scene restart exits on stale feedback.
        if process is None and not pidfile.exists():
            request.touch()
            process = launch()
        time.sleep(.1)
    raise RuntimeError(f"Rerun 未就绪，请检查 {directory / 'viewer.log'}")


def run_viewer(directory):
    from robot.controller.PiperMuJoCo_controller import PiperMuJoCoController
    from robot.runtime.session import Session
    session = Session.attach("cyr_piper_mujoco")
    device = None
    pidfile = directory / "viewer.json"
    request = directory / "viewer.reopen"
    ready = directory / "viewer.ready"
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    pidfile.write_text(json.dumps({"pid": os.getpid()}))
    try:
        spec = next(s for s in session.describe() if s["name"] == "left_arm")
        if spec["class_name"] != "PiperMuJoCoController":
            raise RuntimeError("viewer requires PiperMuJoCoController")
        # No camera renderer or physics stepping: replay measured poses only.
        device = PiperMuJoCoController(cameras=False)
        device.set_up()
        seq = None
        last_fresh = time.monotonic()
        while not stopped:
            frame = session.read_raw("left_arm")
            fresh = frame and 0 <= time.time_ns() - frame["timestamp_ns"] < 2_000_000_000
            reopen = request.exists()
            if fresh and (frame["seq"] != seq or reopen):
                last_fresh = time.monotonic()
                sync_view(device, frame["data"])
                if device.rr is None or reopen:
                    request.unlink(missing_ok=True)
                    device.spawn = True
                    device._start_rerun(port=PORT)
                    ready.write_text("ready")
                device._log_rerun()
                seq = frame["seq"]
            elif time.monotonic() - last_fresh > 5:
                break
            time.sleep(.1)
    finally:
        if device is not None:
            device.stop()
        session.close()
        pidfile.unlink(missing_ok=True)
        ready.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ensure", action="store_true")
    parser.add_argument("--directory", type=Path)
    args = parser.parse_args()
    directory = args.directory or Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state")) / "cyr"
    if args.ensure:
        ensure_viewer(directory)
    else:
        run_viewer(directory)


if __name__ == "__main__":
    main()
