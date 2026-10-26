"""Opt-in MuJoCo platform smoke test through cyr's normal service path.

Default: isolated temporary service; --attach uses an already-started scene.
This checks contact settling, arm control and cameras, NOT completed stacking.
"""
import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


@contextmanager
def scene_session(args):
    from robot.runtime.session import Session
    if args.attach:
        session = Session.attach("cyr_piper_mujoco")
        try:
            yield session
        finally:
            session.close()
        return
    import yaml
    with tempfile.TemporaryDirectory(prefix="cyr_mujoco_") as tmp:
        name = "mj_" + uuid.uuid4().hex[:10]
        config = yaml.safe_load((ROOT / "config/runtime/piper_mujoco.yml").read_text())
        config.update(name=name, shm_name=name)
        config["components"][0]["args"].update(spawn=args.viewer, cameras=not args.no_cameras)
        path = Path(tmp) / "scene.yml"
        path.write_text(yaml.safe_dump(config))
        env = os.environ.copy()
        env["XDG_STATE_HOME"] = tmp
        env["PYTHONPATH"] = str(ROOT / "src")
        env.pop("DEEPSEEK_API_KEY", None)
        command = [sys.executable, "-m", "robot.runtime"]
        session = None
        old_state_home = os.environ.get("XDG_STATE_HOME")
        os.environ["XDG_STATE_HOME"] = tmp
        try:
            subprocess.run(command + ["start", "--config", str(path)], env=env, check=True, timeout=30)
            session = Session.attach(name)
            yield session
        finally:
            if session:
                session.close()
            try:
                subprocess.run(command + ["stop"], env=env, check=True, timeout=30)
            finally:
                if old_state_home is None:
                    os.environ.pop("XDG_STATE_HOME", None)
                else:
                    os.environ["XDG_STATE_HOME"] = old_state_home


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pick-test", action="store_true", help="Also verify fixed-scene physical pick, lift and release")
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument("--out", type=Path, default=ROOT / "save" / f"mujoco_{uuid.uuid4().hex[:10]}.jsonl")
    args = parser.parse_args()
    if args.attach and (args.viewer or args.no_cameras):
        parser.error("--attach uses the running service's camera/viewer settings")
    from robot.harness import PositionExecutor
    from robot.harness.stacking import StackVerifier
    import numpy as np
    with scene_session(args) as session:
        time.sleep(1.0)
        before = session.read_raw("left_arm")
        state = before["data"]
        assert len(state["objects"]) == 3
        assert state["contacts"], "no physical contacts observed"
        for item in state["objects"].values():
            assert -.01 < item["pose"][2] < .03, "bowl did not settle on tabletop"
            assert np.linalg.norm(item["velocity"]) < .1
        if not args.no_cameras:
            assert set(state["cameras"]) == {"head", "wrist"}
            for camera in state["cameras"].values():
                assert camera["rgb"].shape == (240, 320, 3)
                assert np.isfinite(camera["depth_m"]).all()
                assert np.std(camera["rgb"]) > 5, "blank camera image"
                assert time.time_ns() - camera["timestamp_ns"] < 1e9
        q = np.array(state["joint"])
        goal = q.copy(); goal[1] += .03
        with PositionExecutor(session, lower=[-2.618, 0, -2.697, -1.832, -1.22, -3.14],
                              upper=[2.618, 3.14, 2.697, 1.832, 1.22, 3.14],
                              max_age_ms=500, journal=args.out) as executor:
            for rid, target in (("move", goal), ("return", q)):
                result = executor.execute(rid, joint=target.tolist(), timeout_s=3)
                print(rid, result["status"], result["reason"])
                assert result["status"] == "succeeded", result
        after = session.read_raw("left_arm")["data"]
        assert after["sim_time"] > state["sim_time"]
        if not args.no_cameras:
            assert after["cameras"]["wrist"]["frame_id"] > state["cameras"]["wrist"]["frame_id"]
        if args.pick_test:
            from robot.harness.mujoco_checks import run_pick_check
            import json
            evidence = run_pick_check(session)
            evidence_path = args.out.with_suffix(".pick.json")
            with evidence_path.open("x") as f:
                json.dump(evidence, f, ensure_ascii=False, indent=2)
            print("PASS: 物理夹取、抬升、松手落桌。证据:", evidence_path.resolve())
            after = session.read_raw("left_arm")["data"]
        print("叠碗判定（初始三碗分开，应为 False）:", StackVerifier().update(after)["stacked"])
        print("PASS: cyr 生命周期、碗落桌接触、关节反馈、停止确认" + ("、双相机 RGB/深度" if not args.no_cameras else ""))
        print("完整三碗叠放尚未验收。日志:", args.out.resolve())


if __name__ == "__main__":
    main()
