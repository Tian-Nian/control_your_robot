"""Run verified position skills in an isolated Piper Sim (no existing service).

python examples/piper_harness.py --out /tmp/piper-harness.jsonl
Optional --viewer opens a separate Rerun window. Output must not already exist.
"""
from pathlib import Path
import argparse
import os
import subprocess
import sys
import tempfile
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("save") / f"harness_{uuid.uuid4().hex[:12]}.jsonl")
    parser.add_argument("--agent", action="store_true", help="Exercise model-independent skills, EEF and persistent tasks offline")
    parser.add_argument("--store", type=Path, help="Persistent SQLite request store (defaults beside --out)")
    parser.add_argument("--deepseek", action="store_true", help="Run the fixed task through live DeepSeek tool calls")
    parser.add_argument("--model", default="deepseek-flash", help="DeepSeek model ID")
    args = parser.parse_args()
    if args.agent and args.deepseek:
        parser.error("choose --agent or --deepseek")
    if args.deepseek and not os.environ.get("DEEPSEEK_API_KEY"):
        parser.error("DEEPSEEK_API_KEY must be exported")
    import yaml
    from robot.controller.PiperSim_controller import PiperSimController, READY_JOINT
    from robot.harness import PositionExecutor
    from robot.runtime.session import Session

    model = PiperSimController("limits", spawn=False)
    model.set_up(tool_in_urdf=True)
    old_state = os.environ.get("XDG_STATE_HOME")
    process = session = None
    with tempfile.TemporaryDirectory(prefix="piper_harness_") as tmp:
        try:
            os.environ["XDG_STATE_HOME"] = tmp
            name = "harness_" + uuid.uuid4().hex[:10]
            config = dict(name=name, shm_name=name, components=[dict(
                name="left_arm", class_path="robot.controller.PiperSim_controller",
                class_name="PiperSimController", args=dict(name="left_arm", spawn=args.viewer),
                setup=dict(tool_in_urdf=True, control_mode="position"),
                publish=["joint", "gripper", "control_mode", "servo_fault"], rate_hz=50,
                max_bytes=65536, stream_timeout_s=5.0,
                gate=dict(lower=model._lower[:6].tolist(), upper=model._upper[:6].tolist(), max_step_rad=0.02),
            )])
            path = Path(tmp) / "config.yml"
            path.write_text(yaml.safe_dump(config))
            env = os.environ.copy()
            env.pop("DEEPSEEK_API_KEY", None)  # Simulator does not need model credentials.
            env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1] / "src") + os.pathsep + env.get("PYTHONPATH", "")
            with (Path(tmp) / "driver.log").open("w") as log:
                process = subprocess.Popen([sys.executable, "-m", "robot.runtime", "serve", str(path)],
                                           env=env, stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError((Path(tmp) / "driver.log").read_text())
                    try:
                        session = Session.attach(name)
                        if session.read_raw("left_arm"):
                            break
                        session.close()
                        session = None
                    except RuntimeError:
                        pass
                    time.sleep(0.05)
                if session is None:
                    raise RuntimeError("Piper Sim did not start")
                goal = [float(v) for v in READY_JOINT]
                goal[1] += 0.05
                with PositionExecutor(session, lower=model._lower[:6], upper=model._upper[:6],
                                      journal=args.out) as executor:
                    print("能力:", executor.capabilities())
                    requests = [dict(request_id="move", joint=goal),
                                dict(request_id="gripper", gripper=0.5),
                                dict(request_id="return", joint=list(READY_JOINT))]
                    if args.agent:
                        from robot.harness import AgentHarness
                        from robot.harness.kinematics import PiperKinematics
                        kin = PiperKinematics()
                        harness = AgentHarness(executor, robot_id="piper-isolated-demo",
                                               store_path=args.store or args.out.with_suffix(".sqlite3"), kinematics=kin)
                        observation = harness.dispatch("observe", {})
                        print("观测:", observation)
                        task = [dict(skill="move_joint_absolute", arguments=dict(joint=goal)),
                                dict(skill="set_gripper", arguments=dict(gripper=.5)),
                                dict(skill="move_eef", arguments=dict(pose=kin.pose(list(READY_JOINT), "tool"), frame="base", point="tool"))]
                        outcome = harness.run_task("piper-demo-v1", task)
                        print("任务:", outcome)
                        assert outcome["status"] == "succeeded"
                        assert harness.run_task("piper-demo-v1", task) == outcome
                        print("PASS：观测、关节、夹爪、工具点位姿已验证；任务重复 ID 返回历史结果。")
                        print("状态:", harness.dispatch("get_status", {}))
                    elif args.deepseek:
                        from deepseek_piper_task import run_deepseek
                        results = run_deepseek(executor, requests, args.model)
                    else:
                        results = executor.run(requests)
                    if not args.agent:
                        for result in results:
                            print(result["request_id"], result["status"], result["reason"])
                        assert len(results) == 3 and all(r["status"] == "succeeded" for r in results)
                        assert executor.execute(**requests[0]) == results[0]
                        print("PASS：三步反馈已验证；重复 ID 返回原结果，不重复移动。")
                print("事件日志:", args.out.resolve())
        finally:
            if session is not None:
                session.close()
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            if old_state is None:
                os.environ.pop("XDG_STATE_HOME", None)
            else:
                os.environ["XDG_STATE_HOME"] = old_state


if __name__ == "__main__":
    main()
