"""用关节目标采集重力矩。点位用 cyr 的 joint 命令下发，到位后再记。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
HERE = Path(__file__).resolve().parent
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from identify import DEMO_COM, DEMO_MASS, DEMO_POSES, identify_payload  # noqa: E402
from robot.controller.payload_gravity import fit_payload  # noqa: E402


def demo() -> dict:
    """URDF 不含这块负载。先在 8 个姿态上读托住臂的力矩，拟合后再做重力补偿。"""
    from robot.controller.PiperSim_controller import PiperSimController

    arm = PiperSimController("left", spawn=False)
    arm.set_up(control_mode="position", plant_mass=DEMO_MASS, plant_com=DEMO_COM)
    before = arm.get_state()["joint"].copy()
    arm.set_control_mode("gravity_compensation")
    for _ in range(30):
        arm.servo_step(0.02)
        if arm.fault:
            break
    sag = arm.get_state()["joint"].copy()
    unidentified_fault = arm.fault
    fit = identify_payload(arm, DEMO_POSES)
    arm.set_control_mode("gravity_compensation")
    held = arm.get_state()["joint"].copy()
    for _ in range(30):
        arm.servo_step(0.02)
    after = arm.get_state()["joint"].copy()
    return {
        "mass_kg": fit["mass_kg"],
        "com_m": [float(item) for item in fit["com_m"]],
        "rms": fit["rms"],
        "sag_joint2": float(sag[1] - before[1]),
        "unidentified_fault": unidentified_fault,
        "hold_delta": float(np.max(np.abs(after - held))),
        "plant_mass": DEMO_MASS,
        "plant_com": DEMO_COM,
    }


def collect_live(session, arm_name: str) -> dict:
    session.call(arm_name, {"control_mode": "position"}, timeout=2.0)
    print(
        "先处于 position。没有重力补偿时人搬不动臂，点位用 cyr 下发，臂走到停稳后再记。\n"
        f"cyr call {arm_name} '{{\"joint\":[0, 1.3, -1, 0, 0.5, 0]}}'\n"
        "已经下发过就按 Enter 记录当前姿态。把 6 个关节贴在这里，也会用同样的 joint 命令下发并在到位后记录。\n"
        "输入 done 开始辨识。"
    )
    samples = []
    while True:
        line = input(f"已采集 {len(samples)} 组 > ").strip()
        if line.lower() == "done":
            break
        joint = _parse_joint(line)
        if joint is not None:
            called = session.call(arm_name, {"joint": joint}, timeout=15.0)
            if not called.get("accepted"):
                print(called.get("error") or "关节目标没有被接受")
                continue
        samples.append(_sample(session, arm_name))
        print(
            f"关节 {np.array2string(samples[-1]['joint'], precision=3)} "
            f"力矩 {np.array2string(samples[-1]['torque'], precision=3)}"
        )
    fit = fit_payload(samples)
    session.call(
        arm_name,
        {"op": "set_payload", "mass": fit["mass_kg"], "com": fit["com_m"].tolist()},
        timeout=2.0,
    )
    return fit


def _parse_joint(line: str):
    if not line:
        return None
    if line.startswith("["):
        values = json.loads(line)
    else:
        values = [float(part) for part in line.replace(",", " ").split()]
    values = [float(item) for item in values]
    if len(values) != 6:
        raise ValueError(f"关节目标要 6 个数，得到 {len(values)}")
    return values


def _sample(session, arm_name: str) -> dict:
    called = session.call(arm_name, {"op": "hold_torque"}, timeout=2.0)
    data = ((called.get("state") or {}).get("data") or {})
    result = data.get("result") or {}
    if "jacobian" in result and "link_gravity" in result:
        torque = np.asarray(result["torque"], dtype=float).reshape(6)
        link = np.asarray(result["link_gravity"], dtype=float).reshape(6)
        return {
            "joint": np.asarray(result["joint"], dtype=float).reshape(6),
            "jacobian": np.asarray(result["jacobian"], dtype=float).reshape(6, 6),
            "rotation": np.asarray(result["rotation"], dtype=float).reshape(3, 3),
            "torque": torque,
            "residual": torque - link,
        }
    torque = np.asarray(data.get("force"), dtype=float).reshape(-1)
    if torque.size < 6:
        raise RuntimeError("这一拍没有关节力矩。真机需要 publish 里的 force，仿真需要 op hold_torque 的 result。")
    raise RuntimeError("只有测到的力矩，没有该姿态的连杆 g(q) 和法兰雅可比，残差无法从负载里分开。")


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="下发关节目标，到位后采集力矩并辨识法兰负载")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--service", default="piper_sim")
    parser.add_argument("--arm", default="left_arm")
    args = parser.parse_args(argv)
    if args.demo:
        report = demo()
        ok = abs(report["mass_kg"] - DEMO_MASS) < 0.05 and report["hold_delta"] < 0.05 and bool(report["unidentified_fault"])
        report["ok"] = ok
        print(json.dumps(report, ensure_ascii=False))
        return 0 if ok else 2
    from robot.runtime.session import Session

    session = Session.attach(args.service)
    try:
        fit = collect_live(session, args.arm)
    finally:
        session.close()
    print(json.dumps({"mass_kg": fit["mass_kg"], "com_m": fit["com_m"].tolist(), "rms": fit["rms"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
