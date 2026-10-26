"""Fixed-scene physical pick/release check, not a general-purpose pick skill.

Uses simulation ground truth. No welds, body teleportation or model calls.
Only invoked explicitly by the optional MuJoCo harness example.
"""
import time
import numpy as np
from robot.runtime.lease import ControlLease
from robot.harness.stacking import grasp_evidence


def run_pick_check(session):
    from robot.controller.PiperMuJoCo_controller import PiperMuJoCoController

    def read():
        frame = session.read_raw("left_arm")
        if frame is None or not 0 <= (time.time_ns() - frame["timestamp_ns"]) / 1e6 < 500:
            raise RuntimeError("missing/stale simulation feedback")
        state = frame["data"]
        if state.get("servo_fault") or "objects" not in state:
            raise RuntimeError("requires healthy MuJoCo scene feedback")
        return state

    initial = read()
    if np.linalg.norm(np.asarray(initial["objects"]["bowl_0"]["pose"][:2]) - [.42, -.12]) > .01:
        raise ValueError("fixed pick check requires initial bowl_0 at (0.42, -0.12); restart the scene")
    initial_z = float(initial["objects"]["bowl_0"]["pose"][2])
    solver = PiperMuJoCoController(cameras=False)
    solver.set_up()
    evidence = []
    lease = None
    try:
        lease = ControlLease(session, "model", 2.0)
        release = session.call("left_arm", {"op": "release"})
        if not release["accepted"] or release["error"]:
            raise RuntimeError("could not release verified prior hold")

        def phase(name, q, gripper, duration):
            before = read()
            receipt = session.write_target("left_arm", {"joint": np.asarray(q).tolist(), "gripper": gripper}, valid_for_s=2.8)
            if not receipt["accepted"]:
                raise RuntimeError(receipt["error"])
            deadline = time.monotonic() + 8
            refresh = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                if not lease.alive:
                    raise RuntimeError("lease lost")
                state = read()
                if (state["sim_time"] - before["sim_time"] >= duration
                        and np.max(np.abs(np.asarray(state["joint"]) - q)) <= .025):
                    evidence.append({"phase": name, "sim_time": float(state["sim_time"]),
                                     **grasp_evidence(state, "bowl_0", initial_z)})
                    return state
                if time.monotonic() >= refresh:
                    # Explicitly renew this live phase, only while fresh feedback is verified.
                    receipt = session.write_target("left_arm", {"joint": np.asarray(q).tolist(), "gripper": gripper}, valid_for_s=2.8)
                    if not receipt["accepted"]:
                        raise RuntimeError(receipt["error"])
                    refresh = time.monotonic() + 1.5
                time.sleep(.02)
            raise RuntimeError(f"{name}: feedback deadline exceeded")

        def ik(xyz, angle):
            current = read()
            # Only the local IK scratch model is seeded; runtime body poses are never changed.
            solver.data.qpos[solver._qadr[:6]] = current["joint"]
            half = np.deg2rad(angle) / 2
            return solver.solve_ik([*xyz, np.cos(half), 0., np.sin(half), 0.], "tool")

        phase("open", initial["joint"], 1., 1.)
        phase("approach", ik([.42, -.04, .17], 145), 1., 3.)
        grasp_q = ik([.42, -.04, .035], 180)
        phase("descend", grasp_q, 1., 3.)
        grasped = phase("close", grasp_q, 0., 2.)
        if not grasp_evidence(grasped, "bowl_0", initial_z)["bilateral_contact"]:
            raise RuntimeError("closing did not produce bilateral bowl contact")
        lift_q = ik([.42, -.04, .17], 145)
        lifted = phase("lift", lift_q, 0., 3.)
        if not grasp_evidence(lifted, "bowl_0", initial_z)["grasped_and_lifted"]:
            raise RuntimeError("bowl was not physically grasped and lifted")
        final = phase("release", lift_q, 1., 3.)
        final_evidence = grasp_evidence(final, "bowl_0", initial_z)
        if final_evidence["bilateral_contact"] or abs(final_evidence["lift_m"]) > .01:
            raise RuntimeError("released bowl did not return to tabletop")
        return {"status": "succeeded", "check": "physical_pick_lift_release", "evidence": evidence,
                "source": "simulation_ground_truth", "stacking_completed": False}
    finally:
        try:
            if lease is not None and lease.alive:
                receipt = session.stop_target("left_arm")
                if not receipt["accepted"]:
                    raise RuntimeError("final hold request rejected")
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    status = session.runtime_status("left_arm") or {}
                    if status.get("stream_seq") == receipt["seq"] and status.get("stream_state") == "stopped":
                        break
                    time.sleep(.02)
                else:
                    raise RuntimeError("runtime did not acknowledge final hold")
        finally:
            if lease is not None:
                lease.close()
            solver.stop()
