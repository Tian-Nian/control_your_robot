"""Pure visual-loop scheduling. Decisions never contain robot motion targets."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Decision:
    kind: str = "continue"
    reason: str = ""
    trigger: str = ""


def decide_next(*, record, observation, stagnation, stage_turns, stage_rounds,
                failures, waits, max_waits, replans, max_replans,
                recovery_plans, max_recovery_plans):
    periodic = stage_turns >= stage_rounds
    feedback = record.get("actual_feedback", {})
    changed = (max((abs(a - b) for a, b in zip(
        feedback.get("joint_rad", observation["joint_rad"]), observation["joint_rad"])), default=0) > .001
        or abs(feedback.get("gripper", observation["gripper"]) - observation["gripper"]) > .01)
    read_only = record["status"] == "queried" and record.get("action", {}).get("action") in (
        "track_point", "forget_track", "plan_motion")
    stalled = bool(stagnation) or failures >= 2 or (periodic and not read_only and not (record["status"] == "succeeded" and changed))
    explicit = record["status"] == "replan_requested"
    recovery = (explicit or waits >= max_waits or
                (max_recovery_plans > 0 and waits > 0 and stalled and replans >= max_replans))
    if recovery:
        if recovery_plans >= max_recovery_plans:
            return Decision("pause", f"等待恢复规划已用尽 {max_recovery_plans} 次，仍未得到可执行策略")
        trigger = ("controller_replan_request" if explicit else "repeated_wait" if waits >= max_waits
                   else "waiting_after_stage_budget")
        return Decision("wait_recovery", "re-evaluate fresh images, robot options and grasp attempts; revise the remaining strategy", trigger)
    if stalled:
        if replans >= max_replans:
            return Decision("pause", "阶段停滞且已达到重新规划上限")
        return Decision("stage", "measured translation cycle; change strategy using the same target" if stagnation
                        else "stage budget or repeated execution rejection/failure")
    if periodic:
        return Decision("stage_review", "periodic review; check visual progress and retain or revise the remaining plan")
    return Decision()


def verification_trigger(record, stage, *, interval, stage_turns, grasp_state):
    """Image checks are independent of controller confidence; never authorize motion.

    A close with a confirmed stop may be image-checked even after a recoverable
    timeout. Unknown/cancelled/unsafe failures must never advance a stage.
    """
    if interval <= 0 or not stage or stage["motion"] not in ("GRASP", "LIFT"):
        return None
    status, action = record["status"], record.get("action", {})
    safe = status == "succeeded" or (status == "failed" and record.get("stop_confirmed")
                                     and record.get("recovery", {}).get("status") == "ready")
    if not safe:
        return None
    if action.get("action") == "set_gripper" and action.get("gripper") == 0:
        return "close_attempt"
    if action.get("action") == "set_gripper" and action.get("gripper") == 1:
        return None
    closed = grasp_state in ("close_attempted_unverified", "lift_visually_verified")
    if closed and record.get("measured_effect", {}).get("delta_base_m", [0, 0, 0])[2] >= .005:
        return "closed_upward_probe"
    if stage_turns % interval == 0 and (closed or stage["motion"] == "LIFT"):
        return "periodic_closed_observation"
    return None
