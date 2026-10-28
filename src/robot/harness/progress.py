"""Bounded episode memory from executed actions and image checks, never scene truth.

The target is a textual selection, not a visual tracker. Stagnation means measured
robot motion returned near its starting point; it is not a grasp/progress verdict.
"""
import copy
import numpy as np


class EpisodeProgress:
    def __init__(self):
        self.target = None
        self.recent = []
        self.completed = []
        self.replan_reason = "initial plan"
        self.grasp_state = "not_attempted"
        self.stagnation = None
        self.plan_start_round = 0
        self.recovery_context = None
        self.attempts = []
        self.attempt_count = 0
        self.last_check = None

    def begin_plan(self):
        self.plan_start_round = self.recent[-1]["round"] if self.recent else 0
        self.stagnation = None

    def adopt_plan(self, stages):
        """Keep the same textual grasp selection across replans in this episode."""
        stages = copy.deepcopy(stages)
        grasp = next((s for s in stages if s["motion"] == "GRASP"), None)
        if self.target is None and grasp is not None:
            self.target = {"target": grasp["target"], "affordance": grasp["affordance"]}
        if self.target:
            grasp_count = 0
            for stage in stages:
                if stage["motion"] == "GRASP":
                    grasp_count += 1
                if grasp_count > 1:
                    break  # Later objects retain their own selections.
                if stage["motion"] in ("GRASP", "LIFT"):
                    stage["target"] = self.target["target"]
                    stage["affordance"] = self.target["affordance"]
        return stages

    def enter_stage(self, stage):
        if stage and stage["motion"] == "GRASP":
            selected = {"target": stage["target"], "affordance": stage["affordance"]}
            if selected != self.target:
                self.grasp_state = "not_attempted"
                self.attempts = []
                self.attempt_count = 0
                self.last_check = None
            self.target = selected

    def record(self, record, stage, round_index):
        item = {"round": round_index, "stage_id": stage["id"] if stage else None,
                **{k: copy.deepcopy(record[k]) for k in (
                    "action", "status", "reason", "actual_feedback", "eef_verification",
                    "recovery", "action_resolution", "measured_effect", "verification", "verification_trigger", "tool_result") if k in record}}
        if isinstance(item.get("action_resolution"), dict):
            item["action_resolution"].pop("waypoints", None)  # Full preflight lives in the local audit, not model memory.
        self.recent = (self.recent + [item])[-8:]
        action = record.get("action")
        if isinstance(action, dict) and action.get("action") == "set_gripper":
            # Commands are attempts even if verification times out against an object.
            if record.get("status") in ("succeeded", "failed"):
                self.grasp_state = "close_attempted_unverified" if action["gripper"] == 0 else "opened_unverified"
        if isinstance(action, dict) and action.get("action") == "set_gripper" and record.get("status") in ("succeeded", "failed"):
            if action["gripper"] == 0:
                if not self.attempts or self.attempts[-1]["ended_by_open"]:
                    self.attempt_count += 1
                    self.attempts = (self.attempts + [{"attempt": self.attempt_count, "close_round": round_index,
                        "close_commands": 0, "ended_by_open": False, "closed_translation_m": [0., 0., 0.],
                        "upward_travel_m": 0., "last_image_check": None}])[-4:]
                self.attempts[-1]["close_commands"] += 1
            elif self.attempts:
                self.attempts[-1]["ended_by_open"] = True
                self.attempts[-1]["open_round"] = round_index
        if self.attempts and not self.attempts[-1]["ended_by_open"]:
            delta = np.asarray(record.get("measured_effect", {}).get("delta_base_m", [0., 0., 0.]), dtype=float)
            if delta.shape == (3,) and np.isfinite(delta).all():
                current = self.attempts[-1]
                current["closed_translation_m"] = (np.asarray(current["closed_translation_m"]) + delta).tolist()
                current["upward_travel_m"] += max(0., float(delta[2]))
        check = record.get("verification", {})
        if check:
            self.last_check = {"round": round_index, "stage_id": stage["id"] if stage else None,
                               "complete": check["complete"], "reason": check["reason"]}
            if self.attempts and not self.attempts[-1]["ended_by_open"]:
                self.attempts[-1]["last_image_check"] = copy.deepcopy(self.last_check)

        if check.get("complete") and stage:
            self.completed = (self.completed + [{"stage": copy.deepcopy(stage),
                                "round": round_index, "evidence": check["reason"]}])[-6:]
            if stage["motion"] == "LIFT":
                self.grasp_state = "lift_visually_verified"
        # Four successful measured translations with substantial travel but near-zero
        # net displacement. Rotations, failures, queries and gripper changes break it.
        # Read-only previews/track bookkeeping must not hide a motion cycle.
        window = [r for r in self.recent if not (r["status"] == "queried" and
                  isinstance(r.get("action"), dict) and r["action"].get("action") in
                  ("plan_motion", "track_point", "forget_track"))][-4:]
        self.stagnation = None
        if (len(window) != 4 or len({r["stage_id"] for r in window}) != 1
                or window[0]["round"] <= self.plan_start_round):
            return
        deltas = []
        for row in window:
            a = row.get("action", {})
            effect = row.get("measured_effect", {})
            if (not isinstance(a, dict) or a.get("action") not in ("move", "tool_move", "execute_plan")
                    or row["status"] != "succeeded" or "delta_base_m" not in effect):
                return
            if a.get("action") == "execute_plan" and np.linalg.norm(
                    row.get("action_resolution", {}).get("delta_rotvec_base_rad", [0, 0, 0])) > 1e-6:
                return
            deltas.append(effect["delta_base_m"])
        values = np.asarray(deltas, dtype=float)
        if values.shape != (4, 3) or not np.isfinite(values).all():
            return
        travel = float(np.linalg.norm(values, axis=1).sum())
        net = float(np.linalg.norm(values.sum(axis=0)))
        if travel >= .03 and net < .003:
            self.stagnation = {"kind": "measured_translation_cycle", "rounds": [r["round"] for r in window],
                               "travel_m": travel, "net_displacement_m": net,
                               "meaning": "robot returned near its start; object progress unknown"}

    def snapshot(self, stage=None):
        return copy.deepcopy({"target_selection": self.target,
                "target_selection_source": "planner text, not an object pose or visual track",
                "current_stage": stage, "completed_stages": self.completed,
                "recent_actions": self.recent, "grasp_state": self.grasp_state,
                "stagnation": self.stagnation, "replan_reason": self.replan_reason,
                "recovery_context": self.recovery_context,
                "grasp_attempts": self.attempts, "attempt_count": self.attempt_count,
                "attempts_source": "executed command history and robot FK deltas, not object motion or contact",
                "last_image_check": self.last_check})
