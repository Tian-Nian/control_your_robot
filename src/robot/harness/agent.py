"""Model-independent bounded skills, durable outcomes, and sequential tasks."""
from copy import deepcopy
import hashlib
import json
import math
import threading

from robot.harness.store import RequestStore


class AgentHarness:
    def __init__(self, executor, *, robot_id, store_path, kinematics=None):
        if not isinstance(robot_id, str) or not robot_id.strip():
            raise ValueError("explicit robot_id required for durable request scope")
        self.executor = executor
        self.kinematics = kinematics
        self.store = RequestStore(store_path)
        profile = {k: v for k, v in executor.capabilities().items() if k != "episode_id"}
        profile["calibration_id"] = getattr(kinematics, "calibration_id", None)
        self.scope = robot_id + ":" + hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest()
        self._busy = threading.Lock()
        self._task = None
        self._model_lock = threading.RLock()

    def tools(self):
        """Chat-completions-style schemas; usable by any model adapter."""
        def tool(name, description, properties, required):
            return {"type": "function", "function": {"name": name, "description": description,
                "parameters": {"type": "object", "properties": properties, "required": required, "additionalProperties": False}}}
        rid = {"request_id": {"type": "string"}}
        result = [tool("observe", "Read fresh robot feedback; no images or object claims", {}, []),
                  tool("get_status", "Read runtime and task state", {}, []),
                  tool("cancel", "Request cancellation of the currently running request", rid, ["request_id"]),
                  tool("recover", "Explicitly verify position hold and acknowledge recovery", {}, []),
                  tool("move_joint_absolute", "Move absolute joint angles in radians", {**rid, "joint": {"type": "array", "items": {"type": "number"}}}, ["request_id", "joint"]),
                  tool("set_gripper", "Set scalar opening, not a grasp-success claim", {**rid, "gripper": {"type": "number", "minimum": 0, "maximum": 1}}, ["request_id", "gripper"])]
        if self.kinematics:
            result.append(tool("move_eef", "Move base-frame flange/tool pose (xyz metres + unit wxyz)",
                {**rid, "pose": {"type": "array", "items": {"type": "number"}, "minItems": 7, "maxItems": 7},
                 "frame": {"type": "string", "enum": ["base"]}, "point": {"type": "string", "enum": ["flange", "tool"]}},
                ["request_id", "pose", "frame", "point"]))
        return result

    def dispatch(self, name, arguments):
        self._validate(name, arguments)
        if name == "observe":
            feedback = self.executor.observe()
            if self.kinematics:
                # Local FK is not an independently measured external pose.
                with self._busy_model():
                    feedback["eef_fk"] = self.kinematics.pose(feedback["joint"])
                    feedback["calibration_id"] = self.kinematics.calibration_id
            return feedback
        if name == "get_status":
            status = self.executor.get_status()
            requests = self.store.unresolved(self.scope)
            tasks = self.store.unresolved(self.scope + ":tasks")
            return {**status, "requires_recovery": bool(status["requires_recovery"] or ((requests or tasks) and not self._busy.locked())),
                    "task": deepcopy(self._task), "unresolved_requests": requests, "unresolved_tasks": tasks}
        if name == "cancel":
            return {"cancel_requested": self.executor.cancel(arguments["request_id"])}
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("agent harness busy")
        try:
            if name == "recover":
                result = self.executor.recover()
                self.store.acknowledge(self.scope)
                self.store.acknowledge(self.scope + ":tasks")
                return result
            if self.store.unresolved(self.scope + ":tasks"):
                raise RuntimeError("explicit recovery required for interrupted task")
            return self._execute(name, arguments)
        finally:
            self._busy.release()

    def _validate(self, name, arguments):
        definitions = {t["function"]["name"]: t["function"]["parameters"] for t in self.tools()}
        if name not in definitions:
            raise ValueError("unsupported skill")
        schema = definitions[name]
        if not isinstance(arguments, dict) or set(arguments) != set(schema["required"]):
            raise ValueError("skill fields do not match schema")
        for key, value in arguments.items():
            rule = schema["properties"][key]
            if rule["type"] == "string":
                if not isinstance(value, str) or not value.strip() or ("enum" in rule and value not in rule["enum"]):
                    raise ValueError(f"invalid {key}")
            else:
                values = value if rule["type"] == "array" else [value]
                if not isinstance(values, list) or not values or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                    raise ValueError(f"finite numeric {key} required")
                if key == "joint" and len(values) != len(self.executor.lower):
                    raise ValueError("wrong joint dimension")
                if key == "joint" and any(v < low or v > high for v, low, high in zip(values, self.executor.lower, self.executor.upper)):
                    raise ValueError("joint target outside limits")
                if key == "pose" and (len(values) != 7 or abs(sum(v*v for v in values[3:]) ** .5 - 1) > 1e-5):
                    raise ValueError("pose requires xyz + unit wxyz")
                if key == "gripper" and not 0 <= value <= 1:
                    raise ValueError("gripper outside [0, 1]")

    def _busy_model(self):
        return self._model_lock

    def _execute(self, name, arguments):
        rid = arguments["request_id"]
        signature = json.dumps({"skill": name, "arguments": arguments}, sort_keys=True, allow_nan=False)
        cached = self.store.lookup(self.scope, rid, signature)
        if cached is not None:
            return cached
        if self.store.unresolved(self.scope) or self.executor.get_status()["requires_recovery"]:
            raise RuntimeError("explicit recovery required before a new request")
        self.store.begin(self.scope, rid, signature)
        result = {"request_id": rid, "status": "unknown", "reason": "execution interrupted"}
        attempted = False
        try:
            fresh = self.executor.observe()
            if name == "move_eef":
                with self._busy_model():
                    joint = self.kinematics.solve(fresh["joint"], arguments["pose"],
                                                  frame=arguments["frame"], point=arguments["point"])
                payload = {"joint": joint}
            elif name == "move_joint_absolute":
                payload = {"joint": arguments["joint"]}
            else:
                payload = {"gripper": arguments["gripper"]}
            if "joint" in payload and any(v < lo or v > hi for v, lo, hi in zip(payload["joint"], self.executor.lower, self.executor.upper)):
                raise ValueError("IK target outside joint limits")
            attempted = True
            result = self.executor.execute(rid, **payload)
            result["skill"] = name
            result["input_observation_id"] = fresh["observation_id"]
            if name == "move_eef" and result["status"] == "succeeded":
                with self._busy_model():
                    pos_error, rot_error = self.kinematics.error(result["stop_feedback"]["joint"], arguments["pose"], arguments["point"])
                result["eef_verification"] = dict(position_error_m=pos_error, rotation_error_rad=rot_error,
                                                  calibration_id=self.kinematics.calibration_id, source="joint feedback + local FK")
                if pos_error > .001 or rot_error > .01:
                    result.update(status="failed", reason="EEF verification outside tolerance")
        except Exception as exc:
            result.update(status="unknown" if attempted else "rejected", reason=str(exc))
        finally:
            self.store.finish(self.scope, rid, result)
        return result

    def run_task(self, task_id, steps):
        """Finite explicit plan. No automatic retries or implicit grasp success."""
        if not isinstance(task_id, str) or not task_id.strip() or not isinstance(steps, list) or not 1 <= len(steps) <= 32:
            raise ValueError("task_id and 1..32 explicit steps required")
        for step in steps:
            if not isinstance(step, dict) or set(step) != {"skill", "arguments"}:
                raise ValueError("step requires skill and arguments")
            if step["skill"] not in ("move_joint_absolute", "set_gripper", "move_eef"):
                raise ValueError("task steps must be movement skills")
            if not isinstance(step["arguments"], dict) or "request_id" in step["arguments"]:
                raise ValueError("task owns step request IDs")
            self._validate(step["skill"], {**step["arguments"], "request_id": "preflight"})
        signature = json.dumps(steps, sort_keys=True, allow_nan=False)
        scope = self.scope + ":tasks"
        cached = self.store.lookup(scope, task_id, signature)
        if cached is not None:
            return cached
        if not self._busy.acquire(blocking=False):
            raise RuntimeError("agent harness busy")
        try:
            if self.store.unresolved(scope) or self.store.unresolved(self.scope):
                raise RuntimeError("explicit recovery required before a new task")
            self.store.begin(scope, task_id, signature)
            self._task = {"task_id": task_id, "status": "running", "step": 0, "results": []}
            try:
                for index, step in enumerate(steps):
                    self._task["step"] = index
                    args = dict(step["arguments"])
                    args["request_id"] = f"task:{task_id}:{index}"
                    result = self._execute(step["skill"], args)
                    self._task["results"].append(result)
                    if result["status"] != "succeeded":
                        self._task["status"] = result["status"]
                        break
                else:
                    self._task["status"] = "succeeded"
            except BaseException:
                self._task["status"] = "unknown"
                raise
            finally:
                self.store.finish(scope, task_id, self._task)
            return deepcopy(self._task)
        finally:
            self._busy.release()
