"""Execute preflighted visual actions through the verified position executor."""
from .vision import Stopped, validate_action


def execute_motion(executor, request_id, target, resolution, kinematics, stop):
    """Execute a fully preflighted path; verify each waypoint, never continue after failure."""
    path = resolution.get("waypoints") if resolution else None
    if not path:
        return executor.execute(request_id, **target), []
    segments = []
    if len(path) > 1:
        print(f"  长步长预检通过，分 {len(path)} 段执行；每段验收后再继续。", flush=True)
    for index, waypoint in enumerate(path):
        if stop.is_set():
            raise Stopped()
        # A hold/external drift between waypoints must not bypass the joint-step bound.
        actual = executor.observe()["joint"]
        try:
            validate_action({"action": "move_joint_absolute", "joint": waypoint["joint"]}, actual)
        except ValueError:
            # Preserve a structured result even if drift invalidates the FIRST
            # waypoint. Confirm a hold of fresh measured joints; never send the
            # now-invalid waypoint or continue the remaining path.
            result = executor.execute(f"{request_id}-revalidation-hold", joint=actual)
            hold_status = result["status"]
            if hold_status == "succeeded":
                result = {**result, "status": "failed", "reason": "waypoint joint revalidation failed"}
            segments.append({"index": index + 1, "status": "rejected", "reason": result["reason"],
                             "hold_status": hold_status})
            break
        result = executor.execute(f"{request_id}-step-{index + 1}", joint=waypoint["joint"])
        if result.get("stop_feedback"):
            pe, re = kinematics.error(result["stop_feedback"]["joint"], waypoint["pose"], resolution["point"])
            if result["status"] == "succeeded" and (pe > .001 or re > .01):
                result.update(status="failed", reason="EEF feedback FK outside tolerance")
        segments.append({"index": index + 1, "status": result["status"], "reason": result["reason"]})
        if result["status"] != "succeeded":
            break
    return result, segments
