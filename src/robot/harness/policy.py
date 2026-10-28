"""Image-space decisions for the visual loop.

The model names a direction, a pixel, or a stage. This module turns that into the
existing bounded end-effector delta, or into one base-frame point. It never writes
a robot target and never reads simulator object truth.
"""
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from robot.harness.vision import resolve_action, validate_action

PROMPTS = Path(__file__).with_name("prompts")

DIRECTIONS = {
    "forward": [0.01, 0.0, 0.0],
    "back": [-0.01, 0.0, 0.0],
    "left": [0.0, 0.01, 0.0],
    "right": [0.0, -0.01, 0.0],
    "up": [0.0, 0.0, 0.01],
    "down": [0.0, 0.0, -0.01],
}
TOOL_DIRECTIONS = {
    f"{axis}{suffix}": [0.01 * sign if i == index else 0.0 for i in range(3)]
    for index, axis in enumerate("xyz") for suffix, sign in (("+", 1), ("-", -1))
}
STAGE_MOTIONS = {"APPROACH", "ALIGN", "DESCEND", "GRASP", "LIFT", "PLACE", "RELEASE", "RETREAT"}


def load_prompt(name):
    return (PROMPTS / f"{name}.txt").read_text(encoding="utf-8").strip()


def _reason(action):
    reason = action.get("reason", "")
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 200:
        raise ValueError("reason must be a nonempty string of at most 200 characters")
    return reason.strip()


def parse_action(action):
    """Check an image-space action. Returns a copy; does not move anything."""
    if not isinstance(action, dict) or "action" not in action:
        raise ValueError("JSON object with action required")
    _reason(action)
    name = action["action"]
    if not isinstance(name, str):
        raise ValueError("action must be a string")
    allowed = {"action", "reason"}
    if name == "move":
        allowed.add("direction")
        if not isinstance(action.get("direction"), str) or action["direction"] not in DIRECTIONS:
            raise ValueError("direction must be forward, back, left, right, up, or down")
    elif name == "tool_move":
        allowed.add("direction")
        if not isinstance(action.get("direction"), str) or action["direction"] not in TOOL_DIRECTIONS:
            raise ValueError("tool direction must be x+, x-, y+, y-, z+, or z-")
    elif name == "tool_rotate":
        allowed.update({"axis", "sign"})
        if action.get("axis") not in ("x", "y", "z") or type(action.get("sign")) is not int or action["sign"] not in (-1, 1):
            raise ValueError("tool rotation requires axis x/y/z and integer sign -1 or 1")
    elif name == "joint_step":
        allowed.update({"joint_index", "sign"})
        if (type(action.get("joint_index")) is not int or not 1 <= action["joint_index"] <= 6
                or type(action.get("sign")) is not int or action["sign"] not in (-1, 1)):
            raise ValueError("joint_step requires joint_index 1..6 and integer sign -1 or 1")
    elif name == "rotate":
        allowed.add("sense")
        if action.get("sense") not in ("down", "cw", "ccw"):
            raise ValueError("sense must be down, cw, or ccw")
    elif name == "set_gripper":
        allowed.add("gripper")
        if type(action.get("gripper")) not in (int, float) or action["gripper"] not in (0, 1):
            raise ValueError("gripper must be 0 (closed) or 1 (open)")
    elif name == "point":
        allowed.update({"camera", "pixel"})
        if action.get("camera") not in ("head", "wrist"):
            raise ValueError("camera must be head or wrist")
        _pixel_values(action.get("pixel"))
        if "second" in action:
            allowed.add("second")
            second = action["second"]
            if (not isinstance(second, dict) or set(second) != {"camera", "pixel"}
                    or second["camera"] not in ("head", "wrist")
                    or second["camera"] == action["camera"]):
                raise ValueError("second requires a different camera and pixel")
            _pixel_values(second["pixel"])
    elif name == "track_point":
        allowed.update({"camera", "image_id", "pixel", "label"})
        if action.get("camera") not in ("head", "wrist"):
            raise ValueError("track camera must be head or wrist")
        _pixel_values(action.get("pixel"))
        if not isinstance(action.get("label"), str) or not 0 < len(action["label"].strip()) <= 80:
            raise ValueError("track label must be 1..80 characters")
    elif name == "forget_track":
        allowed.add("track_id")
    elif name == "plan_motion":
        allowed.update({"observation_id", "frame", "delta_cm"})
        if action.get("frame") not in ("base", "tool", "head_camera", "wrist_camera"):
            raise ValueError("plan frame must be base/tool/head_camera/wrist_camera")
        for key in ("delta_cm", "rot_deg"):
            if key == "rot_deg" and key not in action:
                continue
            if key == "rot_deg":
                allowed.add(key)
            values = action.get(key)
            if (not isinstance(values, list) or len(values) != 3
                    or any(type(v) not in (int, float) or not np.isfinite(v) for v in values)):
                raise ValueError(f"{key} requires three finite numeric values")
        size = float(np.linalg.norm(action["delta_cm"]))
        angle = float(np.linalg.norm(np.deg2rad(action.get("rot_deg", [0, 0, 0]))))
        if angle > .150000001 or not (1.-1e-7 <= size <= 5.+1e-7 or size <= 1e-8 and angle > 1e-6):
            raise ValueError("plan translation norm must be 1..5 cm or pure rotation; rotation <=0.15 rad")
        if "anchor_track_id" in action:
            allowed.add("anchor_track_id")
    elif name == "execute_plan":
        allowed.update({"plan_id", "observation_id"})
    elif name not in ("wait", "done", "replan"):
        raise ValueError("unsupported visual action")
    if name in ("move", "tool_move") and "step_cm" in action:
        allowed.add("step_cm")
        step = action["step_cm"]
        if type(step) not in (int, float) or not np.isfinite(step) or not 1 <= step <= 5:
            raise ValueError("step_cm must be a finite number from 1 to 5 (centimetres)")
    for key in ("image_id", "observation_id", "track_id", "anchor_track_id", "plan_id"):
        if key in allowed and (not isinstance(action.get(key), str) or not 0 < len(action[key]) <= 160):
            raise ValueError(f"{key} must be a nonempty bounded string")
    if set(action) != allowed:
        raise ValueError("unexpected visual action fields")
    return dict(action)


def interpret(action, joint, kinematics, *, cameras=None):
    """Map one visual action to an executor payload, a point query, or no motion.

    ``done``, ``wait`` and ``replan`` return an empty payload. ``point`` returns None and puts
    the measured point in the resolution. A mapping that fails IK raises ValueError
    before any target exists. For EEF motion, resolution["waypoints"] must be
    executed and verified in order; the returned target is the FINAL waypoint,
    not a replacement for the preflighted path.
    """
    action = parse_action(action)
    name = action["action"]
    if name in ("track_point", "forget_track", "plan_motion", "execute_plan"):
        raise ValueError("stateful visual tool requires the active grounded session")
    if name in ("wait", "done", "replan"):
        return {}, {"visual_action": name}
    if name == "point":
        second = action.get("second")
        names = [action["camera"]] + ([second["camera"]] if second else [])
        now = time.time_ns()
        stamps = {}
        for camera_name in names:
            snapshot = cameras.get(camera_name) if isinstance(cameras, dict) else None
            if not isinstance(snapshot, dict):
                raise ValueError(f"pictured camera snapshot {camera_name} is missing or invalid")
            stamp = snapshot.get("timestamp_ns")
            if type(stamp) is not int or stamp <= 0:
                raise ValueError(f"pictured camera snapshot {camera_name} timestamp_ns is missing or invalid")
            if stamp > now:
                raise ValueError(f"pictured camera snapshot {camera_name} has a future timestamp_ns")
            stamps[camera_name] = stamp
        # observation() checked freshness at acquisition. Inference latency does
        # not invalidate a read-only query of that exact snapshot; never refresh it.
        point = pixel_to_base(cameras, action["camera"], action["pixel"],
                              second=(second["camera"], second["pixel"]) if second else None)
        return None, {"visual_action": "point", **point, "source": "captured_snapshot",
                      "evidence": "historical", "live_motion_target": False,
                      "image_timestamps_ns": stamps, "queried_at_ns": now,
                      "image_age_ms_at_query": {name: (now - stamp) / 1e6 for name, stamp in stamps.items()}}
    if name == "set_gripper":
        return validate_action({"action": "set_gripper", "gripper": action["gripper"]}, joint), {"visual_action": "set_gripper"}
    if name == "joint_step":
        q = list(joint)
        q[action["joint_index"] - 1] += .04 * action["sign"]
        return validate_action({"action": "move_joint_absolute", "joint": q}, joint), {
            "visual_action": name, "joint_index": action["joint_index"], "delta_rad": .04 * action["sign"]}
    if kinematics is None:
        raise ValueError("visual move and rotate require local kinematics")
    if name == "tool_move":
        delta = (np.asarray(TOOL_DIRECTIONS[action["direction"]]) * action.get("step_cm", 1)).tolist()
        low = {"action": "move_eef_delta", "delta_xyz": delta,
               "frame": "tool", "point": "tool"}
    elif name == "tool_rotate":
        rotvec = [0.04 * action["sign"] if a == action["axis"] else 0.0 for a in "xyz"]
        low = {"action": "move_eef_delta", "delta_xyz": [0.0, 0.0, 0.0],
               "delta_rotvec": rotvec, "frame": "tool", "point": "tool"}
    elif name == "move":
        delta = (np.asarray(DIRECTIONS[action["direction"]]) * action.get("step_cm", 1)).tolist()
        low = {"action": "move_eef_delta", "delta_xyz": delta, "frame": "base", "point": "tool"}
    else:
        rotvec = _rotation(action["sense"], joint, kinematics)
        low = {"action": "move_eef_delta", "delta_xyz": [0.0, 0.0, 0.0],
               "delta_rotvec": rotvec, "frame": "base", "point": "tool"}
    resolved, resolution = resolve_action(low, joint, kinematics)
    target = validate_action(resolved, joint, kinematics)
    resolution["visual_action"] = name
    for key in ("direction", "step_cm", "sense", "axis", "sign"):
        if key in action:
            resolution[key] = action[key]
    return target, resolution


def wrist_tool_guide(joint, kinematics, cameras):
    """Robot-only image reference. Never samples depth or object pixels.

    The camera and FK belong to the pictured snapshot. Optical axes are RDF.
    Axes describe directions, not a metric image servo or a grasp detector.
    """
    result = {"available": False, "source": "pictured robot FK + wrist calibration; no scene depth"}
    if kinematics is None:
        return result
    try:
        camera = _camera(cameras, "wrist")
        tool = _pose(kinematics.pose(joint, "tool"))
        camera_from_tool = np.linalg.inv(camera["world_from_camera"]) @ tool
        p = camera_from_tool[:3, 3]
        result.update(available=True, image_timestamp_ns=cameras["wrist"].get("timestamp_ns"),
                      optical_axes="+X image-right, +Y image-down, +Z away from camera",
                      tool_axes_optical={a: camera_from_tool[:3, i].tolist() for i, a in enumerate("xyz")},
                      finger_opening_axis_tool="Y (opposing fingers); tool +Z is the approach axis",
                      tool_origin_pixel=None, tool_origin_in_image=False)
        if p[2] > 1e-6:
            uv = camera["intrinsics"] @ p
            u, v = (uv[:2] / uv[2]).tolist()
            height, width = camera["rgb_shape"]
            result.update(tool_origin_pixel=[u, v], tool_origin_in_image=0 <= u < width and 0 <= v < height)
        return result
    except (ValueError, TypeError, KeyError, np.linalg.LinAlgError):
        # A missing calibration must not silently invent image directions.
        return {**result, "available": False, "reason": "wrist robot reference unavailable; use observed image changes"}


def _rotation(sense, joint, kinematics):
    if sense == "down":
        pose = np.asarray(kinematics.pose(joint, "tool"))
        axis = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()[:, 2]
        down = np.array([0.0, 0.0, -1.0])
        cross = np.cross(axis, down)
        length = float(np.linalg.norm(cross))
        if length < 1e-6:
            if float(axis @ down) > 0:
                raise ValueError("tool axis is already down")
            cross, length = np.array([1.0, 0.0, 0.0]), 1.0
        angle = float(np.arccos(np.clip(float(axis @ down), -1.0, 1.0)))
        return (cross / length * min(0.04, angle)).tolist()
    pose = np.asarray(kinematics.pose(joint, "tool"))
    tool_z = Rotation.from_quat(pose[[4, 5, 6, 3]]).as_matrix()[:, 2]
    sign = 1.0 if sense == "ccw" else -1.0
    return (tool_z * sign * 0.04).tolist()


def pixel_to_base(cameras, name, pixel, *, second=None):
    """One pixel on one depth frame, or two corresponding pixels, to one base point.

    ``cameras`` maps a camera name to depth_m, intrinsics, and world_from_camera.
    The caller supplies only the frame the model pointed at. No object truth.
    Published depth is the camera optical-axis Z distance, not the ray length.
    """
    if second is not None:
        return _triangulate(cameras, name, pixel, second)
    camera = _camera(cameras, name, require_depth=True)
    u, v = _pixel(pixel, camera["depth_m"].shape)
    depth = float(camera["depth_m"][v, u])
    if not np.isfinite(depth) or depth <= 0 or depth > 5:
        raise ValueError("no finite depth at the selected pixel")
    point = camera["origin"] + camera["rotation"] @ _camera_point(camera["intrinsics"], u, v, depth)
    if not np.isfinite(point).all():
        raise ValueError("pixel projection is not finite")
    return {"point_base_m": point.tolist(), "method": "depth", "camera": name,
            "pixel": [u, v], "depth_m": depth}


def _triangulate(cameras, first_name, first_pixel, second):
    other_name, other_pixel = second
    if first_name == other_name:
        raise ValueError("triangulation requires two different cameras")
    first, second_camera = _camera(cameras, first_name), _camera(cameras, other_name)
    a, direction_a = _ray(first, _pixel(first_pixel, first["rgb_shape"]))
    b, direction_b = _ray(second_camera, _pixel(other_pixel, second_camera["rgb_shape"]))
    cross = np.cross(direction_a, direction_b)
    if float(np.linalg.norm(cross)) < 0.01:
        raise ValueError("pixel rays are nearly parallel")
    matrix = np.column_stack((direction_a, -direction_b))
    try:
        scale, _, _, _ = np.linalg.lstsq(matrix, b - a, rcond=None)
    except np.linalg.LinAlgError as exc:
        raise ValueError("pixel rays could not be triangulated") from exc
    if float(scale[0]) <= 0 or float(scale[1]) <= 0:
        raise ValueError("pixel rays do not meet in front of both cameras")
    p = a + direction_a * float(scale[0])
    q = b + direction_b * float(scale[1])
    gap = float(np.linalg.norm(p - q))
    if not np.isfinite(gap) or gap > 0.02:
        raise ValueError("pixel rays disagree by more than 2 cm")
    point = (p + q) / 2
    if not np.isfinite(point).all():
        raise ValueError("pixel projection is not finite")
    return {"point_base_m": point.tolist(), "method": "two_view", "residual_m": gap,
            "cameras": [first_name, other_name], "pixels": [
                list(_pixel(first_pixel, first["rgb_shape"])),
                list(_pixel(other_pixel, second_camera["rgb_shape"]))]}


def _camera(cameras, name, *, require_depth=False):
    try:
        camera = cameras[name]
        # Stereo needs only RGB dimensions and calibration; even malformed depth is ignored.
        depth = np.asarray(camera["depth_m"], dtype=float) if require_depth else None
        intrinsics = np.asarray(camera["intrinsics"], dtype=float)
        transform = _pose(camera["world_from_camera"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"camera {name} has no local depth and calibration") from exc
    if (intrinsics.shape != (3, 3) or not np.isfinite(intrinsics).all()
            or not np.allclose(intrinsics[2], [0, 0, 1], atol=1e-9)
            or abs(intrinsics[0, 1]) > 1e-9 or abs(intrinsics[1, 0]) > 1e-9):
        raise ValueError(f"camera {name} calibration is invalid")
    if float(intrinsics[0, 0]) <= 1e-9 or float(intrinsics[1, 1]) <= 1e-9:
        raise ValueError(f"camera {name} focal length is invalid")
    if not np.isfinite(transform).all():
        raise ValueError(f"camera {name} pose is invalid")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError(f"camera {name} rotation is invalid")
    rgb_shape = camera.get("rgb_shape")
    if rgb_shape is None and "rgb" in camera:
        rgb_shape = np.asarray(camera["rgb"]).shape[:2]
    if depth is None:
        if rgb_shape is None:
            raise ValueError(f"camera {name} has no image dimensions")
        shape = tuple(rgb_shape)
    else:
        if depth.ndim != 2:
            raise ValueError(f"camera {name} depth is invalid")
        shape = depth.shape
        if rgb_shape is not None and tuple(rgb_shape) != shape:
            raise ValueError(f"camera {name} image and depth shapes disagree")
    if len(shape) != 2 or any(type(v) not in (int, np.int64, np.int32) or v <= 0 for v in shape):
        raise ValueError(f"camera {name} image dimensions are invalid")
    return {"depth_m": depth, "intrinsics": intrinsics, "world_from_camera": transform,
            "origin": transform[:3, 3], "rotation": rotation, "rgb_shape": shape}


def _pose(value):
    raw = np.asarray(value, dtype=float)
    if not np.isfinite(raw).all():
        raise ValueError("camera pose must be finite")
    if raw.shape == (4, 4):
        if not np.allclose(raw[3], [0, 0, 0, 1], atol=1e-9):
            raise ValueError("camera homogeneous pose is invalid")
        return raw
    if raw.shape != (7,) or abs(np.linalg.norm(raw[3:]) - 1) > 1e-5:
        raise ValueError("camera pose must be a matrix or xyz + unit wxyz")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(raw[[4, 5, 6, 3]]).as_matrix()
    matrix[:3, 3] = raw[:3]
    return matrix


def _pixel_values(pixel):
    if (not isinstance(pixel, list) or len(pixel) != 2
            or any(type(v) not in (int, float) or isinstance(v, bool) or not np.isfinite(v) or v < 0 for v in pixel)):
        raise ValueError("pixel requires two nonnegative numeric values [u, v]")
    return pixel


def _pixel(pixel, shape):
    _pixel_values(pixel)
    height, width = shape
    if not 0 <= pixel[0] <= width - 1 or not 0 <= pixel[1] <= height - 1:
        raise ValueError("pixel is outside the camera image")
    u, v = int(round(float(pixel[0]))), int(round(float(pixel[1])))
    return u, v


def _camera_point(intrinsics, u, v, depth):
    return np.array([(u - intrinsics[0, 2]) / intrinsics[0, 0] * depth,
                     (v - intrinsics[1, 2]) / intrinsics[1, 1] * depth, depth])


def _ray(camera, pixel):
    u, v = pixel
    local = _camera_point(camera["intrinsics"], u, v, 1.0)
    direction = camera["rotation"] @ local
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("pixel ray is invalid")
    return camera["origin"], direction / norm


def parse_plan(payload, *, task="task", diagnostics=None):
    """Accept a planner object, or degrade to one approach stage. Never invents motion."""
    text = str(task).strip() or "task"
    fallback = [{"id": "task", "target": text[:200], "affordance": "visible task region",
                 "motion": "APPROACH", "description": "Approach using the images.",
                 "completion": text[:200]}]
    try:
        if isinstance(payload, str):
            payload = json.loads(payload)
        if (not isinstance(payload, dict) or not {"stages"} <= set(payload)
                or set(payload) - {"stages", "reason"}):
            raise ValueError
        if "reason" in payload:
            _reason(payload)
        stages = payload["stages"]
        if not isinstance(stages, list) or not 1 <= len(stages) <= 6:
            raise ValueError
        clean, seen = [], set()
        for stage in stages:
            fields = ("id", "target", "affordance", "motion", "description", "completion")
            if not isinstance(stage, dict) or set(stage) != set(fields):
                raise ValueError
            if any(not isinstance(stage[key], str) for key in fields):
                raise ValueError
            item = {key: stage[key].strip() for key in fields}
            if any(not item[key] or len(item[key]) > 200 for key in fields):
                raise ValueError
            if item["motion"] not in STAGE_MOTIONS or item["id"] in seen:
                raise ValueError
            seen.add(item["id"])
            clean.append(item)
        if diagnostics is not None:
            diagnostics.update(status="accepted", stage_count=len(clean))
        return clean
    except (TypeError, ValueError, KeyError, json.JSONDecodeError):
        if diagnostics is not None:
            diagnostics.update(status="fallback", reason="invalid or unavailable planner response", stage_count=1)
        return fallback


def parse_verification(payload):
    """A verifier answer is evidence about an image, not permission to move."""
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, dict) or set(payload) != {"complete", "reason"}:
        raise ValueError("verifier must return complete and reason")
    if type(payload["complete"]) is not bool:
        raise ValueError("complete must be boolean")
    reason = payload["reason"]
    if not isinstance(reason, str) or not reason.strip() or len(reason) > 200:
        raise ValueError("verifier reason must be a short string")
    return {"complete": payload["complete"], "reason": reason.strip(), "source": "image_verification"}
