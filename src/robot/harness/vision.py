"""Explicit RGB/proprioception boundary for a remote visual policy.

The full runtime frame stays local. Only build_messages() output goes to the API.
No object truth, contact data, depth, camera calibration, or scene files are sent.
Optional EEF poses are derived locally from joint feedback through Pinocchio FK.
"""
import base64
import json
import queue
import struct
import threading
import time
import urllib.error
import urllib.request
import zlib

import numpy as np
from scipy.spatial.transform import Rotation

LOWER = [-2.618, 0, -2.697, -1.832, -1.22, -3.14]
UPPER = [2.618, 3.14, 2.697, 1.832, 1.22, 3.14]


def png_url(rgb):
    """Encode uint8 RGB without adding an imaging dependency."""
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or not rgb.size:
        raise ValueError("uint8 HxWx3 RGB required")
    height, width, _ = rgb.shape
    def chunk(kind, data):
        return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", zlib.crc32(kind + data))
    rows = b"".join(b"\0" + row.tobytes() for row in rgb)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


def observation(session, previous=None):
    frame = session.read_raw("left_arm")
    now = time.time_ns()
    if not frame or not 0 <= now - frame["timestamp_ns"] <= 500_000_000:
        raise RuntimeError("robot feedback missing or stale")
    data = frame["data"]
    if data.get("control_mode") != "position" or data.get("servo_fault", True):
        raise RuntimeError("healthy position feedback required")
    q = np.asarray(data["joint"], dtype=float)
    g = np.asarray(data["gripper"], dtype=float).reshape(-1)
    if (q.shape != (6,) or g.shape != (1,) or not np.isfinite(q).all()
            or not np.isfinite(g).all() or not 0 <= g[0] <= 1
            or np.any(q < LOWER) or np.any(q > UPPER)):
        raise RuntimeError("invalid robot feedback")
    images, stamps = {}, {}
    for name in ("head", "wrist"):
        camera = data["cameras"][name]
        stamp = int(camera["timestamp_ns"])
        if not 0 <= now - stamp <= 1_000_000_000:
            raise RuntimeError("camera feedback stale")
        if previous and stamp <= previous[name]:
            raise RuntimeError("camera did not advance")
        images[name] = png_url(camera["rgb"])
        stamps[name] = stamp
    if abs(stamps["head"] - stamps["wrist"]) > 200_000_000:
        raise RuntimeError("camera skew exceeds 200 ms")
    # Construct a new dict; never serialize or copy through the full frame.
    return {"joint_rad": q.tolist(), "gripper": float(g[0]), "rgb": images}, stamps


def build_messages(obs, task, history):
    prompt = (
        "You control a simulated Piper using only head and wrist RGB images and proprioception. "
        "No object poses, contact sensors, depth, or calibrated camera geometry are available. "
        "Choose ONE small action per turn. Output a JSON object only, using one of these action forms "
        "plus the reason field described below: "
        "{\"action\":\"move_joint_absolute\",\"joint\":[six absolute radians]} or "
        "{\"action\":\"set_gripper\",\"gripper\":0.0} or {\"action\":\"wait\"}. "
        "Joint order is 1..6. Each joint may change at most 0.08 rad per turn. "
        "Gripper 0 means closed, 1 open. Opening feedback is not proof of grasp. "
        "Add a short 'reason' string to every action, explaining the decision in Chinese. "
        "For wait, state the specific missing information or capability, or why waiting will help. "
        "If uncertain or finished, wait. Unchanged images alone do not resolve missing capabilities. "
        "Never invent contact or task-success evidence. "
        "No shell, filesystem, network, or other tools are available. "
        f"Joint lower limits: {LOWER}; upper limits: {UPPER}."
    )
    text = {"task": task, "robot": {"joint_rad": obs["joint_rad"], "gripper": obs["gripper"]},
            "recent_actions": history[-8:]}
    if "eef_fk" in obs:
        text["robot"]["eef_fk"] = obs["eef_fk"]
        text["scene_priors"] = {
            "source": "authored scene nominal dimensions and validated baseline, not live object measurements",
            "table_z_m": 0.0, "nominal_bowl_rim_z_m": .05,
            "tool_grasp_z_range_m": [.03, .04], "validated_tool_grasp_z_m": .035,
            "tool_staging_z_range_m": [.12, .17], "flange_to_tool_local_z_m": .13503,
        }
        text["coordinate_guide"] = {
            "frame": "base", "origin": "fixed robot base origin; same frame as tabletop z=0",
            "handedness": "right-handed: X cross Y = Z",
            "positive_axes": {"X": "forward along the initial arm extension into the work area",
                              "Y": "left when standing behind the base and facing +X",
                              "Z": "vertically up, away from tabletop"},
            "one_cm_examples_m": {"forward": [.01,0,0], "backward": [-.01,0,0],
                                  "left": [0,.01,0], "right": [0,-.01,0],
                                  "up": [0,0,.01], "down": [0,0,-.01]},
            "head_view": "fixed camera: gripper +Y motion appears left and -Y right; +X and +Z can both appear upward",
            "wrist_view": "moves and rotates with wrist; image axes are not fixed base axes",
            "source": "robot/frame and qualitative mounting conventions; no object measurements or camera calibration matrices",
        }
        prompt += (
            " You also have move_eef: {\"action\":\"move_eef\",\"pose\":[x,y,z,qw,qx,qy,qz],"
            "\"frame\":\"base\",\"point\":\"tool\",\"reason\":\"short explanation\"}. "
            "point may be tool or flange. Absolute xyz is in metres in the robot base frame; "
            "quaternion is unit wxyz, NOT Euler angles. Base +Z is up; coordinates are not image pixels. "
            "eef_fk contains both poses computed from measured joints by local Pinocchio FK, "
            "not an external sensor or object estimate. tool is the fixed URDF tool point, "
            "not a visually inferred contact point. Prefer tool for manipulation. "
            "Use its current quaternion unchanged for pure translation. Per move_eef, translation "
            "must be between 0.01 and 0.02 m for translation, or zero for rotation-only; orientation change <=0.15 rad. Local IK also limits each joint "
            "change to 0.08 rad. Smaller steps may be needed near singularities. "
            "IK does not plan collision-free paths or localize objects from images."
            "eef_fk.one_cm_translation_options lists six base-axis tool translations with locally checked "
            "IK/joint-step feasibility. Choose a feasible option that fits visual evidence; these options "
            "are NOT collision or object-location checks. If none suits the image, adjust joint posture "
            "instead of repeating the same rejected direction. requested_action records rejected attempts."
        )
        prompt += (
            " Prefer move_eef_delta for small visual corrections: "
            '{"action":"move_eef_delta","delta_xyz":[0,0,0.01],"frame":"base","point":"tool","reason":"..."}. '
            "delta_xyz is a BASE-frame displacement in metres. Optional delta_rotvec is a BASE-frame "
            "right-hand rotation vector in radians, composed as R_new=Exp(delta_rotvec)*R_current. "
            "Omit delta_rotvec to preserve orientation. For rotation-only use delta_xyz=[0,0,0]. "
            "For example delta_xyz=[0,0,0], delta_rotvec=[0,0.04,0] rotates 0.04 rad about base +Y "
            "while holding the selected tool/flange point fixed. Orientation changes may accompany translation. "
            "The norm MUST be exactly 0.01 m (1 cm). Choose direction, not millimetre-sized steps. "
            "This 1 cm rule applies to nonzero translation only; rotation-only is allowed. "
            "Rotation-vector norm must be <=0.15 rad, prefer 0.02 to 0.04 rad so joint changes stay <=0.08 rad. "
            "The local adapter NEVER shrinks the delta; infeasible IK or joint steps are rejected. Inspect action_resolution "
            "in feedback for the executed displacement. Do not repeat an unachievable absolute target. "
            "VISUAL PROCEDURE: use head view for object choice/arm clearance and wrist for local alignment. "
            "Describe visible rim, gripper tips and relative image positions. Image up is NOT base +Z. "
            "Do not infer base-frame movement direction from image direction without evidence. "
            "If direction is unknown but a probe is visually reasonable, choose ONE 1cm translation "
            "at unchanged orientation and compare previous/current images to learn its effect. "
            "Change one axis at a time, then keep/reverse direction based on observed image motion. "
            "Approach in small steps only after alignment improves; do not descend blindly because an object "
            "appears above the gripper in wrist pixels. Use RGB appearance/relative motion only, no depth map. "
            "For a rejected absolute step, use the 1cm delta interface; if that is infeasible, change direction, "
            "use move_joint_absolute to adjust posture, or use rotation-only move_eef. A rejection caused no movement. "
            "Distinguish execution success from grasp success. Closing/opening is not grasp evidence. "
            "In reason, briefly state visible evidence, effect of previous action, next hypothesis and expected "
            "image change. If no defensible movement exists, explain the specific blockage and wait even if "
            "the task asks for perpetual movement. Do not fabricate calibration or task completion."
        )
        prompt += (
            " COORDINATE DIRECTIONS: Commands with frame=base always use a fixed right-handed robot-base frame, "
            "even after rotating the wrist or joint1. +X is forward along the INITIAL arm extension into the "
            "work area; -X is backward. Stand behind the robot looking along +X: +Y is your left, -Y your right; "
            "+Z is vertically up from the tabletop, -Z down. For a 1cm move use forward [0.01,0,0], "
            "backward [-0.01,0,0], left [0,0.01,0], right [0,-0.01,0], up [0,0,0.01], down [0,0,-0.01]. "
            "Left/right here refer to this BASE definition, not your changing wrist view. "
            "point=tool only selects the controlled point/rotation pivot: it DOES NOT change the frame. "
            "Thus a base -Z descent remains vertical after wrist rotation; it is not along the optical axis. "
            "For absolute move_eef, xyz are absolute base coordinates; move_eef_delta adds displacement to "
            "current FK xyz. Do not put an absolute position into delta_xyz. "
            "HEAD IMAGE GUIDE for this fixed mounting: moving the GRIPPER +Y moves it left in the head image, "
            "and -Y right. Forward +X usually moves its projection upward over the work area; lifting +Z "
            "can also move it upward, so image-up alone cannot distinguish forward from lift. Use FK height. "
            "A target right of the gripper in head view suggests a -Y lateral correction if locally feasible. "
            "A stationary bowl stays fixed in head pixels until touched; distinguish gripper motion from "
            "bowl motion. In wrist view, the CAMERA moves: stationary objects can shift opposite camera motion, "
            "with a different mapping after rotation. Re-check before/after images, especially after a rotation; "
            "never assume wrist-image right means base -Y or wrist-image up means base +Z. "
            "tool_x_axis_base/tool_y_axis_base/tool_z_axis_base give the LOCAL tool axes expressed in BASE "
            "from FK; they rotate with the wrist, while base axes do not. No tool-frame delta commands are "
            "supported. To move 1cm along local tool +Z, explicitly use 0.01*tool_z_axis_base as base "
            "delta_xyz. This is different from vertical descent unless tool +Z is already base -Z. "
            "delta_rotvec also uses BASE axes: positive rotation follows the right-hand rule. "
            "Positive X rotation turns +Y toward +Z; positive Y turns +Z toward +X; positive Z turns +X "
            "toward +Y. Reverse the sign for the opposite turn. These are radians, not degrees. "
            "In reason name the base axis/sign, expected gripper motion, and which view/FK feedback will "
            "check the effect. If the observed sign is opposite the hypothesis, revise rather than repeat."
        )
        prompt += (
            " FAILURE RECOVERY: Read recent_actions.status/reason, actual_feedback, action_resolution and "
            "eef_verification before deciding. Failed commands can partially move; use CURRENT FK and RGB, "
            "never assume the requested target was reached. Verification timeout can be contact, tracking "
            "error or a deadline; without contact sensors you cannot claim which. When recovery.status=ready, "
            "the harness has verified hold and permits a new plan. Do not request recovery yourself or repeat "
            "the same failed action. If descent stalls near the rim/table, consider a 1cm +Z retreat, lateral "
            "repositioning or small rotation at clearance, then reapproach. Do not repeatedly descend or close "
            "harder on the assumption this must succeed. Prefer adjusting one variable and compare image/FK "
            "feedback. A gripper timeout is not a grasp-success signal. In reason explicitly explain what "
            "failed, the measured actual state, and how the NEW action differs. User Enter, unconfirmed stop, "
            "lost lease, stale feedback or recovery failure halt this loop; never bypass those states. "
        )
        prompt += (
            " GRASP POSTURE/HEIGHT: Prefer a vertical top-down grasp: tool local +Z should point along "
            "base -Z. Read tool_z_axis_base and tool_down_tilt_deg from joint-derived FK; aim for tilt <=10 "
            "degrees near grasp. Wrist camera looking down in its image is not proof that tool +Z is down. "
            "Correct orientation gradually while high (tool z about 0.12 to 0.17m), using rotation-only "
            "move_eef_delta or joint adjustments. orientation_hint is a local FK/IK-tested proposal to reduce "
            "tilt, not a collision check. Keep checking both views and don't rotate rapidly beside the rim. "
            "Task scene priors: tabletop base z=0; bowl rim approximately z=0.05m. The validated nominal "
            "grasp uses tool z about 0.035m, with a 0.03 to 0.04m tool-height band. These are approximate "
            "setup dimensions, NOT current object pose estimates or depth-camera measurements. Flange height "
            "is different: flange-to-tool offset is +0.13503m along tool Z. Use tool pose for descent. "
            "When the rim is close to the wrist-view gripper and the top-down posture is established, stop "
            "endlessly refining pixels: descend by 1cm base -Z steps, reobserve each step, and stop descent "
            "around the nominal tool grasp band. Do not drive below the table or keep descending to force "
            "alignment. The task's rim-on-right heuristic is a cue to try closing, not proof of grasp. "
            "After closing, an observed bowl moving with the gripper during a lift is stronger visual "
            "evidence than a successful position command. Mention current tool height/tilt and why to rotate, "
            "descend, close, or lift in reason. Never replace RGB with object/contact truth."
        )
    content = [{"type": "text", "text": json.dumps(text, ensure_ascii=False, allow_nan=False)}]
    for label, images in (("previous", obs.get("previous_rgb")), ("current", obs["rgb"])):
        if images is None:
            continue
        for name in ("head", "wrist"):
            content.extend([{"type": "text", "text": f"{label} {name} RGB view"},
                            {"type": "image_url", "image_url": {"url": images[name]}}])
    return [{"role": "system", "content": prompt}, {"role": "user", "content": content}]


def resolve_action(action, joint, kinematics=None):
    """Resolve an explicit bounded delta using only joint feedback and local FK/IK."""
    if not isinstance(action, dict) or action.get("action") != "move_eef_delta":
        return action, None
    required = {"action", "delta_xyz", "frame", "point"}
    if set(action) - {"reason", "delta_rotvec"} != required or kinematics is None:
        raise ValueError("move_eef_delta requires delta_xyz, frame, point and Pinocchio")
    if action["frame"] != "base" or action["point"] not in ("tool", "flange"):
        raise ValueError("only base-frame tool/flange supported")
    if "reason" in action and (not isinstance(action["reason"], str) or not 0 < len(action["reason"].strip()) <= 500):
        raise ValueError("reason must be a nonempty string of at most 500 characters")
    values = action["delta_xyz"]
    if not isinstance(values, list) or len(values) != 3 or any(type(v) not in (int, float) for v in values):
        raise ValueError("delta_xyz requires three numeric metre values")
    delta = np.asarray(values, dtype=float)
    size = float(np.linalg.norm(delta))
    rotation = action.get("delta_rotvec", [0., 0., 0.])
    if not isinstance(rotation, list) or len(rotation) != 3 or any(type(v) not in (int, float) for v in rotation):
        raise ValueError("delta_rotvec requires three numeric base-frame radians")
    rotvec = np.asarray(rotation, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    if not np.isfinite(rotvec).all() or angle > .150000001:
        raise ValueError("delta_rotvec norm must be <=0.15 rad; try 0.02 to 0.04 rad")
    pure_rotation = size <= 1e-10 and angle > 1e-6
    if not np.isfinite(delta).all() or (not pure_rotation and abs(size - .01) > 1e-7):
        raise ValueError("delta_xyz norm must be exactly 0.01 m (1 cm); choose a unit direction times 0.01")
    current = np.asarray(kinematics.pose(joint, action["point"]), dtype=float)
    pose = current.copy(); pose[:3] += delta
    if angle > 0:
        new_rotation = Rotation.from_rotvec(rotvec) * Rotation.from_quat(current[[4, 5, 6, 3]])
        xyzw = new_rotation.as_quat(); pose[3:] = xyzw[[3, 0, 1, 2]]
    resolved = {"action": "move_eef", "pose": pose.tolist(), "frame": "base", "point": action["point"]}
    validate_action(resolved, joint, kinematics)
    return resolved, {"requested_delta_xyz_m": delta.tolist(), "resolved_delta_xyz_m": delta.tolist(),
                      "delta_rotvec_base_rad": rotvec.tolist(), "rotation_angle_rad": angle,
                      "scale": 1.0, "target_pose": pose.tolist(), "frame": "base", "point": action["point"]}


def validate_action(action, joint, kinematics=None):
    if not isinstance(action, dict):
        raise ValueError("JSON object required")
    action = dict(action)
    if "reason" in action:
        reason = action.pop("reason")
        if not isinstance(reason, str) or not reason.strip() or len(reason) > 500:
            raise ValueError("reason must be a nonempty string of at most 500 characters")
    name = action.get("action")
    if name == "move_eef_delta":
        resolved, _ = resolve_action(action, joint, kinematics)
        return validate_action(resolved, joint, kinematics)
    if name == "move_eef":
        if kinematics is None:
            raise ValueError("move_eef requires local Pinocchio kinematics")
        if set(action) != {"action", "pose", "frame", "point"}:
            raise ValueError("move_eef requires pose, frame, point")
        if action["frame"] != "base" or action["point"] not in ("tool", "flange"):
            raise ValueError("only base-frame tool/flange supported")
        values = action["pose"]
        if (not isinstance(values, list) or len(values) != 7
                or any(type(v) not in (float, int) for v in values)):
            raise ValueError("pose requires seven numeric xyz + wxyz values")
        pose = np.asarray(values, dtype=float)
        if not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:]) - 1) > 1e-5:
            raise ValueError("pose requires finite xyz and unit wxyz quaternion")
        distance, angle = kinematics.error(joint, pose, action["point"])
        if distance > .020000001 or angle > .150000001:
            raise ValueError("EEF step exceeds 0.02 m or 0.15 rad; prefer a 1 cm move_eef_delta")
        if 1e-8 < distance < .0099999:
            raise ValueError("EEF translation must be at least 0.01 m; use a 1 cm move_eef_delta")
        if distance <= 1e-8 and angle <= 1e-6:
            raise ValueError("zero EEF motion; choose a 1 cm delta, rotation, joint adjustment, or wait")
        try:
            solved = kinematics.solve(joint, pose, frame="base", point=action["point"])
        except RuntimeError:
            raise ValueError("Pinocchio IK did not converge; choose a smaller or different EEF step") from None
        target = validate_action({"action": "move_joint_absolute", "joint": solved}, joint)
        position_error, rotation_error = kinematics.error(solved, pose, action["point"])
        if position_error > .001 or rotation_error > .01:
            raise ValueError("IK solution failed FK verification")
        return target
    if name == "wait" and set(action) == {"action"}:
        return {}
    if name == "move_joint_absolute" and set(action) == {"action", "joint"}:
        values = action["joint"]
        if (not isinstance(values, list) or len(values) != 6
                or any(type(v) not in (float, int) for v in values)):
            raise ValueError("six numeric joint angles required")
        q = np.asarray(values, dtype=float)
        if not np.isfinite(q).all() or np.any(q < LOWER) or np.any(q > UPPER):
            raise ValueError("joint limits exceeded")
        if np.max(np.abs(q - np.asarray(joint))) > .080000001:
            raise ValueError("joint step exceeds 0.08 rad")
        return {"joint": q.tolist()}
    if name == "set_gripper" and set(action) == {"action", "gripper"}:
        g = action["gripper"]
        if type(g) not in (float, int) or not np.isfinite(g) or not 0 <= g <= 1:
            raise ValueError("gripper must be a number in [0,1]")
        return {"gripper": float(g)}
    raise ValueError("unsupported action or extra fields")


def one_cm_options(joint, kinematics):
    """Robot-only feasibility hints; no scene geometry or ground truth inputs."""
    options = []
    for axis in range(3):
        for sign in (1, -1):
            delta = [0., 0., 0.]; delta[axis] = .01 * sign
            action = dict(action="move_eef_delta", delta_xyz=delta, frame="base", point="tool")
            try:
                resolve_action(action, joint, kinematics)
                item = {"delta_xyz": delta, "ik_joint_feasible": True}
            except ValueError as exc:
                item = {"delta_xyz": delta, "ik_joint_feasible": False, "reason": str(exc)}
            options.append(item)
    return options


def orientation_feedback(joint, kinematics):
    """Tool-axis inclination and feasible correction derived only from joint FK."""
    p = np.asarray(kinematics.pose(joint, "tool"))
    matrix = Rotation.from_quat(p[[4, 5, 6, 3]]).as_matrix()
    axis = matrix[:, 2]
    down = np.array([0., 0., -1.])
    angle = float(np.arccos(np.clip(axis @ down, -1, 1)))
    result = {"tool_x_axis_base": matrix[:, 0].tolist(), "tool_y_axis_base": matrix[:, 1].tolist(),
              "tool_z_axis_base": axis.tolist(), "tool_down_tilt_deg": float(np.rad2deg(angle)),
              "tool_height_base_m": float(p[2])}
    if angle < 1e-6:
        result["orientation_hint"] = {"status": "already_down"}
        return result
    cross = np.cross(axis, down)
    direction = cross / np.linalg.norm(cross) if np.linalg.norm(cross) > 1e-8 else np.array([1., 0., 0.])
    for step in (.04, .02, .01):
        rv = direction * min(step, angle)
        action = dict(action="move_eef_delta", delta_xyz=[0.,0.,0.], delta_rotvec=rv.tolist(), frame="base", point="tool")
        try:
            resolve_action(action, joint, kinematics)
            result["orientation_hint"] = {"status": "ik_joint_feasible", "action": action,
                                           "source": "local FK/IK only; no collision check"}
            return result
        except ValueError:
            continue
    result["orientation_hint"] = {"status": "use_joint_posture_adjustment", "reason": "No feasible small tilt correction at this fixed tool point"}
    return result


class Stopped(Exception):
    pass


def interruptible_call(fn, stop, timeout_s=65):
    """Discard late HTTP replies; network workers never receive a robot handle."""
    result = queue.Queue(maxsize=1)
    def worker():
        try:
            result.put((True, fn()))
        except Exception as exc:
            result.put((False, exc))
    if stop.is_set():
        raise Stopped()
    threading.Thread(target=worker, daemon=True).start()
    deadline = time.monotonic() + timeout_s
    while not stop.wait(.05):
        try:
            ok, value = result.get_nowait()
        except queue.Empty:
            if time.monotonic() >= deadline:
                raise RuntimeError("model request deadline exceeded")
            continue
        if ok:
            return value
        raise value
    raise Stopped()


def chat_action(messages, key, model, base_url, *, deepseek_options=False):
    payload = {"model": model, "messages": messages, "response_format": {"type": "json_object"},
               "max_tokens": 1024}
    if deepseek_options:
        payload.update(thinking={"type": "disabled"}, temperature=0)
    request = urllib.request.Request(base_url.rstrip("/") + "/chat/completions",
        data=json.dumps(payload, allow_nan=False).encode(),
        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            body = json.load(response)
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f"Model API HTTP {exc.code}; response body hidden") from None
    except Exception as exc:
        raise RuntimeError(f"Model API transport failure ({type(exc).__name__}); details hidden") from None
    if body["choices"][0].get("finish_reason") != "stop":
        raise RuntimeError("incomplete model response")
    return json.loads(body["choices"][0]["message"]["content"])


def deepseek_action(messages, key, model="deepseek-flash"):
    return chat_action(messages, key, model, "https://api.deepseek.com", deepseek_options=True)
