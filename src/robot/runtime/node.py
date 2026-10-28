"""单个组件的周期循环。驱动类由调用方或 YAML 里的路径加载。"""

from __future__ import annotations

import importlib
import os
import sys
import time
import traceback
from collections import deque

import numpy as np

from robot.harness.gate import CommandRejected, validate_joints
from robot.harness.motion import JointMotion
from robot.runtime.log import configure_logging, get_logger
from robot.runtime.safety import engage_safe_stop
from robot.runtime.transport import Bus

_CONTROL_MODES = frozenset(
    {
        "impedance",
        "imp",
        "imp_joint",
        "teleop",
        "teleop_imp",
        "position",
        "pos",
        "cartesian",
        "imp_cart",
        "cart",
        "force",
        "imp_force",
        "mit",
        "mit_control",
        "gravity_compensation",
        "gravity",
    }
)


def _servo(component, dt: float) -> None:
    step = getattr(component, "servo_step", None)
    if step is None:
        return
    step(float(dt))


def load_component(spec: dict):
    module = importlib.import_module(spec["class_path"])
    cls = getattr(module, spec["class_name"])
    return cls(**(spec.get("args") or {}))


def run_component(shm_name: str, spec: dict, stop_event, factory=None) -> None:
    name = spec["name"]
    configure_logging()
    log = get_logger(name)
    bus = Bus.attach(shm_name)
    component = None
    logged_error = ""
    fatal = False
    try:
        try:
            component = factory() if factory is not None else load_component(spec)
            setattr(component, "_cyr_raise_move_errors", True)
            setup = spec.get("setup") or {}
            if setup:
                component.set_up(**setup)
            else:
                component.set_up()
            component.set_collect_info(list(spec["publish"]))
            _init_runtime_state(component, spec)
            log.info("set_up 完成 publish=%s rate_hz=%s", ",".join(spec["publish"]), spec["rate_hz"])
        except Exception as exc:
            log.exception("set_up 失败")
            bus.latch_safety(f"{name} set_up 失败: {exc}")
            _status(bus, name, alive=False, error=f"set_up failed: {exc}", drops=0, hz=0.0, cycle_ms=0.0)
            raise
        period = 1.0 / float(spec["rate_hz"])
        next_tick = time.monotonic()
        state_drops = 0
        error = ""
        while not stop_event.is_set():
            if bus.read_safety()["latched"]:
                log.error("安全保护已触发: %s", bus.read_safety()["reason"])
                break
            started = time.monotonic()
            try:
                _advance(bus, spec, component, log)
            except Exception as exc:
                fatal = True
                log.exception("命令执行失败，触发安全保护")
                bus.latch_safety(f"{name} 命令失败: {exc}")
                break
            move_error = str(getattr(component, "_cyr_last_cmd_error", "") or "")
            try:
                _servo(component, period)
                data = component.get()
                timestamp_ns = time.time_ns()
                if isinstance(data, dict) and data.get("timestamp") is not None:
                    timestamp_ns = int(data["timestamp"])
                wrote = bus.write_state(name, {"timestamp_ns": timestamp_ns, "data": data})
                if wrote:
                    error = move_error
                else:
                    state_drops += 1
                    error = "state exceeds max_bytes" if not move_error else f"{move_error}; state exceeds max_bytes"
            except Exception as exc:
                error = str(exc)
            if error != logged_error:
                if error:
                    log.error("周期失败: %s", error)
                elif logged_error:
                    log.info("周期恢复")
                logged_error = error
            finished = time.monotonic()
            _read, write_seq, cmd_drops = bus.cmd_cursors(name)
            state = bus.read_state(name)
            _status(
                bus,
                name,
                alive=True,
                error=error,
                drops=state_drops + cmd_drops,
                hz=_hz(component),
                cycle_ms=(finished - started) * 1000.0,
                state_seq=0 if state is None else int(state["seq"]),
                cmd_read_seq=_read,
                cmd_write_seq=write_seq,
                **_motion_status(component),
            )
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                stop_event.wait(delay)
            else:
                next_tick = time.monotonic()
    finally:
        exiting = sys.exc_info()[1]
        if isinstance(exiting, SystemExit) and exiting.code not in (0, None):
            bus.latch_safety(f"{name} 异常退出: {exiting}")
        _stop_device(log, component)
        try:
            safety = bus.read_safety()
            if safety["latched"]:
                _status(
                    bus,
                    name,
                    alive=False,
                    error=safety["reason"],
                    drops=0,
                    hz=0.0,
                    cycle_ms=0.0,
                )
        except Exception:
            log.exception("写入安全状态失败")
        bus.close()
    if fatal:
        raise SystemExit(1)


def bundle_process(shm_name: str, specs: list[dict], stop_event) -> None:
    """一个进程跑一组组件。天机左右臂必须这样，才能共用一条 UDP。"""
    if len(specs) == 1:
        component_process(shm_name, specs[0], stop_event)
        return
    configure_logging()
    label = "+".join(str(spec.get("name") or "component") for spec in specs)
    log = get_logger(label)
    log.info("进程启动 components=%s", label)
    try:
        run_bundle(shm_name, specs, stop_event)
    except Exception:
        log.exception("进程异常退出")
        traceback.print_exc()
        raise SystemExit(1)
    log.info("进程正常结束")


def run_bundle(shm_name: str, specs: list[dict], stop_event) -> None:
    names = [spec["name"] for spec in specs]
    log = get_logger("+".join(names))
    bus = Bus.attach(shm_name)
    components: list = []
    fatal = False
    try:
        try:
            for spec in specs:
                component = load_component(spec)
                setattr(component, "_cyr_raise_move_errors", True)
                setup = spec.get("setup") or {}
                if setup:
                    component.set_up(**setup)
                else:
                    component.set_up()
                component.set_collect_info(list(spec["publish"]))
                _init_runtime_state(component, spec)
                components.append(component)
                log.info("%s set_up 完成 publish=%s", spec["name"], ",".join(spec["publish"]))
        except Exception as exc:
            failed = specs[len(components)]["name"]
            log.exception("%s set_up 失败", failed)
            bus.latch_safety(f"{failed} set_up 失败: {exc}")
            _status(bus, failed, alive=False, error=f"set_up failed: {exc}", drops=0, hz=0.0, cycle_ms=0.0)
            raise
        period = 1.0 / max(float(spec["rate_hz"]) for spec in specs)
        next_tick = time.monotonic()
        logged: dict[str, str] = {}
        while not stop_event.is_set():
            if bus.read_safety()["latched"]:
                log.error("安全保护已触发: %s", bus.read_safety()["reason"])
                break
            for spec, component in zip(specs, components):
                name = spec["name"]
                started = time.monotonic()
                try:
                    _advance(bus, spec, component, log)
                except Exception as exc:
                    fatal = True
                    log.exception("%s 命令执行失败，触发安全保护", name)
                    bus.latch_safety(f"{name} 命令失败: {exc}")
                    break
                error = str(getattr(component, "_cyr_last_cmd_error", "") or "")
                try:
                    _servo(component, 1.0 / float(spec["rate_hz"]))
                    data = component.get()
                    timestamp_ns = time.time_ns()
                    if isinstance(data, dict) and data.get("timestamp") is not None:
                        timestamp_ns = int(data["timestamp"])
                    if not bus.write_state(name, {"timestamp_ns": timestamp_ns, "data": data}):
                        error = "state exceeds max_bytes" if not error else f"{error}; state exceeds max_bytes"
                except Exception as exc:
                    error = str(exc)
                if error and error != logged.get(name):
                    log.error("%s 周期失败: %s", name, error)
                logged[name] = error
                state = bus.read_state(name)
                _read, write_seq, cmd_drops = bus.cmd_cursors(name)
                _status(
                    bus,
                    name,
                    alive=True,
                    error=error,
                    drops=cmd_drops,
                    hz=_hz(component),
                    cycle_ms=(time.monotonic() - started) * 1000.0,
                    state_seq=0 if state is None else int(state["seq"]),
                    cmd_read_seq=_read,
                    cmd_write_seq=write_seq,
                    **_motion_status(component),
                )
            if fatal or bus.read_safety()["latched"]:
                break
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                stop_event.wait(delay)
            else:
                next_tick = time.monotonic()
    finally:
        for component in components:
            _stop_device(log, component)
        bus.close()
    if fatal:
        raise SystemExit(1)


def component_process(shm_name: str, spec: dict, stop_event) -> None:
    configure_logging()
    log = get_logger(str(spec.get("name") or "component"))
    log.info(
        "进程启动 class=%s.%s",
        spec.get("class_path"),
        spec.get("class_name"),
    )
    try:
        run_component(shm_name, spec, stop_event)
    except Exception:
        log.exception("进程异常退出")
        traceback.print_exc()
        raise SystemExit(1)
    log.info("进程正常结束")


def _stop_device(log, component) -> None:
    if component is None:
        return
    try:
        action = engage_safe_stop(component)
    except Exception:
        log.exception("安全停止失败")
        return
    log.warning("安全停止 action=%s", action)


def _init_runtime_state(component, spec: dict) -> None:
    component._cyr_body = _load_body(spec)
    component._cyr_motion = None
    component._cyr_motion_hold = False
    component._cyr_last_cmd_seq = 0
    component._cyr_last_cmd_error = ""
    component._cyr_tracking_err = 0.0
    component._cyr_motion_timeout_s = float((spec.get("motion") or {}).get("timeout_s") or 0.0)
    component._cyr_stream_seq = 0
    component._cyr_stream_target = None
    component._cyr_stream_deadline_ns = 0
    component._cyr_stream_state = "idle"
    component._cyr_stream_reason = ""
    component._cyr_stream_hold = False
    component._cyr_cycles = deque()


def _load_body(spec: dict):
    path = spec.get("body") or ""
    if not path:
        return None
    from robot.harness.body import BodyModel

    return BodyModel.load(path)


def _hz(component) -> float:
    samples = component._cyr_cycles
    now = time.monotonic()
    samples.append(now)
    while len(samples) > 1 and now - samples[0] > 1.0:
        samples.popleft()
    if len(samples) >= 2 and samples[-1] > samples[0]:
        return (len(samples) - 1) / (samples[-1] - samples[0])
    return 0.0


def _motion_status(component) -> dict:
    plan = getattr(component, "_cyr_motion", None)
    return {
        "last_cmd_seq": int(getattr(component, "_cyr_last_cmd_seq", 0) or 0),
        "last_cmd_error": str(getattr(component, "_cyr_last_cmd_error", "") or ""),
        "motion_active": plan is not None,
        "motion_seq": 0 if plan is None else int(plan.seq),
        "motion_timeout_s": float(getattr(component, "_cyr_motion_timeout_s", 0.0) or 0.0),
        "tracking_err": float(getattr(component, "_cyr_tracking_err", 0.0) or 0.0),
        "stream_seq": int(getattr(component, "_cyr_stream_seq", 0)),
        "stream_state": getattr(component, "_cyr_stream_state", "idle"),
        "stream_reason": getattr(component, "_cyr_stream_reason", ""),
    }


def _advance(bus: Bus, spec: dict, component, log) -> None:
    _stream_watchdog(bus, spec, component)
    if getattr(component, "_cyr_motion", None) is not None:
        _step_motion(component)
        return
    if bus.read_safety()["latched"]:
        return
    command = bus.pop_cmd(spec["name"])
    if command is not None:
        component._cyr_last_cmd_seq = int(command["seq"])
        data = command.get("data")
        if isinstance(data, dict) and data.get("op") == "reset":
            frame = bus.read_stream(spec["name"])
            component._cyr_stream_reset_seq = 0 if frame is None else int(frame["seq"])
        try:
            _apply_command(spec, component, command)
        except CommandRejected as exc:
            component._cyr_motion = None
            component._cyr_motion_hold = True
            component._cyr_last_cmd_error = str(exc)
            log.error("%s 拒绝命令: %s", spec["name"], exc)
            return
        if getattr(component, "_cyr_motion", None) is None:
            component._cyr_last_cmd_error = ""
        return
    _follow_stream(bus, spec, component)


def _apply_command(spec: dict, component, command: dict) -> None:
    data = command["data"]
    if not isinstance(data, dict):
        raise CommandRejected("命令必须是对象")
    if data.get("op") == "release":
        component._cyr_stream_hold = False
        component._cyr_stream_state = "idle"
        component._cyr_stream_target = None
        component._cyr_stream_reset_seq = component._cyr_stream_seq
        component._cyr_motion_hold = False
        component._cyr_last_cmd_error = ""
        return
    if getattr(component, "_cyr_motion_hold", False):
        raise CommandRejected("组件保持当前位置，后续命令已拒绝")
    if data.get("op") == "reset":
        _apply_reset(spec, component, data.get("start_state"), int(command["seq"]))
        return
    if data.get("op") in (
        "gravity",
        "hold_torque",
        "set_payload",
        "set_plant_payload",
        "impedance",
        "cartesian",
        "mit",
        "pose",
        "tool_delta",
        "touch",
    ):
        handler = getattr(component, "handle_runtime_op", None)
        if handler is None:
            raise CommandRejected(f"组件不接受 {data.get('op')}")
        try:
            handler(data)
        except Exception as exc:
            raise CommandRejected(str(exc)) from exc
        return
    payload = dict(data)
    mode = payload.pop("control_mode", None)
    eef = payload.get("eef")
    if eef is not None and "joint" not in payload:
        solved = _solve_eef(spec, component, eef, bool(command["is_delta"]))
        if solved is not None:
            joints, absolute = solved
            payload = {**payload, "joint": joints.tolist(), "eef": np.asarray(absolute, dtype=float).reshape(-1).tolist()}
    elif command["is_delta"] and "joint" in payload:
        current = _current_joint(component)
        try:
            delta = np.asarray(payload["joint"], dtype=float).reshape(-1)
            payload["joint"] = (current + delta).tolist()
        except (TypeError, ValueError) as exc:
            raise CommandRejected("关节目标必须是数字") from exc
        command = {**command, "is_delta": False}
    checked = None
    if "joint" in payload and (_motion_enabled(spec) or _gate_configured(spec, component)):
        current = _current_joint(component)
        try:
            checked = _gate(
                spec,
                component,
                current,
                payload["joint"],
                eef=payload.get("eef"),
                trajectory=_motion_enabled(spec),
            )
        except (TypeError, ValueError) as exc:
            raise CommandRejected("关节目标必须是数字") from exc
        payload = {**payload, "joint": checked.tolist()}
    if mode is not None:
        _apply_mode(component, mode)
    if checked is not None and _motion_enabled(spec):
        if "gripper" in payload:
            try:
                component.set_gripper(payload["gripper"])
            except (TypeError, ValueError) as exc:
                raise CommandRejected(str(exc)) from exc
        _start_joint(spec, component, checked, int(command["seq"]))
        return
    move = getattr(component, "move", None)
    if not any(key in payload for key in ("joint", "eef", "gripper", "action", "velocity", "force")):
        return
    if move is None:
        return
    move(payload, is_delta=bool(command["is_delta"]) and "joint" not in payload)


def _apply_reset(spec: dict, component, start_state, seq: int) -> None:
    """调用控制器自己的 reset。整段轨迹由 set_joint 插值发出，不把零位一次下发。"""
    runtime_goal = getattr(component, "runtime_reset_target", None)
    if runtime_goal is not None and _motion_enabled(spec):
        try:
            _begin_joint(spec, component, runtime_goal(start_state), seq, eef=None)
        except (TypeError, ValueError) as exc:
            raise CommandRejected(str(exc)) from exc
        return
    goal = _reset_goal(spec, component, start_state)
    if goal is not None and (_motion_enabled(spec) or _gate_configured(spec, component)):
        goal = _gate(spec, component, _current_joint(component), goal, eef=None, trajectory=True)
    reset = getattr(component, "reset", None)
    if reset is None:
        raise CommandRejected("组件没有 reset")
    try:
        reset(None if goal is None else np.asarray(goal, dtype=float).reshape(-1).tolist())
    except CommandRejected:
        raise
    except Exception as exc:
        raise CommandRejected(str(exc)) from exc


def _reset_goal(spec: dict, component, start_state):
    if start_state is not None:
        return np.asarray(start_state, dtype=float).reshape(-1)
    body = getattr(component, "_cyr_body", None)
    arm = _arm_name(spec)
    if body is not None and arm in body.arm_names():
        return np.asarray(body.home(arm), dtype=float).reshape(-1)
    return None


def _begin_joint(spec: dict, component, goal, seq: int, eef) -> None:
    current = _current_joint(component)
    checked = _gate(spec, component, current, goal, eef=eef, trajectory=True)
    _start_joint(spec, component, checked, seq)


def _start_joint(spec: dict, component, goal, seq: int) -> None:
    motion = spec.get("motion") or {}
    component._cyr_motion_timeout_s = _cfg_float(motion, "timeout_s", 8.0)
    try:
        plan = JointMotion(
            _current_joint(component),
            goal,
            max_step_rad=_cfg_float(motion, "max_step_rad", 0.05),
            tol_rad=_cfg_float(motion, "tol_rad", 0.02),
            timeout_s=component._cyr_motion_timeout_s,
            seq=seq,
        )
    except ValueError as exc:
        raise CommandRejected(str(exc)) from exc
    component._cyr_motion = plan
    component._cyr_last_cmd_error = ""
    _step_motion(component)


def _apply_mode(component, mode) -> None:
    if str(mode).strip().lower() not in _CONTROL_MODES:
        raise CommandRejected(
            f"control_mode 只能是 position、mit、gravity_compensation、impedance、cartesian 或 force，得到 {mode!r}"
        )
    move = getattr(component, "move", None)
    if move is not None:
        move({"control_mode": mode}, is_delta=False)


def _step_motion(component) -> None:
    plan = component._cyr_motion
    if plan is None:
        return
    measured = _current_joint(component)
    component._cyr_tracking_err = plan.error(measured)
    if plan.tracked(measured):
        component._cyr_motion = None
        component._cyr_last_cmd_error = ""
        return
    if plan.timed_out():
        component._cyr_motion = None
        component._cyr_motion_hold = True
        component._cyr_last_cmd_error = "跟踪超时"
        return
    point = plan.goal if plan.command_reached() else plan.advance()
    move = getattr(component, "move", None)
    if move is not None:
        move({"joint": np.asarray(point, dtype=float).tolist()}, is_delta=False)
    measured = _current_joint(component)
    component._cyr_tracking_err = plan.error(measured)
    if plan.tracked(measured):
        component._cyr_motion = None
        component._cyr_last_cmd_error = ""


def _solve_eef(spec: dict, component, eef, is_delta: bool):
    """返回 (关节解, 绝对末端)。优先 body，随后设备 IK；都没有才原样下发。"""
    body = getattr(component, "_cyr_body", None)
    arm = _arm_name(spec)
    if body is None or arm not in body.arm_names():
        solver = getattr(component, "solve_ik", None)
        if solver is None:
            return None
        try:
            from robot.runtime.frames import compose_delta

            target = (compose_delta(component.get_state()["eef"], eef)
                      if is_delta else np.asarray(eef, dtype=float))
            return np.asarray(solver(target), dtype=float), target
        except (TypeError, ValueError, RuntimeError) as exc:
            raise CommandRejected(str(exc)) from exc
    current = _current_joint(component)
    try:
        raw = np.asarray(eef, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise CommandRejected("末端目标必须是数字") from exc
    if is_delta:
        from robot.runtime.frames import compose_delta

        pose = np.asarray(body.flange_pose(arm, current), dtype=float).reshape(-1)
        try:
            target = compose_delta(pose, raw)
        except ValueError as exc:
            raise CommandRejected(str(exc)) from exc
    else:
        target = raw
    gate = spec.get("gate") or {}
    solved = body.ik(arm, current, target, max_error_m=_cfg_float(gate, "max_fk_error_m", 0.008))
    return solved, target


def _gate(spec: dict, component, current, target, *, eef, trajectory: bool) -> np.ndarray:
    gate = spec.get("gate") or {}
    body = getattr(component, "_cyr_body", None)
    arm = _arm_name(spec)
    clearance = None
    flange = None
    if body is not None and arm in body.arm_names():
        lower, upper = body.limits(arm)
        clearance = lambda sample, arm=arm, body=body: body.clearance(arm, sample)
        flange = lambda sample, arm=arm, body=body: body.flange_pose(arm, sample)
    elif "lower" in gate and "upper" in gate:
        lower = gate["lower"]
        upper = gate["upper"]
    else:
        return np.asarray(target, dtype=float).reshape(-1)
    motion = spec.get("motion") or {}
    return validate_joints(
        current,
        target,
        lower=lower,
        upper=upper,
        max_step_rad=_cfg_float(gate, "max_step_rad", 0.2),
        min_clearance_m=_cfg_float(gate, "min_clearance_m", 0.0),
        clearance=clearance,
        eef=eef,
        flange=flange,
        max_fk_error_m=_cfg_float(gate, "max_fk_error_m", 0.008),
        trajectory=trajectory,
        path_step_rad=_cfg_float(motion, "max_step_rad", _cfg_float(gate, "max_step_rad", 0.05)),
    )


def _cfg_float(mapping, key: str, default: float) -> float:
    if not isinstance(mapping, dict) or mapping.get(key) is None:
        return float(default)
    return float(mapping[key])


def _gate_configured(spec: dict, component) -> bool:
    gate = spec.get("gate") or {}
    body = getattr(component, "_cyr_body", None)
    if body is not None and _arm_name(spec) in body.arm_names():
        return True
    return "lower" in gate and "upper" in gate


def _motion_enabled(spec: dict) -> bool:
    return bool((spec.get("motion") or {}).get("enabled"))


def _arm_name(spec: dict) -> str:
    gate = spec.get("gate") or {}
    if gate.get("arm"):
        return str(gate["arm"])
    name = str(spec.get("name") or "")
    if name.endswith("_arm"):
        return name[: -len("_arm")]
    return name


def _stop_stream(component, reason):
    component._cyr_motion = None
    component._cyr_motion_hold = True
    component._cyr_stream_hold = True
    component._cyr_stream_target = None
    component._cyr_stream_deadline_ns = 0
    component._cyr_stream_reason = reason
    component._cyr_last_cmd_error = "" if reason == "stop requested" else reason
    state = component.get_state() if callable(getattr(component, "get_state", None)) else {}
    if state.get("control_mode") == "position" and state.get("joint") is not None:
        payload = {"joint": np.asarray(state["joint"], dtype=float).tolist()}
        if state.get("gripper") is not None:
            payload["gripper"] = float(np.asarray(state["gripper"]).reshape(-1)[0])
        component.move(payload, is_delta=False)
        component._cyr_stream_state = "stopped"
    else:
        engage_safe_stop(component)
        component._cyr_stream_state = "stop_unconfirmed"


def _stream_watchdog(bus, spec, component):
    """Runs in the device process before motion/stream advancement.

    Deadlines use the host monotonic clock. No Python loop can enforce them
    while the driver itself blocks. Position hold is not a hardware e-stop.
    """
    held = bus.read_control()["held"]
    frame = bus.read_stream(spec["name"]) if held else None
    if frame is not None and int(frame["seq"]) != component._cyr_stream_seq:
        component._cyr_stream_seq = int(frame["seq"])
        if frame["data"].get("op") == "stop":
            _stop_stream(component, "stop requested")
            return
        if not component._cyr_stream_hold:
            component._cyr_stream_target = frame["data"]
            deadline = int(frame.get("deadline_ns", 0))
            cap = float(spec.get("stream_timeout_s") or 0)
            if cap > 0:
                written = int(frame.get("written_ns", 0))
                configured = written + int(cap * 1e9) if written > 0 else 1
                deadline = min(deadline, configured) if deadline else configured
            component._cyr_stream_deadline_ns = deadline
            component._cyr_stream_state = "active"
            component._cyr_stream_reason = ""
    if component._cyr_stream_state == "active":
        if not held:
            if component._cyr_stream_deadline_ns:
                _stop_stream(component, "control lease lost")
            else:
                component._cyr_stream_state = "idle"
                component._cyr_stream_target = None
        elif component._cyr_stream_deadline_ns and time.monotonic_ns() >= component._cyr_stream_deadline_ns:
            _stop_stream(component, "stream target expired")


def _follow_stream(bus: Bus, spec: dict, component) -> None:
    if component._cyr_motion_hold or component._cyr_stream_hold:
        return
    if not bus.read_control()["held"]:
        return
    frame = bus.read_stream(spec["name"])
    if frame is None:
        return
    if int(frame["seq"]) == int(getattr(component, "_cyr_stream_reset_seq", 0) or 0):
        component._cyr_stream_target = None
        return
    target = component._cyr_stream_target
    if not isinstance(target, dict):
        return
    try:
        payload = _stream_payload(spec, component, target)
    except CommandRejected as exc:
        component._cyr_last_cmd_error = str(exc)
        return
    if payload:
        component.move(payload, is_delta=False)
        component._cyr_last_cmd_error = ""


def _stream_payload(spec: dict, component, target: dict) -> dict:
    payload = {}
    if "gripper" in target:
        payload["gripper"] = target["gripper"]
    goal = None
    if "joint" in target:
        goal = np.asarray(target["joint"], dtype=float).reshape(-1)
    elif "eef" in target:
        solved = _solve_eef(spec, component, target["eef"], False)
        if solved is None:
            payload["eef"] = np.asarray(target["eef"], dtype=float).reshape(-1).tolist()
        else:
            goal = np.asarray(solved[0], dtype=float).reshape(-1)
    if goal is None:
        return payload
    current = _current_joint(component)
    if current.size != goal.size:
        raise CommandRejected(f"关节长度 {goal.size}，当前是 {current.size}")
    step = _stream_limit(spec)
    delta = goal - current
    if np.isfinite(step):
        delta = np.clip(delta, -step, step)
    point = _clip_limits(spec, component, current + delta)
    if _stream_blocked(spec, component, point):
        raise CommandRejected("间隙")
    payload["joint"] = point.tolist()
    return payload


def _stream_limit(spec: dict) -> float:
    gate = spec.get("gate") or {}
    motion = spec.get("motion") or {}
    if isinstance(gate, dict) and gate.get("max_step_rad") is not None:
        return float(gate["max_step_rad"])
    if isinstance(motion, dict) and motion.get("max_step_rad") is not None:
        return float(motion["max_step_rad"])
    return float("inf")


def _clip_limits(spec: dict, component, point: np.ndarray) -> np.ndarray:
    body = getattr(component, "_cyr_body", None)
    arm = _arm_name(spec)
    if body is not None and arm in body.arm_names():
        lower, upper = body.limits(arm)
    else:
        gate = spec.get("gate") or {}
        if "lower" not in gate or "upper" not in gate:
            return point
        lower = np.asarray(gate["lower"], dtype=float).reshape(-1)
        upper = np.asarray(gate["upper"], dtype=float).reshape(-1)
    count = min(point.size, lower.size, upper.size)
    point = point.copy()
    point[:count] = np.clip(point[:count], lower[:count], upper[:count])
    return point


def _stream_blocked(spec: dict, component, point: np.ndarray) -> bool:
    gate = spec.get("gate") or {}
    minimum = float(gate.get("min_clearance_m") or 0.0)
    if minimum <= 0:
        return False
    body = getattr(component, "_cyr_body", None)
    arm = _arm_name(spec)
    if body is None or arm not in body.arm_names():
        return False
    return float(body.clearance(arm, point)) < minimum


def _current_joint(component) -> np.ndarray:
    getter = getattr(component, "get_state", None)
    if callable(getter):
        state = getter()
        if isinstance(state, dict) and state.get("joint") is not None:
            return np.asarray(state["joint"], dtype=float).reshape(-1)
    data = component.get()
    if isinstance(data, dict) and data.get("joint") is not None:
        return np.asarray(data["joint"], dtype=float).reshape(-1)
    raise CommandRejected("读不到当前关节")


def _status(
    bus: Bus,
    name: str,
    *,
    alive: bool,
    error: str,
    drops: int,
    hz: float,
    cycle_ms: float,
    state_seq: int = 0,
    cmd_read_seq: int = 0,
    cmd_write_seq: int = 0,
    last_cmd_seq: int = 0,
    last_cmd_error: str = "",
    motion_active: bool = False,
    motion_seq: int = 0,
    motion_timeout_s: float = 0.0,
    tracking_err: float = 0.0,
    stream_seq: int = 0,
    stream_state: str = "idle",
    stream_reason: str = "",
) -> None:
    bus.write_status(
        name,
        {
            "pid": os.getpid(),
            "alive": alive,
            "hz": float(hz),
            "cycle_ms": float(cycle_ms),
            "state_seq": int(state_seq),
            "cmd_read_seq": int(cmd_read_seq),
            "cmd_write_seq": int(cmd_write_seq),
            "drops": int(drops),
            "error": error,
            "last_cmd_seq": int(last_cmd_seq),
            "last_cmd_error": last_cmd_error,
            "motion_active": bool(motion_active),
            "motion_seq": int(motion_seq),
            "motion_timeout_s": float(motion_timeout_s),
            "tracking_err": float(tracking_err),
            "stream_seq": stream_seq,
            "stream_state": stream_state,
            "stream_reason": stream_reason,
        },
    )
