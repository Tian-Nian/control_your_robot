"""单个组件的周期循环。驱动类由调用方或 YAML 里的路径加载。"""

from __future__ import annotations

import importlib
import os
import sys
import time
import traceback
from collections import deque

from robot.runtime.log import configure_logging, get_logger
from robot.runtime.safety import engage_safe_stop
from robot.runtime.transport import Bus


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
            log.info("set_up 完成 publish=%s rate_hz=%s", ",".join(spec["publish"]), spec["rate_hz"])
        except Exception as exc:
            log.exception("set_up 失败")
            bus.latch_safety(f"{name} set_up 失败: {exc}")
            _status(bus, name, alive=False, error=f"set_up failed: {exc}", drops=0, hz=0.0, cycle_ms=0.0)
            raise
        period = 1.0 / float(spec["rate_hz"])
        next_tick = time.monotonic()
        cycles: deque[float] = deque()
        state_drops = 0
        error = ""
        while not stop_event.is_set():
            if bus.read_safety()["latched"]:
                log.error("安全保护已触发: %s", bus.read_safety()["reason"])
                break
            started = time.monotonic()
            try:
                move_error = _drain_commands(bus, component, name)
            except Exception as exc:
                fatal = True
                log.exception("命令执行失败，触发安全保护")
                bus.latch_safety(f"{name} 命令失败: {exc}")
                break
            try:
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
            cycles.append(finished)
            while len(cycles) > 1 and finished - cycles[0] > 1.0:
                cycles.popleft()
            if len(cycles) >= 2 and cycles[-1] > cycles[0]:
                hz = (len(cycles) - 1) / (cycles[-1] - cycles[0])
            else:
                hz = 0.0
            _read, write_seq, cmd_drops = bus.cmd_cursors(name)
            state = bus.read_state(name)
            _status(
                bus,
                name,
                alive=True,
                error=error,
                drops=state_drops + cmd_drops,
                hz=hz,
                cycle_ms=(finished - started) * 1000.0,
                state_seq=0 if state is None else int(state["seq"]),
                cmd_read_seq=_read,
                cmd_write_seq=write_seq,
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


def _drain_commands(bus: Bus, component, name: str) -> str:
    error = ""
    move = getattr(component, "move", None)
    while True:
        if bus.read_safety()["latched"]:
            return error
        command = bus.pop_cmd(name)
        if command is None:
            return error
        if move is None:
            continue
        move(command["data"], is_delta=command["is_delta"])


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
        },
    )
