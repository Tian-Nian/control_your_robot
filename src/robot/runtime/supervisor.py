"""拉起组件进程，并在 SIGINT/SIGTERM 时回收共享内存。"""

from __future__ import annotations

import multiprocessing as mp
import os
import signal
import time

from robot.runtime.config import RuntimeConfig
from robot.runtime.discover import remove_record, write_record
from robot.runtime.log import configure_logging, get_logger
from robot.runtime.node import bundle_process
from robot.runtime.transport import Bus, SlotSpec


def run_supervisor(config: RuntimeConfig) -> int:
    configure_logging()
    log = get_logger("supervisor")
    names = [item.name for item in config.components]
    log.info("启动 shm=%s config=%s components=%s", config.shm_name, config.path, ",".join(names))
    bus = None
    ctx = mp.get_context("spawn")
    stop_event = ctx.Event()
    processes: list[mp.Process] = []
    exit_code = 0

    def _stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        try:
            bus = Bus.create(
                config.shm_name,
                [SlotSpec(item.name, item.max_bytes) for item in config.components],
                config.cmd_capacity,
            )
            for group in process_groups(config.components):
                specs = [item.as_spec() for item in group]
                label = "+".join(item.name for item in group)
                process = ctx.Process(
                    target=bundle_process,
                    name=f"cyr-{label}",
                    args=(config.shm_name, specs, stop_event),
                )
                process.start()
                processes.append(process)
                log.info("拉起 %s pid=%s", label, process.pid)
            if not _wait_until_ready(bus, config, processes, stop_event, log):
                if stop_event.is_set():
                    exit_code = 0
                else:
                    exit_code = 1
                    bus.latch_safety("组件启动失败")
                    log.error("启动失败，已触发安全保护")
                stop_event.set()
            else:
                write_record(
                    {
                        "name": config.name,
                        "pid": os.getpid(),
                        "config_path": str(config.path),
                        "shm_name": config.shm_name,
                        "components": names,
                    }
                )
                log.info("就绪 pid=%s", os.getpid())
                while not stop_event.is_set():
                    failed = [process for process in processes if not process.is_alive() and process.exitcode not in (0, None)]
                    if failed:
                        detail = ", ".join(f"{process.name} code={process.exitcode}" for process in failed)
                        log.error("组件退出: %s", detail)
                        bus.latch_safety(f"组件退出: {detail}")
                        log.error("已触发安全保护，正在停止其余设备")
                        exit_code = 1
                        stop_event.set()
                        break
                    time.sleep(0.2)
        except Exception:
            log.exception("supervisor 失败")
            exit_code = 1
            stop_event.set()
    finally:
        log.info("正在关闭")
        if bus is not None:
            bus.latch_safety("服务关闭")
        stop_event.set()
        _join_all(processes)
        remove_record(shm_name=config.shm_name)
        if bus is not None:
            bus.unlink()
        log.info("已关闭 exit=%s", exit_code)
    return exit_code


def process_groups(components):
    """同一台天机控制器的左右臂放进一个进程，才能共用那一条 UDP。"""
    groups = []
    shared: dict[str, list] = {}
    for item in components:
        robot_ip = (item.setup or {}).get("robot_ip") if item.class_name == "MarvinArmController" else None
        if not robot_ip:
            groups.append([item])
            continue
        bucket = shared.get(robot_ip)
        if bucket is None:
            bucket = []
            shared[robot_ip] = bucket
            groups.append(bucket)
        bucket.append(item)
    return groups


def _wait_until_ready(bus: Bus, config: RuntimeConfig, processes: list[mp.Process], stop_event, log) -> bool:
    deadline = time.monotonic() + 10.0
    _wait_until_ready.last_error = ""
    names = [item.name for item in config.components]
    while time.monotonic() < deadline:
        if stop_event.is_set():
            log.info("就绪前收到停止")
            return False
        for process in processes:
            if not process.is_alive() and process.exitcode not in (0, None):
                log.error("%s 在启动阶段退出 code=%s", process.name, process.exitcode)
                return False
        statuses = [bus.read_status(name) for name in names]
        if all(status and status.get("alive") for status in statuses):
            return True
        if any(status and status.get("error") for status in statuses):
            detail = "; ".join(
                f"{name}: {status.get('error')}"
                for name, status in zip(names, statuses)
                if status and status.get("error")
            )
            if detail != _wait_until_ready.last_error:
                log.error("组件尚未就绪 %s", detail)
                _wait_until_ready.last_error = detail
        time.sleep(0.05)
    log.error("10 秒内组件没有全部就绪")
    return False


def _join_all(processes: list[mp.Process]) -> None:
    for process in processes:
        process.join(timeout=2)
    for process in processes:
        if process.is_alive():
            process.terminate()
    for process in processes:
        process.join(timeout=1)
    for process in processes:
        if process.is_alive():
            process.kill()
            process.join(timeout=1)
