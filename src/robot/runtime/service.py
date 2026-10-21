"""start / stop / restart / status。前台 serve 留给调试和 systemd。"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from robot.runtime.config import load_config
from robot.runtime.discover import (
    clear_dead_records,
    live_records,
    remove_record,
    user_state_dir,
)
from robot.runtime.log import configure_logging, get_logger, install_excepthook, log_path, tail_log
from robot.runtime.supervisor import run_supervisor
from robot.runtime.transport import BusBusyError, BusCorruptError, pid_alive


def source_root() -> Path:
    return Path(__file__).resolve().parents[2]


def serve(config_path: Path) -> int:
    configure_logging()
    install_excepthook()
    log = get_logger("serve")
    try:
        config = load_config(config_path)
    except (OSError, ValueError) as exc:
        log.error("配置无法加载: %s", exc)
        print(exc, file=sys.stderr)
        return 1
    log.info("serve config=%s", config.path)
    try:
        return run_supervisor(config)
    except (BusBusyError, BusCorruptError) as exc:
        log.exception("无法占用共享内存")
        print(exc, file=sys.stderr)
        return 1
    except Exception:
        log.exception("serve 异常退出")
        return 1


def start(config_path: Path) -> int:
    configure_logging()
    log = get_logger("service")
    clear_dead_records()
    try:
        config = load_config(config_path)
    except (OSError, ValueError) as exc:
        log.error("配置无效: %s", exc)
        print(exc, file=sys.stderr)
        return 1
    same = [record for record in live_records() if record.get("shm_name") == config.shm_name]
    if same:
        print(f"已在运行 pid={same[0]['pid']}")
        return 0
    driver_path = user_state_dir() / "driver.log"
    driver_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = open(driver_path, "ab", buffering=0)
    try:
        env = os.environ.copy()
        src = str(source_root())
        previous = env.get("PYTHONPATH")
        env["PYTHONPATH"] = src if not previous else src + os.pathsep + previous
        resolved = str(Path(config_path).resolve())
        log.info("start config=%s", resolved)
        process = subprocess.Popen(
            [sys.executable, "-m", "robot.runtime", "serve", resolved],
            start_new_session=True,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            env=env,
            cwd=str(source_root().parent),
        )
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                _report_start_failure(f"启动失败，退出码 {process.returncode}")
                return 1
            matched = [record for record in live_records() if int(record["pid"]) == process.pid]
            if matched:
                print(f"已启动 pid={process.pid} 日志={log_path()}")
                return 0
            time.sleep(0.05)
        _signal(process.pid, signal.SIGTERM)
        _report_start_failure("启动超时")
        return 1
    finally:
        log_file.close()


def stop(*, quiet: bool = False) -> int:
    live = live_records()
    if not live:
        clear_dead_records()
        _say("已停止", quiet=quiet)
        return 0
    for record in live:
        _stop_one(record)
    _say("已停止", quiet=quiet)
    return 0


def _stop_one(record: dict) -> None:
    pid = int(record["pid"])
    if not _signal(pid, signal.SIGTERM):
        _reap_if_child(pid)
        remove_record(shm_name=str(record.get("shm_name") or ""))
        return
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if not pid_alive(pid):
            break
        time.sleep(0.05)
    else:
        _signal(pid, signal.SIGKILL)
        kill_deadline = time.monotonic() + 2.0
        while time.monotonic() < kill_deadline and pid_alive(pid):
            time.sleep(0.05)
    _reap_if_child(pid)
    remove_record(shm_name=str(record.get("shm_name") or ""))


def restart() -> int:
    live = live_records()
    if len(live) != 1:
        print("未运行，无法 restart", file=sys.stderr)
        return 1
    config_path = Path(live[0]["config_path"])
    stopped = stop()
    if stopped != 0:
        return stopped
    return start(config_path)


def status() -> int:
    live = live_records()
    if not live:
        print("stopped")
        return 3
    for record in live:
        print(
            f"running pid={record['pid']} shm={record['shm_name']} config={record['config_path']} log={log_path()}"
        )
    return 0


def _report_start_failure(message: str) -> None:
    get_logger("service").error(message)
    excerpt = tail_log()
    if excerpt:
        print(excerpt, file=sys.stderr)
    print(f"{message}，日志：{log_path()}", file=sys.stderr)


def _say(message: str, *, quiet: bool, error: bool = False) -> None:
    if quiet:
        return
    print(message, file=sys.stderr if error else sys.stdout)


def _signal(pid: int, sig: int) -> bool:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        return False
    return True


def _reap_if_child(pid: int) -> None:
    try:
        os.waitpid(pid, os.WNOHANG)
    except (ChildProcessError, OSError):
        return


