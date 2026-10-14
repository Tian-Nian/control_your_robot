"""运行时日志。多进程追加到同一个文件，用来抓启动和组件异常。"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

from robot.runtime.discover import user_state_dir

_READY = False


def log_path() -> Path:
    return user_state_dir() / "cyr.log"


def configure_logging() -> None:
    global _READY
    if _READY:
        return
    path = log_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger("cyr")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-5s [pid=%(process)d %(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler = _AppendHandler(path)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    if sys.stderr.isatty():
        stream = logging.StreamHandler()
        stream.setLevel(logging.INFO)
        stream.setFormatter(formatter)
        logger.addHandler(stream)
    _READY = True


def get_logger(component: str | None = None) -> logging.Logger:
    configure_logging()
    if not component:
        return logging.getLogger("cyr")
    return logging.getLogger(f"cyr.{component}")


def install_excepthook() -> None:
    logger = get_logger("excepthook")

    def _hook(exc_type, exc, tb) -> None:
        if exc_type is SystemExit:
            return
        logger.critical("未捕获异常", exc_info=(exc_type, exc, tb))

    sys.excepthook = _hook


def tail_log(limit: int = 40) -> str:
    try:
        lines = log_path().read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(lines[-limit:])


class _AppendHandler(logging.Handler):
    """一次 write 写完一条记录，避免多进程把同一行写碎。"""

    def __init__(self, path: Path):
        super().__init__()
        self._fd = os.open(path, os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o644)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            payload = (self.format(record) + "\n").encode("utf-8", errors="replace")
            os.write(self._fd, payload)
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        try:
            os.close(self._fd)
        except OSError:
            pass
        super().close()
