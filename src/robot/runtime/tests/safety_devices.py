"""用来确认故障和关闭都会走到安全停止。"""

from __future__ import annotations

import time
from pathlib import Path


class HoldDevice:
    def __init__(self, marker: str):
        self.marker = marker

    def set_up(self):
        return None

    def set_collect_info(self, _info):
        return None

    def get(self):
        return {"joint": [0.0], "timestamp": time.time_ns()}

    def safe_stop(self) -> str:
        Path(self.marker).write_text("stopped", encoding="utf-8")
        return "hold"


class CrashDevice:
    def __init__(self):
        self.ticks = 0

    def set_up(self):
        return None

    def set_collect_info(self, _info):
        return None

    def get(self):
        self.ticks += 1
        if self.ticks > 3:
            raise SystemExit(1)
        return {"joint": [1.0], "timestamp": time.time_ns()}

    def safe_stop(self) -> str:
        return "hold"
