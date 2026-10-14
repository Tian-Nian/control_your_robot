"""墨现触觉手套只读。串口收 460 通道压力，不写设备。"""

from __future__ import annotations

import os
import threading
import time

import numpy as np

from robot.sensor.sensor import Sensor

FRAME_TOTAL = 468
CHANNELS = 460


def parse_frame(frame: bytes) -> np.ndarray | None:
    if len(frame) != FRAME_TOTAL:
        return None
    if frame[0] != 0xA5 or frame[1] != 0x5A or frame[2] != 0x01:
        return None
    frame_len = frame[3] | (frame[4] << 8)
    if frame_len != 466 or frame[5] != 0x01:
        return None
    checksum = frame[466] | (frame[467] << 8)
    if checksum != (sum(frame[:466]) & 0xFFFF):
        return None
    return np.frombuffer(frame[6:466], dtype=np.uint8).copy()


class MoxianTactileSensor(Sensor):
    def __init__(self, name: str = "moxian_tactile"):
        super().__init__()
        self.name = name
        self.type = "tactile"
        self._ser = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._pressure: np.ndarray | None = None

    def set_up(self, device: str, baudrate: int = 921600):
        import serial

        if not os.path.exists(device):
            raise FileNotFoundError(f"{self.name}: {device} 不存在")
        self._ser = serial.Serial(device, int(baudrate), timeout=0.1)
        self._ser.reset_input_buffer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name=f"{self.name}_serial", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._pressure is not None:
                    return
            time.sleep(0.02)
        raise RuntimeError(f"{self.name} 没有读到压力帧")

    def get_information(self):
        with self._lock:
            if self._pressure is None:
                raise RuntimeError(f"{self.name} 没有压力")
            return {"pressure": self._pressure.astype(np.float32, copy=True)}

    def safe_stop(self) -> str:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        ser = self._ser
        self._ser = None
        if ser is not None:
            ser.close()
        return "hold"

    def _loop(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            ser = self._ser
            if ser is None:
                return
            try:
                data = ser.read(ser.in_waiting or 1)
            except Exception:
                time.sleep(0.01)
                continue
            if not data:
                continue
            buffer.extend(data)
            while len(buffer) >= FRAME_TOTAL:
                if buffer[0] == 0xA5 and buffer[1] == 0x5A:
                    frame = parse_frame(bytes(buffer[:FRAME_TOTAL]))
                    buffer = buffer[FRAME_TOTAL:]
                    if frame is None or frame.size != CHANNELS:
                        continue
                    with self._lock:
                        self._pressure = frame
                else:
                    del buffer[0]
