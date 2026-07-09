"""Fast process-shared dict backed by msgpack instead of Manager proxies."""

from __future__ import annotations

from multiprocessing import Array, Lock, Value

from robot.utils.base import msgpack_numpy


class MsgpackSharedDict:
    """Single-writer friendly shared blob for numpy-heavy teleop payloads."""

    def __init__(self, max_bytes: int = 4 * 1024 * 1024):
        self._lock = Lock()
        self._size = Value("i", 0)
        self._buf = Array("B", max_bytes)
        self._max_bytes = max_bytes

    def replace(self, data: dict) -> None:
        packed = msgpack_numpy.packb(data, use_bin_type=True)
        if len(packed) > self._max_bytes:
            raise ValueError(
                f"MsgpackSharedDict overflow: payload={len(packed)} bytes, max={self._max_bytes}"
            )
        with self._lock:
            self._size.value = len(packed)
            self._buf[: len(packed)] = packed

    def read(self) -> dict:
        with self._lock:
            size = self._size.value
            if size <= 0:
                return {}
            payload = bytes(self._buf[:size])
        return msgpack_numpy.unpackb(payload, raw=False)
