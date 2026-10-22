"""策略进程的 TCP 客户端。call 的参数和 XPolicyLab 的 eval_one_episode 一致。"""

from __future__ import annotations

import socket

from robot.deploy.codec import json_to_numpy, numpy_to_json


class ModelClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 9999, timeout: float = 30.0):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def call(self, func_name=None, obs=None):
        self._send({"cmd": func_name, "obs": obs})
        response = self._recv()
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(str(response.get("error") or "策略服务返回错误"))
        if isinstance(response, dict) and "res" in response:
            return response["res"]
        return None

    def close(self) -> None:
        sock = self.sock
        self.sock = None
        if sock is not None:
            sock.close()

    def _send(self, data) -> None:
        raw = numpy_to_json(data).encode("utf-8")
        self.sock.sendall(len(raw).to_bytes(4, "big"))
        self.sock.sendall(raw)

    def _recv(self):
        size = int.from_bytes(self._read(4), "big")
        return json_to_numpy(self._read(size).decode("utf-8"))

    def _read(self, size: int) -> bytes:
        chunks = []
        received = 0
        while received < size:
            chunk = self.sock.recv(min(size - received, 4096))
            if not chunk:
                raise ConnectionError("策略服务断开了")
            chunks.append(chunk)
            received += len(chunk)
        return b"".join(chunks)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
