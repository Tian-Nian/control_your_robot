"""共享内存槽位。锁内只拷贝字节，调用方在锁外 pack/unpack。"""

from __future__ import annotations

import os
import math
import secrets
import struct
import time
from contextlib import contextmanager
from dataclasses import dataclass

from robot.utils.base.msgpack_numpy import packb, unpackb

MAGIC = b"CYR1"
VERSION = 3
NAME_LEN = 64
STATUS_MAX = 4096
CMD_SLOT_MAX = 65536
MAX_COMPONENTS = 32

_HEADER = struct.Struct("<4sIIIII")
_DIR = struct.Struct(f"<{NAME_LEN}sIIIIII")
_SLOT_HDR = struct.Struct("<QII")
_CMD_HDR = struct.Struct("<QQQQ")
_SAFETY_REASON = 256
_SAFETY = struct.Struct(f"<QI{_SAFETY_REASON}s")
_CONTROL_ROLE = 32
_CONTROL = struct.Struct(f"<QQII{_CONTROL_ROLE}s")
STREAM_MAX = 65536
_ROLE_LABEL = {"teleop": "遥操", "model": "部署", "replay": "回放"}

assert _HEADER.size == 24
assert _SLOT_HDR.size == 16
assert _CMD_HDR.size == 32


class BusBusyError(RuntimeError):
    """同名共享内存的属主进程还活着。"""


class BusCorruptError(RuntimeError):
    """共享内存头部无法识别。"""


class LayoutMismatchError(RuntimeError):
    """服务还是旧布局。读观测可以，流式目标和长线控制不行。"""

    def __init__(self, found: int, expected: int):
        self.found = int(found)
        self.expected = int(expected)
        super().__init__(
            f"服务仍是第 {self.found} 版共享内存，当前程序是第 {self.expected} 版。先 cyr restart，再遥操或部署"
        )


class SafetyStopError(RuntimeError):
    """安全保护已经锁存，命令被拒绝。"""


class ControlBusyError(RuntimeError):
    """长线控制正持有锁，这条写入没有令牌。"""

    def __init__(self, label: str):
        self.label = label
        super().__init__(f"控制锁被{label}持有")


def pid_alive(pid: int) -> bool:
    """进程还在跑才算活着。已经退出、尚未被父进程回收的僵尸不算。"""
    if pid <= 0:
        return False
    if not os.path.isdir("/proc"):
        return _pid_alive_without_proc(pid)
    try:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
            stat = handle.read()
    except FileNotFoundError:
        return False
    except OSError:
        return _pid_alive_by_signal(pid)
    end = stat.rfind(")")
    if end < 0 or end + 2 >= len(stat):
        return True
    return stat[end + 2] != "Z"


def _pid_alive_without_proc(pid: int) -> bool:
    """macOS 没有 /proc。用信号判断进程是否存在，再用 ps 排除僵尸。"""
    if not _pid_alive_by_signal(pid):
        return False
    import subprocess

    try:
        stat = subprocess.check_output(
            ["ps", "-o", "stat=", "-p", str(pid)],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return _pid_alive_by_signal(pid)
    return not stat.startswith("Z")


def _pid_alive_by_signal(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _safety_offset() -> int:
    return _align(_HEADER.size + MAX_COMPONENTS * _DIR.size)


def _control_offset() -> int:
    return _align(_safety_offset() + _SAFETY.size)


def _stream_offset(status_off: int, status_max: int) -> int:
    return _align(status_off + _SLOT_HDR.size + status_max)


def _align(value: int, boundary: int = 8) -> int:
    return (value + boundary - 1) & ~(boundary - 1)


@dataclass(frozen=True)
class SlotSpec:
    name: str
    state_max: int


@dataclass
class _Component:
    name: str
    state_max: int
    cmd_slot_max: int
    status_max: int
    state_off: int
    cmd_off: int
    status_off: int
    stream_off: int
    stream_max: int


def _shm_dir() -> str:
    """锁文件目录。Linux 用 /dev/shm；macOS 没有这个目录，放到临时目录。"""
    if os.path.isdir("/dev/shm"):
        return "/dev/shm"
    path = os.path.join(os.environ.get("TMPDIR") or "/tmp", "cyr-shm")
    os.makedirs(path, exist_ok=True)
    return path


def _lock_path(shm_name: str, component: str) -> str:
    return os.path.join(_shm_dir(), f"{shm_name}.{component}.lock")


def _shm_fs_path(shm_name: str) -> str:
    return os.path.join(_shm_dir(), shm_name.lstrip("/"))


class _Locks:
    def __init__(self, shm_name: str, names: list[str], *, control: bool = True):
        self._fds: dict[str, int] = {}
        names = [*names, "safety"]
        if control:
            names.append("control")
        self._paths = [_lock_path(shm_name, name) for name in names]
        for name in names:
            path = _lock_path(shm_name, name)
            fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
            os.chmod(path, 0o666)
            self._fds[name] = fd

    @contextmanager
    def hold(self, name: str):
        fd = self._fds[name]
        import fcntl

        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)

    def close(self) -> None:
        for fd in self._fds.values():
            os.close(fd)
        self._fds.clear()

    def unlink_files(self) -> None:
        for path in self._paths:
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass


def _read_owner(shm) -> int:
    if len(shm.buf) < _HEADER.size:
        raise BusCorruptError("shared memory is too small")
    magic, version, owner_pid, _count, _capacity, _reserved = _HEADER.unpack_from(shm.buf, 0)
    if magic != MAGIC or int(version) not in (2, 3):
        raise BusCorruptError(
            f"共享内存无法识别（magic={magic!r}，第 {int(version)} 版）。当前程序认识第 2 和第 3 版"
        )
    return int(owner_pid)


class Bus:
    def __init__(self, shm, locks: _Locks, components: dict[str, _Component], cmd_capacity: int, version: int):
        self._shm = shm
        self._locks = locks
        self._components = components
        self._cmd_capacity = cmd_capacity
        self._version = int(version)
        self._closed = False

    @property
    def version(self) -> int:
        return self._version

    @property
    def name(self) -> str:
        return self._shm.name.lstrip("/")

    @property
    def cmd_capacity(self) -> int:
        return self._cmd_capacity

    def component_names(self) -> list[str]:
        return list(self._components)

    @classmethod
    def create(cls, shm_name: str, slots: list[SlotSpec], cmd_capacity: int, owner_pid: int | None = None) -> Bus:
        _release_stale(shm_name)
        components, total = _layout(slots, cmd_capacity)
        previous = os.umask(0o022)
        try:
            shm = _open_shared_memory(shm_name, create=True, size=total)
        finally:
            os.umask(previous)
        try:
            os.chmod(_shm_fs_path(shm.name), 0o644)
        except OSError:
            pass
        pid = os.getpid() if owner_pid is None else owner_pid
        _HEADER.pack_into(shm.buf, 0, MAGIC, VERSION, pid, len(slots), cmd_capacity, 0)
        for index, comp in enumerate(components.values()):
            encoded = comp.name.encode("utf-8")
            _DIR.pack_into(
                shm.buf,
                _HEADER.size + index * _DIR.size,
                encoded,
                comp.state_max,
                comp.cmd_slot_max,
                comp.status_max,
                comp.state_off,
                comp.cmd_off,
                comp.status_off,
            )
        try:
            locks = _Locks(shm_name, list(components))
        except Exception:
            _destroy_segment(shm)
            shm.close()
            raise
        return cls(shm, locks, components, cmd_capacity, VERSION)

    @classmethod
    def attach(cls, shm_name: str) -> Bus:
        shm = _open_shared_memory(shm_name, create=False)
        try:
            _read_owner(shm)
            _magic, version, _pid, count, capacity, _reserved = _HEADER.unpack_from(shm.buf, 0)
            version = int(version)
            components: dict[str, _Component] = {}
            for index in range(int(count)):
                raw_name, state_max, cmd_slot_max, status_max, state_off, cmd_off, status_off = _DIR.unpack_from(
                    shm.buf, _HEADER.size + index * _DIR.size
                )
                name = raw_name.split(b"\0", 1)[0].decode("utf-8")
                has_stream = version >= 3
                components[name] = _Component(
                    name=name,
                    state_max=int(state_max),
                    cmd_slot_max=int(cmd_slot_max),
                    status_max=int(status_max),
                    state_off=int(state_off),
                    cmd_off=int(cmd_off),
                    status_off=int(status_off),
                    stream_off=_stream_offset(int(status_off), int(status_max)) if has_stream else 0,
                    stream_max=STREAM_MAX if has_stream else 0,
                )
            locks = _Locks(shm_name, list(components), control=version >= 3)
        except Exception:
            shm.close()
            raise
        return cls(shm, locks, components, int(capacity), version)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._locks.close()
        self._shm.close()

    def unlink(self) -> None:
        """属主退出时删除段和锁文件。"""
        if not self._closed:
            self._locks.close()
            self._closed = True
        self._locks.unlink_files()
        _destroy_segment(self._shm)
        self._shm.close()

    def write_state(self, name: str, body: dict) -> bool:
        comp = self._require(name)
        payload = packb(body, use_bin_type=True)
        if len(payload) > comp.state_max:
            return False
        with self._locks.hold(name):
            seq, _nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.state_off)
            self._write_bytes(comp.state_off + _SLOT_HDR.size, payload)
            _SLOT_HDR.pack_into(self._shm.buf, comp.state_off, int(seq) + 1, len(payload), 0)
        return True

    def read_state(self, name: str) -> dict | None:
        comp = self._require(name)
        with self._locks.hold(name):
            seq, nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.state_off)
            if seq == 0 or nbytes <= 0 or nbytes > comp.state_max:
                return None
            blob = bytes(self._shm.buf[comp.state_off + _SLOT_HDR.size : comp.state_off + _SLOT_HDR.size + nbytes])
        body = unpackb(blob, raw=False)
        return {"seq": int(seq), "timestamp_ns": int(body["timestamp_ns"]), "data": body["data"]}

    def latch_safety(self, reason: str) -> bool:
        """锁存安全保护。已经锁存时保留第一次的原因。"""
        text = reason.encode("utf-8", errors="replace")[:_SAFETY_REASON]
        with self._locks.hold("safety"):
            seq, latched, _raw = _SAFETY.unpack_from(self._shm.buf, _safety_offset())
            if int(latched):
                return False
            _SAFETY.pack_into(self._shm.buf, _safety_offset(), int(seq) + 1, 1, text)
        return True

    def read_safety(self) -> dict:
        with self._locks.hold("safety"):
            seq, latched, raw = _SAFETY.unpack_from(self._shm.buf, _safety_offset())
        return {
            "seq": int(seq),
            "latched": bool(latched),
            "reason": raw.split(b"\0", 1)[0].decode("utf-8", errors="replace"),
        }

    def read_control(self) -> dict:
        """当前长线控制。过期视为无人持有，不改共享内存。"""
        if self._version < 3:
            return {"held": False, "role": "", "label": ""}
        with self._locks.hold("control"):
            view = self._control_view(time.monotonic_ns())
        return {"held": view["active"], "role": view["role"], "label": view["label"]}

    def acquire_control(self, role: str, lease_s: float = 2.0) -> int:
        """取得长线控制。环里还没执行的命令丢掉，已经在跟踪的那一段继续走完。"""
        self._require_layout()
        label = _ROLE_LABEL.get(role)
        if label is None:
            raise ValueError("role 只能是 teleop、model 或 replay")
        if lease_s <= 0:
            raise ValueError("lease_s 必须为正")
        token = secrets.randbits(63) or 1
        with self._locks.hold("control"):
            view = self._control_view(time.monotonic_ns())
            if view["active"]:
                raise ControlBusyError(view["label"])
            self._write_control(token, role, lease_s)
        for name in self._components:
            self._drop_pending(name)
            self._drop_stream(name)
        return token

    def renew_control(self, token: int, lease_s: float = 2.0) -> bool:
        """目标不变也要续。令牌不对或已经过期就失败。"""
        if self._version < 3:
            return False
        if lease_s <= 0:
            raise ValueError("lease_s 必须为正")
        with self._locks.hold("control"):
            view = self._control_view(time.monotonic_ns())
            if not view["active"] or int(view["token"]) != int(token):
                return False
            self._write_control(int(token), view["role"], lease_s)
        return True

    def release_control(self, token: int) -> bool:
        if self._version < 3:
            return False
        with self._locks.hold("control"):
            view = self._control_view(time.monotonic_ns())
            if int(view["token"]) != int(token) or not view["token"]:
                return False
            _CONTROL.pack_into(self._shm.buf, _control_offset(), 0, 0, 0, 0, b"")
        return True

    def write_cmd(self, name: str, data: dict, is_delta: bool = False, token: int = 0) -> int:
        self._check_writer(token)
        comp = self._require(name)
        payload = packb({"is_delta": bool(is_delta), "data": data}, use_bin_type=True)
        if len(payload) > comp.cmd_slot_max:
            raise ValueError(f"{name}: command is {len(payload)} bytes, max is {comp.cmd_slot_max}")
        with self._locks.hold(name):
            write_seq, read_seq, drops, _reserved = _CMD_HDR.unpack_from(self._shm.buf, comp.cmd_off)
            if int(write_seq) - int(read_seq) >= self._cmd_capacity:
                read_seq = int(read_seq) + 1
                drops = int(drops) + 1
            seq = int(write_seq) + 1
            slot = (seq - 1) % self._cmd_capacity
            offset = self._cmd_slot_offset(comp, slot)
            self._write_bytes(offset + _SLOT_HDR.size, payload)
            _SLOT_HDR.pack_into(self._shm.buf, offset, seq, len(payload), 0)
            _CMD_HDR.pack_into(self._shm.buf, comp.cmd_off, seq, int(read_seq), int(drops), 0)
        return seq

    def pop_cmd(self, name: str) -> dict | None:
        comp = self._require(name)
        with self._locks.hold(name):
            write_seq, read_seq, drops, _reserved = _CMD_HDR.unpack_from(self._shm.buf, comp.cmd_off)
            if int(read_seq) >= int(write_seq):
                return None
            seq = int(read_seq) + 1
            slot = (seq - 1) % self._cmd_capacity
            offset = self._cmd_slot_offset(comp, slot)
            stored_seq, nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, offset)
            if int(stored_seq) != seq or nbytes <= 0 or nbytes > comp.cmd_slot_max:
                raise BusCorruptError(f"{name}: command slot {seq} is inconsistent")
            blob = bytes(self._shm.buf[offset + _SLOT_HDR.size : offset + _SLOT_HDR.size + nbytes])
            _CMD_HDR.pack_into(self._shm.buf, comp.cmd_off, int(write_seq), seq, int(drops), 0)
        body = unpackb(blob, raw=False)
        return {"seq": seq, "is_delta": bool(body["is_delta"]), "data": body["data"]}

    def write_stream(self, name: str, data: dict, token: int = 0, valid_for_s: float | None = None) -> int:
        """盖住这份最新目标。不进命令环，调用方也不等它被执行。"""
        self._require_layout()
        self._check_writer(token)
        comp = self._require(name)
        written_ns = time.monotonic_ns()
        deadline_ns = 0
        if valid_for_s is not None:
            ttl = float(valid_for_s)
            if not math.isfinite(ttl) or ttl <= 0:
                raise ValueError("valid_for_s must be finite and positive")
            deadline_ns = written_ns + int(ttl * 1e9)
        payload = packb({"data": data, "written_ns": written_ns, "deadline_ns": deadline_ns}, use_bin_type=True)
        if len(payload) > comp.stream_max:
            raise ValueError(f"{name}: stream is {len(payload)} bytes, max is {comp.stream_max}")
        with self._locks.hold(name):
            seq, _nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.stream_off)
            seq = int(seq) + 1
            self._write_bytes(comp.stream_off + _SLOT_HDR.size, payload)
            _SLOT_HDR.pack_into(self._shm.buf, comp.stream_off, seq, len(payload), 0)
        return seq

    def read_stream(self, name: str) -> dict | None:
        if self._version < 3:
            return None
        comp = self._require(name)
        with self._locks.hold(name):
            seq, nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.stream_off)
            if seq == 0 or nbytes <= 0 or nbytes > comp.stream_max:
                return None
            blob = bytes(self._shm.buf[comp.stream_off + _SLOT_HDR.size : comp.stream_off + _SLOT_HDR.size + nbytes])
        body = unpackb(blob, raw=False)
        return {"seq": int(seq), "data": body["data"],
                "written_ns": int(body.get("written_ns", 0)),
                "deadline_ns": int(body.get("deadline_ns", 0))}

    def cmd_cursors(self, name: str) -> tuple[int, int, int]:
        comp = self._require(name)
        with self._locks.hold(name):
            write_seq, read_seq, drops, _reserved = _CMD_HDR.unpack_from(self._shm.buf, comp.cmd_off)
        return int(read_seq), int(write_seq), int(drops)

    def write_status(self, name: str, body: dict) -> None:
        comp = self._require(name)
        payload = packb(body, use_bin_type=True)
        if len(payload) > comp.status_max:
            payload = packb({"alive": False, "error": "status exceeds max_bytes"}, use_bin_type=True)
        with self._locks.hold(name):
            seq, _nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.status_off)
            self._write_bytes(comp.status_off + _SLOT_HDR.size, payload)
            _SLOT_HDR.pack_into(self._shm.buf, comp.status_off, int(seq) + 1, len(payload), 0)

    def read_status(self, name: str) -> dict | None:
        comp = self._require(name)
        with self._locks.hold(name):
            seq, nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.status_off)
            if seq == 0 or nbytes <= 0 or nbytes > comp.status_max:
                return None
            blob = bytes(self._shm.buf[comp.status_off + _SLOT_HDR.size : comp.status_off + _SLOT_HDR.size + nbytes])
        return unpackb(blob, raw=False)

    def _check_writer(self, token: int) -> None:
        safety = self.read_safety()
        if safety["latched"]:
            raise SafetyStopError(f"安全保护已触发: {safety['reason']}")
        if self._version < 3:
            return
        with self._locks.hold("control"):
            view = self._control_view(time.monotonic_ns())
        if view["active"] and int(token) != int(view["token"]):
            raise ControlBusyError(view["label"])

    def _control_view(self, now_ns: int) -> dict:
        token, expires, pid, _pad, raw = _CONTROL.unpack_from(self._shm.buf, _control_offset())
        role = raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")
        active = bool(token) and int(expires) > int(now_ns) and pid_alive(int(pid))
        return {
            "token": int(token),
            "role": role if active else "",
            "label": _ROLE_LABEL.get(role, role) if active else "",
            "active": active,
        }

    def _write_control(self, token: int, role: str, lease_s: float) -> None:
        expires = time.monotonic_ns() + int(float(lease_s) * 1e9)
        _CONTROL.pack_into(
            self._shm.buf,
            _control_offset(),
            int(token),
            int(expires),
            os.getpid(),
            0,
            role.encode("utf-8"),
        )

    def _require_layout(self) -> None:
        if self._version < 3:
            raise LayoutMismatchError(self._version, VERSION)

    def _drop_stream(self, name: str) -> None:
        """换持有者时丢掉上一份最新目标。已经走到的位置保持不动。"""
        comp = self._components[name]
        with self._locks.hold(name):
            seq, _nbytes, _pad = _SLOT_HDR.unpack_from(self._shm.buf, comp.stream_off)
            _SLOT_HDR.pack_into(self._shm.buf, comp.stream_off, int(seq) + 1, 0, 0)

    def _drop_pending(self, name: str) -> None:
        comp = self._components[name]
        with self._locks.hold(name):
            write_seq, read_seq, drops, _reserved = _CMD_HDR.unpack_from(self._shm.buf, comp.cmd_off)
            skipped = max(0, int(write_seq) - int(read_seq))
            _CMD_HDR.pack_into(
                self._shm.buf,
                comp.cmd_off,
                int(write_seq),
                int(write_seq),
                int(drops) + skipped,
                0,
            )

    def _require(self, name: str) -> _Component:
        try:
            return self._components[name]
        except KeyError as exc:
            raise KeyError(f"unknown component {name!r}") from exc

    def _cmd_slot_offset(self, comp: _Component, slot: int) -> int:
        stride = _SLOT_HDR.size + comp.cmd_slot_max
        return comp.cmd_off + _CMD_HDR.size + slot * stride

    def _write_bytes(self, offset: int, payload: bytes) -> None:
        self._shm.buf[offset : offset + len(payload)] = payload


def _layout(slots: list[SlotSpec], cmd_capacity: int) -> tuple[dict[str, _Component], int]:
    if not slots:
        raise ValueError("at least one component is required")
    if len(slots) > MAX_COMPONENTS:
        raise ValueError(f"at most {MAX_COMPONENTS} components")
    if cmd_capacity < 1:
        raise ValueError("cmd_capacity must be positive")
    cursor = _align(_control_offset() + _CONTROL.size)
    components: dict[str, _Component] = {}
    for slot in slots:
        if slot.name in components:
            raise ValueError(f"duplicate component {slot.name}")
        state_off = cursor
        cursor = _align(cursor + _SLOT_HDR.size + slot.state_max)
        cmd_off = cursor
        cursor = _align(cursor + _CMD_HDR.size + cmd_capacity * (_SLOT_HDR.size + CMD_SLOT_MAX))
        status_off = cursor
        cursor = _align(cursor + _SLOT_HDR.size + STATUS_MAX)
        stream_off = cursor
        cursor = _align(cursor + _SLOT_HDR.size + STREAM_MAX)
        components[slot.name] = _Component(
            name=slot.name,
            state_max=slot.state_max,
            cmd_slot_max=CMD_SLOT_MAX,
            status_max=STATUS_MAX,
            state_off=state_off,
            cmd_off=cmd_off,
            status_off=status_off,
            stream_off=stream_off,
            stream_max=STREAM_MAX,
        )
    return components, cursor


def _open_shared_memory(name: str, *, create: bool, size: int = 0):
    """打开段，并立刻从 resource tracker 摘掉。

    CPython 会在进程退出时让 resource tracker 按名字 unlink。HMI 这类后来
    attach 的进程如果也登记了，一退出就会把服务还在用的段从 /dev/shm 删掉。
    段的生命周期只由属主的 unlink 负责。
    """
    from multiprocessing import shared_memory

    shm = shared_memory.SharedMemory(name=name, create=create, size=size)
    _drop_tracker_claim(shm)
    return shm


def _drop_tracker_claim(shm) -> None:
    name = getattr(shm, "_name", None)
    if not name:
        return
    try:
        from multiprocessing import resource_tracker

        resource_tracker.unregister(name, "shared_memory")
    except Exception:
        return


def _destroy_segment(shm) -> None:
    name = getattr(shm, "_name", None)
    if not name:
        return
    try:
        import _posixshmem

        _posixshmem.shm_unlink(name)
    except FileNotFoundError:
        pass


def _release_stale(shm_name: str) -> None:
    try:
        existing = _open_shared_memory(shm_name, create=False)
    except FileNotFoundError:
        return
    try:
        owner = _read_owner(existing)
    except BusCorruptError:
        existing.close()
        raise BusCorruptError(
            f"shared memory {shm_name!r} exists but is not a runtime segment; remove /dev/shm/{shm_name}"
        ) from None
    existing.close()
    if pid_alive(owner):
        raise BusBusyError(f"runtime already running pid={owner} shm={shm_name}")
    stale = _open_shared_memory(shm_name, create=False)
    _destroy_segment(stale)
    stale.close()
    root = _shm_dir()
    try:
        names = os.listdir(root)
    except FileNotFoundError:
        return
    for path in names:
        prefix = f"{shm_name}."
        if path.startswith(prefix) and path.endswith(".lock"):
            try:
                os.unlink(os.path.join(root, path))
            except FileNotFoundError:
                pass
