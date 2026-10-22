"""curses 全屏界面。查看状态，并可以在界面里关闭服务。"""

from __future__ import annotations

import curses
import time

from robot.runtime.client import RuntimeClient
from robot.runtime.discover import live_records
from robot.runtime.hmi.model import HmiModel
from robot.runtime.hmi.render import frame
from robot.runtime.service import stop
from robot.runtime.transport import BusCorruptError


def run_hmi() -> int:
    curses.wrapper(_loop)
    return 0


def _loop(stdscr) -> None:
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    stdscr.keypad(True)
    stdscr.timeout(100)
    if curses.has_colors():
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_CYAN, -1)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN)
    model = HmiModel()
    client: RuntimeClient | None = None
    attached = ""
    try:
        while True:
            key = stdscr.getch()
            actions = []
            if key != -1:
                actions.append(key)
                stdscr.nodelay(True)
                while True:
                    extra = stdscr.getch()
                    if extra == -1:
                        break
                    actions.append(extra)
                stdscr.timeout(100)
            for item in actions:
                action = _translate(item, model.confirming_stop)
                if action is not None and not model.apply_key(action):
                    return
                if model.take_action() == "stop":
                    model.notice = "正在关闭服务"
                    _draw(stdscr, model)
                    if client is not None:
                        client.close()
                        client = None
                        attached = ""
                    code = stop(quiet=True)
                    model.notice = "服务已关闭" if code == 0 else "关闭失败"
            client, attached = _poll(model, client, attached)
            _draw(stdscr, model)
    finally:
        if client is not None:
            client.close()


def _poll(model: HmiModel, client: RuntimeClient | None, attached: str) -> tuple[RuntimeClient | None, str]:
    records = live_records()
    if len(records) != 1:
        if client is not None:
            client.close()
        model.set_link(False, 0, attached)
        return None, ""
    record = records[0]
    shm_name = str(record["shm_name"])
    if client is None or attached != shm_name:
        if client is not None:
            client.close()
        try:
            client = RuntimeClient(shm_name)
        except FileNotFoundError:
            model.set_link(False, int(record["pid"]), shm_name)
            model.notice = "共享内存不存在，请重启服务"
            return None, ""
        except BusCorruptError as exc:
            model.set_link(False, int(record["pid"]), shm_name)
            model.notice = str(exc)
            return None, ""
        attached = shm_name
    model.set_link(True, int(record["pid"]), shm_name)
    if client.version < 3:
        model.notice = "服务仍是第 2 版，可以查看。遥操和部署需要先 cyr restart"
    model.set_names([str(name) for name in record.get("components") or []])
    try:
        safety = client.read_safety()
    except Exception:
        safety = {"latched": False, "reason": ""}
    if safety.get("latched"):
        model.notice = "安全保护 " + str(safety.get("reason") or "")
    now_ns = time.time_ns()
    for name in list(model.names):
        try:
            status = client.read_status(name)
            state = client.read_state(name)
        except FileNotFoundError:
            client.close()
            model.set_link(False, 0, shm_name)
            return None, ""
        model.update_component(name, status, state, now_ns)
    return client, attached


def _draw(stdscr, model: HmiModel) -> None:
    height, width = stdscr.getmaxyx()
    lines = frame(model, width, height)
    stdscr.erase()
    for row, line in enumerate(lines):
        if row >= height:
            break
        attr = curses.A_NORMAL
        if row == 0 and curses.has_colors():
            attr = curses.color_pair(1) | curses.A_BOLD
        elif line.startswith(">") and curses.has_colors():
            attr = curses.color_pair(2)
        try:
            stdscr.addnstr(row, 0, line, max(0, width - 1), attr)
        except curses.error:
            pass
    stdscr.refresh()


def _translate(key: int, confirming: bool = False) -> str | None:
    if key in (27,):
        return "escape"
    if key in (10, 13, curses.KEY_ENTER):
        return "enter"
    if confirming:
        if key in (ord("y"), ord("Y")):
            return "yes"
        if key in (ord("n"), ord("N"), ord("q"), ord("Q")):
            return "no"
        return None
    if key in (ord("q"), ord("Q")):
        return "quit"
    if key in (curses.KEY_UP, ord("k")):
        return "up"
    if key in (curses.KEY_DOWN, ord("j")):
        return "down"
    if key in (ord("x"), ord("X")):
        return "stop"
    return None
