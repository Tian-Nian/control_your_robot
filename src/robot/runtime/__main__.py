"""cyr 命令入口。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from robot.runtime.present import format_data
from robot.runtime.service import restart, serve, start, status, stop


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cyr")
    sub = parser.add_subparsers(dest="command", required=True)

    serve_parser = sub.add_parser("serve", help="前台运行，供调试和 systemd 使用")
    serve_parser.add_argument("config")

    start_parser = sub.add_parser("start", help="后台启动服务")
    start_parser.add_argument("--config", required=True)

    sub.add_parser("stop", help="停止服务")
    sub.add_parser("restart", help="用上次的配置重启")
    sub.add_parser("status", help="查看服务进程是否在运行")
    sub.add_parser("hmi", help="打开实时状态界面")

    echo_parser = sub.add_parser("echo", help="打印一个组件的最新观测")
    echo_parser.add_argument("component")

    sub.add_parser("snapshot", help="打印全部组件的最新观测，一行 JSON")

    call_parser = sub.add_parser("call", help="向一个组件发送一条命令，一行 JSON")
    call_parser.add_argument("component")
    call_parser.add_argument("data", help="JSON 对象，复位用 {\"op\":\"reset\"}")
    call_parser.add_argument("--delta", action="store_true", help="把命令当成增量")
    call_parser.add_argument("--timeout", type=float, default=2.0)

    args = parser.parse_args(argv)
    if args.command == "serve":
        return serve(Path(args.config))
    if args.command == "start":
        return start(Path(args.config))
    if args.command == "stop":
        return stop()
    if args.command == "restart":
        return restart()
    if args.command == "status":
        return status()
    if args.command == "hmi":
        from robot.runtime.hmi.app import run_hmi

        return run_hmi()
    if args.command == "echo":
        return echo(args.component)
    if args.command == "snapshot":
        return snapshot()
    if args.command == "call":
        return call(args.component, args.data, is_delta=args.delta, timeout=args.timeout)
    parser.error(f"unknown command {args.command}")
    return 2


def snapshot() -> int:
    from robot.runtime.session import Session

    try:
        session = Session.attach()
    except RuntimeError as exc:
        _print_json({"error": str(exc)})
        return 1
    try:
        _print_json(session.snapshot())
    finally:
        session.close()
    return 0


def call(component: str, payload: str, *, is_delta: bool, timeout: float) -> int:
    from robot.runtime.session import Session

    try:
        data = json.loads(payload)
    except json.JSONDecodeError as exc:
        _print_json({"accepted": False, "seq": None, "error": f"JSON 无效: {exc}", "state": None})
        return 1
    if not isinstance(data, dict):
        _print_json({"accepted": False, "seq": None, "error": "命令必须是 JSON 对象", "state": None})
        return 1
    try:
        session = Session.attach()
    except RuntimeError as exc:
        _print_json({"accepted": False, "seq": None, "error": str(exc), "state": None})
        return 1
    if data.get("op") == "reset" and timeout == 2.0:
        timeout = 15.0
    try:
        result = session.call(component, data, is_delta=is_delta, timeout=timeout)
    finally:
        session.close()
    _print_json(result)
    return 0 if result["accepted"] else 1


def _print_json(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


def echo(component: str) -> int:
    from robot.runtime.session import Session

    try:
        session = Session.attach()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    try:
        names = [item["name"] for item in session.describe()]
        if component not in names:
            print(f"没有组件 {component}", file=sys.stderr)
            return 1
        state = session.read_raw(component)
    finally:
        session.close()
    if state is None:
        print("无数据")
        return 1
    print(f"seq={state['seq']} timestamp_ns={state['timestamp_ns']}")
    print(format_data(state["data"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
