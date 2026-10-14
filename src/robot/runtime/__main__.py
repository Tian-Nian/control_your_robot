"""cyr 命令入口。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from robot.runtime.client import RuntimeClient
from robot.runtime.discover import live_records
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
    parser.error(f"unknown command {args.command}")
    return 2


def echo(component: str) -> int:
    records = live_records()
    if not records:
        print("未运行", file=sys.stderr)
        return 1
    client = RuntimeClient(str(records[0]["shm_name"]))
    try:
        if component not in client.component_names():
            print(f"没有组件 {component}", file=sys.stderr)
            return 1
        state = client.read_state(component)
    finally:
        client.close()
    if state is None:
        print("无数据")
        return 1
    print(f"seq={state['seq']} timestamp_ns={state['timestamp_ns']}")
    print(format_data(state["data"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
