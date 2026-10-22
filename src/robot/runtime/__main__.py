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

    teleop_parser = sub.add_parser("teleop", help="关节主从。源的关节原样写给从臂")
    teleop_parser.add_argument("--source", required=True)
    teleop_parser.add_argument("--follower", required=True)
    teleop_parser.add_argument("--rate", type=float, default=50.0)
    teleop_parser.add_argument("--lease", type=float, default=2.0)

    collect_parser = sub.add_parser("collect", help="按保存频率把最新观测写成 HDF5")
    collect_parser.add_argument("--task", required=True)
    collect_parser.add_argument("--out", default="save")
    collect_parser.add_argument("--freq", type=float, default=30.0)

    replay_parser = sub.add_parser("replay", help="用采集文件里的关节走部署的 take_action")
    replay_parser.add_argument("--file", required=True)
    replay_parser.add_argument("--hz", type=float, default=30.0)
    replay_parser.add_argument("--lease", type=float, default=2.0)

    deploy_parser = sub.add_parser("deploy", help="把运行时交给 XPolicyLab 的 eval_one_episode")
    deploy_parser.add_argument("--policy", required=True)
    deploy_parser.add_argument("--instruction", default="")
    deploy_parser.add_argument("--hz", type=float, default=25.0)
    deploy_parser.add_argument("--steps", type=int, default=0)
    deploy_parser.add_argument("--host", default="127.0.0.1")
    deploy_parser.add_argument("--port", type=int, default=9999)
    deploy_parser.add_argument("--lease", type=float, default=2.0)

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
    if args.command == "teleop":
        return teleop(args.source, args.follower, args.rate, args.lease)
    if args.command == "collect":
        return collect(args.task, args.out, args.freq)
    if args.command == "replay":
        return replay_file(args.file, args.hz, args.lease)
    if args.command == "deploy":
        return deploy(args.policy, args.instruction, args.hz, args.steps, args.host, args.port, args.lease)
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


def teleop(source: str, follower: str, rate: float, lease: float) -> int:
    import threading

    from robot.runtime.session import Session
    from robot.teleop.joint_follow import follow

    try:
        session = Session.attach()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    stop = threading.Event()
    try:
        follow(session, source, follower, rate, stop, lease)
    except KeyboardInterrupt:
        stop.set()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        session.close()
    return 0


def collect(task: str, out: str, freq: float) -> int:
    import threading

    from robot.data.hdf5_episode import collect_until
    from robot.runtime.session import Session

    try:
        session = Session.attach()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print("回车开始采集，再回车结束。采集只读，不复位。")
    try:
        input()
    except EOFError:
        session.close()
        return 1
    stop = threading.Event()

    def _finish() -> None:
        try:
            input()
        except EOFError:
            pass
        stop.set()

    threading.Thread(target=_finish, daemon=True).start()
    try:
        path = collect_until(session, task, out, freq, stop)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        session.close()
    print(path)
    return 0


def replay_file(path: str, hz: float, lease: float) -> int:
    from robot.data.xspark import load_actions
    from robot.harness.runtime_env import RuntimeEnv, replay
    from robot.runtime.session import Session

    try:
        session = Session.attach()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    try:
        actions = load_actions(path)
        env = RuntimeEnv(session, control_hz=hz, lease_s=lease, role="replay", max_steps=len(actions))
        try:
            replay(env, actions)
        finally:
            env.close()
    except (OSError, ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        session.close()
    return 0


def deploy(policy: str, instruction: str, hz: float, steps: int, host: str, port: int, lease: float) -> int:
    from robot.deploy.model_client import ModelClient
    from robot.harness.runtime_env import RuntimeEnv, run_policy
    from robot.runtime.session import Session

    try:
        session = Session.attach()
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    try:
        env = RuntimeEnv(
            session,
            control_hz=hz,
            instruction=instruction,
            max_steps=steps,
            lease_s=lease,
            role="model",
        )
    except RuntimeError as exc:
        session.close()
        print(str(exc), file=sys.stderr)
        return 1
    client = None
    try:
        client = ModelClient(host, port)
        run_policy(env, client, policy)
    except (RuntimeError, OSError, ImportError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if client is not None:
            client.close()
        env.close()
        session.close()
    return 0


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
