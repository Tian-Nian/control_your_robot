#!/usr/bin/env bash
# Keep the optional Python environment and discovery namespace consistent.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$ROOT/.venv-harness/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "未找到 $PYTHON；请先安装专用 .venv-harness 环境。" >&2
    exit 1
fi
cd "$ROOT"
COMMAND="${1:-status}"
if [[ $# -gt 0 ]]; then shift; fi
# Respect the same discovery directory as a direct cyr invocation. Do not
# silently strand a service launched by the older project-local wrapper.
if [[ "$COMMAND" != report ]]; then
    "$PYTHON" - "$ROOT" <<'PYTHON'
import sys
from pathlib import Path
from robot.runtime.discover import load_records, user_state_dir
from robot.runtime.transport import pid_alive
legacy = Path(sys.argv[1]) / "save/mujoco-runtime/cyr"
if legacy.resolve() != user_state_dir().resolve():
    old = [r for r in load_records(legacy / "runtime.json")
           if r.get("shm_name") == "cyr_piper_mujoco" and pid_alive(int(r["pid"]))]
    if old:
        print(f"旧目录仍有 MuJoCo 服务 pid={old[0]['pid']}；未停止或重置它。", file=sys.stderr)
        print('先退出 agent；迁移会重置场景。在仓库根目录执行：', file=sys.stderr)
        print('XDG_STATE_HOME="$PWD/save/mujoco-runtime" .venv-harness/bin/python -m robot.runtime stop', file=sys.stderr)
        print('unset XDG_STATE_HOME', file=sys.stderr)
        print('bash examples/piper_mujoco.sh start', file=sys.stderr)
        print('若要保留旧场景，临时 export XDG_STATE_HOME="$PWD/save/mujoco-runtime" 后继续使用。', file=sys.stderr)
        sys.exit(2)
PYTHON
fi
case "$COMMAND" in
    start)
        "$PYTHON" -m robot.runtime start --config "$ROOT/config/runtime/piper_mujoco.yml" "$@"
        exec "$PYTHON" "$ROOT/examples/piper_mujoco_viewer.py" --ensure
        ;;
    stop|restart|status) exec "$PYTHON" -m robot.runtime "$COMMAND" "$@" ;;
    viewer) exec "$PYTHON" "$ROOT/examples/piper_mujoco_viewer.py" --ensure "$@" ;;
    agent)
        continuing=false
        for option in "$@"; do
            if [[ "$option" == --resume || "$option" == --resume=* ]]; then continuing=true; fi
        done
        if [[ "$continuing" == false ]]; then
            "$PYTHON" -m robot.runtime start --config "$ROOT/config/runtime/piper_mujoco.yml"
        fi
        "$PYTHON" "$ROOT/examples/piper_mujoco_viewer.py" --ensure
        exec "$PYTHON" "$ROOT/examples/piper_mujoco_agent.py" --attach "$@"
        ;;
    report) exec "$PYTHON" "$ROOT/examples/piper_mujoco_report.py" "$@" ;;
    resume)
        if [[ $# -lt 1 ]]; then
            echo "用法：bash examples/piper_mujoco.sh resume save/vision_<id> [agent 参数]" >&2
            exit 2
        fi
        SOURCE_RUN="$1"
        shift
        # Continue only the existing scene; do not silently create a reset scene.
        "$PYTHON" "$ROOT/examples/piper_mujoco_viewer.py" --ensure
        exec "$PYTHON" "$ROOT/examples/piper_mujoco_agent.py" --attach --resume "$SOURCE_RUN" "$@"
        ;;
    teleop)
        "$PYTHON" -m robot.runtime start --config "$ROOT/config/runtime/piper_mujoco.yml"
        "$PYTHON" "$ROOT/examples/piper_mujoco_viewer.py" --ensure
        exec "$PYTHON" "$ROOT/examples/piper_mujoco_teleop.py" --attach "$@"
        ;;
    pick-test) exec "$PYTHON" "$ROOT/examples/piper_mujoco_harness.py" --attach --pick-test "$@" ;;
    *) echo "用法：bash examples/piper_mujoco.sh {start|stop|restart|status|viewer|agent|resume|report|teleop|pick-test} [参数]" >&2; exit 2 ;;
esac
