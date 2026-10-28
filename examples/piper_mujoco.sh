#!/usr/bin/env bash
# Keep the optional Python environment and discovery namespace consistent.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="$ROOT/.venv-harness/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "未找到 $PYTHON；请先安装专用 .venv-harness 环境。" >&2
    exit 1
fi
export XDG_STATE_HOME="$ROOT/save/mujoco-runtime"
cd "$ROOT"
COMMAND="${1:-status}"
if [[ $# -gt 0 ]]; then shift; fi
case "$COMMAND" in
    start) exec "$PYTHON" -m robot.runtime start --config "$ROOT/config/runtime/piper_mujoco.yml" "$@" ;;
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
