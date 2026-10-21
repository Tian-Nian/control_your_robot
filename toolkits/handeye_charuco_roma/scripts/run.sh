#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CORE="${HANDEYE_CORE:-$ROOT/../handeye_charuco}"

_sdk_candidates=(
  "${MARVIN_SDK_ROOT:-}"
  "$ROOT/../X-Robot-Pipeline_new/third_party/TJ_FX_ROBOT_CONTRL_SDK"
  "/home/user/kehe/TJ_FX_ROBOT_CONTRL_SDK-master"
)
SDK=""
for c in "${_sdk_candidates[@]}"; do
  if [[ -n "$c" && -f "$c/SDK_PYTHON/libKine.so" ]]; then
    SDK="$c"
    break
  fi
done
SDK="${SDK:-$ROOT/../X-Robot-Pipeline_new/third_party/TJ_FX_ROBOT_CONTRL_SDK}"
export MARVIN_SDK_ROOT="$SDK"

export PYTHONPATH="$ROOT:$CORE:${SDK}/SDK_PYTHON${PYTHONPATH:+:$PYTHONPATH}"
if [[ -d "${SDK}/SDK_PYTHON" ]]; then
  export LD_LIBRARY_PATH="${SDK}/SDK_PYTHON${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

_py_ok() {
  local py="$1"
  if [[ "$py" == */* ]]; then
    [[ -x "$py" ]] || return 1
  else
    command -v "$py" >/dev/null 2>&1 || return 1
  fi
  "$py" -c "import bus_py,cv2" >/dev/null 2>&1
}

PY=""
for c in \
  ${PYTHON:+"$PYTHON"} \
  /home/user/kehe/ether/.venv/bin/python \
  /home/user/miniconda3/envs/roma/bin/python \
  /opt/xrobot/venv/bin/python \
  python3
do
  if _py_ok "$c"; then
    PY="$c"
    break
  fi
done
PY="${PY:-${PYTHON:-python3}}"
exec "$PY" -m handeye_charuco_roma "$@"
